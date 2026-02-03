//! Build recovery and retry mechanisms

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use backoff::backoff::Backoff;
use backoff::{ExponentialBackoff, ExponentialBackoffBuilder};
use futures::Future;
use serde::{Deserialize, Serialize};
use tracing::{debug, info, warn};

use crate::error::{BuildError, Result};

/// Different recovery strategies for build failures
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BuildRecoveryStrategy {
    /// Don't attempt recovery
    None,
    /// Retry the failed operation
    Retry,
    /// Clean environment and retry
    CleanRetry,
    /// Skip the failed package and continue
    Skip,
    /// Attempt graceful degradation
    Graceful,
}

/// Configuration for retry behavior
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RetryConfig {
    /// Maximum number of retry attempts
    pub max_attempts: u32,
    /// Initial delay between retries
    pub initial_delay: Duration,
    /// Maximum delay between retries
    pub max_delay: Duration,
    /// Multiplier for exponential backoff
    pub multiplier: f64,
    /// Maximum total time spent retrying
    pub max_elapsed_time: Option<Duration>,
}

impl Default for RetryConfig {
    fn default() -> Self {
        Self {
            max_attempts: 3,
            initial_delay: Duration::from_millis(500),
            max_delay: Duration::from_secs(30),
            multiplier: 2.0,
            max_elapsed_time: Some(Duration::from_secs(300)), // 5 minutes
        }
    }
}

/// Recovery context containing information about the failure
#[derive(Debug, Clone)]
pub struct RecoveryContext {
    /// The operation that failed
    pub operation: String,
    /// The error message that occurred
    pub error_message: String,
    /// Whether the error is retryable
    pub is_retryable: bool,
    /// Number of attempts made so far
    pub attempt_count: u32,
    /// Time spent on retries so far
    pub elapsed_time: Duration,
    /// Package name if applicable
    pub package_name: Option<String>,
}

impl RecoveryContext {
    /// Create a new recovery context
    pub fn new(operation: impl Into<String>, error: &BuildError) -> Self {
        Self {
            operation: operation.into(),
            error_message: error.to_string(),
            is_retryable: error.is_retryable(),
            attempt_count: 1,
            elapsed_time: Duration::ZERO,
            package_name: None,
        }
    }

    /// Create a recovery context for a package operation
    pub fn for_package(
        operation: impl Into<String>,
        package: impl Into<String>,
        error: &BuildError,
    ) -> Self {
        Self {
            operation: operation.into(),
            error_message: error.to_string(),
            is_retryable: error.is_retryable(),
            attempt_count: 1,
            elapsed_time: Duration::ZERO,
            package_name: Some(package.into()),
        }
    }

    /// Increment the attempt count
    pub fn increment_attempt(&mut self) {
        self.attempt_count += 1;
    }

    /// Add elapsed time
    pub fn add_elapsed_time(&mut self, duration: Duration) {
        self.elapsed_time += duration;
    }
}

/// Recovery manager for handling build failures and implementing retry logic
pub struct RecoveryManager {
    /// Recovery strategy to use
    strategy: BuildRecoveryStrategy,
    /// Retry configuration
    retry_config: RetryConfig,
    /// Shutdown signal
    shutdown_signal: Arc<AtomicBool>,
}

impl RecoveryManager {
    /// Create a new recovery manager
    pub fn new(strategy: BuildRecoveryStrategy, retry_config: RetryConfig) -> Self {
        Self {
            strategy,
            retry_config,
            shutdown_signal: Arc::new(AtomicBool::new(false)),
        }
    }

    /// Create a recovery manager with default configuration
    pub fn with_strategy(strategy: BuildRecoveryStrategy) -> Self {
        Self::new(strategy, RetryConfig::default())
    }

    /// Set the shutdown signal
    pub fn set_shutdown_signal(&mut self, signal: Arc<AtomicBool>) {
        self.shutdown_signal = signal;
    }

    /// Check if shutdown was requested
    pub fn is_shutdown_requested(&self) -> bool {
        self.shutdown_signal.load(Ordering::Acquire)
    }

    /// Execute an operation with retry logic
    pub async fn execute_with_retry<F, Fut, T>(
        &self,
        operation_name: &str,
        operation: F,
    ) -> Result<T>
    where
        F: Fn() -> Fut,
        Fut: Future<Output = Result<T>>,
    {
        if self.strategy == BuildRecoveryStrategy::None {
            return operation().await;
        }

        let mut backoff = self.create_backoff();
        let initial_error = BuildError::transient("Initial attempt");
        let mut context = RecoveryContext::new(operation_name, &initial_error);

        loop {
            // Check for shutdown
            if self.is_shutdown_requested() {
                return Err(BuildError::ShutdownInProgress);
            }

            let start_time = std::time::Instant::now();

            match operation().await {
                Ok(result) => {
                    if context.attempt_count > 1 {
                        info!(
                            "Operation '{}' succeeded after {} attempts",
                            operation_name, context.attempt_count
                        );
                    }
                    return Ok(result);
                }
                Err(error) => {
                    context.error_message = error.to_string();
                    context.is_retryable = error.is_retryable();
                    context.add_elapsed_time(start_time.elapsed());

                    // Check if we should retry this error
                    if !error.is_retryable() {
                        debug!("Error is not retryable: {}", error);
                        return Err(error);
                    }

                    // Check if we've exceeded max attempts
                    if context.attempt_count >= self.retry_config.max_attempts {
                        warn!(
                            "Max retry attempts ({}) exceeded for operation '{}'",
                            self.retry_config.max_attempts, operation_name
                        );
                        return Err(BuildError::max_retries_exceeded(
                            operation_name,
                            self.retry_config.max_attempts,
                        ));
                    }

                    // Check if we've exceeded max elapsed time
                    if let Some(max_elapsed) = self.retry_config.max_elapsed_time {
                        if context.elapsed_time >= max_elapsed {
                            warn!(
                                "Max elapsed time ({:?}) exceeded for operation '{}'",
                                max_elapsed, operation_name
                            );
                            return Err(BuildError::max_retries_exceeded(
                                operation_name,
                                self.retry_config.max_attempts,
                            ));
                        }
                    }

                    // Get next delay from backoff
                    let delay = match backoff.next_backoff() {
                        Some(delay) => delay,
                        None => {
                            warn!("Backoff exhausted for operation '{}'", operation_name);
                            return Err(BuildError::max_retries_exceeded(
                                operation_name,
                                self.retry_config.max_attempts,
                            ));
                        }
                    };

                    warn!(
                        "Operation '{}' failed (attempt {}): {}. Retrying in {:?}",
                        operation_name, context.attempt_count, error, delay
                    );

                    // Wait before retrying
                    tokio::time::sleep(delay).await;

                    context.increment_attempt();
                }
            }
        }
    }

    /// Execute a package operation with retry logic
    pub async fn execute_package_operation_with_retry<F, Fut, T>(
        &self,
        package_name: &str,
        operation_name: &str,
        operation: F,
    ) -> Result<T>
    where
        F: Fn() -> Fut,
        Fut: Future<Output = Result<T>>,
    {
        let full_operation_name = format!("{package_name}:{operation_name}");
        self.execute_with_retry(&full_operation_name, operation)
            .await
    }

    /// Determine recovery action for a given error
    pub fn determine_recovery_action(&self, context: &RecoveryContext) -> RecoveryAction {
        // Always abort on shutdown
        if context.error_message.contains("Shutdown")
            || context.error_message.contains("cancelled")
            || context.error_message.contains("Cancelled")
        {
            return RecoveryAction::Abort;
        }

        match (&self.strategy, context.is_retryable) {
            // No recovery strategy
            (BuildRecoveryStrategy::None, _) => RecoveryAction::Fail,

            // Retry strategy
            (BuildRecoveryStrategy::Retry, true) => {
                if context.attempt_count < self.retry_config.max_attempts {
                    RecoveryAction::Retry
                } else {
                    RecoveryAction::Fail
                }
            }

            // Clean retry strategy
            (BuildRecoveryStrategy::CleanRetry, true) => {
                if context.attempt_count < self.retry_config.max_attempts {
                    RecoveryAction::CleanAndRetry
                } else {
                    RecoveryAction::Fail
                }
            }

            // Skip strategy
            (BuildRecoveryStrategy::Skip, _) => RecoveryAction::Skip,

            // Graceful degradation
            (BuildRecoveryStrategy::Graceful, _) => RecoveryAction::Graceful,

            // Default to fail
            _ => RecoveryAction::Fail,
        }
    }

    /// Create an exponential backoff instance
    fn create_backoff(&self) -> ExponentialBackoff {
        ExponentialBackoffBuilder::new()
            .with_initial_interval(self.retry_config.initial_delay)
            .with_max_interval(self.retry_config.max_delay)
            .with_multiplier(self.retry_config.multiplier)
            .with_max_elapsed_time(self.retry_config.max_elapsed_time)
            .build()
    }

    /// Report recovery metrics
    pub fn report_metrics(&self, context: &RecoveryContext) {
        if context.attempt_count > 1 {
            info!(
                "Recovery metrics for '{}': {} attempts, {:?} elapsed time",
                context.operation, context.attempt_count, context.elapsed_time
            );
        }
    }
}

/// Actions that can be taken during recovery
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecoveryAction {
    /// Retry the operation
    Retry,
    /// Clean environment and retry
    CleanAndRetry,
    /// Skip the failed operation
    Skip,
    /// Attempt graceful degradation
    Graceful,
    /// Fail the operation
    Fail,
    /// Abort the entire build
    Abort,
}

/// Retry a future with exponential backoff
pub async fn retry_with_backoff<F, Fut, T>(
    operation: F,
    config: &RetryConfig,
    shutdown_signal: Option<Arc<AtomicBool>>,
) -> Result<T>
where
    F: Fn() -> Fut,
    Fut: Future<Output = Result<T>>,
{
    let recovery_manager = RecoveryManager::new(BuildRecoveryStrategy::Retry, config.clone());
    if let Some(signal) = shutdown_signal {
        let mut manager = recovery_manager;
        manager.set_shutdown_signal(signal);
        manager
            .execute_with_retry("retry_operation", operation)
            .await
    } else {
        recovery_manager
            .execute_with_retry("retry_operation", operation)
            .await
    }
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::{AtomicU32, Ordering};

    use super::*;

    #[tokio::test]
    async fn test_successful_operation_no_retry() {
        let manager = RecoveryManager::with_strategy(BuildRecoveryStrategy::Retry);
        let counter = Arc::new(AtomicU32::new(0));
        let counter_clone = Arc::clone(&counter);

        let result = manager
            .execute_with_retry("test", || async {
                counter_clone.fetch_add(1, Ordering::SeqCst);
                Ok(42)
            })
            .await;

        assert!(result.is_ok());
        assert_eq!(result.unwrap(), 42);
        assert_eq!(counter.load(Ordering::SeqCst), 1);
    }

    #[tokio::test]
    async fn test_retry_with_eventual_success() {
        let manager = RecoveryManager::with_strategy(BuildRecoveryStrategy::Retry);
        let counter = Arc::new(AtomicU32::new(0));
        let counter_clone = Arc::clone(&counter);

        let result = manager
            .execute_with_retry("test", || async {
                let count = counter_clone.fetch_add(1, Ordering::SeqCst);
                if count < 2 {
                    Err(BuildError::transient("temporary failure"))
                } else {
                    Ok(42)
                }
            })
            .await;

        assert!(result.is_ok());
        assert_eq!(result.unwrap(), 42);
        assert_eq!(counter.load(Ordering::SeqCst), 3);
    }

    #[tokio::test]
    async fn test_non_retryable_error() {
        let manager = RecoveryManager::with_strategy(BuildRecoveryStrategy::Retry);
        let counter = Arc::new(AtomicU32::new(0));
        let counter_clone = Arc::clone(&counter);

        let result: Result<i32> = manager
            .execute_with_retry("test", || async {
                counter_clone.fetch_add(1, Ordering::SeqCst);
                Err(BuildError::permanent("permanent failure"))
            })
            .await;

        assert!(result.is_err());
        assert_eq!(counter.load(Ordering::SeqCst), 1);
    }

    #[tokio::test]
    async fn test_max_retries_exceeded() {
        let config = RetryConfig {
            max_attempts: 2,
            initial_delay: Duration::from_millis(1),
            max_delay: Duration::from_millis(10),
            multiplier: 2.0,
            max_elapsed_time: None,
        };
        let manager = RecoveryManager::new(BuildRecoveryStrategy::Retry, config);
        let counter = Arc::new(AtomicU32::new(0));
        let counter_clone = Arc::clone(&counter);

        let result: Result<i32> = manager
            .execute_with_retry("test", || async {
                counter_clone.fetch_add(1, Ordering::SeqCst);
                Err(BuildError::transient("always fails"))
            })
            .await;

        assert!(result.is_err());
        assert!(matches!(result.unwrap_err(), BuildError::MaxRetriesExceeded { .. }));
        assert_eq!(counter.load(Ordering::SeqCst), 2);
    }

    #[tokio::test]
    async fn test_shutdown_signal() {
        let manager = RecoveryManager::with_strategy(BuildRecoveryStrategy::Retry);
        let shutdown = Arc::new(AtomicBool::new(false));
        let mut manager_with_signal = manager;
        manager_with_signal.set_shutdown_signal(Arc::clone(&shutdown));

        // Set shutdown signal
        shutdown.store(true, Ordering::Release);

        let result = manager_with_signal
            .execute_with_retry("test", || async { Ok(42) })
            .await;

        assert!(result.is_err());
        assert!(matches!(result.unwrap_err(), BuildError::ShutdownInProgress));
    }

    #[test]
    fn test_recovery_context() {
        let error = BuildError::transient("test error");
        let mut context = RecoveryContext::new("test_operation", &error);

        assert_eq!(context.operation, "test_operation");
        assert_eq!(context.attempt_count, 1);
        assert_eq!(context.elapsed_time, Duration::ZERO);
        assert!(context.package_name.is_none());
        assert!(context.is_retryable);

        context.increment_attempt();
        assert_eq!(context.attempt_count, 2);

        context.add_elapsed_time(Duration::from_millis(100));
        assert_eq!(context.elapsed_time, Duration::from_millis(100));
    }

    #[test]
    fn test_recovery_actions() {
        let config = RetryConfig::default();
        let manager = RecoveryManager::new(BuildRecoveryStrategy::Retry, config);

        // Test retryable error with attempts remaining
        let error = BuildError::transient("retry me");
        let context = RecoveryContext::new("test", &error);
        assert_eq!(manager.determine_recovery_action(&context), RecoveryAction::Retry);

        // Test non-retryable error
        let error = BuildError::permanent("don't retry");
        let context = RecoveryContext::new("test", &error);
        assert_eq!(manager.determine_recovery_action(&context), RecoveryAction::Fail);

        // Test shutdown error
        let error = BuildError::Cancelled;
        let context = RecoveryContext::new("test", &error);
        assert_eq!(manager.determine_recovery_action(&context), RecoveryAction::Abort);
    }
}
