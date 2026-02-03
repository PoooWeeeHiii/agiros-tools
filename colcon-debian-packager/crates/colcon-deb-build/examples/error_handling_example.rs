//! Example demonstrating error handling and recovery mechanisms

use std::sync::Arc;
use std::time::Duration;

use colcon_deb_build::{
    setup_signal_handlers, BuildError, BuildRecoveryStrategy, RecoveryManager, Result, RetryConfig,
    ShutdownGuard, ShutdownManager,
};
use tracing::{info, warn};

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize tracing
    tracing_subscriber::fmt::init();

    info!("Starting error handling and recovery example");

    // Create shutdown manager
    let shutdown_manager = Arc::new(ShutdownManager::new(Duration::from_secs(30)));

    // Setup signal handlers for graceful shutdown
    setup_signal_handlers(Arc::clone(&shutdown_manager)).await?;

    // Create recovery manager with retry strategy
    let retry_config = RetryConfig {
        max_attempts: 3,
        initial_delay: Duration::from_millis(500),
        max_delay: Duration::from_secs(5),
        multiplier: 2.0,
        max_elapsed_time: Some(Duration::from_secs(30)),
    };

    let mut recovery_manager = RecoveryManager::new(BuildRecoveryStrategy::Retry, retry_config);
    recovery_manager.set_shutdown_signal(shutdown_manager.shutdown_signal());

    // Create a shutdown guard for this operation
    let _guard = ShutdownGuard::new(Arc::clone(&shutdown_manager), "main_operation".to_string());

    // Example 1: Operation that succeeds immediately
    info!("=== Example 1: Successful operation ===");
    let result = recovery_manager
        .execute_with_retry("successful_operation", || async {
            info!("Executing successful operation");
            Ok(42)
        })
        .await?;
    info!("Operation succeeded with result: {}", result);

    // Example 2: Operation that fails but eventually succeeds
    info!("=== Example 2: Operation with transient failures ===");
    let attempt_count = Arc::new(std::sync::atomic::AtomicU32::new(0));
    let attempt_count_clone = Arc::clone(&attempt_count);

    let result = recovery_manager
        .execute_with_retry("retry_operation", move || {
            let count = attempt_count_clone.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
            async move {
                if count < 2 {
                    warn!("Operation failed (attempt {})", count + 1);
                    Err(BuildError::transient("Temporary network failure"))
                } else {
                    info!("Operation succeeded on attempt {}", count + 1);
                    Ok("Success!")
                }
            }
        })
        .await?;
    info!("Operation succeeded with result: {}", result);

    // Example 3: Operation that fails permanently
    info!("=== Example 3: Operation with permanent failure ===");
    let result: Result<()> = recovery_manager
        .execute_with_retry("permanent_failure", || async {
            Err(BuildError::permanent("Configuration error"))
        })
        .await;

    match result {
        Ok(_) => info!("Unexpected success"),
        Err(e) => info!("Expected permanent failure: {}", e),
    }

    // Example 4: Package-level operation with retry
    info!("=== Example 4: Package operation with retry ===");
    let package_attempt_count = Arc::new(std::sync::atomic::AtomicU32::new(0));
    let package_attempt_count_clone = Arc::clone(&package_attempt_count);

    recovery_manager
        .execute_package_operation_with_retry("my_package", "build", move || {
            let count =
                package_attempt_count_clone.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
            async move {
                if count < 1 {
                    warn!("Package build failed (attempt {})", count + 1);
                    Err(BuildError::build_failed("my_package", "Dependency not found"))
                } else {
                    info!("Package build succeeded on attempt {}", count + 1);
                    Ok(())
                }
            }
        })
        .await?;
    info!("Package operation completed successfully");

    // Example 5: Demonstration of shutdown handling
    info!("=== Example 5: Shutdown handling ===");

    // Check if shutdown was requested
    if shutdown_manager.is_shutdown_requested() {
        info!("Shutdown was requested during execution");
        // Perform graceful shutdown
        shutdown_manager.graceful_shutdown().await?;
    } else {
        info!("No shutdown requested, continuing normally");
    }

    info!("Error handling and recovery example completed successfully");

    Ok(())
}
