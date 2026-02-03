//! Error types for the build orchestration module

use thiserror::Error;

/// Build-specific error types
#[derive(Error, Debug)]
pub enum BuildError {
    /// Environment preparation failed
    #[error("Failed to prepare build environment: {reason}")]
    EnvironmentPreparation { reason: String },

    /// Container creation failed
    #[error("Failed to create container: {reason}")]
    ContainerCreation { reason: String },

    /// Build execution failed
    #[error("Build execution failed for package {package}: {reason}")]
    BuildExecution { package: String, reason: String },

    /// Artifact collection failed
    #[error("Failed to collect artifacts: {reason}")]
    ArtifactCollection { reason: String },

    /// Build timeout
    #[error("Build timed out after {duration_secs} seconds")]
    BuildTimeout { duration_secs: u64 },

    /// Missing build dependency
    #[error("Missing build dependency: {dependency}")]
    MissingDependency { dependency: String },

    /// Invalid build configuration
    #[error("Invalid build configuration: {reason}")]
    InvalidConfiguration { reason: String },

    /// Progress monitoring error
    #[error("Progress monitoring error: {reason}")]
    ProgressMonitoring { reason: String },

    /// Recovery operation failed
    #[error("Recovery operation failed: {reason}")]
    RecoveryFailed { reason: String },

    /// Maximum retry attempts exceeded
    #[error("Maximum retry attempts ({max_attempts}) exceeded for operation: {operation}")]
    MaxRetriesExceeded {
        operation: String,
        max_attempts: u32,
    },

    /// Build was cancelled by user
    #[error("Build was cancelled by user")]
    Cancelled,

    /// Shutdown in progress
    #[error("Shutdown in progress, operation aborted")]
    ShutdownInProgress,

    /// Transient error that may be retryable
    #[error("Transient error: {reason}")]
    Transient { reason: String },

    /// Permanent error that should not be retried
    #[error("Permanent error: {reason}")]
    Permanent { reason: String },

    /// Core library error
    #[error("Core error: {0}")]
    Core(#[from] colcon_deb_core::error::Error),

    /// Docker error
    #[error("Docker error: {0}")]
    Docker(#[from] colcon_deb_docker::error::DockerError),

    /// Configuration error
    #[error("Configuration error: {0}")]
    Config(String),

    /// I/O error
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    /// Serialization error
    #[error("Serialization error: {0}")]
    Serialization(#[from] serde_json::Error),
}

/// Result type alias for build operations
pub type Result<T> = std::result::Result<T, BuildError>;

impl BuildError {
    /// Add context to an error (simplified)
    pub fn context(context: impl Into<String>, source: impl Into<String>) -> Self {
        Self::Config(format!("{}: {}", context.into(), source.into()))
    }

    /// Create a configuration error
    pub fn config(msg: impl Into<String>) -> Self {
        Self::Config(msg.into())
    }

    /// Create an environment preparation error
    pub fn environment(reason: impl Into<String>) -> Self {
        Self::EnvironmentPreparation { reason: reason.into() }
    }

    /// Create a build execution error
    pub fn build_failed(package: impl Into<String>, reason: impl Into<String>) -> Self {
        Self::BuildExecution { package: package.into(), reason: reason.into() }
    }

    /// Create a recovery failed error
    pub fn recovery_failed(reason: impl Into<String>) -> Self {
        Self::RecoveryFailed { reason: reason.into() }
    }

    /// Create a max retries exceeded error
    pub fn max_retries_exceeded(operation: impl Into<String>, max_attempts: u32) -> Self {
        Self::MaxRetriesExceeded { operation: operation.into(), max_attempts }
    }

    /// Create a transient error
    pub fn transient(reason: impl Into<String>) -> Self {
        Self::Transient { reason: reason.into() }
    }

    /// Create a permanent error
    pub fn permanent(reason: impl Into<String>) -> Self {
        Self::Permanent { reason: reason.into() }
    }

    /// Check if an error is retryable
    pub fn is_retryable(&self) -> bool {
        match self {
            Self::Transient { .. }
            | Self::BuildTimeout { .. }
            | Self::ContainerCreation { .. }
            | Self::Docker(_)
            | Self::Io(_) => true,

            Self::Permanent { .. }
            | Self::Cancelled
            | Self::ShutdownInProgress
            | Self::InvalidConfiguration { .. }
            | Self::MissingDependency { .. }
            | Self::MaxRetriesExceeded { .. } => false,

            // For other errors, be conservative and don't retry
            _ => false,
        }
    }

    /// Check if an error indicates shutdown
    pub fn is_shutdown(&self) -> bool {
        matches!(self, Self::Cancelled | Self::ShutdownInProgress)
    }
}
