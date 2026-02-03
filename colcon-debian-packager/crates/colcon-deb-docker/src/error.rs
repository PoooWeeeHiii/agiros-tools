//! Docker-specific error types

use thiserror::Error;

/// Docker-specific error type
#[derive(Error, Debug)]
pub enum DockerError {
    /// Bollard client error
    #[error("Docker client error: {0}")]
    Client(#[from] bollard::errors::Error),

    /// Container not found
    #[error("Container not found: {id}")]
    ContainerNotFound { id: String },

    /// Image not found
    #[error("Image not found: {name}")]
    ImageNotFound { name: String },

    /// Build failed
    #[error("Docker build failed: {reason}")]
    BuildFailed { reason: String },

    /// Pull failed
    #[error("Docker pull failed for image {image}: {reason}")]
    PullFailed { image: String, reason: String },

    /// Container execution failed
    #[error("Container execution failed: {reason}")]
    ExecutionFailed { reason: String },

    /// Invalid configuration
    #[error("Invalid Docker configuration: {reason}")]
    InvalidConfig { reason: String },

    /// Volume mount error
    #[error("Volume mount error: {path} - {reason}")]
    VolumeError { path: String, reason: String },

    /// Network error
    #[error("Docker network error: {reason}")]
    NetworkError { reason: String },

    /// Timeout error
    #[error("Docker operation timed out after {duration:?}")]
    Timeout { duration: std::time::Duration },

    /// Stream error
    #[error("Docker stream error: {reason}")]
    StreamError { reason: String },
}

/// Result type alias for Docker operations
pub type Result<T> = std::result::Result<T, DockerError>;
