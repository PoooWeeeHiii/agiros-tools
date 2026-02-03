//! Error types for the core library

use thiserror::Error;

/// Core error type for colcon-deb operations
#[derive(Error, Debug)]
pub enum Error {
    /// I/O operation failed
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    /// Serialization/deserialization error
    #[error("Serialization error: {0}")]
    Serialization(#[from] serde_json::Error),

    /// Package not found
    #[error("Package not found: {name}")]
    PackageNotFound { name: String },

    /// Invalid package format
    #[error("Invalid package format: {reason}")]
    InvalidPackage { reason: String },

    /// Dependency error
    #[error("Dependency error: {message}")]
    DependencyError { message: String },

    /// Build failed
    #[error("Build failed for package {package}: {reason}")]
    BuildFailed { package: String, reason: String },

    /// Configuration error
    #[error("Configuration error: {message}")]
    ConfigError { message: String },

    /// Docker error
    #[error("Docker error: {message}")]
    DockerError { message: String },

    /// Parse error
    #[error("Parse error: {message}")]
    ParseError { message: String },

    /// Generic error with context
    #[error("{context}: {source}")]
    WithContext {
        context: String,
        #[source]
        source: Box<dyn std::error::Error + Send + Sync>,
    },
}

/// Result type alias for colcon-deb operations
pub type Result<T> = std::result::Result<T, Error>;

impl Error {
    /// Add context to an error
    pub fn context<E>(context: impl Into<String>, source: E) -> Self
    where
        E: std::error::Error + Send + Sync + 'static,
    {
        Self::WithContext { context: context.into(), source: Box::new(source) }
    }
}
