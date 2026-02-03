//! Error types for Debian package management

use thiserror::Error;

/// Debian-specific error types
#[derive(Error, Debug)]
pub enum DebianError {
    /// Invalid package name
    #[error("Invalid package name: {name}")]
    InvalidPackageName { name: String },

    /// Missing required file in debian directory
    #[error("Missing required file: {file} in debian directory for package {package}")]
    MissingRequiredFile { package: String, file: String },

    /// Invalid debian control file
    #[error("Invalid debian control file for package {package}: {reason}")]
    InvalidControlFile { package: String, reason: String },

    /// Invalid version format
    #[error("Invalid version format: {version} - {reason}")]
    InvalidVersion { version: String, reason: String },

    /// Dependency mapping failed
    #[error("Failed to map dependency {dependency}: {reason}")]
    DependencyMappingFailed { dependency: String, reason: String },

    /// bloom-generate execution failed
    #[error("bloom-generate failed for package {package}: {reason}")]
    BloomGenerateFailed { package: String, reason: String },

    /// Repository generation failed
    #[error("Repository generation failed: {reason}")]
    RepositoryGenerationFailed { reason: String },

    /// File system error
    #[error("File system error: {0}")]
    Io(#[from] std::io::Error),

    /// JSON serialization error
    #[error("JSON error: {0}")]
    Json(#[from] serde_json::Error),

    /// Core library error
    #[error("Core error: {0}")]
    Core(#[from] colcon_deb_core::error::Error),
}

/// Result type alias for Debian operations
pub type Result<T> = std::result::Result<T, DebianError>;

impl DebianError {
    /// Create an invalid package name error
    pub fn invalid_package_name(name: impl Into<String>) -> Self {
        Self::InvalidPackageName { name: name.into() }
    }

    /// Create a missing required file error
    pub fn missing_required_file(package: impl Into<String>, file: impl Into<String>) -> Self {
        Self::MissingRequiredFile { package: package.into(), file: file.into() }
    }

    /// Create an invalid control file error
    pub fn invalid_control_file(package: impl Into<String>, reason: impl Into<String>) -> Self {
        Self::InvalidControlFile { package: package.into(), reason: reason.into() }
    }

    /// Create an invalid version error
    pub fn invalid_version(version: impl Into<String>, reason: impl Into<String>) -> Self {
        Self::InvalidVersion { version: version.into(), reason: reason.into() }
    }

    /// Create a dependency mapping error
    pub fn dependency_mapping_failed(
        dependency: impl Into<String>,
        reason: impl Into<String>,
    ) -> Self {
        Self::DependencyMappingFailed { dependency: dependency.into(), reason: reason.into() }
    }

    /// Create a bloom-generate error
    pub fn bloom_generate_failed(package: impl Into<String>, reason: impl Into<String>) -> Self {
        Self::BloomGenerateFailed { package: package.into(), reason: reason.into() }
    }

    /// Create a repository generation error
    pub fn repository_generation_failed(reason: impl Into<String>) -> Self {
        Self::RepositoryGenerationFailed { reason: reason.into() }
    }
}
