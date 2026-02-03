//! Build result tracking

use std::path::PathBuf;

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

/// Result of building a package
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BuildResult {
    /// Package name
    pub package_name: String,

    /// Build status
    pub status: BuildStatus,

    /// Start time
    pub start_time: DateTime<Utc>,

    /// End time
    pub end_time: DateTime<Utc>,

    /// Duration in seconds
    pub duration_secs: f64,

    /// Generated .deb file paths
    pub deb_files: Vec<PathBuf>,

    /// Build log output
    pub log: Option<String>,

    /// Error message if failed
    pub error: Option<String>,
}

/// Build status enum
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum BuildStatus {
    /// Build succeeded
    Success,

    /// Build failed
    Failed,

    /// Build was skipped
    Skipped,

    /// Build is in progress
    InProgress,

    /// Build is queued
    Queued,
}

impl BuildResult {
    /// Create a new build result
    pub fn new(package_name: impl Into<String>) -> Self {
        let now = Utc::now();
        Self {
            package_name: package_name.into(),
            status: BuildStatus::Queued,
            start_time: now,
            end_time: now,
            duration_secs: 0.0,
            deb_files: Vec::new(),
            log: None,
            error: None,
        }
    }

    /// Mark build as started
    pub fn start(&mut self) {
        self.status = BuildStatus::InProgress;
        self.start_time = Utc::now();
    }

    /// Mark build as completed successfully
    pub fn succeed(&mut self, deb_files: Vec<PathBuf>) {
        self.status = BuildStatus::Success;
        self.end_time = Utc::now();
        self.duration_secs = (self.end_time - self.start_time).num_milliseconds() as f64 / 1000.0;
        self.deb_files = deb_files;
    }

    /// Mark build as failed
    pub fn fail(&mut self, error: impl Into<String>) {
        self.status = BuildStatus::Failed;
        self.end_time = Utc::now();
        self.duration_secs = (self.end_time - self.start_time).num_milliseconds() as f64 / 1000.0;
        self.error = Some(error.into());
    }

    /// Mark build as skipped
    pub fn skip(&mut self, reason: impl Into<String>) {
        self.status = BuildStatus::Skipped;
        self.end_time = Utc::now();
        self.error = Some(reason.into());
    }
}

impl BuildStatus {
    /// Check if the build is complete
    pub fn is_complete(&self) -> bool {
        matches!(self, Self::Success | Self::Failed | Self::Skipped)
    }

    /// Check if the build was successful
    pub fn is_success(&self) -> bool {
        matches!(self, Self::Success)
    }

    /// Check if the build failed
    pub fn is_failed(&self) -> bool {
        matches!(self, Self::Failed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_build_result_lifecycle() {
        let mut result = BuildResult::new("test_package");
        assert_eq!(result.status, BuildStatus::Queued);

        result.start();
        assert_eq!(result.status, BuildStatus::InProgress);
        assert!(!result.status.is_complete());

        result.succeed(vec![PathBuf::from("test.deb")]);
        assert_eq!(result.status, BuildStatus::Success);
        assert!(result.status.is_success());
        assert!(result.status.is_complete());
        assert_eq!(result.deb_files.len(), 1);
        assert!(result.duration_secs >= 0.0);
    }
}
