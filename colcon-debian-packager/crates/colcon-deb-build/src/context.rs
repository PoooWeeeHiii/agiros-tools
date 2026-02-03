//! Build context management

use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::Mutex;
use std::time::Duration;

use chrono::{DateTime, Utc};
use colcon_deb_config::Config;
use colcon_deb_core::package::Package;
use serde::{Deserialize, Serialize};

/// Build state enum
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BuildState {
    /// Initial state
    Idle,
    /// Preparing environment
    Preparing,
    /// Ready to build
    Ready,
    /// Currently building
    Building,
    /// Collecting artifacts
    CollectingArtifacts,
    /// Build completed successfully
    Completed,
    /// Build failed
    Failed,
}

/// Build statistics
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct BuildStats {
    /// Total packages to build
    pub total_packages: usize,
    /// Successfully built packages
    pub built_packages: usize,
    /// Failed packages
    pub failed_packages: usize,
    /// Skipped packages
    pub skipped_packages: usize,
    /// Build start time
    #[serde(skip_serializing_if = "Option::is_none")]
    pub start_time: Option<DateTime<Utc>>,
    /// Build end time
    #[serde(skip_serializing_if = "Option::is_none")]
    pub end_time: Option<DateTime<Utc>>,
}

impl BuildStats {
    /// Get build duration
    pub fn duration(&self) -> Option<Duration> {
        match (&self.start_time, &self.end_time) {
            (Some(start), Some(end)) => {
                let duration_ms = end.timestamp_millis() - start.timestamp_millis();
                Some(Duration::from_millis(duration_ms as u64))
            }
            (Some(start), None) => {
                let now = Utc::now();
                let duration_ms = now.timestamp_millis() - start.timestamp_millis();
                Some(Duration::from_millis(duration_ms as u64))
            }
            _ => None,
        }
    }

    /// Check if build is complete
    pub fn is_complete(&self) -> bool {
        self.built_packages + self.failed_packages + self.skipped_packages >= self.total_packages
    }
}

/// Package build result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PackageBuildResult {
    /// Package name
    pub name: String,
    /// Success status
    pub success: bool,
    /// Error message if failed
    pub error: Option<String>,
    /// Build duration
    pub duration: Duration,
    /// Generated artifacts
    pub artifacts: Vec<PathBuf>,
}

/// Build context containing all build state
#[derive(Debug, Clone)]
pub struct BuildContext {
    /// Build configuration
    config: Config,
    /// Current build state
    state: Arc<Mutex<BuildState>>,
    /// Build statistics
    stats: Arc<Mutex<BuildStats>>,
    /// Package build results
    results: Arc<Mutex<HashMap<String, PackageBuildResult>>>,
    /// Discovered packages
    packages: Arc<Mutex<Vec<Package>>>,
    /// Container ID if running
    container_id: Arc<Mutex<Option<String>>>,
}

impl BuildContext {
    /// Create a new build context
    pub fn new(config: Config) -> Self {
        Self {
            config,
            state: Arc::new(Mutex::new(BuildState::Idle)),
            stats: Arc::new(Mutex::new(BuildStats::default())),
            results: Arc::new(Mutex::new(HashMap::new())),
            packages: Arc::new(Mutex::new(Vec::new())),
            container_id: Arc::new(Mutex::new(None)),
        }
    }

    /// Get the build configuration
    pub fn config(&self) -> &Config {
        &self.config
    }

    /// Get current build state
    pub fn state(&self) -> BuildState {
        *self.state.lock().unwrap()
    }

    /// Set build state
    pub fn set_state(&mut self, state: BuildState) {
        *self.state.lock().unwrap() = state;

        // Update timestamps
        match state {
            BuildState::Building => {
                self.stats.lock().unwrap().start_time = Some(Utc::now());
            }
            BuildState::Completed | BuildState::Failed => {
                self.stats.lock().unwrap().end_time = Some(Utc::now());
            }
            _ => {}
        }
    }

    /// Get build statistics
    pub fn stats(&self) -> BuildStats {
        self.stats.lock().unwrap().clone()
    }

    /// Update build statistics
    pub fn update_stats<F>(&mut self, f: F)
    where
        F: FnOnce(&mut BuildStats),
    {
        let mut stats = self.stats.lock().unwrap();
        f(&mut stats);
    }

    /// Add package build result
    pub fn add_result(&mut self, result: PackageBuildResult) {
        let name = result.name.clone();
        let success = result.success;

        {
            let mut results = self.results.lock().unwrap();
            results.insert(name, result);
        }

        // Update stats
        self.update_stats(|stats| {
            if success {
                stats.built_packages += 1;
            } else {
                stats.failed_packages += 1;
            }
        });
    }

    /// Get all build results
    pub fn results(&self) -> HashMap<String, PackageBuildResult> {
        self.results.lock().unwrap().clone()
    }

    /// Set discovered packages
    pub fn set_packages(&mut self, packages: Vec<Package>) {
        let count = packages.len();
        *self.packages.lock().unwrap() = packages;

        self.update_stats(|stats| {
            stats.total_packages = count;
        });
    }

    /// Get discovered packages
    pub fn packages(&self) -> Vec<Package> {
        self.packages.lock().unwrap().clone()
    }

    /// Set container ID
    pub fn set_container_id(&mut self, id: Option<String>) {
        *self.container_id.lock().unwrap() = id;
    }

    /// Get container ID
    pub fn container_id(&self) -> Option<String> {
        self.container_id.lock().unwrap().clone()
    }

    /// Check if build was successful
    pub fn is_successful(&self) -> bool {
        let stats = self.stats();
        stats.is_complete() && stats.failed_packages == 0
    }

    /// Get build summary
    pub fn summary(&self) -> String {
        let stats = self.stats();
        let duration = stats
            .duration()
            .map(|d| format!(" in {:.1}s", d.as_secs_f32()))
            .unwrap_or_default();

        format!(
            "Built {}/{} packages successfully{}. {} failed, {} skipped.",
            stats.built_packages,
            stats.total_packages,
            duration,
            stats.failed_packages,
            stats.skipped_packages
        )
    }
}
