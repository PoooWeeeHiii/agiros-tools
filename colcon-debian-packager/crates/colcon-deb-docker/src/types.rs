//! Docker-related types and structures

use std::collections::HashMap;

use serde::{Deserialize, Serialize};

/// Docker build context
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BuildContext {
    /// Dockerfile content
    pub dockerfile: String,
    /// Build arguments
    pub build_args: HashMap<String, String>,
    /// Labels to apply
    pub labels: HashMap<String, String>,
    /// Target stage (for multi-stage builds)
    pub target: Option<String>,
}

/// Container state
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ContainerState {
    /// Container is being created
    Creating,
    /// Container is running
    Running,
    /// Container has exited
    Exited(i32),
    /// Container is paused
    Paused,
    /// Container is being removed
    Removing,
    /// Container is dead
    Dead,
}

/// Container info
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContainerInfo {
    /// Container ID
    pub id: String,
    /// Container name
    pub name: String,
    /// Image used
    pub image: String,
    /// Current state
    pub state: ContainerState,
    /// Creation time
    pub created: String,
    /// Labels
    pub labels: HashMap<String, String>,
}

/// Image info
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImageInfo {
    /// Image ID
    pub id: String,
    /// Repository tags
    pub tags: Vec<String>,
    /// Image size in bytes
    pub size: i64,
    /// Creation time
    pub created: String,
    /// Labels
    pub labels: HashMap<String, String>,
}

/// Docker registry credentials
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegistryAuth {
    /// Username
    pub username: String,
    /// Password
    pub password: String,
    /// Registry server URL
    pub server_address: Option<String>,
}

/// Container resource limits
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResourceLimits {
    /// Memory limit in bytes
    pub memory: Option<i64>,
    /// CPU quota (100000 = 1 CPU)
    pub cpu_quota: Option<i64>,
    /// CPU shares (relative weight)
    pub cpu_shares: Option<i64>,
}

/// Container health check
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HealthCheck {
    /// Command to run
    pub test: Vec<String>,
    /// Interval between checks
    pub interval: Option<u64>,
    /// Timeout for each check
    pub timeout: Option<u64>,
    /// Number of retries
    pub retries: Option<u32>,
    /// Start period
    pub start_period: Option<u64>,
}

/// Docker network mode
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum NetworkMode {
    /// Default bridge network
    Bridge,
    /// Host network
    Host,
    /// No networking
    None,
    /// Custom network
    Custom(String),
}

impl Default for NetworkMode {
    fn default() -> Self {
        Self::Bridge
    }
}
