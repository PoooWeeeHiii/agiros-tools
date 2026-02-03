//! Configuration management for Colcon Debian Packager
//!
//! This crate handles YAML configuration parsing, validation,
//! and environment variable substitution.

use std::path::{Path, PathBuf};

use colcon_deb_core::error::{Error, Result};
use regex::Regex;
use serde::{Deserialize, Serialize};

/// Main configuration structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Config {
    /// Path to the colcon repository
    pub colcon_repo: PathBuf,

    /// Path to debian directories collection
    pub debian_dirs: PathBuf,

    /// Docker configuration
    pub docker: DockerConfig,

    /// Optional ROS distribution (can be auto-detected)
    pub ros_distro: Option<String>,

    /// Output directory for .deb files
    #[serde(default = "default_output_dir")]
    pub output_dir: PathBuf,

    /// Number of parallel build jobs
    #[serde(default = "default_parallel_jobs")]
    pub parallel_jobs: usize,
}

/// Docker configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum DockerConfig {
    /// Use an existing Docker image
    Image { image: String },

    /// Build from a Dockerfile
    Dockerfile { dockerfile: PathBuf },
}

impl Config {
    /// Load configuration from a YAML file
    pub fn from_file(path: impl AsRef<Path>) -> Result<Self> {
        let path = path.as_ref();
        let content = std::fs::read_to_string(path).map_err(|e| Error::ConfigError {
            message: format!("Failed to read config file {path:?}: {e}"),
        })?;

        let mut config: Config = serde_yaml::from_str(&content)
            .map_err(|e| Error::ConfigError { message: format!("Failed to parse YAML: {e}") })?;

        // Expand environment variables
        config.expand_env_vars()?;

        // Validate configuration
        config.validate()?;

        Ok(config)
    }

    /// Expand environment variables in paths
    fn expand_env_vars(&mut self) -> Result<()> {
        self.colcon_repo = expand_path(&self.colcon_repo)?;
        self.debian_dirs = expand_path(&self.debian_dirs)?;
        self.output_dir = expand_path(&self.output_dir)?;

        if let DockerConfig::Dockerfile { ref mut dockerfile } = self.docker {
            *dockerfile = expand_path(dockerfile)?;
        }

        Ok(())
    }

    /// Validate configuration
    pub fn validate(&self) -> Result<()> {
        // Check colcon_repo exists and has src directory
        if !self.colcon_repo.exists() {
            return Err(Error::ConfigError {
                message: format!("Colcon repository does not exist: {0:?}", self.colcon_repo),
            });
        }

        let src_dir = self.colcon_repo.join("src");
        if !src_dir.exists() {
            let override_dir = std::env::var("COLCON_SRC_DIR").ok();
            if let Some(override_dir) = override_dir {
                let resolved = resolve_src_dir(&self.colcon_repo, &override_dir);
                if !resolved.exists() {
                    return Err(Error::ConfigError {
                        message: format!(
                            "Colcon repository missing src directory: {src_dir:?} (override COLCON_SRC_DIR={override_dir} not found)"
                        ),
                    });
                }
            } else {
                return Err(Error::ConfigError {
                    message: format!("Colcon repository missing src directory: {src_dir:?}"),
                });
            }
        }

        // Create debian_dirs if it doesn't exist
        if !self.debian_dirs.exists() {
            std::fs::create_dir_all(&self.debian_dirs).map_err(|e| Error::ConfigError {
                message: format!("Failed to create debian_dirs: {e}"),
            })?;
        }

        // Create output_dir if it doesn't exist
        if !self.output_dir.exists() {
            std::fs::create_dir_all(&self.output_dir).map_err(|e| Error::ConfigError {
                message: format!("Failed to create output_dir: {e}"),
            })?;
        }

        // Validate Docker config
        match &self.docker {
            DockerConfig::Dockerfile { dockerfile } => {
                if !dockerfile.exists() {
                    return Err(Error::ConfigError {
                        message: format!("Dockerfile does not exist: {dockerfile:?}"),
                    });
                }
            }
            DockerConfig::Image { image } => {
                if image.is_empty() {
                    return Err(Error::ConfigError {
                        message: "Docker image name cannot be empty".to_string(),
                    });
                }
            }
        }

        // Validate parallel jobs
        if self.parallel_jobs == 0 {
            return Err(Error::ConfigError {
                message: "parallel_jobs must be at least 1".to_string(),
            });
        }

        Ok(())
    }

    /// Get Docker image name (pull or build tag)
    pub fn docker_image(&self) -> String {
        match &self.docker {
            DockerConfig::Image { image } => image.clone(),
            DockerConfig::Dockerfile { .. } => "colcon-deb-builder:latest".to_string(),
        }
    }
}

/// Expand environment variables in a path
fn expand_path(path: &Path) -> Result<PathBuf> {
    let path_str = path.to_string_lossy();
    let env_var_re =
        Regex::new(r"\$\{([^}]+)\}|\$([A-Za-z_][A-Za-z0-9_]*)").expect("Invalid regex");

    let mut result = path_str.to_string();
    for cap in env_var_re.captures_iter(&path_str) {
        let var_name = cap.get(1).or_else(|| cap.get(2)).unwrap().as_str();
        let var_value = std::env::var(var_name).map_err(|_| Error::ConfigError {
            message: format!("Environment variable not found: {var_name}"),
        })?;

        result = result.replace(&cap[0], &var_value);
    }

    Ok(PathBuf::from(result))
}

fn default_output_dir() -> PathBuf {
    PathBuf::from("./output")
}

fn default_parallel_jobs() -> usize {
    std::thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(4)
}

fn resolve_src_dir(colcon_repo: &Path, src_dir: &str) -> PathBuf {
    let src_path = PathBuf::from(src_dir);
    if src_path.is_absolute() {
        src_path
    } else {
        colcon_repo.join(src_path)
    }
}

#[cfg(test)]
mod tests {
    use std::env;

    use tempfile::TempDir;

    use super::*;

    #[test]
    fn test_expand_path() {
        env::set_var("TEST_VAR", "/test/path");

        let path = PathBuf::from("${TEST_VAR}/sub");
        let expanded = expand_path(&path).unwrap();
        assert_eq!(expanded, PathBuf::from("/test/path/sub"));

        let path = PathBuf::from("$TEST_VAR/sub");
        let expanded = expand_path(&path).unwrap();
        assert_eq!(expanded, PathBuf::from("/test/path/sub"));
    }

    #[test]
    fn test_config_validation() {
        let temp_dir = TempDir::new().unwrap();
        let colcon_repo = temp_dir.path().join("workspace");
        std::fs::create_dir_all(colcon_repo.join("src")).unwrap();

        let dockerfile = temp_dir.path().join("Dockerfile");
        std::fs::write(&dockerfile, "FROM ros:loong").unwrap();

        let config = Config {
            colcon_repo,
            debian_dirs: temp_dir.path().join("debian_dirs"),
            docker: DockerConfig::Dockerfile { dockerfile },
            ros_distro: Some("loong".to_string()),
            output_dir: temp_dir.path().join("output"),
            parallel_jobs: 4,
        };

        // Should create missing directories and validate successfully
        assert!(config.validate().is_ok());
    }
}
