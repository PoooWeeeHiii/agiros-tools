//! Clean command implementation

use std::path::{Path, PathBuf};

use colcon_deb_config::Config;
use color_eyre::eyre::{Context, Result};
use tracing::info;

/// Clean command implementation
pub struct CleanCommand {
    config_path: Option<PathBuf>,
    clean_all: bool,
}

impl CleanCommand {
    pub fn new(config_path: Option<PathBuf>, clean_all: bool) -> Self {
        Self { config_path, clean_all }
    }

    pub async fn execute(&self) -> Result<()> {
        info!("Cleaning build artifacts");

        let config =
            if let Some(config_path) = &self.config_path {
                Some(Config::from_file(config_path).with_context(|| {
                    format!("Failed to load config from {}", config_path.display())
                })?)
            } else {
                None
            };

        // Clean output directory
        if let Some(config) = &config {
            self.clean_output_directory(&config.output_dir).await?;
        } else {
            // Default output directory
            let default_output = PathBuf::from("./build");
            if default_output.exists() {
                self.clean_output_directory(&default_output).await?;
            }
        }

        // Clean Docker containers and images if requested
        if self.clean_all {
            self.clean_docker_artifacts().await?;
        }

        // Clean rust-script cache if requested
        if self.clean_all {
            self.clean_rust_script_cache().await?;
        }

        println!("✓ Cleanup completed");
        Ok(())
    }

    async fn clean_output_directory(&self, output_dir: &Path) -> Result<()> {
        if !output_dir.exists() {
            info!("Output directory does not exist, nothing to clean: {}", output_dir.display());
            return Ok(());
        }

        info!("Cleaning output directory: {}", output_dir.display());

        // Remove .deb files
        let deb_pattern = output_dir.join("*.deb");
        self.remove_files_matching(&deb_pattern, "*.deb").await?;

        // Remove repository metadata
        let repo_dir = output_dir.join("repository");
        if repo_dir.exists() {
            std::fs::remove_dir_all(&repo_dir).with_context(|| {
                format!("Failed to remove repository directory: {}", repo_dir.display())
            })?;
            println!("  Removed repository directory");
        }

        // Remove build logs
        let logs_pattern = output_dir.join("*.log");
        self.remove_files_matching(&logs_pattern, "*.log").await?;

        // Remove temporary debian directories
        let temp_debian_pattern = output_dir.join("debian_*");
        self.remove_directories_matching(&temp_debian_pattern, "debian_*")
            .await?;

        Ok(())
    }

    async fn clean_docker_artifacts(&self) -> Result<()> {
        info!("Cleaning Docker artifacts");

        // TODO: Implement Docker cleanup once DockerManager is available
        println!("  Docker cleanup not yet implemented");
        println!("  You can manually clean Docker artifacts with:");
        println!("    docker container prune");
        println!("    docker image prune");
        println!("    docker system prune");

        Ok(())
    }

    async fn clean_rust_script_cache(&self) -> Result<()> {
        info!("Cleaning rust-script cache");

        // rust-script typically caches in ~/.cargo/script-cache
        let home = std::env::var("HOME").unwrap_or_else(|_| ".".to_string());
        let cache_dir = PathBuf::from(home).join(".cargo").join("script-cache");

        if cache_dir.exists() {
            let entries = std::fs::read_dir(&cache_dir).with_context(|| {
                format!("Failed to read cache directory: {}", cache_dir.display())
            })?;

            let mut removed_count = 0;
            for entry in entries {
                let entry = entry?;
                let path = entry.path();

                // Look for colcon-deb related cache entries
                if let Some(name) = path.file_name().and_then(|n| n.to_str()) {
                    if name.contains("colcon-deb")
                        || name.contains("debian-preparer")
                        || name.contains("scanner")
                    {
                        if path.is_dir() {
                            std::fs::remove_dir_all(&path).with_context(|| {
                                format!("Failed to remove cache directory: {}", path.display())
                            })?;
                        } else {
                            std::fs::remove_file(&path).with_context(|| {
                                format!("Failed to remove cache file: {}", path.display())
                            })?;
                        }
                        removed_count += 1;
                        println!("  Removed cache entry: {name}");
                    }
                }
            }

            if removed_count == 0 {
                println!("  No colcon-deb cache entries found");
            } else {
                println!("  Removed {removed_count} cache entries");
            }
        } else {
            println!("  rust-script cache directory not found");
        }

        Ok(())
    }

    async fn remove_files_matching(&self, pattern: &Path, description: &str) -> Result<()> {
        if let Some(parent) = pattern.parent() {
            if !parent.exists() {
                return Ok(());
            }

            let pattern_str = pattern.file_name().unwrap().to_str().unwrap();
            let entries = std::fs::read_dir(parent)
                .with_context(|| format!("Failed to read directory: {}", parent.display()))?;

            let mut removed_count = 0;
            for entry in entries {
                let entry = entry?;
                let path = entry.path();

                if path.is_file() {
                    if let Some(name) = path.file_name().and_then(|n| n.to_str()) {
                        if Self::matches_pattern(name, pattern_str) {
                            std::fs::remove_file(&path).with_context(|| {
                                format!("Failed to remove file: {}", path.display())
                            })?;
                            removed_count += 1;
                        }
                    }
                }
            }

            if removed_count > 0 {
                println!("  Removed {removed_count} {description} files");
            }
        }

        Ok(())
    }

    async fn remove_directories_matching(&self, pattern: &Path, description: &str) -> Result<()> {
        if let Some(parent) = pattern.parent() {
            if !parent.exists() {
                return Ok(());
            }

            let pattern_str = pattern.file_name().unwrap().to_str().unwrap();
            let entries = std::fs::read_dir(parent)
                .with_context(|| format!("Failed to read directory: {}", parent.display()))?;

            let mut removed_count = 0;
            for entry in entries {
                let entry = entry?;
                let path = entry.path();

                if path.is_dir() {
                    if let Some(name) = path.file_name().and_then(|n| n.to_str()) {
                        if Self::matches_pattern(name, pattern_str) {
                            std::fs::remove_dir_all(&path).with_context(|| {
                                format!("Failed to remove directory: {}", path.display())
                            })?;
                            removed_count += 1;
                        }
                    }
                }
            }

            if removed_count > 0 {
                println!("  Removed {removed_count} {description} directories");
            }
        }

        Ok(())
    }

    fn matches_pattern(name: &str, pattern: &str) -> bool {
        // Simple glob pattern matching for * wildcard
        if pattern.contains('*') {
            let parts: Vec<&str> = pattern.split('*').collect();
            if parts.len() == 2 {
                let prefix = parts[0];
                let suffix = parts[1];
                return name.starts_with(prefix) && name.ends_with(suffix);
            }
        }
        name == pattern
    }
}
