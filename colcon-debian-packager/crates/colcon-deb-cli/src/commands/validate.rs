//! Validate command implementation

use std::path::PathBuf;

use colcon_deb_config::Config;
use colcon_deb_ros::scan_workspace;
use color_eyre::eyre::{Context, Result};
use tracing::{error, info, warn};

/// Validate command implementation
pub struct ValidateCommand {
    config_path: PathBuf,
}

impl ValidateCommand {
    pub fn new(config_path: PathBuf) -> Self {
        Self { config_path }
    }

    pub async fn execute(&self) -> Result<()> {
        info!("Validating configuration and workspace");

        // Load and validate configuration
        let config = Config::from_file(&self.config_path).with_context(|| {
            format!("Failed to load config from {}", self.config_path.display())
        })?;

        println!("✓ Configuration loaded successfully");

        // Check colcon repo path exists
        if !config.colcon_repo.exists() {
            error!("Colcon repository does not exist: {}", config.colcon_repo.display());
            return Err(color_eyre::eyre::eyre!("Colcon repository does not exist"));
        }
        println!("✓ Colcon repository exists: {}", config.colcon_repo.display());

        // Check output directory is writable
        if let Some(parent) = config.output_dir.parent() {
            if !parent.exists() {
                warn!("Output directory parent does not exist: {}", parent.display());
            }
        }
        println!("✓ Output directory: {}", config.output_dir.display());

        // Scan workspace for packages
        info!("Scanning workspace for ROS packages");
        let src_dir = config.colcon_repo.join("src");
        let packages = scan_workspace(&src_dir).context("Failed to scan workspace")?;

        if packages.is_empty() {
            warn!("No ROS packages found in workspace");
        } else {
            println!("✓ Found {} ROS packages:", packages.len());
            for package in &packages {
                println!("  - {} ({})", package.name, package.version);
                if !package.path.exists() {
                    warn!("    Package path does not exist: {}", package.path.display());
                }
            }
        }

        // Check Docker availability
        info!("Checking Docker availability");
        match Self::check_docker_available().await {
            Ok(()) => {
                println!("✓ Docker is available");
            }
            Err(e) => {
                error!("Docker check failed: {}", e);
                return Err(e).context("Docker availability check failed");
            }
        }

        // Validate debian directories configuration
        if !config.debian_dirs.exists() {
            warn!("Debian directories path does not exist: {}", config.debian_dirs.display());
        } else {
            println!("✓ Debian directories path exists: {}", config.debian_dirs.display());
        }

        println!("\n✓ All validation checks passed!");
        Ok(())
    }

    async fn check_docker_available() -> Result<()> {
        // TODO: Implement Docker availability check once DockerManager is available
        // For now, try to run docker --version command
        use tokio::process::Command;

        match Command::new("docker").arg("--version").output().await {
            Ok(output) => {
                if output.status.success() {
                    Ok(())
                } else {
                    Err(color_eyre::eyre::eyre!("Docker command failed"))
                }
            }
            Err(e) => Err(color_eyre::eyre::eyre!("Docker not available: {}", e)),
        }
    }
}
