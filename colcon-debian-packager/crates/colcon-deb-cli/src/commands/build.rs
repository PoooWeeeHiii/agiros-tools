//! Build command implementation

use std::path::PathBuf;

use colcon_deb_config::Config;
use colcon_deb_ros::scan_workspace;
use color_eyre::eyre::{Context, Result};
use indicatif::{ProgressBar, ProgressStyle};
use tracing::{info, warn};

/// Build command implementation
pub struct BuildCommand {
    config_path: PathBuf,
    output_dir: Option<PathBuf>,
    parallel_jobs: Option<usize>,
    agiros_distro: Option<String>,
}

impl BuildCommand {
    pub fn new(
        config_path: PathBuf,
        output_dir: Option<PathBuf>,
        parallel_jobs: Option<usize>,
        agiros_distro: Option<String>,
    ) -> Self {
        Self { config_path, output_dir, parallel_jobs, agiros_distro }
    }

    pub async fn execute(&self) -> Result<()> {
        info!("Starting build process");

        // Load configuration
        let mut config = Config::from_file(&self.config_path).with_context(|| {
            format!("Failed to load config from {}", self.config_path.display())
        })?;

        // Override config with command line options
        if let Some(output_dir) = &self.output_dir {
            config.output_dir = output_dir.clone();
        }
        if let Some(jobs) = self.parallel_jobs {
            config.parallel_jobs = jobs;
        }
        if let Some(distro) = &self.agiros_distro {
            config.ros_distro = Some(distro.clone());
        }

        println!("✓ Configuration loaded and validated");
        info!("Output directory: {}", config.output_dir.display());
        info!("Parallel jobs: {}", config.parallel_jobs);

        // Create build orchestrator
        // This initializes the Docker service and prepares for the build
        info!("Initializing build orchestrator...");
        let mut orchestrator = colcon_deb_build::BuildOrchestrator::new(config).await;

        // Run the build
        // This will:
        // 1. Prepare the build environment
        // 2. Execute the build in the container
        // 3. Collect artifacts
        info!("Executing build workflow...");
        orchestrator.build().await.map_err(|e| {
            color_eyre::eyre::eyre!("Build failed: {}", e)
        })?;

        info!("Build completed successfully");
        println!("\n✨ Build completed successfully!");

        Ok(())
    }
}
