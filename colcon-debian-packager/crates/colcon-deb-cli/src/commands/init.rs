//! Init command implementation

use std::path::PathBuf;

use colcon_deb_config::{Config, DockerConfig};
use color_eyre::eyre::{Context, Result};
use tracing::info;

/// Init command implementation
pub struct InitCommand {
    output_path: PathBuf,
    force: bool,
}

impl InitCommand {
    pub fn new(output_path: PathBuf, force: bool) -> Self {
        Self { output_path, force }
    }

    pub async fn execute(&self) -> Result<()> {
        info!("Initializing configuration file at {}", self.output_path.display());

        // Check if file already exists
        if self.output_path.exists() && !self.force {
            return Err(color_eyre::eyre::eyre!(
                "Configuration file already exists: {}. Use --force to overwrite.",
                self.output_path.display()
            ));
        }

        // Create default configuration
        let config = Self::create_default_config()?;

        // Write configuration to file
        let yaml_content =
            serde_yaml::to_string(&config).with_context(|| "Failed to serialize config to YAML")?;

        std::fs::write(&self.output_path, yaml_content)
            .with_context(|| format!("Failed to write config to {}", self.output_path.display()))?;

        println!("✓ Created configuration file: {}", self.output_path.display());
        println!("\nNext steps:");
        println!("1. Edit the configuration file to match your workspace");
        println!(
            "2. Run 'colcon-deb validate -c {}' to validate the configuration",
            self.output_path.display()
        );
        println!("3. Run 'colcon-deb build -c {}' to build packages", self.output_path.display());

        Ok(())
    }

    fn create_default_config() -> Result<Config> {
        let current_dir = std::env::current_dir().context("Failed to get current directory")?;

        Ok(Config {
            colcon_repo: current_dir.clone(),
            debian_dirs: current_dir.join("debian_dirs"),
            docker: DockerConfig::Image { image: "ros:loong-ros-base".to_string() },
            ros_distro: Some("loong".to_string()),
            output_dir: current_dir.join("output"),
            parallel_jobs: 4,
        })
    }
}
