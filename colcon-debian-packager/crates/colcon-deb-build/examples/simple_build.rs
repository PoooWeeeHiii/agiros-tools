//! Example of using the colcon-deb-build crate

use std::path::PathBuf;

use colcon_deb_build::{BuildOrchestratorTrait, ColconDebBuilder};
use colcon_deb_config::{Config, DockerConfig};
use tracing::info;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize logging
    tracing_subscriber::fmt::init();

    // Create configuration
    let config = Config {
        colcon_repo: PathBuf::from("./test_workspace"),
        debian_dirs: PathBuf::from("./debian_dirs"),
        docker: DockerConfig::Image { image: "ros:loong".to_string() },
        ros_distro: Some("loong".to_string()),
        output_dir: PathBuf::from("./output"),
        parallel_jobs: 4,
    };

    // Create build orchestrator
    info!("Creating build orchestrator");
    let mut builder = match ColconDebBuilder::new(config).await {
        Ok(builder) => builder,
        Err(e) => {
            eprintln!("Failed to create builder: {e}");
            eprintln!("Make sure Docker is installed and running");
            return Ok(());
        }
    };

    // Prepare environment
    info!("Preparing build environment");
    if let Err(e) = builder.prepare_environment().await {
        eprintln!("Failed to prepare environment: {e}");
        return Ok(());
    }

    // Run build
    info!("Running build");
    if let Err(e) = builder.run_build().await {
        eprintln!("Build failed: {e}");
        return Ok(());
    }

    // Collect artifacts
    info!("Collecting artifacts");
    match builder.collect_artifacts().await {
        Ok(artifacts) => {
            info!("Build completed successfully!");
            info!("Generated {} artifacts:", artifacts.len());
            for artifact in artifacts {
                info!("  - {}", artifact.display());
            }
        }
        Err(e) => {
            eprintln!("Failed to collect artifacts: {e}");
        }
    }

    // Print summary
    let context = builder.context();
    info!("{}", context.summary());

    Ok(())
}
