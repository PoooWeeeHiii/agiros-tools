//! Example showing how to integrate progress UI with build orchestration

use std::path::PathBuf;

use colcon_deb_build::progress_ui::{ProgressUI, ProgressUIFactory};
use colcon_deb_build::{BuildContext, BuildState};
use colcon_deb_config::{Config, DockerConfig};
use colcon_deb_docker::progress::{LogLevel, ProgressEvent};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize logging
    tracing_subscriber::fmt::init();

    // Create a mock configuration
    let config = create_mock_config();

    // Create progress UI
    let progress_ui = ProgressUIFactory::create_indicatif();

    // Show how progress UI would be integrated with build orchestrator
    demonstrate_integration(&config, &*progress_ui).await?;

    progress_ui.finish();
    Ok(())
}

fn create_mock_config() -> Config {
    Config {
        colcon_repo: PathBuf::from("/tmp/mock_workspace"),
        debian_dirs: PathBuf::from("/tmp/mock_debian"),
        docker: DockerConfig::Image { image: "ros:loong".to_string() },
        ros_distro: Some("loong".to_string()),
        output_dir: PathBuf::from("/tmp/mock_output"),
        parallel_jobs: 4,
    }
}

async fn demonstrate_integration(
    _config: &Config,
    progress_ui: &dyn ProgressUI,
) -> Result<(), Box<dyn std::error::Error>> {
    // This demonstrates how the progress UI would be used in a real build scenario

    progress_ui.update(&ProgressEvent::Log {
        level: LogLevel::Info,
        message: "Starting build orchestration".to_string(),
    });

    // Stage 1: Environment preparation
    progress_ui.update(&ProgressEvent::Stage {
        name: "environment_setup".to_string(),
        description: Some("Preparing build environment".to_string()),
    });

    // Simulate environment preparation steps
    for (i, step) in [
        "Docker initialization",
        "Workspace validation",
        "Dependency resolution",
    ]
    .iter()
    .enumerate()
    {
        progress_ui.update(&ProgressEvent::Progress {
            current: i + 1,
            total: 3,
            message: Some(step.to_string()),
        });
        tokio::time::sleep(std::time::Duration::from_millis(500)).await;
    }

    // Stage 2: Package discovery
    progress_ui.update(&ProgressEvent::Stage {
        name: "discovery".to_string(),
        description: Some("Discovering packages in workspace".to_string()),
    });

    let mock_packages = ["package_a", "package_b", "package_c"];

    progress_ui.update(&ProgressEvent::Log {
        level: LogLevel::Info,
        message: format!("Found {} packages to build", mock_packages.len()),
    });

    // Stage 3: Building packages
    progress_ui.update(&ProgressEvent::Stage {
        name: "building".to_string(),
        description: Some("Building packages".to_string()),
    });

    for (i, package) in mock_packages.iter().enumerate() {
        let current = i + 1;
        let total = mock_packages.len();

        // Start package build
        progress_ui.update(&ProgressEvent::PackageStart {
            name: package.to_string(),
            current: Some(current),
            total: Some(total),
        });

        // Simulate build steps
        for step in 1..=4 {
            let message = match step {
                1 => "CMake configuration",
                2 => "Compilation",
                3 => "Testing",
                4 => "Packaging",
                _ => "Unknown step",
            };

            progress_ui.update(&ProgressEvent::Progress {
                current: step,
                total: 4,
                message: Some(format!("{package}: {message}")),
            });

            tokio::time::sleep(std::time::Duration::from_millis(300)).await;
        }

        // Complete package
        progress_ui.update(&ProgressEvent::PackageComplete {
            name: package.to_string(),
            success: true,
            error: None,
        });
    }

    // Stage 4: Artifact collection
    progress_ui.update(&ProgressEvent::Stage {
        name: "artifact_collection".to_string(),
        description: Some("Collecting build artifacts".to_string()),
    });

    progress_ui.update(&ProgressEvent::Progress {
        current: 1,
        total: 1,
        message: Some("Copying artifacts from container".to_string()),
    });

    tokio::time::sleep(std::time::Duration::from_millis(400)).await;

    progress_ui.update(&ProgressEvent::Log {
        level: LogLevel::Info,
        message: "Build completed successfully!".to_string(),
    });

    Ok(())
}

/// This function shows how you would integrate progress UI with the actual
/// build context
#[allow(dead_code)]
fn integrate_with_build_context(context: &mut BuildContext, progress_ui: &dyn ProgressUI) {
    // Monitor build state changes
    match context.state() {
        BuildState::Idle => {
            progress_ui.update(&ProgressEvent::Log {
                level: LogLevel::Debug,
                message: "Build context is idle".to_string(),
            });
        }
        BuildState::Preparing => {
            progress_ui.update(&ProgressEvent::Stage {
                name: "preparation".to_string(),
                description: Some("Preparing build environment".to_string()),
            });
        }
        BuildState::Ready => {
            progress_ui.update(&ProgressEvent::Log {
                level: LogLevel::Info,
                message: "Build environment ready".to_string(),
            });
        }
        BuildState::Building => {
            progress_ui.update(&ProgressEvent::Stage {
                name: "building".to_string(),
                description: Some("Building packages".to_string()),
            });
        }
        BuildState::CollectingArtifacts => {
            progress_ui.update(&ProgressEvent::Stage {
                name: "collecting".to_string(),
                description: Some("Collecting build artifacts".to_string()),
            });
        }
        BuildState::Completed => {
            progress_ui.update(&ProgressEvent::Log {
                level: LogLevel::Info,
                message: "Build completed successfully".to_string(),
            });
        }
        BuildState::Failed => {
            progress_ui.update(&ProgressEvent::Log {
                level: LogLevel::Error,
                message: "Build failed".to_string(),
            });
        }
    }

    // Report package results
    for (_name, result) in context.results() {
        progress_ui.update(&ProgressEvent::PackageComplete {
            name: result.name.clone(),
            success: result.success,
            error: result.error.clone(),
        });
    }
}
