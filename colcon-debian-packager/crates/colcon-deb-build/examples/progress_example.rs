//! Example demonstrating progress UI usage with Docker events

use std::time::Duration;

use colcon_deb_build::progress_ui::{ProgressUI, ProgressUIFactory};
use colcon_deb_docker::progress::{LogLevel, ProgressEvent};

#[tokio::main]
async fn main() {
    // Create a progress UI
    let progress_ui = ProgressUIFactory::create_indicatif();

    // Simulate a build process with progress events
    simulate_build_process(&*progress_ui).await;

    // Finish the progress display
    progress_ui.finish();
}

async fn simulate_build_process(progress_ui: &dyn ProgressUI) {
    let packages = ["std_msgs", "geometry_msgs", "sensor_msgs", "nav_msgs"];

    // Start build stage
    progress_ui.update(&ProgressEvent::Stage {
        name: "preparation".to_string(),
        description: Some("Preparing build environment".to_string()),
    });

    tokio::time::sleep(Duration::from_millis(500)).await;

    // Log some initial messages
    progress_ui.update(&ProgressEvent::Log {
        level: LogLevel::Info,
        message: "Starting build process".to_string(),
    });

    // Move to building stage
    progress_ui.update(&ProgressEvent::Stage {
        name: "building".to_string(),
        description: Some("Building packages".to_string()),
    });

    // Simulate building each package
    for (i, package) in packages.iter().enumerate() {
        let current = i + 1;
        let total = packages.len();

        // Start building package
        progress_ui.update(&ProgressEvent::PackageStart {
            name: package.to_string(),
            current: Some(current),
            total: Some(total),
        });

        // Simulate progress updates
        for step in 1..=5 {
            progress_ui.update(&ProgressEvent::Progress {
                current: step,
                total: 5,
                message: Some(format!("Building {package} step {step}")),
            });
            tokio::time::sleep(Duration::from_millis(200)).await;
        }

        // Complete package (most succeed, one fails for demo)
        let success = package != &"sensor_msgs";
        progress_ui.update(&ProgressEvent::PackageComplete {
            name: package.to_string(),
            success,
            error: if success {
                None
            } else {
                Some("Compilation failed: missing dependency".to_string())
            },
        });

        tokio::time::sleep(Duration::from_millis(100)).await;
    }

    // Final stage
    progress_ui.update(&ProgressEvent::Stage {
        name: "cleanup".to_string(),
        description: Some("Cleaning up build artifacts".to_string()),
    });

    tokio::time::sleep(Duration::from_millis(300)).await;

    progress_ui.update(&ProgressEvent::Log {
        level: LogLevel::Info,
        message: "Build process completed".to_string(),
    });
}
