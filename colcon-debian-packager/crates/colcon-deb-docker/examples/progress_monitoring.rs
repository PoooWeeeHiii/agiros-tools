//! Example demonstrating architecture detection and progress monitoring

use std::path::PathBuf;

use colcon_deb_docker::{
    check_architecture_compatibility, ContainerSpec, DockerConfig, DockerService,
    DockerServiceTrait, ProgressEvent, VolumeMount,
};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize Docker service
    let config = DockerConfig::default();
    let service = DockerService::new(config).await?;
    let docker = service.client();

    // Check architecture compatibility
    let image = "ros:loong-ros-base";
    println!("Checking architecture compatibility for {image}");

    let compatibility = check_architecture_compatibility(docker, image).await?;
    println!("Host architecture: {}", compatibility.host.arch);
    println!("Image architecture: {}", compatibility.image.arch);
    println!("Needs emulation: {}", compatibility.needs_emulation);
    println!("Summary: {}", compatibility.summary());

    if !compatibility.can_run() {
        eprintln!("Cannot run container!");
        for warning in &compatibility.warnings {
            eprintln!("Warning: {warning}");
        }
        return Ok(());
    }

    // Create container with progress monitoring
    let spec = ContainerSpec::new(image.to_string())
        .with_command(vec![
            "/helpers/build-orchestrator.rs".to_string(),
            "--workspace".to_string(),
            "/workspace".to_string(),
        ])
        .with_volume(VolumeMount {
            host_path: PathBuf::from("/tmp/workspace"),
            container_path: "/workspace".to_string(),
            read_only: false,
        })
        .with_env("ROS_DISTRO".to_string(), "loong".to_string());

    println!("\nStarting container...");
    let container_id = service.run_container(&spec).await?;
    println!("Container started: {container_id}");

    // Monitor progress
    println!("\nMonitoring build progress...");
    let container_manager = colcon_deb_docker::container::ContainerManager::new(docker);

    let progress = container_manager
        .stream_with_progress(&container_id, |event| match event {
            ProgressEvent::PackageStart { name, current, total } => {
                if let (Some(c), Some(t)) = (current, total) {
                    println!("[{c}/{t}] Starting package: {name}");
                } else {
                    println!("Starting package: {name}");
                }
            }
            ProgressEvent::PackageComplete { name, success, error } => {
                if *success {
                    println!("✓ Completed package: {name}");
                } else {
                    println!(
                        "✗ Failed package: {} - {}",
                        name,
                        error.as_deref().unwrap_or("Unknown error")
                    );
                }
            }
            ProgressEvent::Stage { name, description } => {
                println!("\n=== Stage: {name} ===");
                if let Some(desc) = description {
                    println!("    {desc}");
                }
            }
            ProgressEvent::Progress { current, total, message } => {
                let percentage = (*current as f64 / *total as f64) * 100.0;
                println!("Progress: {:.1}% - {}", percentage, message.as_deref().unwrap_or(""));
            }
            ProgressEvent::Log { level, message } => {
                println!("[{}] {message}", format!("{level:?}").to_uppercase());
            }
        })
        .await?;

    // Print final summary
    println!("\n=== Build Summary ===");
    println!("Total packages: {:?}", progress.total_packages);
    println!("Successful: {}", progress.successful_packages());
    println!("Failed: {}", progress.failed_packages());

    if let Some(percentage) = progress.completion_percentage() {
        println!("Completion: {percentage:.1}%");
    }

    // Cleanup
    println!("\nCleaning up...");
    service.remove_container(&container_id).await?;
    println!("Container removed");

    Ok(())
}
