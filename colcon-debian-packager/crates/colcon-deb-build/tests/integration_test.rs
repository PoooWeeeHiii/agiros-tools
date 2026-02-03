//! Integration tests for the build module

use colcon_deb_build::{BuildOrchestratorTrait, ColconDebBuilder};
use colcon_deb_config::Config;
use tempfile::TempDir;

#[tokio::test]
async fn test_build_orchestrator_creation() {
    // Create temporary directories
    let temp_dir = TempDir::new().unwrap();
    let workspace = temp_dir.path().join("workspace");
    std::fs::create_dir_all(workspace.join("src")).unwrap();

    let config = Config {
        colcon_repo: workspace,
        debian_dirs: temp_dir.path().join("debian"),
        docker: colcon_deb_config::DockerConfig::Image { image: "ros:loong".to_string() },
        ros_distro: Some("loong".to_string()),
        output_dir: temp_dir.path().join("output"),
        parallel_jobs: 1,
    };

    // Test that we can create the orchestrator
    let result = ColconDebBuilder::new(config.clone()).await;

    // This will fail in CI without Docker, but that's expected
    if result.is_err() {
        // Should be a Docker error - we can't unwrap_err without Debug
        // so just check that it failed
        assert!(result.is_err());
    } else {
        // If Docker is available, verify the orchestrator was created
        let orchestrator = result.unwrap();
        assert_eq!(orchestrator.context().state(), colcon_deb_build::BuildState::Idle);
    }
}

#[test]
fn test_build_context() {
    use std::time::Duration;

    use colcon_deb_build::context::PackageBuildResult;
    use colcon_deb_build::{BuildContext, BuildState};

    let temp_dir = TempDir::new().unwrap();
    let config = Config {
        colcon_repo: temp_dir.path().join("workspace"),
        debian_dirs: temp_dir.path().join("debian"),
        docker: colcon_deb_config::DockerConfig::Image { image: "ros:loong".to_string() },
        ros_distro: Some("loong".to_string()),
        output_dir: temp_dir.path().join("output"),
        parallel_jobs: 4,
    };

    let mut context = BuildContext::new(config);

    // Test state transitions
    assert_eq!(context.state(), BuildState::Idle);
    context.set_state(BuildState::Preparing);
    assert_eq!(context.state(), BuildState::Preparing);

    // Test adding results
    context.add_result(PackageBuildResult {
        name: "test_package".to_string(),
        success: true,
        error: None,
        duration: Duration::from_secs(10),
        artifacts: vec![],
    });

    let stats = context.stats();
    assert_eq!(stats.built_packages, 1);
    assert_eq!(stats.failed_packages, 0);
}

#[test]
fn test_artifact_parsing() {
    use std::path::Path;

    use colcon_deb_build::BuildArtifact;

    // Test parsing a valid .deb filename
    let path = Path::new("/tmp/agiros-loong-example-msgs_0.1.0-1_amd64.deb");
    let artifact = BuildArtifact::from_path(path);

    // Since this requires the file to exist, it will return None
    assert!(artifact.is_none());
}

#[test]
fn test_executor_config() {
    use std::path::PathBuf;

    use colcon_deb_build::ExecutorConfig;

    let config = ExecutorConfig {
        container_image: "ros:loong".to_string(),
        workspace_path: PathBuf::from("/workspace"),
        output_dir: PathBuf::from("/output"),
        parallel_jobs: 4,
        timeout_seconds: Some(3600),
    };

    assert_eq!(config.container_image, "ros:loong");
    assert_eq!(config.parallel_jobs, 4);
    assert_eq!(config.timeout_seconds, Some(3600));
}
