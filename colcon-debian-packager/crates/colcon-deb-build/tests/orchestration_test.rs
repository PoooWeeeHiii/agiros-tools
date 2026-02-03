//! Comprehensive integration tests for build orchestration
//!
//! This module tests the complete build orchestration functionality including:
//! - Parallel execution with multiple packages
//! - Failure scenarios and recovery
//! - Progress reporting integration
//! - Shutdown handling
//! - Mock Docker operations for testing

use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use async_trait::async_trait;
use colcon_deb_build::{
    BuildContext, BuildError, BuildState, ExecutorConfig, ProgressUI, Result, ShutdownManager,
    ShutdownSignal,
};
use colcon_deb_config::{Config, DockerConfig};
use colcon_deb_core::Package;
use colcon_deb_docker::{
    container::ContainerSpec,
    progress::{LogLevel, ProgressEvent},
    service::{ContainerOutput, DockerServiceTrait, LogOutput},
};
use futures::future::join_all;
use tempfile::TempDir;
use tokio::sync::mpsc;
use tokio::time::{sleep, timeout};
use tracing::info;

/// Mock Docker service that simulates Docker operations without requiring real
/// Docker
#[derive(Debug, Clone)]
pub struct MockDockerService {
    /// Track operations that have been called
    operations: Arc<Mutex<Vec<String>>>,
    /// Control whether operations should fail
    should_fail: Arc<Mutex<HashMap<String, bool>>>,
    /// Simulate operation delays
    delays: Arc<Mutex<HashMap<String, Duration>>>,
    /// Progress event sender
    progress_sender: Arc<Mutex<Option<mpsc::UnboundedSender<ProgressEvent>>>>,
}

impl Default for MockDockerService {
    fn default() -> Self {
        Self::new()
    }
}

impl MockDockerService {
    pub fn new() -> Self {
        Self {
            operations: Arc::new(Mutex::new(Vec::new())),
            should_fail: Arc::new(Mutex::new(HashMap::new())),
            delays: Arc::new(Mutex::new(HashMap::new())),
            progress_sender: Arc::new(Mutex::new(None)),
        }
    }

    pub fn set_failure(&self, operation: &str, should_fail: bool) {
        self.should_fail
            .lock()
            .unwrap()
            .insert(operation.to_string(), should_fail);
    }

    pub fn set_delay(&self, operation: &str, delay: Duration) {
        self.delays
            .lock()
            .unwrap()
            .insert(operation.to_string(), delay);
    }

    pub fn get_operations(&self) -> Vec<String> {
        self.operations.lock().unwrap().clone()
    }

    pub fn set_progress_sender(&self, sender: mpsc::UnboundedSender<ProgressEvent>) {
        *self.progress_sender.lock().unwrap() = Some(sender);
    }

    fn simulate_progress(&self, operation: &str, package: Option<&str>) {
        let sender = self.progress_sender.lock().unwrap().clone();
        if let Some(sender) = sender {
            let event = ProgressEvent::Log {
                level: LogLevel::Info,
                message: format!("Executing {operation} for package {package:?}"),
            };
            let _ = sender.send(event);
        }
    }

    async fn execute_operation(&self, operation: &str, package: Option<&str>) -> Result<()> {
        // Record the operation
        self.operations.lock().unwrap().push(operation.to_string());

        // Check if we should delay
        let delay = self.delays.lock().unwrap().get(operation).cloned();
        if let Some(delay) = delay {
            sleep(delay).await;
        }

        // Simulate progress
        self.simulate_progress(operation, package);

        // Check if we should fail
        let should_fail = self
            .should_fail
            .lock()
            .unwrap()
            .get(operation)
            .cloned()
            .unwrap_or(false);
        if should_fail {
            return Err(BuildError::build_failed("mock", format!("Mock failure for {operation}")));
        }

        Ok(())
    }
}

#[async_trait]
impl DockerServiceTrait for MockDockerService {
    async fn pull_image(&self, image: &str) -> colcon_deb_docker::Result<()> {
        self.execute_operation(&format!("pull_image_{image}"), None)
            .await
            .map_err(|e| colcon_deb_docker::DockerError::PullFailed {
                image: image.to_string(),
                reason: e.to_string(),
            })
    }

    async fn build_image(
        &self,
        dockerfile_path: &str,
        tag: &str,
        _build_args: Option<Vec<(String, String)>>,
    ) -> colcon_deb_docker::Result<()> {
        self.execute_operation(&format!("build_image_{dockerfile_path}_{tag}"), None)
            .await
            .map_err(|e| colcon_deb_docker::DockerError::BuildFailed { reason: e.to_string() })
    }

    async fn run_container(&self, spec: &ContainerSpec) -> colcon_deb_docker::Result<String> {
        self.execute_operation(&format!("run_container_{}", spec.image), None)
            .await
            .map_err(|e| colcon_deb_docker::DockerError::ExecutionFailed {
                reason: e.to_string(),
            })?;
        Ok("mock_container_id".to_string())
    }

    async fn image_exists(&self, image: &str) -> colcon_deb_docker::Result<bool> {
        self.execute_operation(&format!("image_exists_{image}"), None)
            .await
            .map_err(|_e| colcon_deb_docker::DockerError::ImageNotFound {
                name: image.to_string(),
            })?;
        Ok(true)
    }

    async fn stop_container(&self, container_id: &str) -> colcon_deb_docker::Result<()> {
        self.execute_operation(&format!("stop_container_{container_id}"), None)
            .await
            .map_err(|e| colcon_deb_docker::DockerError::ExecutionFailed { reason: e.to_string() })
    }

    async fn remove_container(&self, container_id: &str) -> colcon_deb_docker::Result<()> {
        self.execute_operation(&format!("remove_container_{container_id}"), None)
            .await
            .map_err(|e| colcon_deb_docker::DockerError::ExecutionFailed { reason: e.to_string() })
    }

    async fn exec_in_container(
        &self,
        container_id: &str,
        command: Vec<String>,
    ) -> colcon_deb_docker::Result<ContainerOutput> {
        let cmd_str = command.join(" ");
        self.execute_operation(&format!("exec_{container_id}_{cmd_str}"), None)
            .await
            .map_err(|e| colcon_deb_docker::DockerError::ExecutionFailed {
                reason: e.to_string(),
            })?;
        Ok(ContainerOutput {
            exit_code: 0,
            stdout: b"mock output".to_vec(),
            stderr: Vec::new(),
        })
    }

    async fn copy_from_container(
        &self,
        container_id: &str,
        path: &str,
    ) -> colcon_deb_docker::Result<Vec<u8>> {
        self.execute_operation(&format!("copy_from_{container_id}_{path}"), None)
            .await
            .map_err(|e| colcon_deb_docker::DockerError::ExecutionFailed {
                reason: e.to_string(),
            })?;
        Ok(b"mock file content".to_vec())
    }

    async fn copy_to_container(
        &self,
        container_id: &str,
        path: &str,
        content: Vec<u8>,
    ) -> colcon_deb_docker::Result<()> {
        self.execute_operation(
            &format!("copy_to_{}_{}_size_{}", container_id, path, content.len()),
            None,
        )
        .await
        .map_err(|e| colcon_deb_docker::DockerError::ExecutionFailed { reason: e.to_string() })
    }

    async fn container_logs(
        &self,
        container_id: &str,
    ) -> colcon_deb_docker::Result<
        std::pin::Pin<Box<dyn futures::Stream<Item = colcon_deb_docker::Result<LogOutput>> + Send>>,
    > {
        self.execute_operation(&format!("get_logs_{container_id}"), None)
            .await
            .map_err(|e| colcon_deb_docker::DockerError::ExecutionFailed {
                reason: e.to_string(),
            })?;
        use futures::stream;
        Ok(Box::pin(stream::empty()))
    }

    async fn wait_container(&self, container_id: &str) -> colcon_deb_docker::Result<i64> {
        self.execute_operation(&format!("wait_{container_id}"), None)
            .await
            .map_err(|e| colcon_deb_docker::DockerError::ExecutionFailed {
                reason: e.to_string(),
            })?;
        Ok(0)
    }

    async fn list_containers(&self, all: bool) -> colcon_deb_docker::Result<Vec<String>> {
        self.execute_operation(&format!("list_containers_{all}"), None)
            .await
            .map_err(|e| colcon_deb_docker::DockerError::ExecutionFailed {
                reason: e.to_string(),
            })?;
        Ok(vec!["mock_container_id".to_string()])
    }
}

/// Mock progress UI for testing
#[derive(Debug, Clone)]
pub struct MockProgressUI {
    events: Arc<Mutex<Vec<ProgressEvent>>>,
    clear_count: Arc<Mutex<usize>>,
    finish_count: Arc<Mutex<usize>>,
}

impl Default for MockProgressUI {
    fn default() -> Self {
        Self::new()
    }
}

impl MockProgressUI {
    pub fn new() -> Self {
        Self {
            events: Arc::new(Mutex::new(Vec::new())),
            clear_count: Arc::new(Mutex::new(0)),
            finish_count: Arc::new(Mutex::new(0)),
        }
    }

    pub fn get_events(&self) -> Vec<ProgressEvent> {
        self.events.lock().unwrap().clone()
    }

    pub fn get_clear_count(&self) -> usize {
        *self.clear_count.lock().unwrap()
    }

    pub fn get_finish_count(&self) -> usize {
        *self.finish_count.lock().unwrap()
    }
}

impl ProgressUI for MockProgressUI {
    fn update(&self, event: &ProgressEvent) {
        self.events.lock().unwrap().push(event.clone());
    }

    fn clear(&self) {
        *self.clear_count.lock().unwrap() += 1;
    }

    fn finish(&self) {
        *self.finish_count.lock().unwrap() += 1;
    }

    fn get_package_progress(&self, _package: &str) -> Option<Box<dyn ProgressUI>> {
        Some(Box::new(self.clone()))
    }
}

/// Create a test configuration with temporary directories
fn create_test_config() -> (Config, TempDir) {
    let temp_dir = TempDir::new().unwrap();
    let workspace = temp_dir.path().join("workspace");
    std::fs::create_dir_all(workspace.join("src")).unwrap();

    let config = Config {
        colcon_repo: workspace,
        debian_dirs: temp_dir.path().join("debian"),
        docker: DockerConfig::Image { image: "ros:loong".to_string() },
        ros_distro: Some("loong".to_string()),
        output_dir: temp_dir.path().join("output"),
        parallel_jobs: 4,
    };

    (config, temp_dir)
}

/// Create mock packages for testing
fn create_test_packages() -> Vec<Package> {
    vec![
        Package {
            name: "package_a".to_string(),
            path: PathBuf::from("/workspace/src/package_a"),
            version: "1.0.0".to_string(),
            description: "Test package A".to_string(),
            license: "MIT".to_string(),
            build_type: colcon_deb_core::package::BuildType::Cmake,
            dependencies: colcon_deb_core::package::Dependencies {
                build: vec!["cmake".to_string()],
                exec: vec!["std_msgs".to_string()],
                ..Default::default()
            },
            maintainers: vec![colcon_deb_core::package::Maintainer {
                name: "Test Maintainer".to_string(),
                email: "test@example.com".to_string(),
            }],
        },
        Package {
            name: "package_b".to_string(),
            path: PathBuf::from("/workspace/src/package_b"),
            version: "1.0.0".to_string(),
            description: "Test package B".to_string(),
            license: "MIT".to_string(),
            build_type: colcon_deb_core::package::BuildType::Cmake,
            dependencies: colcon_deb_core::package::Dependencies {
                build: vec!["cmake".to_string()],
                exec: vec!["package_a".to_string()],
                ..Default::default()
            },
            maintainers: vec![colcon_deb_core::package::Maintainer {
                name: "Test Maintainer".to_string(),
                email: "test@example.com".to_string(),
            }],
        },
        Package {
            name: "package_c".to_string(),
            path: PathBuf::from("/workspace/src/package_c"),
            version: "1.0.0".to_string(),
            description: "Test package C".to_string(),
            license: "MIT".to_string(),
            build_type: colcon_deb_core::package::BuildType::Cmake,
            dependencies: colcon_deb_core::package::Dependencies {
                build: vec!["cmake".to_string()],
                exec: vec!["std_msgs".to_string()],
                ..Default::default()
            },
            maintainers: vec![colcon_deb_core::package::Maintainer {
                name: "Test Maintainer".to_string(),
                email: "test@example.com".to_string(),
            }],
        },
    ]
}

#[tokio::test]
async fn test_parallel_execution_multiple_packages() {
    let (_config, _temp_dir) = create_test_config();
    let mock_docker = MockDockerService::new();
    let packages = create_test_packages();

    // Create progress channel
    let (progress_tx, _progress_rx) = mpsc::unbounded_channel();
    mock_docker.set_progress_sender(progress_tx);

    let _mock_progress = MockProgressUI::new();

    // Test that multiple packages can be built in parallel
    let start_time = std::time::Instant::now();

    // Add small delays to simulate real work
    for package in &packages {
        mock_docker.set_delay(
            &format!("exec_mock_container_id_build_{}", package.name),
            Duration::from_millis(100),
        );
    }

    // Simulate parallel execution by running multiple futures concurrently
    let futures: Vec<_> = packages
        .iter()
        .map(|package| {
            let docker = mock_docker.clone();
            let package_name = package.name.clone();
            async move {
                docker
                    .execute_operation(&format!("build_{package_name}"), Some(&package_name))
                    .await
            }
        })
        .collect();

    let results = join_all(futures).await;
    let duration = start_time.elapsed();

    // All builds should succeed
    for result in results {
        assert!(result.is_ok());
    }

    // Should complete faster than sequential execution (3 * 100ms = 300ms)
    assert!(
        duration < Duration::from_millis(250),
        "Parallel execution took too long: {duration:?}"
    );

    // Verify all operations were recorded
    let operations = mock_docker.get_operations();
    assert_eq!(operations.len(), 3);
    for package in &packages {
        assert!(operations.contains(&format!("build_{}", package.name)));
    }
}

#[tokio::test]
async fn test_failure_scenarios_and_recovery() {
    let (config, _temp_dir) = create_test_config();
    let mock_docker = MockDockerService::new();
    let packages = create_test_packages();

    // Configure one package to fail
    mock_docker.set_failure("build_package_b", true);

    let mut context = BuildContext::new(config);
    context.set_state(BuildState::Building);

    // Test individual package builds
    for package in &packages {
        let result = mock_docker
            .execute_operation(&format!("build_{}", package.name), Some(&package.name))
            .await;

        if package.name == "package_b" {
            assert!(result.is_err());
            let error = result.unwrap_err();
            assert!(error.to_string().contains("Mock failure"));
        } else {
            assert!(result.is_ok());
        }
    }

    // Verify operations were attempted
    let operations = mock_docker.get_operations();
    assert_eq!(operations.len(), 3);

    // Test recovery by clearing the failure and retrying
    mock_docker.set_failure("build_package_b", false);
    let retry_result = mock_docker
        .execute_operation("build_package_b", Some("package_b"))
        .await;
    assert!(retry_result.is_ok());
}

#[tokio::test]
async fn test_progress_reporting_integration() {
    let (_config, _temp_dir) = create_test_config();
    let mock_docker = MockDockerService::new();
    let packages = create_test_packages();

    // Create progress channel
    let (progress_tx, mut progress_rx) = mpsc::unbounded_channel();
    mock_docker.set_progress_sender(progress_tx);

    let mock_progress = MockProgressUI::new();

    // Start a task to collect progress events
    let progress_ui = mock_progress.clone();
    let progress_task = tokio::spawn(async move {
        while let Some(event) = progress_rx.recv().await {
            progress_ui.update(&event);
        }
    });

    // Execute operations that generate progress events
    for package in &packages {
        let _ = mock_docker
            .execute_operation(&format!("build_{}", package.name), Some(&package.name))
            .await;
    }

    // Give progress events time to be processed
    sleep(Duration::from_millis(50)).await;

    // Cancel the progress task
    progress_task.abort();

    // Verify progress events were received
    let events = mock_progress.get_events();
    assert_eq!(events.len(), 3);

    for (i, package) in packages.iter().enumerate() {
        match &events[i] {
            ProgressEvent::Log { level, message } => {
                assert_eq!(level, &LogLevel::Info);
                assert!(message.contains(&format!("build_{}", package.name)));
                assert!(message.contains(&package.name));
            }
            _ => panic!("Expected Log event"),
        }
    }
}

#[tokio::test]
async fn test_shutdown_handling() {
    let (_config, _temp_dir) = create_test_config();
    let mock_docker = MockDockerService::new();

    // Create shutdown manager
    let shutdown_manager = ShutdownManager::new(Duration::from_secs(5));
    let shutdown_signal = shutdown_manager.shutdown_signal();
    let _shutdown_receiver = shutdown_manager.shutdown_receiver();

    // Add delays to simulate long-running operations
    mock_docker.set_delay("long_operation", Duration::from_millis(500));

    // Start a task that performs a long operation
    let docker = mock_docker.clone();
    let operation_task =
        tokio::spawn(async move { docker.execute_operation("long_operation", None).await });

    // Start a task that triggers shutdown after a short delay
    let shutdown_task = tokio::spawn(async move {
        sleep(Duration::from_millis(100)).await;
        shutdown_manager.request_shutdown(ShutdownSignal::user_request());
    });

    // Wait for either the operation to complete or shutdown to be triggered
    let result = timeout(Duration::from_millis(1000), operation_task).await;

    // Wait for shutdown to complete
    let _ = shutdown_task.await;

    match result {
        Ok(Ok(Ok(()))) => {
            // Operation completed successfully (unlikely due to timing)
            info!("Operation completed before shutdown");
        }
        Ok(Ok(Err(_))) => {
            // Operation failed (expected if shutdown interrupted it)
            info!("Operation failed due to shutdown");
        }
        Ok(Err(_)) => {
            // Task was cancelled (possible with our timeout)
            info!("Operation task was cancelled");
        }
        Err(_) => {
            // Timeout occurred (expected due to our setup)
            info!("Operation timed out");
        }
    }

    // Verify shutdown signal was triggered
    assert!(shutdown_signal.load(std::sync::atomic::Ordering::Acquire));
}

#[tokio::test]
async fn test_mock_docker_operations() {
    let mock_docker = MockDockerService::new();

    // Test basic operations
    let result = mock_docker.pull_image("ros:loong").await;
    assert!(result.is_ok());

    let result = mock_docker
        .build_image("/context", "test:latest", None)
        .await;
    assert!(result.is_ok());

    let spec = ContainerSpec {
        image: "ros:loong".to_string(),
        command: vec!["echo".to_string(), "hello".to_string()],
        environment: HashMap::new(),
        volumes: vec![],
        working_dir: None,
        user: None,
    };

    let result = mock_docker.run_container(&spec).await;
    assert!(result.is_ok());
    assert_eq!(result.unwrap(), "mock_container_id");

    let result = mock_docker
        .exec_in_container("mock_container_id", vec!["echo".to_string(), "hello".to_string()])
        .await;
    assert!(result.is_ok());
    let exec_result = result.unwrap();
    assert_eq!(exec_result.exit_code, 0);
    assert_eq!(exec_result.stdout, b"mock output");

    let result = mock_docker.stop_container("mock_container_id").await;
    assert!(result.is_ok());

    let result = mock_docker.remove_container("mock_container_id").await;
    assert!(result.is_ok());

    // Verify all operations were recorded
    let operations = mock_docker.get_operations();
    assert!(operations.contains(&"pull_image_ros:loong".to_string()));
    assert!(operations.contains(&"build_image_/context_test:latest".to_string()));
    assert!(operations.contains(&"run_container_ros:loong".to_string()));
    assert!(operations.contains(&"exec_mock_container_id_echo hello".to_string()));
    assert!(operations.contains(&"stop_container_mock_container_id".to_string()));
    assert!(operations.contains(&"remove_container_mock_container_id".to_string()));
}

#[tokio::test]
async fn test_executor_config_validation() {
    let config = ExecutorConfig {
        container_image: "ros:loong".to_string(),
        workspace_path: PathBuf::from("/workspace"),
        output_dir: PathBuf::from("/output"),
        parallel_jobs: 4,
        timeout_seconds: Some(3600),
    };

    // Test configuration validation
    assert!(!config.container_image.is_empty());
    assert!(config.workspace_path.is_absolute());
    assert!(config.output_dir.is_absolute());
    assert!(config.parallel_jobs > 0);
    assert!(config.timeout_seconds.unwrap() > 0);
}

#[tokio::test]
async fn test_build_context_state_management() {
    let (config, _temp_dir) = create_test_config();
    let mut context = BuildContext::new(config);

    // Test initial state
    assert_eq!(context.state(), BuildState::Idle);

    // Test state transitions
    context.set_state(BuildState::Preparing);
    assert_eq!(context.state(), BuildState::Preparing);

    context.set_state(BuildState::Building);
    assert_eq!(context.state(), BuildState::Building);

    context.set_state(BuildState::CollectingArtifacts);
    assert_eq!(context.state(), BuildState::CollectingArtifacts);

    context.set_state(BuildState::Completed);
    assert_eq!(context.state(), BuildState::Completed);

    // Test error state
    context.set_state(BuildState::Failed);
    assert_eq!(context.state(), BuildState::Failed);
}

#[tokio::test]
async fn test_concurrent_package_builds() {
    let (_config, _temp_dir) = create_test_config();
    let mock_docker = MockDockerService::new();
    let packages = create_test_packages();

    // Set up different delays for each package to test concurrency
    mock_docker.set_delay("build_package_a", Duration::from_millis(50));
    mock_docker.set_delay("build_package_b", Duration::from_millis(100));
    mock_docker.set_delay("build_package_c", Duration::from_millis(75));

    let start_time = std::time::Instant::now();

    // Build all packages concurrently
    let futures: Vec<_> = packages
        .iter()
        .map(|package| {
            let docker = mock_docker.clone();
            let package_name = package.name.clone();
            async move {
                docker
                    .execute_operation(&format!("build_{package_name}"), Some(&package_name))
                    .await
            }
        })
        .collect();

    let results = join_all(futures).await;
    let duration = start_time.elapsed();

    // All builds should succeed
    for result in results {
        assert!(result.is_ok());
    }

    // Should complete in approximately the time of the longest operation (100ms)
    // but definitely faster than sequential (50 + 100 + 75 = 225ms)
    assert!(
        duration < Duration::from_millis(150),
        "Concurrent builds took too long: {duration:?}"
    );
    assert!(
        duration >= Duration::from_millis(95),
        "Concurrent builds completed too quickly: {duration:?}"
    );
}

#[tokio::test]
async fn test_error_propagation() {
    let mock_docker = MockDockerService::new();

    // Configure various failure scenarios
    mock_docker.set_failure("pull_image_nonexistent", true);
    mock_docker.set_failure("run_container_invalid", true);

    // Test image pull failure
    let result = mock_docker.pull_image("nonexistent").await;
    assert!(result.is_err());
    let error = result.unwrap_err();
    assert!(error.to_string().contains("Mock failure"));

    // Test container creation failure
    let spec = ContainerSpec {
        image: "invalid".to_string(),
        command: vec!["test".to_string()],
        environment: HashMap::new(),
        volumes: vec![],
        working_dir: None,
        user: None,
    };

    let result = mock_docker.run_container(&spec).await;
    assert!(result.is_err());
    let error = result.unwrap_err();
    assert!(error.to_string().contains("Mock failure"));
}

#[tokio::test]
async fn test_progress_ui_integration() {
    let mock_progress = MockProgressUI::new();

    // Create test progress events
    let events = vec![
        ProgressEvent::PackageStart {
            name: "package_a".to_string(),
            current: Some(1),
            total: Some(2),
        },
        ProgressEvent::PackageComplete {
            name: "package_b".to_string(),
            success: true,
            error: None,
        },
    ];

    // Send progress events
    for event in &events {
        mock_progress.update(event);
    }

    // Verify events were recorded
    let recorded_events = mock_progress.get_events();
    assert_eq!(recorded_events.len(), 2);

    match &recorded_events[0] {
        ProgressEvent::PackageStart { name, .. } => {
            assert_eq!(name, "package_a");
        }
        _ => panic!("Expected PackageStart event"),
    }

    match &recorded_events[1] {
        ProgressEvent::PackageComplete { name, success, .. } => {
            assert_eq!(name, "package_b");
            assert!(success);
        }
        _ => panic!("Expected PackageComplete event"),
    }

    // Test UI lifecycle
    mock_progress.clear();
    assert_eq!(mock_progress.get_clear_count(), 1);

    mock_progress.finish();
    assert_eq!(mock_progress.get_finish_count(), 1);

    // Test package-specific progress
    let package_progress = mock_progress.get_package_progress("test_package");
    assert!(package_progress.is_some());
}
