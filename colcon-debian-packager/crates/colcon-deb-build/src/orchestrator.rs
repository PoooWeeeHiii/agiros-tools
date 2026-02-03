//! Build orchestration logic

use std::path::PathBuf;
use std::sync::Arc;

use async_trait::async_trait;
use colcon_deb_config::Config;
use colcon_deb_docker::{DockerService, DockerServiceTrait};
use tracing::{debug, info, warn};

use crate::{
    artifact::ArtifactCollector,
    context::{BuildContext, BuildState},
    error::{BuildError, Result},
    executor::{BuildExecutor, ExecutorConfig},
    graceful_shutdown::{ShutdownGuard, ShutdownManager},
    recovery::{BuildRecoveryStrategy, RecoveryManager, RetryConfig},
};

/// Trait for build orchestration
#[async_trait]
pub trait BuildOrchestratorTrait {
    /// Prepare the build environment
    async fn prepare_environment(&mut self) -> Result<()>;

    /// Run the build process
    async fn run_build(&mut self) -> Result<()>;

    /// Collect build artifacts
    async fn collect_artifacts(&mut self) -> Result<Vec<PathBuf>>;

    /// Get the current build context
    fn context(&self) -> &BuildContext;

    /// Get mutable build context
    fn context_mut(&mut self) -> &mut BuildContext;
}

/// Main build orchestrator implementation
pub struct ColconDebBuilder {
    /// Build configuration
    config: Config,
    /// Build context
    context: BuildContext,
    /// Docker service (wrapped in Arc for sharing)
    docker: Arc<DockerService>,
    /// Build executor
    executor: Option<BuildExecutor>,
    /// Artifact collector
    artifact_collector: ArtifactCollector,
    /// Shutdown manager for graceful shutdown
    shutdown_manager: Arc<ShutdownManager>,
    /// Recovery manager for retry logic
    recovery_manager: RecoveryManager,
    /// Shutdown guard for cleanup
    _shutdown_guard: Option<ShutdownGuard>,
}

impl ColconDebBuilder {
    /// Create a new build orchestrator
    pub async fn new(config: Config) -> Result<Self> {
        let docker = DockerService::new(Default::default())
            .await
            .map_err(BuildError::Docker)?;

        let context = BuildContext::new(config.clone());
        let artifact_collector = ArtifactCollector::new(config.output_dir.clone());

        // Create shutdown manager with appropriate timeout
        let shutdown_manager = Arc::new(ShutdownManager::new(
            std::time::Duration::from_secs(60), // 1 minute graceful shutdown timeout
        ));

        // Create recovery manager with retry strategy
        let recovery_strategy = BuildRecoveryStrategy::Retry; // Could be configurable
        let retry_config = RetryConfig {
            max_attempts: 3,
            initial_delay: std::time::Duration::from_millis(1000),
            max_delay: std::time::Duration::from_secs(30),
            multiplier: 2.0,
            max_elapsed_time: Some(std::time::Duration::from_secs(300)),
        };
        let mut recovery_manager = RecoveryManager::new(recovery_strategy, retry_config);
        recovery_manager.set_shutdown_signal(shutdown_manager.shutdown_signal());

        Ok(Self {
            config,
            context,
            docker: Arc::new(docker),
            executor: None,
            artifact_collector,
            shutdown_manager,
            recovery_manager,
            _shutdown_guard: None,
        })
    }

    /// Initialize the Docker service
    #[allow(dead_code)]
    async fn init_docker(&mut self) -> Result<()> {
        info!("Initializing Docker service");

        // Verify connection by checking if we can list containers
        self.docker
            .list_containers(false)
            .await
            .map_err(BuildError::Docker)?;

        debug!("Docker service initialized successfully");
        Ok(())
    }

    /// Create build executor
    fn create_executor(&mut self) -> Result<()> {
        let executor_config = ExecutorConfig {
            container_image: self.config.docker_image(),
            workspace_path: self.config.colcon_repo.clone(),
            output_dir: self.config.output_dir.clone(),
            parallel_jobs: self.config.parallel_jobs,
            timeout_seconds: None,
        };

        self.executor = Some(BuildExecutor::new(executor_config, Arc::clone(&self.docker)));

        Ok(())
    }

    /// Clean up resources
    async fn cleanup(&mut self) -> Result<()> {
        if let Some(executor) = &mut self.executor {
            executor.cleanup().await?;
        }

        // Execute graceful shutdown
        self.shutdown_manager.graceful_shutdown().await?;

        Ok(())
    }

    /// Get the shutdown manager
    pub fn shutdown_manager(&self) -> Arc<ShutdownManager> {
        Arc::clone(&self.shutdown_manager)
    }

    /// Get the recovery manager reference
    pub fn recovery_manager(&self) -> &RecoveryManager {
        &self.recovery_manager
    }

    /// Check if shutdown was requested
    pub fn is_shutdown_requested(&self) -> bool {
        self.shutdown_manager.is_shutdown_requested()
    }

    /// Setup signal handlers for graceful shutdown
    pub async fn setup_signal_handlers(&self) -> Result<()> {
        crate::graceful_shutdown::setup_signal_handlers(Arc::clone(&self.shutdown_manager)).await
    }
}

#[async_trait]
impl BuildOrchestratorTrait for ColconDebBuilder {
    async fn prepare_environment(&mut self) -> Result<()> {
        info!("Preparing build environment");

        // Check for shutdown before starting
        if self.is_shutdown_requested() {
            return Err(BuildError::ShutdownInProgress);
        }

        // Update build state
        self.context.set_state(BuildState::Preparing);

        // Create shutdown guard for this operation
        self._shutdown_guard = Some(ShutdownGuard::new(
            Arc::clone(&self.shutdown_manager),
            "prepare_environment".to_string(),
        ));

        // Initialize Docker with retry logic
        let docker = Arc::clone(&self.docker);
        self.recovery_manager
            .execute_with_retry("init_docker", || async {
                // Verify connection by checking if we can list containers
                docker
                    .list_containers(false)
                    .await
                    .map_err(BuildError::Docker)?;
                Ok(())
            })
            .await?;

        // Create build executor (no retry needed for local operation)
        self.create_executor()?;

        // Validate workspace
        if !self.config.colcon_repo.exists() {
            return Err(BuildError::environment(format!(
                "Workspace path does not exist: {}",
                self.config.colcon_repo.display()
            )));
        }

        // Create output directory with retry logic
        self.recovery_manager
            .execute_with_retry("create_output_dir", || async {
                std::fs::create_dir_all(&self.config.output_dir).map_err(|e| {
                    BuildError::environment(format!("Failed to create output directory: {e}"))
                })
            })
            .await?;

        // Check for shutdown before completing
        if self.is_shutdown_requested() {
            return Err(BuildError::ShutdownInProgress);
        }

        info!("Build environment prepared successfully");
        self.context.set_state(BuildState::Ready);

        Ok(())
    }

    async fn run_build(&mut self) -> Result<()> {
        info!("Starting build process");

        // Check for shutdown before starting
        if self.is_shutdown_requested() {
            return Err(BuildError::ShutdownInProgress);
        }

        // Update build state
        self.context.set_state(BuildState::Building);

        // Update shutdown guard for this operation
        self._shutdown_guard =
            Some(ShutdownGuard::new(Arc::clone(&self.shutdown_manager), "run_build".to_string()));

        // Check that executor is initialized
        if self.executor.is_none() {
            return Err(BuildError::InvalidConfiguration {
                reason: "Build executor not initialized".to_string(),
            });
        }

        // Run build with basic error handling (retry logic can be added at the package
        // level)
        let result = {
            // Check for shutdown before execution
            if self.is_shutdown_requested() {
                return Err(BuildError::ShutdownInProgress);
            }

            let executor = self.executor.as_mut().unwrap();
            executor.execute_build(&mut self.context).await
        };

        match result {
            Ok(()) => {
                info!("Build completed successfully");
                self.context.set_state(BuildState::Completed);
                Ok(())
            }
            Err(e) => {
                if e.is_shutdown() {
                    info!("Build was cancelled during execution");
                } else {
                    warn!("Build failed: {}", e);
                }
                self.context.set_state(BuildState::Failed);
                Err(e)
            }
        }
    }

    async fn collect_artifacts(&mut self) -> Result<Vec<PathBuf>> {
        info!("Collecting build artifacts");

        // Check for shutdown before starting
        if self.is_shutdown_requested() {
            return Err(BuildError::ShutdownInProgress);
        }

        // Update build state
        self.context.set_state(BuildState::CollectingArtifacts);

        // Update shutdown guard for this operation
        self._shutdown_guard = Some(ShutdownGuard::new(
            Arc::clone(&self.shutdown_manager),
            "collect_artifacts".to_string(),
        ));

        // Get executor
        let executor = self
            .executor
            .as_ref()
            .ok_or_else(|| BuildError::InvalidConfiguration {
                reason: "Build executor not initialized".to_string(),
            })?;

        // Collect artifacts from container
        let artifacts = {
            // Check for shutdown before collection
            if self.is_shutdown_requested() {
                return Err(BuildError::ShutdownInProgress);
            }

            self.artifact_collector
                .collect_from_executor(executor, &self.context)
                .await?
        };

        info!("Collected {} artifacts", artifacts.len());

        // Clean up
        self.cleanup().await?;

        Ok(artifacts)
    }

    fn context(&self) -> &BuildContext {
        &self.context
    }

    fn context_mut(&mut self) -> &mut BuildContext {
        &mut self.context
    }
}

/// Legacy build orchestrator for compatibility  
pub struct BuildOrchestrator {
    inner: ColconDebBuilder,
}

impl BuildOrchestrator {
    /// Create a new build orchestrator
    pub async fn new(config: Config) -> Self {
        Self {
            inner: ColconDebBuilder::new(config)
                .await
                .expect("Failed to create builder"),
        }
    }

    /// Run the build process
    pub async fn build(&mut self) -> std::result::Result<(), colcon_deb_core::error::Error> {
        // Prepare environment
        self.inner.prepare_environment().await.map_err(|e| {
            colcon_deb_core::error::Error::BuildFailed {
                package: "workspace".to_string(),
                reason: e.to_string(),
            }
        })?;

        // Run build
        self.inner
            .run_build()
            .await
            .map_err(|e| colcon_deb_core::error::Error::BuildFailed {
                package: "workspace".to_string(),
                reason: e.to_string(),
            })?;

        // Collect artifacts
        self.inner.collect_artifacts().await.map_err(|e| {
            colcon_deb_core::error::Error::BuildFailed {
                package: "workspace".to_string(),
                reason: e.to_string(),
            }
        })?;

        Ok(())
    }
}
