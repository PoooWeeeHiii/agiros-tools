//! Build executor for running builds in containers

use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;

use colcon_deb_docker::{ContainerSpec, DockerService, DockerServiceTrait, VolumeMount};
use futures::StreamExt;
use tracing::{debug, info, warn};

use crate::{
    context::{BuildContext, PackageBuildResult},
    error::{BuildError, Result},
};

/// Build executor configuration
#[derive(Debug, Clone)]
pub struct ExecutorConfig {
    /// Container image to use
    pub container_image: String,
    /// Workspace path on host
    pub workspace_path: PathBuf,
    /// Output directory on host
    pub output_dir: PathBuf,
    /// Number of parallel jobs
    pub parallel_jobs: usize,
    /// Build timeout in seconds
    pub timeout_seconds: Option<u64>,
}

/// Build executor for running builds in Docker containers
pub struct BuildExecutor {
    /// Executor configuration
    config: ExecutorConfig,
    /// Docker service
    docker: Arc<DockerService>,
    /// Active container ID
    container_id: Option<String>,
}

impl BuildExecutor {
    /// Create a new build executor
    pub fn new(config: ExecutorConfig, docker: Arc<DockerService>) -> Self {
        Self { config, docker, container_id: None }
    }

    /// Get the container ID
    pub fn container_id(&self) -> Option<&str> {
        self.container_id.as_deref()
    }

    /// Create and run container for build
    async fn create_and_run_container(&mut self) -> Result<String> {
        info!("Creating build container");

        // Set up volume mounts
        let mounts = vec![
            VolumeMount {
                host_path: self.config.workspace_path.clone(),
                container_path: "/workspace".to_string(),
                read_only: false,
            },
            VolumeMount {
                host_path: self.config.output_dir.clone(),
                container_path: "/output".to_string(),
                read_only: false,
            },
        ];

        // Set up environment variables
        let mut env = HashMap::new();
        env.insert("PARALLEL_JOBS".to_string(), self.config.parallel_jobs.to_string());
        env.insert("BUILD_OUTPUT_DIR".to_string(), "/output".to_string());
        if let Ok(src_dir) = std::env::var("COLCON_SRC_DIR") {
            env.insert("COLCON_SRC_DIR".to_string(), src_dir);
        }

        // Create container spec
        let spec = ContainerSpec {
            image: self.config.container_image.clone(),
            command: vec!["/entrypoint.sh".to_string(), "build".to_string()],
            environment: env,
            volumes: mounts,
            working_dir: Some("/workspace".to_string()),
            user: None,
        };

        // Run container
        let container_id = self
            .docker
            .run_container(&spec)
            .await
            .map_err(BuildError::Docker)?;

        debug!("Started container: {}", container_id);
        self.container_id = Some(container_id.clone());

        Ok(container_id)
    }

    /// Monitor build progress from container logs
    #[allow(dead_code)]
    async fn monitor_progress(&mut self, context: &mut BuildContext) -> Result<()> {
        if let Some(container_id) = &self.container_id {
            let mut log_stream = self
                .docker
                .container_logs(container_id)
                .await
                .map_err(BuildError::Docker)?;

            while let Some(log_result) = log_stream.next().await {
                match log_result {
                    Ok(log_output) => {
                        match log_output {
                            colcon_deb_docker::service::LogOutput::Stdout(line) => {
                                debug!("Build output: {}", line);
                                // Parse progress from output
                                self.parse_progress_line(&line, context);
                            }
                            colcon_deb_docker::service::LogOutput::Stderr(line) => {
                                debug!("Build error: {}", line);
                            }
                        }
                    }
                    Err(e) => {
                        warn!("Error reading logs: {}", e);
                    }
                }
            }
        }

        Ok(())
    }

    /// Parse progress line from build output
    #[allow(dead_code)]
    fn parse_progress_line(&self, line: &str, context: &mut BuildContext) {
        // Simple parsing for now - look for package build status
        if line.contains("Building package:") {
            if let Some(name) = line.split("Building package:").nth(1) {
                let name = name.trim();
                info!("Building package: {}", name);
            }
        } else if line.contains("Package") && line.contains("built successfully") {
            if let Some(name) = line.split("Package").nth(1) {
                if let Some(name) = name.split("built").next() {
                    let name = name.trim();
                    info!("Package {} built successfully", name);

                    context.add_result(PackageBuildResult {
                        name: name.to_string(),
                        success: true,
                        error: None,
                        duration: Duration::from_secs(0), // Placeholder
                        artifacts: vec![],
                    });
                }
            }
        }
    }

    /// Execute the build
    pub async fn execute_build(&mut self, context: &mut BuildContext) -> Result<()> {
        // Create and run container
        let container_id = self.create_and_run_container().await?;
        context.set_container_id(Some(container_id.clone()));

        // Wait for container with timeout
        let timeout = self
            .config
            .timeout_seconds
            .map(Duration::from_secs)
            .unwrap_or(Duration::from_secs(3600)); // 1 hour default

        let start = std::time::Instant::now();

        // Monitor progress in background
        let monitor_handle = {
            let container_id = container_id.clone();
            tokio::spawn(async move {
                // Simple monitoring - just track that we're running
                debug!("Progress monitoring started for container {}", container_id);
            })
        };

        // Wait for container to finish
        match tokio::time::timeout(timeout, self.docker.wait_container(&container_id)).await {
            Ok(Ok(exit_code)) => {
                monitor_handle.abort();

                if exit_code == 0 {
                    info!("Build completed successfully");
                    Ok(())
                } else {
                    Err(BuildError::build_failed(
                        "workspace",
                        format!("Container exited with code {exit_code}"),
                    ))
                }
            }
            Ok(Err(e)) => {
                monitor_handle.abort();
                Err(BuildError::Docker(e))
            }
            Err(_) => {
                monitor_handle.abort();

                // Timeout
                warn!("Build timed out after {:?}", start.elapsed());

                // Stop container
                if let Err(e) = self.docker.stop_container(&container_id).await {
                    warn!("Failed to stop container: {}", e);
                }

                Err(BuildError::BuildTimeout { duration_secs: timeout.as_secs() })
            }
        }
    }

    /// Clean up resources
    pub async fn cleanup(&mut self) -> Result<()> {
        if let Some(container_id) = &self.container_id {
            info!("Cleaning up container: {}", container_id);

            // Remove container
            if let Err(e) = self.docker.remove_container(container_id).await {
                warn!("Failed to remove container: {}", e);
            }

            self.container_id = None;
        }

        Ok(())
    }

    /// Get build logs
    pub async fn get_logs(&self) -> Result<Vec<String>> {
        if let Some(container_id) = &self.container_id {
            let mut logs = Vec::new();
            let mut log_stream = self
                .docker
                .container_logs(container_id)
                .await
                .map_err(BuildError::Docker)?;

            while let Some(log_result) = log_stream.next().await {
                match log_result {
                    Ok(log_output) => match log_output {
                        colcon_deb_docker::service::LogOutput::Stdout(line) => {
                            logs.push(line);
                        }
                        colcon_deb_docker::service::LogOutput::Stderr(line) => {
                            logs.push(format!("ERROR: {line}"));
                        }
                    },
                    Err(e) => {
                        warn!("Error reading logs: {}", e);
                        break;
                    }
                }
            }

            Ok(logs)
        } else {
            Ok(vec![])
        }
    }

    /// Copy file from container
    pub async fn copy_from_container(&self, container_path: &str) -> Result<Vec<u8>> {
        if let Some(container_id) = &self.container_id {
            self.docker
                .copy_from_container(container_id, container_path)
                .await
                .map_err(BuildError::Docker)
        } else {
            Err(BuildError::InvalidConfiguration { reason: "No active container".to_string() })
        }
    }
}
