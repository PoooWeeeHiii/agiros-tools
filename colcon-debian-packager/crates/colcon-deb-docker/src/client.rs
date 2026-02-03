//! Docker client wrapper

use std::pin::Pin;
use std::time::Duration;

use async_trait::async_trait;
use bollard::Docker;
use futures::{Stream, StreamExt};

use crate::container::{ContainerManager, ContainerSpec};
use crate::error::{DockerError, Result};
use crate::image::ImageManager;
use crate::service::{ContainerOutput, DockerServiceTrait, LogOutput};
use crate::types::BuildContext;

/// Docker service configuration
pub struct DockerConfig {
    /// Socket path (None for default)
    pub socket_path: Option<String>,
    /// Operation timeout
    pub timeout: Duration,
    /// Maximum concurrent image pulls
    pub max_concurrent_pulls: usize,
}

impl Default for DockerConfig {
    fn default() -> Self {
        Self {
            socket_path: None,
            timeout: Duration::from_secs(300),
            max_concurrent_pulls: 2,
        }
    }
}

/// High-level Docker service wrapper
pub struct DockerService {
    client: Docker,
    #[allow(dead_code)]
    config: DockerConfig,
}

impl DockerService {
    /// Create a new Docker service
    pub async fn new(config: DockerConfig) -> Result<Self> {
        let client = match &config.socket_path {
            Some(path) => Docker::connect_with_socket(path, 120, bollard::API_DEFAULT_VERSION),
            None => Docker::connect_with_local_defaults(),
        }
        .map_err(DockerError::Client)?;

        // Test connection
        client.ping().await.map_err(DockerError::Client)?;

        Ok(Self { client, config })
    }

    /// Get the underlying Docker client
    pub fn client(&self) -> &Docker {
        &self.client
    }
}

#[async_trait]
impl DockerServiceTrait for DockerService {
    async fn pull_image(&self, image: &str) -> Result<()> {
        let manager = ImageManager::new(&self.client);
        manager.pull(image).await
    }

    async fn image_exists(&self, image: &str) -> Result<bool> {
        let manager = ImageManager::new(&self.client);
        manager.exists(image).await
    }

    async fn build_image(
        &self,
        dockerfile_path: &str,
        tag: &str,
        build_args: Option<Vec<(String, String)>>,
    ) -> Result<()> {
        let dockerfile_content = tokio::fs::read_to_string(dockerfile_path)
            .await
            .map_err(|e| DockerError::BuildFailed {
                reason: format!("Failed to read Dockerfile: {e}"),
            })?;

        let mut context = BuildContext {
            dockerfile: dockerfile_content,
            build_args: Default::default(),
            labels: Default::default(),
            target: None,
        };

        if let Some(args) = build_args {
            context.build_args = args.into_iter().collect();
        }

        let manager = ImageManager::new(&self.client);
        manager.build(&context, tag).await
    }

    async fn run_container(&self, spec: &ContainerSpec) -> Result<String> {
        let manager = ContainerManager::new(&self.client);
        manager.run(spec).await
    }

    async fn exec_in_container(
        &self,
        container_id: &str,
        command: Vec<String>,
    ) -> Result<ContainerOutput> {
        let manager = ContainerManager::new(&self.client);
        manager.exec(container_id, command).await
    }

    async fn container_logs(
        &self,
        container_id: &str,
    ) -> Result<Pin<Box<dyn Stream<Item = Result<LogOutput>> + Send>>> {
        let manager = ContainerManager::new(&self.client);
        manager.logs(container_id).await
    }

    async fn wait_container(&self, container_id: &str) -> Result<i64> {
        use bollard::container::WaitContainerOptions;

        let options = WaitContainerOptions { condition: "not-running" };

        let mut stream = self.client.wait_container(container_id, Some(options));

        if let Some(result) = stream.next().await {
            match result {
                Ok(wait_response) => Ok(wait_response.status_code),
                Err(e) => Err(DockerError::Client(e)),
            }
        } else {
            Err(DockerError::ExecutionFailed {
                reason: "Container wait stream ended unexpectedly".to_string(),
            })
        }
    }

    async fn remove_container(&self, container_id: &str) -> Result<()> {
        let manager = ContainerManager::new(&self.client);
        manager.remove(container_id, true).await
    }

    async fn copy_from_container(&self, container_id: &str, path: &str) -> Result<Vec<u8>> {
        use bollard::container::DownloadFromContainerOptions;

        let options = DownloadFromContainerOptions { path };

        let stream = self
            .client
            .download_from_container(container_id, Some(options));

        let mut bytes = Vec::new();
        futures::pin_mut!(stream);

        while let Some(chunk) = stream.next().await {
            let chunk = chunk.map_err(DockerError::Client)?;
            bytes.extend_from_slice(&chunk);
        }

        Ok(bytes)
    }

    async fn copy_to_container(
        &self,
        container_id: &str,
        path: &str,
        content: Vec<u8>,
    ) -> Result<()> {
        use bollard::container::UploadToContainerOptions;

        let options = UploadToContainerOptions { path, no_overwrite_dir_non_dir: "false" };

        self.client
            .upload_to_container(container_id, Some(options), content.into())
            .await
            .map_err(DockerError::Client)?;

        Ok(())
    }

    async fn list_containers(&self, all: bool) -> Result<Vec<String>> {
        use bollard::container::ListContainersOptions;

        let options = ListContainersOptions::<String> { all, ..Default::default() };

        let containers = self
            .client
            .list_containers(Some(options))
            .await
            .map_err(DockerError::Client)?;

        Ok(containers.into_iter().filter_map(|c| c.id).collect())
    }

    async fn stop_container(&self, container_id: &str) -> Result<()> {
        use bollard::container::StopContainerOptions;

        let options = StopContainerOptions {
            t: 10, // 10 second timeout
        };

        self.client
            .stop_container(container_id, Some(options))
            .await
            .map_err(DockerError::Client)?;

        Ok(())
    }
}
