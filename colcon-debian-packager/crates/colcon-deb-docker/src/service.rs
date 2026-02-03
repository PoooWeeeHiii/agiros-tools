//! Docker service trait and implementations

use std::pin::Pin;

use async_trait::async_trait;
use futures::Stream;

use crate::container::ContainerSpec;
use crate::error::Result;

/// Output from container execution
#[derive(Debug, Clone)]
pub struct ContainerOutput {
    /// Exit code
    pub exit_code: i64,
    /// Standard output
    pub stdout: Vec<u8>,
    /// Standard error
    pub stderr: Vec<u8>,
}

/// Log output from container
#[derive(Debug, Clone)]
pub enum LogOutput {
    /// Standard output line
    Stdout(String),
    /// Standard error line
    Stderr(String),
}

/// Docker service trait for container operations
#[async_trait]
pub trait DockerServiceTrait: Send + Sync {
    /// Pull an image if not present
    async fn pull_image(&self, image: &str) -> Result<()>;

    /// Check if an image exists locally
    async fn image_exists(&self, image: &str) -> Result<bool>;

    /// Build an image from a Dockerfile
    async fn build_image(
        &self,
        dockerfile_path: &str,
        tag: &str,
        build_args: Option<Vec<(String, String)>>,
    ) -> Result<()>;

    /// Create and run a container
    async fn run_container(&self, spec: &ContainerSpec) -> Result<String>;

    /// Execute a command in a running container
    async fn exec_in_container(
        &self,
        container_id: &str,
        command: Vec<String>,
    ) -> Result<ContainerOutput>;

    /// Get container logs as a stream
    async fn container_logs(
        &self,
        container_id: &str,
    ) -> Result<Pin<Box<dyn Stream<Item = Result<LogOutput>> + Send>>>;

    /// Wait for container to finish
    async fn wait_container(&self, container_id: &str) -> Result<i64>;

    /// Remove a container
    async fn remove_container(&self, container_id: &str) -> Result<()>;

    /// Copy files from container
    async fn copy_from_container(&self, container_id: &str, path: &str) -> Result<Vec<u8>>;

    /// Copy files to container
    async fn copy_to_container(
        &self,
        container_id: &str,
        path: &str,
        content: Vec<u8>,
    ) -> Result<()>;

    /// List containers
    async fn list_containers(&self, all: bool) -> Result<Vec<String>>;

    /// Stop a container
    async fn stop_container(&self, container_id: &str) -> Result<()>;
}
