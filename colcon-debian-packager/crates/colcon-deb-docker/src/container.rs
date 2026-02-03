//! Container management

use std::collections::HashMap;
use std::path::PathBuf;
use std::pin::Pin;

use bollard::container::{Config, CreateContainerOptions, LogsOptions, RemoveContainerOptions};
use bollard::exec::{CreateExecOptions, StartExecResults};
use bollard::models::HostConfig;
use futures::{Stream, StreamExt};
use serde::{Deserialize, Serialize};

use crate::error::{DockerError, Result};
use crate::progress::{Progress, ProgressEvent, ProgressParser, ProgressStream};
use crate::service::{ContainerOutput, LogOutput};

/// Container specification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContainerSpec {
    /// Docker image to use
    pub image: String,

    /// Command to execute
    pub command: Vec<String>,

    /// Environment variables
    pub environment: HashMap<String, String>,

    /// Volume mounts
    pub volumes: Vec<VolumeMount>,

    /// Working directory
    pub working_dir: Option<String>,

    /// User to run as
    pub user: Option<String>,
}

/// Volume mount specification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VolumeMount {
    /// Host path
    pub host_path: PathBuf,

    /// Container path
    pub container_path: String,

    /// Read-only mount
    pub read_only: bool,
}

impl ContainerSpec {
    /// Create a new container specification
    pub fn new(image: String) -> Self {
        Self {
            image,
            command: Vec::new(),
            environment: HashMap::new(),
            volumes: Vec::new(),
            working_dir: None,
            user: None,
        }
    }

    /// Add a command
    pub fn with_command(mut self, command: Vec<String>) -> Self {
        self.command = command;
        self
    }

    /// Add environment variable
    pub fn with_env(mut self, key: String, value: String) -> Self {
        self.environment.insert(key, value);
        self
    }

    /// Add volume mount
    pub fn with_volume(mut self, mount: VolumeMount) -> Self {
        self.volumes.push(mount);
        self
    }

    /// Set working directory
    pub fn with_working_dir(mut self, dir: String) -> Self {
        self.working_dir = Some(dir);
        self
    }

    /// Set user
    pub fn with_user(mut self, user: String) -> Self {
        self.user = Some(user);
        self
    }

    /// Convert to Bollard container config
    pub fn to_container_config(&self) -> Config<String> {
        let mut config = Config {
            image: Some(self.image.clone()),
            cmd: if self.command.is_empty() {
                None
            } else {
                Some(self.command.clone())
            },
            env: Some(
                self.environment
                    .iter()
                    .map(|(k, v)| format!("{k}={v}"))
                    .collect(),
            ),
            working_dir: self.working_dir.clone(),
            user: self.user.clone(),
            ..Default::default()
        };

        // Configure host config with volumes
        if !self.volumes.is_empty() {
            let binds: Vec<String> = self
                .volumes
                .iter()
                .map(|v| {
                    format!(
                        "{}:{}{}",
                        v.host_path.display(),
                        v.container_path,
                        if v.read_only { ":ro" } else { "" }
                    )
                })
                .collect();

            config.host_config = Some(HostConfig { binds: Some(binds), ..Default::default() });
        }

        config
    }
}

/// Container manager for executing operations
pub struct ContainerManager<'a> {
    client: &'a bollard::Docker,
}

impl<'a> ContainerManager<'a> {
    /// Create a new container manager
    pub fn new(client: &'a bollard::Docker) -> Self {
        Self { client }
    }

    /// Create and start a container
    pub async fn run(&self, spec: &ContainerSpec) -> Result<String> {
        let config = spec.to_container_config();
        let options = CreateContainerOptions::<String>::default();

        let container = self
            .client
            .create_container(Some(options), config)
            .await
            .map_err(DockerError::Client)?;

        self.client
            .start_container::<String>(&container.id, None)
            .await
            .map_err(DockerError::Client)?;

        Ok(container.id)
    }

    /// Execute command in container
    pub async fn exec(&self, container_id: &str, command: Vec<String>) -> Result<ContainerOutput> {
        let exec_config = CreateExecOptions {
            attach_stdout: Some(true),
            attach_stderr: Some(true),
            cmd: Some(command),
            ..Default::default()
        };

        let exec = self
            .client
            .create_exec(container_id, exec_config)
            .await
            .map_err(DockerError::Client)?;

        let mut stdout_data = Vec::new();
        let mut stderr_data = Vec::new();

        match self
            .client
            .start_exec(&exec.id, None)
            .await
            .map_err(DockerError::Client)?
        {
            StartExecResults::Attached { mut output, .. } => {
                while let Some(Ok(chunk)) = output.next().await {
                    match chunk {
                        bollard::container::LogOutput::StdOut { message } => {
                            stdout_data.extend_from_slice(&message);
                        }
                        bollard::container::LogOutput::StdErr { message } => {
                            stderr_data.extend_from_slice(&message);
                        }
                        _ => {}
                    }
                }
            }
            StartExecResults::Detached => {
                return Err(DockerError::ExecutionFailed {
                    reason: "Exec started in detached mode".to_string(),
                });
            }
        }

        let inspect = self
            .client
            .inspect_exec(&exec.id)
            .await
            .map_err(DockerError::Client)?;

        Ok(ContainerOutput {
            exit_code: inspect.exit_code.unwrap_or(0),
            stdout: stdout_data,
            stderr: stderr_data,
        })
    }

    /// Get container logs
    pub async fn logs(
        &self,
        container_id: &str,
    ) -> Result<Pin<Box<dyn Stream<Item = Result<LogOutput>> + Send>>> {
        let options = LogsOptions::<String> {
            stdout: true,
            stderr: true,
            follow: true,
            ..Default::default()
        };

        let stream = self
            .client
            .logs(container_id, Some(options))
            .map(|result| match result {
                Ok(output) => match output {
                    bollard::container::LogOutput::StdOut { message } => {
                        Ok(LogOutput::Stdout(String::from_utf8_lossy(&message).to_string()))
                    }
                    bollard::container::LogOutput::StdErr { message } => {
                        Ok(LogOutput::Stderr(String::from_utf8_lossy(&message).to_string()))
                    }
                    _ => Ok(LogOutput::Stdout(String::new())),
                },
                Err(e) => Err(DockerError::Client(e)),
            });

        Ok(Box::pin(stream))
    }

    /// Remove container
    pub async fn remove(&self, container_id: &str, force: bool) -> Result<()> {
        let options = RemoveContainerOptions { force, ..Default::default() };

        self.client
            .remove_container(container_id, Some(options))
            .await
            .map_err(DockerError::Client)?;

        Ok(())
    }

    /// Get container logs as a progress stream
    pub async fn progress_stream(
        &self,
        container_id: &str,
    ) -> Result<Pin<Box<dyn Stream<Item = Result<ProgressEvent>> + Send>>> {
        let options = LogsOptions::<String> {
            stdout: true,
            stderr: true,
            follow: true,
            ..Default::default()
        };

        let log_stream = self
            .client
            .logs(container_id, Some(options))
            .map(|result| match result {
                Ok(output) => match output {
                    bollard::container::LogOutput::StdOut { message } => {
                        Ok(String::from_utf8_lossy(&message).to_string())
                    }
                    bollard::container::LogOutput::StdErr { message } => {
                        Ok(String::from_utf8_lossy(&message).to_string())
                    }
                    _ => Ok(String::new()),
                },
                Err(e) => Err(DockerError::Client(e)),
            });

        Ok(Box::pin(ProgressStream::new(log_stream)))
    }

    /// Execute command with progress monitoring
    pub async fn exec_with_progress<F>(
        &self,
        container_id: &str,
        command: Vec<String>,
        mut progress_callback: F,
    ) -> Result<ContainerOutput>
    where
        F: FnMut(&ProgressEvent),
    {
        let exec_config = CreateExecOptions {
            attach_stdout: Some(true),
            attach_stderr: Some(true),
            cmd: Some(command),
            ..Default::default()
        };

        let exec = self
            .client
            .create_exec(container_id, exec_config)
            .await
            .map_err(DockerError::Client)?;

        let mut stdout_data = Vec::new();
        let mut stderr_data = Vec::new();
        let mut progress = Progress::default();

        match self
            .client
            .start_exec(&exec.id, None)
            .await
            .map_err(DockerError::Client)?
        {
            StartExecResults::Attached { output: mut stream, .. } => {
                let parser = crate::progress::DefaultProgressParser;
                let mut line_buffer = String::new();

                while let Some(Ok(chunk)) = stream.next().await {
                    match chunk {
                        bollard::container::LogOutput::StdOut { message } => {
                            stdout_data.extend_from_slice(&message);
                            let text = String::from_utf8_lossy(&message);
                            line_buffer.push_str(&text);

                            // Process complete lines
                            while let Some(newline_pos) = line_buffer.find('\n') {
                                let line = line_buffer[..newline_pos].to_string();
                                line_buffer.drain(..=newline_pos);

                                if let Some(event) = parser.parse_line(&line) {
                                    progress.update(&event);
                                    progress_callback(&event);
                                }
                            }
                        }
                        bollard::container::LogOutput::StdErr { message } => {
                            stderr_data.extend_from_slice(&message);
                            let text = String::from_utf8_lossy(&message);
                            line_buffer.push_str(&text);

                            // Process complete lines from stderr too
                            while let Some(newline_pos) = line_buffer.find('\n') {
                                let line = line_buffer[..newline_pos].to_string();
                                line_buffer.drain(..=newline_pos);

                                if let Some(event) = parser.parse_line(&line) {
                                    progress.update(&event);
                                    progress_callback(&event);
                                }
                            }
                        }
                        _ => {}
                    }
                }

                // Process any remaining buffer
                if !line_buffer.is_empty() {
                    if let Some(event) = parser.parse_line(&line_buffer) {
                        progress_callback(&event);
                    }
                }
            }
            StartExecResults::Detached => {
                return Err(DockerError::ExecutionFailed {
                    reason: "Exec started in detached mode".to_string(),
                });
            }
        }

        let inspect = self
            .client
            .inspect_exec(&exec.id)
            .await
            .map_err(DockerError::Client)?;

        Ok(ContainerOutput {
            exit_code: inspect.exit_code.unwrap_or(0),
            stdout: stdout_data,
            stderr: stderr_data,
        })
    }

    /// Stream container output with progress tracking
    pub async fn stream_with_progress<F>(
        &self,
        container_id: &str,
        mut progress_callback: F,
    ) -> Result<Progress>
    where
        F: FnMut(&ProgressEvent),
    {
        let mut progress = Progress::default();
        let mut progress_stream = self.progress_stream(container_id).await?;

        while let Some(result) = progress_stream.next().await {
            match result {
                Ok(event) => {
                    progress.update(&event);
                    progress_callback(&event);
                }
                Err(e) => {
                    // Log error but continue processing
                    eprintln!("Error parsing progress event: {e}");
                }
            }
        }

        Ok(progress)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::arch;

    #[tokio::test]
    async fn test_container_spec_creation() {
        let spec = ContainerSpec::new("ubuntu:22.04".to_string())
            .with_command(vec!["echo".to_string(), "hello".to_string()])
            .with_env("TEST_VAR".to_string(), "test_value".to_string())
            .with_working_dir("/workspace".to_string());

        assert_eq!(spec.image, "ubuntu:22.04");
        assert_eq!(spec.command, vec!["echo", "hello"]);
        assert!(spec.environment.contains_key("TEST_VAR"));
        assert_eq!(spec.working_dir, Some("/workspace".to_string()));
    }

    #[tokio::test]
    async fn test_architecture_detection() {
        let host_arch = arch::detect_host_architecture();
        assert!(!host_arch.arch.is_empty());
        assert!(
            host_arch.arch == "amd64" || host_arch.arch == "arm64" || !host_arch.arch.is_empty()
        );
    }

    #[test]
    fn test_progress_parsing() {
        use crate::progress::{DefaultProgressParser, ProgressParser};

        let parser = DefaultProgressParser;

        // Test package start parsing
        let line = "::progress::package_start::my_package [1/10]";
        let event = parser.parse_line(line).unwrap();

        match event {
            ProgressEvent::PackageStart { name, current, total } => {
                assert_eq!(name, "my_package");
                assert_eq!(current, Some(1));
                assert_eq!(total, Some(10));
            }
            _ => panic!("Unexpected event type"),
        }

        // Test log parsing
        let line = "::log::info::Building package";
        let event = parser.parse_line(line).unwrap();

        match event {
            ProgressEvent::Log { level, message } => {
                assert_eq!(level, crate::progress::LogLevel::Info);
                assert_eq!(message, "Building package");
            }
            _ => panic!("Unexpected event type"),
        }
    }
}
