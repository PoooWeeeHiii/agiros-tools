# Design Details - Docker Integration

This document details the design of the Docker integration component (`colcon-deb-docker`).

## Overview

The Docker integration layer runs on the **host system** and provides async-first container management for executing ROS builds in isolated environments. It wraps the Bollard Docker client with domain-specific abstractions.

## Host-Container Architecture

The Docker service operates exclusively on the host side:
- **Host**: Manages container lifecycle, monitors progress, collects artifacts
- **Container**: Executes build scripts, produces .deb packages

This separation ensures the host application never directly builds packages, maintaining system isolation.

## Architecture

```rust
pub struct DockerService {
    client: Docker,
    config: DockerConfig,
}

pub struct DockerConfig {
    socket_path: Option<String>,
    timeout: Duration,
    max_concurrent_pulls: usize,
}
```

## Key Components

### 1. Docker Client Wrapper

**Purpose**: Provide high-level Docker operations with proper error handling.

```rust
impl DockerService {
    pub async fn new(config: DockerConfig) -> Result<Self>;
    pub async fn pull_image(&self, image: &str) -> Result<()>;
    pub async fn build_image(&self, context: BuildContext) -> Result<String>;
    pub async fn run_container(&self, spec: ContainerSpec) -> Result<ContainerResult>;
}
```

### 2. Container Specification

**Purpose**: Declarative container configuration.

```rust
pub struct ContainerSpec {
    pub image: String,
    pub command: Vec<String>,
    pub environment: HashMap<String, String>,
    pub volumes: Vec<VolumeMount>,
    pub working_dir: Option<String>,
    pub user: Option<String>,
    pub network_mode: NetworkMode,
}

pub struct VolumeMount {
    pub host_path: PathBuf,
    pub container_path: String,
    pub read_only: bool,
}
```

### 3. Build Context Management

**Purpose**: Efficient Docker build context creation.

```rust
pub struct BuildContext {
    pub dockerfile_path: PathBuf,
    pub context_dir: PathBuf,
    pub build_args: HashMap<String, String>,
    pub target: Option<String>,
}

impl BuildContext {
    pub async fn create_tar(&self) -> Result<Vec<u8>>;
}
```

### 4. Output Streaming

**Purpose**: Real-time build output with progress tracking.

```rust
pub trait OutputHandler: Send + Sync {
    fn on_stdout(&mut self, data: &[u8]);
    fn on_stderr(&mut self, data: &[u8]);
    fn on_progress(&mut self, progress: BuildProgress);
}

pub struct BuildProgress {
    pub current_step: usize,
    pub total_steps: usize,
    pub message: String,
}
```

## Key Design Decisions

### 1. Async-First API

All Docker operations are async to prevent blocking:
- Image pulls can take minutes
- Builds can be long-running
- Container execution needs streaming

### 2. Resource Management

**Connection Pooling**: Reuse Docker client connections
```rust
lazy_static! {
    static ref DOCKER_CLIENT: Arc<Docker> = Arc::new(Docker::connect_with_local_defaults().unwrap());
}
```

**Concurrent Operations**: Limit parallel operations
```rust
pub struct DockerPool {
    semaphore: Arc<Semaphore>,
    max_concurrent: usize,
}
```

### 3. Error Handling

Comprehensive error types for Docker operations using `thiserror`:
```rust
use thiserror::Error;

#[derive(Error, Debug)]
pub enum DockerError {
    #[error("Image not found: {0}")]
    ImageNotFound(String),
    
    #[error("Container failed with exit code {code}")]
    ContainerFailed { 
        code: i32, 
        #[source]
        logs: Option<std::io::Error>,
    },
    
    #[error("Build failed: {0}")]
    BuildFailed(String),
    
    #[error("Docker daemon not accessible")]
    DaemonNotAccessible(#[from] bollard::errors::Error),
    
    #[error("Invalid container specification")]
    InvalidSpec(#[source] Box<dyn std::error::Error + Send + Sync>),
    
    #[error("Timeout waiting for container")]
    Timeout(#[from] tokio::time::error::Elapsed),
}
```

### 4. Container Lifecycle

State machine for container management:
```rust
pub enum ContainerState {
    Created,
    Starting,
    Running,
    Stopping,
    Stopped,
    Failed(String),
}
```

### 5. Security Considerations

**User Management Strategy**:
- Container starts as root to set up user
- Creates non-root user matching host UID/GID
- Configures sudo without password for specific commands
- Drops to non-root user for build operations

```rust
pub struct UserSetup {
    host_uid: u32,
    host_gid: u32,
    username: String,
}

impl UserSetup {
    pub fn from_current_user() -> Self {
        Self {
            host_uid: current_uid(),
            host_gid: current_gid(),
            username: "builder".to_string(),
        }
    }
    
    pub fn generate_setup_script(&self) -> String {
        format!(r#"
#!/bin/bash
# Create user matching host UID/GID
groupadd -g {gid} {user}
useradd -u {uid} -g {gid} -m -s /bin/bash {user}

# Configure sudo for specific commands
cat > /etc/sudoers.d/{user} << EOF
{user} ALL=(ALL) NOPASSWD: /usr/bin/apt, /usr/bin/apt-get, /usr/bin/agirosdep
EOF

# Switch to non-root user
exec su - {user} -c "$@"
"#,
            uid = self.host_uid,
            gid = self.host_gid,
            user = self.username,
        )
    }
}
```

**Container Security Options**:
```rust
impl Default for ContainerSecurityOptions {
    fn default() -> Self {
        Self {
            privileged: false,
            read_only_rootfs: false,
            drop_capabilities: vec!["ALL"],
            add_capabilities: vec!["CHOWN", "SETUID", "SETGID", "DAC_OVERRIDE"],
            user_ns_mode: None,  // Use default namespace
        }
    }
}
```

## Integration with Build System

### 1. Build Container Specification

```rust
pub fn create_build_container_spec(
    image: &str,
    colcon_repo: &Path,
    debian_dirs: &Path,
    output_dir: &Path,
    config: &BuildConfig,
) -> ContainerSpec {
    // Prepare helper scripts for cross-architecture compatibility
    let helper_scripts = prepare_helper_scripts().await?;
    
    ContainerSpec {
        image: image.to_string(),
        command: vec!["/scripts/entrypoint.sh".to_string()],
        environment: HashMap::from([
            ("HOST_UID".to_string(), current_uid().to_string()),
            ("HOST_GID".to_string(), current_gid().to_string()),
            ("ROS_DISTRO".to_string(), config.ros_distro.clone()),
            ("PARALLEL_JOBS".to_string(), config.parallel_jobs.to_string()),
            ("BUILD_TYPE".to_string(), "Release".to_string()),
        ]),
        volumes: vec![
            VolumeMount {
                host_path: colcon_repo.join("src"),
                container_path: "/workspace/src".to_string(),
                read_only: true,  // Source is read-only
            },
            VolumeMount {
                host_path: debian_dirs.to_path_buf(),
                container_path: "/workspace/debian_dirs".to_string(),
                read_only: false,  // Writable for bloom-generate
            },
            VolumeMount {
                host_path: scripts_dir(),
                container_path: "/scripts".to_string(),
                read_only: true,  // Scripts are read-only
            },
            VolumeMount {
                host_path: output_dir.to_path_buf(),
                container_path: "/workspace/output".to_string(),
                read_only: false,  // Output is writable
            },
            VolumeMount {
                host_path: helper_scripts,
                container_path: "/helpers".to_string(),
                read_only: true,  // Helper scripts are read-only
            },
        ],
        working_dir: Some("/workspace".to_string()),
        user: None,  // Start as root to create user, then drop privileges
        network_mode: NetworkMode::Host,  // For apt operations
    }
}
```

### 2. Cross-Architecture Helper Scripts

**Challenge**: Running ARM64 containers on AMD64 hosts makes pre-compiled binaries incompatible.

**Solution**: Use interpreted scripts (shell/Python) instead of compiled binaries.

```rust
/// Prepare helper scripts for mounting into container
pub async fn prepare_helper_scripts() -> Result<PathBuf> {
    let scripts_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("scripts")
        .join("helpers");
    
    // Verify scripts exist
    if !scripts_dir.exists() {
        bail!("Helper scripts directory not found: {:?}", scripts_dir);
    }
    
    // Check required scripts
    let required_scripts = [
        "package-scanner.py",
        "debian-preparer.sh",
        "build-orchestrator.sh",
        "progress-reporter.sh",
    ];
    
    for script in &required_scripts {
        let script_path = scripts_dir.join(script);
        if !script_path.exists() {
            bail!("Required helper script not found: {}", script);
        }
    }
    
    Ok(scripts_dir)
}

/// Detect architecture compatibility
pub async fn check_architecture_compatibility(image: &str) -> Result<ArchitectureInfo> {
    let host_arch = std::env::consts::ARCH;
    
    // Inspect image to get its architecture
    let docker = Docker::connect_with_local_defaults()?;
    let image_info = docker.inspect_image(image).await?;
    
    let image_arch = image_info.architecture
        .as_deref()
        .unwrap_or("unknown");
    
    Ok(ArchitectureInfo {
        host_arch: host_arch.to_string(),
        image_arch: image_arch.to_string(),
        compatible: host_arch == image_arch,
    })
}
```

### 3. Output Processing

```rust
pub struct BuildOutputHandler {
    tx: mpsc::Sender<BuildEvent>,
    buffer: LineBuffer,
}

impl OutputHandler for BuildOutputHandler {
    fn on_stdout(&mut self, data: &[u8]) {
        self.buffer.append(data);
        while let Some(line) = self.buffer.next_line() {
            // Parse structured output from helper
            if line.starts_with("::progress::") {
                if let Some(event) = parse_progress_line(line) {
                    let _ = self.tx.send(BuildEvent::Progress(event));
                }
            } else if line.starts_with("::log::") {
                if let Some(log) = parse_log_line(line) {
                    let _ = self.tx.send(BuildEvent::Log(log));
                }
            } else {
                // Regular output
                let _ = self.tx.send(BuildEvent::Output {
                    line: line.to_string(),
                    stream: OutputStream::Stdout,
                });
            }
        }
    }
}

fn parse_progress_line(line: &str) -> Option<ProgressEvent> {
    // Parse "::progress::type=package_start,name=foo"
    let params = line.strip_prefix("::progress::")?;
    let mut fields = HashMap::new();
    
    for pair in params.split(',') {
        let (key, value) = pair.split_once('=')?;
        fields.insert(key, value);
    }
    
    match fields.get("type")? {
        "package_start" => Some(ProgressEvent::PackageStart {
            name: fields.get("name")?.to_string(),
        }),
        "package_complete" => Some(ProgressEvent::PackageComplete {
            name: fields.get("name")?.to_string(),
            success: fields.get("success")? == "true",
        }),
        _ => None,
    }
}
```

## Performance Optimizations

### 1. Image Layer Caching

Leverage Docker's layer caching:
```rust
pub struct ImageCache {
    cache: Arc<RwLock<HashMap<String, ImageInfo>>>,
}

impl ImageCache {
    pub async fn get_or_pull(&self, image: &str) -> Result<ImageInfo>;
}
```

### 2. Build Context Optimization

Minimize build context size:
```rust
pub async fn optimize_build_context(context: &Path) -> Result<()> {
    // Create .dockerignore
    // Remove unnecessary files
    // Compress large files
}
```

### 3. Parallel Container Execution

Run multiple containers concurrently:
```rust
pub struct ContainerPool {
    max_concurrent: usize,
    running: Arc<Mutex<HashSet<String>>>,
}

impl ContainerPool {
    pub async fn run_many(&self, specs: Vec<ContainerSpec>) -> Vec<Result<ContainerResult>>;
}
```

## Testing Strategy

### 1. Mock Docker Client

For unit tests:
```rust
pub trait DockerClient: Send + Sync {
    async fn pull_image(&self, image: &str) -> Result<()>;
    async fn run_container(&self, spec: &ContainerSpec) -> Result<ContainerResult>;
}

pub struct MockDockerClient {
    responses: HashMap<String, ContainerResult>,
}
```

### 2. Integration Tests

Using testcontainers-rs:
```rust
#[tokio::test]
async fn test_build_in_container() {
    let docker = DockerService::new(Default::default()).await.unwrap();
    let result = docker.run_container(&test_spec()).await.unwrap();
    assert_eq!(result.exit_code, 0);
}
```

## Cross-Architecture Support

### Architecture Detection

```rust
#[derive(Debug, Clone)]
pub struct ArchitectureInfo {
    pub host_arch: String,
    pub image_arch: String,
    pub compatible: bool,
}

impl ArchitectureInfo {
    pub fn requires_emulation(&self) -> bool {
        !self.compatible
    }
    
    pub fn performance_warning(&self) -> Option<String> {
        if self.requires_emulation() {
            Some(format!(
                "Running {} container on {} host may be slower due to emulation",
                self.image_arch, self.host_arch
            ))
        } else {
            None
        }
    }
}
```

### Helper Script Structure

We use rust-script for cross-architecture compatibility:

```
scripts/helpers/
├── package-scanner.rs      # Rust script for package discovery
├── debian-preparer.rs      # Rust script for debian management
├── build-orchestrator.rs   # Rust script for build coordination
├── progress-reporter.rs    # Rust script for structured logging
└── check-requirements.rs   # Rust script for dependency verification
```

### Performance Considerations

- **Native Architecture**: Near-native performance with rust-script caching
- **Emulated Architecture**: Slower execution but rust-script still efficient
- **Compilation Overhead**: First-run compilation, then cached execution
- **Memory Usage**: Rust provides efficient memory management

### Testing Cross-Architecture

```rust
#[tokio::test]
async fn test_arm64_container_on_amd64() {
    let service = DockerService::new(Default::default()).await.unwrap();
    
    // This test requires Docker with emulation support
    let spec = ContainerSpec {
        image: "arm64v8/rust:1.75".to_string(),
        command: vec!["/helpers/package-scanner.rs".to_string()],
        // ... other fields
    };
    
    let result = service.run_container(&spec).await;
    
    // Should work even with architecture mismatch
    assert!(result.is_ok());
}
```

## Future Enhancements

1. **BuildKit Support**: Use BuildKit for improved caching
2. **Multi-Platform Builds**: Native support for ARM64 builds  
3. **Remote Docker**: Support remote Docker daemons
4. **Container Reuse**: Keep containers warm for multiple builds
5. **GPU Support**: Enable NVIDIA GPU passthrough
6. **rust-script Optimization**: Pre-warm compilation caches in base images