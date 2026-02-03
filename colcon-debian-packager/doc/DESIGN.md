# Design Decisions - Colcon Debian Packager (Rust)

This document captures the key design decisions made for the Rust rewrite of the Colcon Debian Packager.

## Language and Runtime

### Decision: Rust with Tokio Async Runtime
**Rationale:**
- **Performance**: Rust provides zero-cost abstractions and efficient memory management
- **Safety**: Memory safety guarantees prevent common bugs in system-level code
- **Async Support**: Tokio enables efficient concurrent I/O operations
- **Docker Integration**: Excellent async Docker client libraries available

**Trade-offs:**
- Higher initial development complexity
- Steeper learning curve for contributors
- Longer compilation times

## Project Structure

### Decision: Cargo Workspace with Multiple Crates
**Rationale:**
- **Modularity**: Clear separation of concerns
- **Parallel Compilation**: Better build performance
- **Code Reuse**: Shared functionality in dedicated crates
- **Testing**: Isolated unit testing per crate

**Structure:**
```
crates/
├── colcon-deb-cli/      # Binary crate
├── colcon-deb-core/     # Common types
├── colcon-deb-config/   # Configuration
├── colcon-deb-docker/   # Docker ops
├── colcon-deb-ros/      # ROS handling
├── colcon-deb-debian/   # Debian packaging
└── colcon-deb-build/    # Orchestration
```

## Error Handling

### Decision: `thiserror` for Libraries, `eyre` for Applications
**Rationale:**
- **Libraries**: Use `thiserror` for strongly-typed, composable errors
- **Applications**: Use `eyre` for rich error reports with context
- **User Experience**: Detailed error chains and helpful diagnostics
- **Debugging**: Better stack traces and error context

**Library Implementation (using `thiserror`):**
```rust
use thiserror::Error;

#[derive(Error, Debug)]
pub enum BuildError {
    #[error("Package not found: {name}")]
    PackageNotFound { name: String },
    
    #[error("Docker operation failed")]
    Docker(#[from] DockerError),
    
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
}

#[derive(Error, Debug)]
pub enum DockerError {
    #[error("Image not found: {0}")]
    ImageNotFound(String),
    
    #[error("Container failed with exit code {code}")]
    ContainerFailed { code: i32 },
}
```

**Application Implementation (using `eyre`):**
```rust
use eyre::{Result, WrapErr};

fn main() -> Result<()> {
    // Install error hooks for better reports
    color_eyre::install()?;
    
    // Wrap errors with context
    let config = load_config()
        .wrap_err("Failed to load configuration")?;
    
    build_packages(&config)
        .wrap_err_with(|| format!("Build failed for workspace: {:?}", config.workspace))?;
    
    Ok(())
}
```

## Configuration Management

### Decision: YAML Configuration with Three Key Paths
**Rationale:**
- **Simplicity**: Only three required configuration items
- **Flexibility**: User can provide custom Debian directories
- **Compatibility**: Works with existing bloom-generated packages

**Configuration Schema:**
```yaml
# Required configuration
colcon_repo: /path/to/ros/workspace    # Input Colcon repository
debian_dirs: /path/to/debian/configs   # Collection of package debian/ dirs
docker:
  # Option 1: Use existing image
  image: ros:loong-ros-base
  # Option 2: Build from Dockerfile
  dockerfile: /path/to/Dockerfile

# Optional configuration
ros_distro: loong                      # ROS distribution (can auto-detect)
output_dir: ./output                    # Where to place .deb files
parallel_jobs: 4                        # Build parallelism
```

**Debian Directory Structure:**
```
debian_dirs/
├── package_name_1/
│   └── debian/
│       ├── control
│       ├── rules
│       └── ...
├── package_name_2/
│   └── debian/
└── ...
```

## Docker Integration

### Decision: Bollard Async Client
**Rationale:**
- **Async Native**: Built for Tokio
- **Full API Coverage**: Complete Docker API support
- **Active Maintenance**: Well-maintained library
- **Type Safety**: Strong typing for Docker objects

**Key Features:**
- Streaming build output
- Progress callbacks
- Container lifecycle management
- Volume and network handling

## Package Discovery and Analysis

### Decision: Custom Package.xml Parser
**Rationale:**
- **ROS 2 Specific**: Handle ROS 2 package formats (v3)
- **Performance**: Optimized for our use case
- **Ament Focus**: Support ament_cmake and ament_python build systems
- **Validation**: Strict parsing with clear errors

**Implementation:**
- XML parsing with `quick-xml`
- Serde deserialization for structured data
- Support for conditional dependencies
- ROS 2 specific metadata handling

## Build Orchestration

### Decision: Simple Parallel Execution After colcon build
**Rationale:**
- **Simplicity**: colcon already handles all dependency ordering
- **Performance**: Maximum parallelism for .deb creation
- **Correctness**: All dependencies resolved by colcon build
- **Efficiency**: No need for dependency graphs or topological sorting

**Algorithm:**
1. Run `colcon build` (it handles all dependency ordering)
2. Source `install/setup.bash` to load built environment
3. Create all .deb packages in parallel (no ordering needed)
4. Track completion and errors

## Caching Strategy

### Decision: Multi-Level Caching
**Rationale:**
- **Performance**: Avoid redundant work
- **Flexibility**: Different cache strategies
- **Docker Integration**: Leverage Docker layer cache

**Cache Levels:**
1. Docker image layers
2. Built package artifacts
3. Dependency resolution results
4. Workspace state

## CLI Design

### Decision: Clap v4 with Derive API
**Rationale:**
- **Type Safety**: Compile-time argument validation
- **Documentation**: Auto-generated help
- **Subcommands**: Natural command structure
- **Completion**: Shell completion generation

**Command Structure:**
```
colcon-deb
├── build      # Main build command
├── clean      # Cleanup operations
├── validate   # Configuration validation
├── init       # Initialize config
└── version    # Version information
```

## Logging and Observability

### Decision: Tracing with Structured Logging
**Rationale:**
- **Structure**: Machine-readable logs
- **Performance**: Low overhead
- **Flexibility**: Multiple output formats
- **Integration**: Works with standard tools

**Features:**
- Log levels: ERROR, WARN, INFO, DEBUG, TRACE
- Structured fields for filtering
- JSON output for CI/CD
- Pretty printing for interactive use

## Testing Strategy

### Decision: Multi-Level Testing with Test Workspace
**Rationale:**
- **Confidence**: Comprehensive test coverage
- **Speed**: Fast unit tests, slower integration tests
- **Realism**: Test workspace mimics real ROS projects
- **Isolation**: Dedicated test environment

**Test Types:**
1. **Unit Tests**: Per-crate logic testing
2. **Integration Tests**: Component interaction
3. **E2E Tests**: Full builds with test workspace
4. **Property Tests**: Fuzzing with `proptest`

**Test Workspace Structure:**
```
tests/test_workspace/
├── src/
│   ├── simple_publisher/    # Basic C++ package
│   ├── simple_subscriber/   # C++ with dependencies
│   ├── python_node/         # Python package
│   └── meta_package/        # Meta-package example
└── test_configs/
    ├── minimal.yaml         # Minimal configuration
    └── full.yaml           # Full-featured config
```

## Dependency Management

### Decision: Conservative Dependency Selection
**Rationale:**
- **Stability**: Prefer mature, stable crates
- **Maintenance**: Active community support
- **Security**: Regular security updates
- **License**: Compatible open-source licenses

**Key Dependencies:**
- `tokio`: Async runtime (MIT)
- `bollard`: Docker client (Apache-2.0)
- `clap`: CLI framework (MIT/Apache-2.0)
- `serde`: Serialization (MIT/Apache-2.0)
- `tracing`: Logging (MIT)
- `thiserror`: Error derivation for libraries (MIT/Apache-2.0)
- `eyre`: Error reporting for applications (MIT/Apache-2.0)
- `color-eyre`: Enhanced error reports (MIT/Apache-2.0)
- `indicatif`: Progress bars and terminal UI (MIT)
- `tokio-util`: Async utilities and channels (MIT)
- `crossterm`: Terminal manipulation (MIT)
- `ratatui`: Terminal UI framework (MIT)

## File System Operations

### Decision: Async I/O with Tokio
**Rationale:**
- **Performance**: Non-blocking file operations
- **Consistency**: Same async model throughout
- **Scalability**: Handle many files efficiently

**Implementation:**
- `tokio::fs` for async file operations
- Memory-mapped files for large data
- Streaming processing for package files

## Security Considerations

### Decision: Principle of Least Privilege
**Rationale:**
- **Container Security**: No privileged operations
- **File Permissions**: Respect umask
- **Credential Handling**: Secure storage
- **Input Validation**: Strict validation

**Measures:**
- Run containers as non-root
- Validate all external input
- Secure temporary file creation
- Optional GPG signing support

## ROS 2 Focus

### Decision: ROS 2 Only Support
**Rationale:**
- **Modern Ecosystem**: ROS 2 is the future of robotics middleware
- **Clean Architecture**: No legacy ROS 1 complexity
- **Ament Build System**: Consistent modern build tooling
- **Colcon Native**: Built for colcon from the ground up

**Implications:**
- No catkin support
- No rosbuild support
- Package format v3 (ROS 2 style) only
- Ament CMake and Ament Python build types only

## External Command Integration

### Decision: Container-Side Execution of Linux Tools
**Rationale:**
- **Isolation**: Host system doesn't need ROS/Debian tools
- **Compatibility**: Exact behavior matching with ROS ecosystem
- **Flexibility**: Different tool versions per container
- **Reliability**: Battle-tested tools in their native environment

**Host-Side Commands (Rust):**
```rust
// Only Docker commands run on host
- docker pull/build: Image management
- docker run: Container execution
- docker cp: Artifact collection
```

**Container-Side Commands (Shell Scripts):**
```bash
# ROS commands
- agirosdep update/install: Dependency resolution
- colcon build: Package compilation
- agiros pkg list: Package discovery

# Debian commands
- dpkg-deb --build: Package creation
- dpkg-scanpackages: Index generation
- apt-ftparchive: Repository metadata
- lintian: Package validation

# System commands
- tar: Archive operations
- find: File discovery
```

**Container Script Structure:**
```bash
#!/bin/bash
# /scripts/entrypoint.sh - Container entry point (runs as root)

# Create non-root user matching host UID/GID
HOST_UID=${HOST_UID:-1000}
HOST_GID=${HOST_GID:-1000}

groupadd -g $HOST_GID builder 2>/dev/null || true
useradd -u $HOST_UID -g $HOST_GID -m -s /bin/bash builder 2>/dev/null || true

# Configure passwordless sudo for required commands
cat > /etc/sudoers.d/builder << EOF
builder ALL=(ALL) NOPASSWD: /usr/bin/apt, /usr/bin/apt-get, /usr/bin/apt-cache, \
                           /usr/bin/agirosdep, /usr/bin/dpkg, /usr/bin/dpkg-deb
EOF

# Fix ownership of workspace
chown -R builder:builder /workspace

# Drop privileges and run main build script
exec su - builder -c "/scripts/main.sh"
```

```bash
#!/bin/bash
# /scripts/main.sh - Main build script (runs as non-root user)

# 1. Setup environment
source /opt/agiros/$ROS_DISTRO/setup.bash

# 2. Install dependencies (requires sudo)
sudo agirosdep init || true  # May already be initialized
agirosdep update  # Runs as user
sudo agirosdep install --from-paths src --ignore-src -y

# 3. Build packages (as user) - colcon handles all dependency ordering
colcon build --merge-install --parallel-workers $PARALLEL_JOBS

# 4. Source the built workspace
source install/setup.bash

# 5. Create Debian packages (can be done in parallel)
for package in $(colcon list --names-only); do
    package_path="/workspace/src/$package"
    debian_dir="/workspace/debian_dirs/$package/debian"
    
    # Check if custom debian dir exists
    if [ ! -d "$debian_dir" ]; then
        echo "Generating debian dir for $package with bloom-generate"
        bloom-generate rosdebian --ros-distro $ROS_DISTRO "$package_path"
        
        # Copy generated debian dir to collection (as user)
        mkdir -p "/workspace/debian_dirs/$package"
        cp -r "$package_path/debian" "/workspace/debian_dirs/$package/"
    else
        # Use provided debian dir (as user)
        cp -r "$debian_dir" "$package_path/"
    fi
    
    # Build the debian package (may need sudo for dpkg-deb)
    /scripts/build-deb.sh "$package_path"
done

# 6. Generate repository (as user)
/scripts/create-repo.sh /workspace/output
```

## Cross-Architecture Support

### Decision: rust-script for Container-Side Operations
**Rationale:**
- **Cross-Architecture**: ARM64 containers on AMD64 hosts work seamlessly
- **Performance**: Near-native Rust performance with compilation caching
- **Type Safety**: Full Rust type system and error handling
- **Development**: Edit-and-run workflow with Rust tooling

**Problem:**
Pre-compiled Rust binaries built on AMD64 hosts won't run in ARM64 containers, limiting deployment flexibility.

**Solution:**
Use `rust-script` to compile and execute Rust code on-demand inside containers:

```rust
/// Detect architecture compatibility issues
pub async fn check_architecture_compatibility(image: &str) -> Result<ArchitectureInfo> {
    let host_arch = std::env::consts::ARCH;
    
    // Inspect image architecture
    let docker = Docker::connect_with_local_defaults()?;
    let image_info = docker.inspect_image(image).await?;
    let image_arch = image_info.architecture.as_deref().unwrap_or("unknown");
    
    Ok(ArchitectureInfo {
        host_arch: host_arch.to_string(),
        image_arch: image_arch.to_string(),
        compatible: host_arch == image_arch,
        requires_emulation: host_arch != image_arch,
    })
}
```

**Script Structure:**
```
scripts/helpers/
├── package-scanner.rs      # Rust script for package discovery
├── debian-preparer.rs      # Rust script for bloom integration  
├── build-orchestrator.rs   # Rust script for build coordination
├── progress-reporter.rs    # Rust script for structured logging
└── check-requirements.rs   # Rust script for dependency verification
```

**rust-script Example:**
```rust
#!/usr/bin/env rust-script
//! ```cargo
//! [dependencies]
//! serde = { version = "1.0", features = ["derive"] }
//! serde_json = "1.0"
//! tokio = { version = "1.0", features = ["full"] }
//! ```

use serde::{Serialize, Deserialize};

#[derive(Serialize, Deserialize)]
struct Package {
    name: String,
    version: String,
    // ... other fields
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Full Rust functionality with async support
    let packages = scan_packages().await?;
    println!("{}", serde_json::to_string_pretty(&packages)?);
    Ok(())
}
```

**Benefits:**
- Universal compatibility across architectures
- Near-native Rust performance (cached compilation)
- Full Rust type system and error handling
- Async/await support for I/O operations
- Single language ecosystem (no Python/Shell mixing)
- Familiar Rust development experience

**Trade-offs:**
- Compilation overhead on first run (mitigated by caching)
- Requires Rust toolchain in containers
- Larger container images (Rust + dependencies)

**Container Requirements:**
```dockerfile
# Install Rust and rust-script
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"
RUN cargo install rust-script
```

## Performance Optimizations

### Decision: Lazy Evaluation and Streaming
**Rationale:**
- **Memory**: Handle large workspaces
- **Responsiveness**: Quick startup
- **Efficiency**: Only compute what's needed

**Techniques:**
- Stream package discovery
- Parallel .deb package creation
- Efficient string handling with `Cow<str>`
- Let colcon handle all dependency complexity