# colcon-deb-build

Build orchestration crate for the Colcon Debian Packager. This crate manages the entire build process, coordinating Docker containers and tracking build progress.

## Features

- **Build Orchestration**: Manages the complete build lifecycle from environment preparation to artifact collection
- **Docker Integration**: Runs builds in isolated Docker containers for reproducibility
- **Progress Tracking**: Monitors build progress and provides real-time status updates
- **Artifact Collection**: Automatically collects and organizes generated .deb files
- **Error Handling**: Comprehensive error types for all build scenarios

## Architecture

The crate is organized into several key modules:

- **orchestrator**: Main build orchestration logic with the `BuildOrchestratorTrait` and `ColconDebBuilder`
- **executor**: Manages Docker container execution for builds
- **context**: Tracks build state, statistics, and results
- **artifact**: Handles collection and organization of build outputs
- **error**: Build-specific error types

## Usage

```rust
use colcon_deb_build::{BuildOrchestratorTrait, ColconDebBuilder};
use colcon_deb_config::{Config, DockerConfig};

// Create configuration
let config = Config {
    colcon_repo: PathBuf::from("./workspace"),
    debian_dirs: PathBuf::from("./debian"),
    docker: DockerConfig::Image {
        image: "ros:loong".to_string(),
    },
    ros_distro: Some("loong".to_string()),
    output_dir: PathBuf::from("./output"),
    parallel_jobs: 4,
};

// Create and run build
let mut builder = ColconDebBuilder::new(config).await?;
builder.prepare_environment().await?;
builder.run_build().await?;
let artifacts = builder.collect_artifacts().await?;
```

## Integration

This crate integrates with:
- `colcon-deb-config`: For build configuration
- `colcon-deb-docker`: For container management
- `colcon-deb-ros`: For ROS package handling
- `colcon-deb-debian`: For Debian package creation

## Build States

The build process transitions through several states:
1. `Idle`: Initial state
2. `Preparing`: Setting up the build environment
3. `Ready`: Environment prepared, ready to build
4. `Building`: Actively building packages
5. `CollectingArtifacts`: Gathering build outputs
6. `Completed`: Build finished successfully
7. `Failed`: Build encountered errors