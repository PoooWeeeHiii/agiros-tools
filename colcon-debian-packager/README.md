# Colcon Debian Packager (Rust)

A Rust implementation of a Debian package builder for ROS Colcon repositories. This tool builds `.deb` packages for all packages in a ROS workspace, executing the build process in controlled Docker containers.

## Features

- 🚀 **Fast parallel builds** - Leverages Rust's async runtime for maximum performance
- 🐳 **Docker isolation** - Builds run in containers for reproducibility
- 🔧 **Cross-architecture support** - Build ARM64 packages on AMD64 hosts
- 📦 **APT repository generation** - Creates ready-to-use Debian repositories
- 🛡️ **Security-first design** - Non-root builds with minimal privileges
- 🔄 **Incremental builds** - Smart caching for faster rebuilds

## Architecture

The system uses a split architecture:

1. **Host Application (Rust)** - Orchestrates the build process
   - Reads YAML configuration
   - Manages Docker containers
   - Monitors build progress
   - Collects generated artifacts

2. **Container Environment** - Executes the actual builds
   - Uses rust-script for cross-architecture compatibility
   - Runs colcon build (handles all dependency ordering)
   - Creates .deb packages in parallel
   - Generates APT repository metadata

## Quick Start

### Prerequisites

- Rust 1.75 or later
- Docker
- cargo-nextest (for testing)

### Installation

```bash
# Clone the repository
git clone https://github.com/your-org/colcon-deb
cd colcon-deb

# Install development tools
make install-tools

# Build the project
make build-release

# Install the binary
make install
```

### Basic Usage

1. Create a configuration file:

```yaml
# colcon-deb.yaml
colcon_repo: /path/to/your/ros/workspace
debian_dirs: /path/to/debian/configs
docker:
  image: ros:loong-ros-base
output_dir: ./output
parallel_jobs: 4
```

2. Build packages:

```bash
colcon-deb build --config colcon-deb.yaml
```

## Configuration

The tool requires three main configuration items:

- `colcon_repo`: Path to your ROS workspace (must contain `src/` directory)
- `debian_dirs`: Directory containing custom Debian configurations per package
- `docker`: Either an existing image or path to a Dockerfile

### Example Configuration

```yaml
# Required fields
colcon_repo: ${HOME}/my_ros_ws
debian_dirs: ${HOME}/my_debian_dirs
docker:
  # Option 1: Use existing image
  image: ros:loong-ros-base
  # Option 2: Build from Dockerfile
  # dockerfile: ./docker/Dockerfile.loong

# Optional fields
ros_distro: loong  # Auto-detected if not specified
output_dir: ./output
parallel_jobs: 8
```

## Project Structure

```
colcon-deb/
├── crates/                 # Rust crates (workspace members)
│   ├── colcon-deb-cli/     # CLI application
│   ├── colcon-deb-core/    # Core types and traits
│   ├── colcon-deb-config/  # Configuration management
│   ├── colcon-deb-docker/  # Docker integration
│   ├── colcon-deb-ros/     # ROS package handling
│   ├── colcon-deb-debian/  # Debian packaging
│   └── colcon-deb-build/   # Build orchestration
├── scripts/
│   ├── entrypoint.sh       # Container entry script
│   ├── main.sh             # Main build script
│   └── helpers/            # rust-script helpers
│       ├── package-scanner.rs
│       ├── debian-preparer.rs
│       ├── build-orchestrator.rs
│       └── progress-reporter.rs
└── tests/
    └── test_workspace/     # Example ROS packages for testing
```

## Development

### Building

```bash
# Debug build
make build

# Release build
make build-release

# Run tests
make test

# Run linting
make lint

# Format code
make format
```

### Testing

The project uses `cargo-nextest` for testing:

```bash
# Run all tests
make test

# Run with coverage
make test-coverage

# Run specific test
cargo nextest run package_name
```

### Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Run `make check` to ensure code quality
5. Submit a pull request

## How It Works

1. **Configuration**: The host reads the YAML configuration and validates paths
2. **Container Setup**: Docker container is prepared with necessary tools
3. **Dependency Installation**: `agirosdep` installs system dependencies
4. **Build Phase**: `colcon build` compiles all packages (handles dependency order)
5. **Packaging Phase**: `.deb` files are created in parallel for all packages
6. **Repository Generation**: APT repository metadata is generated

The key insight is that `colcon build` already handles all dependency ordering, so after it completes, all packages can be converted to `.deb` files in parallel.

## License

This project is licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT license ([LICENSE-MIT](LICENSE-MIT))

at your option.

## Acknowledgments

- Original Python implementation: [colcon-deb-python](../colcon-deb-python)
- ROS community for the colcon build tool
- Rust async ecosystem (tokio, bollard, etc.)