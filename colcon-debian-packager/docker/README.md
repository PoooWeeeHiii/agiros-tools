# Colcon-Deb Docker Environment

This directory contains Docker configurations for building ROS packages with colcon-deb in isolated containers.

## Quick Start

```bash
# Build Docker image for ROS loong
./build-images.sh --distro loong

# Run colcon-deb in container
./run-colcon-deb.sh colcon-deb build

# Or start interactive shell
./run-colcon-deb.sh
```

## Directory Structure

```
docker/
├── base/
│   └── Dockerfile          # Base image with ROS, Rust, and build tools
├── scripts/
│   ├── entrypoint.sh      # Container entrypoint script
│   ├── setup-environment.sh # Environment configuration
│   └── inject-helpers.sh  # Helper script injection
├── docker-compose.yml     # Docker Compose configuration
├── build-images.sh        # Build Docker images
├── run-colcon-deb.sh     # Run colcon-deb in container
└── test-multiarch.sh     # Test multi-architecture support
```

## Features

### 1. Pre-configured Build Environment
- ROS (loong, Iron, Rolling)
- Rust toolchain with rust-script
- Debian packaging tools (dpkg-dev, debhelper, lintian)
- bloom for ROS package generation
- colcon build system

### 2. Helper Script Injection
The container can automatically inject colcon-deb helper scripts:
- `scanner.rs` - Workspace scanning
- `debian-preparer.rs` - Debian directory preparation

Mount helpers with: `-v /path/to/helpers:/opt/colcon-deb/helpers:ro`

### 3. Multi-Architecture Support
Build for different architectures:
```bash
# AMD64 (default)
./run-colcon-deb.sh --arch amd64

# ARM64
./run-colcon-deb.sh --arch arm64
```

### 4. User Permission Handling
Automatically maps container user to host user ID to avoid permission issues:
```bash
# Container will use your host UID
./run-colcon-deb.sh
```

## Usage Examples

### Build Debian Packages
```bash
# Build all packages in current directory
./run-colcon-deb.sh colcon-deb build

# Build with specific ROS distro
./run-colcon-deb.sh --distro iron colcon-deb build

# Build with parallel jobs
./run-colcon-deb.sh colcon-deb build -j 8
```

### Validate Configuration
```bash
# Check colcon-deb.yaml configuration
./run-colcon-deb.sh colcon-deb validate
```

### Custom Workspace
```bash
# Use different workspace
./run-colcon-deb.sh --workspace /path/to/ros/ws colcon-deb build

# Specify output directory
./run-colcon-deb.sh --output ./my-debs colcon-deb build
```

### Interactive Development
```bash
# Start bash shell in container
./run-colcon-deb.sh

# Inside container:
rust-script --version
bloom-generate --help
colcon-deb --help
```

## Docker Compose

Use Docker Compose for persistent development:

```bash
# Start loong container
docker-compose up -d colcon-deb-loong

# Execute commands
docker-compose exec colcon-deb-loong colcon-deb build

# Stop container
docker-compose down
```

## Building Images

### Single Distribution
```bash
./build-images.sh --distro loong
```

### All Distributions
```bash
./build-images.sh
```

### Push to Registry
```bash
./build-images.sh --push --registry ghcr.io/your-org
```

## Multi-Architecture Testing

Test cross-platform builds:
```bash
# Requires Docker buildx
./test-multiarch.sh
```

## Environment Variables

- `ROS_DISTRO` - ROS distribution (loong, iron, rolling)
- `LOCAL_USER_ID` - Host user ID for permission mapping
- `INJECT_HELPERS` - Enable helper script injection (true/false)
- `TARGET_ARCHITECTURE` - Target build architecture
- `WORKSPACE` - Workspace directory to mount
- `OUTPUT_DIR` - Output directory for .deb files
- `HELPERS_DIR` - Directory containing helper scripts

## Troubleshooting

### Permission Issues
If you encounter permission errors:
```bash
# Ensure output directory is writable
chmod 755 ./debian_output

# Run with explicit UID
LOCAL_USER_ID=$(id -u) ./run-colcon-deb.sh
```

### Missing Tools
If tools are not found in container:
```bash
# Rebuild image
./build-images.sh --distro loong

# Check tool installation
docker run --rm colcon-deb:loong which rust-script
```

### Build Failures
For build issues:
```bash
# Run with verbose output
./run-colcon-deb.sh colcon-deb build -vv

# Check container logs
docker-compose logs colcon-deb-loong
```

## Customization

### Custom Base Image
Modify `docker/base/Dockerfile`:
```dockerfile
ARG BASE_IMAGE=your-custom-image:tag
FROM ${BASE_IMAGE}
```

### Additional Tools
Add tools to the Dockerfile:
```dockerfile
RUN apt-get update && apt-get install -y \
    your-additional-tools
```

### Different ROS Distributions
Add to `build-images.sh`:
```bash
declare -A ROS_DISTROS=(
    ["loong"]="ubuntu:22.04"
    ["your-distro"]="ubuntu:24.04"
)
```