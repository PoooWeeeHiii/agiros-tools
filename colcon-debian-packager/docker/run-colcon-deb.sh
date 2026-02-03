#!/bin/bash
# Wrapper script to run colcon-deb in Docker container

set -e

# Default values
ROS_DISTRO="${ROS_DISTRO:-loong}"
WORKSPACE="${WORKSPACE:-$(pwd)}"
OUTPUT_DIR="${OUTPUT_DIR:-$(pwd)/debian_output}"
HELPERS_DIR="${HELPERS_DIR:-$(dirname "$0")/../scripts/helpers}"
TARGET_ARCH="${TARGET_ARCH:-$(dpkg --print-architecture 2>/dev/null || echo amd64)}"
INTERACTIVE=true
IMAGE_TAG="colcon-deb:${ROS_DISTRO}"

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Usage function
usage() {
    cat <<EOF
Usage: $0 [OPTIONS] [COMMAND]

Run colcon-deb in a Docker container with proper environment setup.

Options:
    -d, --distro DISTRO      ROS distribution (default: loong)
    -w, --workspace PATH     Workspace path (default: current directory)
    -o, --output PATH        Output directory for .deb files (default: ./debian_output)
    -a, --arch ARCH          Target architecture (default: host architecture)
    -i, --image IMAGE        Custom Docker image (overrides distro)
    --no-interactive         Run in non-interactive mode
    -h, --help               Show this help message

Examples:
    # Run interactive shell in loong container
    $0

    # Build packages in iron container
    $0 --distro iron colcon-deb build

    # Validate configuration
    $0 colcon-deb validate

    # Run with custom workspace
    $0 --workspace /path/to/ws colcon-deb build -j 4

EOF
}

# Parse arguments
DOCKER_ARGS=()
COMMAND_ARGS=()
PARSING_COMMAND=false

while [[ $# -gt 0 ]]; do
    if [ "$PARSING_COMMAND" = true ]; then
        COMMAND_ARGS+=("$1")
        shift
        continue
    fi
    
    case $1 in
        -d|--distro)
            ROS_DISTRO="$2"
            IMAGE_TAG="colcon-deb:${ROS_DISTRO}"
            shift 2
            ;;
        -w|--workspace)
            WORKSPACE="$2"
            shift 2
            ;;
        -o|--output)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        -a|--arch)
            TARGET_ARCH="$2"
            shift 2
            ;;
        -i|--image)
            IMAGE_TAG="$2"
            shift 2
            ;;
        --no-interactive)
            INTERACTIVE=false
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            # Start parsing command arguments
            PARSING_COMMAND=true
            COMMAND_ARGS+=("$1")
            shift
            ;;
    esac
done

# Validate workspace
if [ ! -d "$WORKSPACE" ]; then
    echo -e "${RED}Error: Workspace directory does not exist: $WORKSPACE${NC}"
    exit 1
fi

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Check if image exists
if ! docker image inspect "$IMAGE_TAG" >/dev/null 2>&1; then
    echo -e "${YELLOW}Docker image not found: $IMAGE_TAG${NC}"
    echo -e "${YELLOW}Building image...${NC}"
    
    # Try to build the image
    DOCKER_DIR="$(dirname "$0")"
    if [ -f "$DOCKER_DIR/build-images.sh" ]; then
        "$DOCKER_DIR/build-images.sh" --distro "$ROS_DISTRO"
    else
        echo -e "${RED}Cannot find build script${NC}"
        exit 1
    fi
fi

# Build Docker run command
DOCKER_CMD=(
    "docker" "run" "--rm"
    "-e" "ROS_DISTRO=$ROS_DISTRO"
    "-e" "LOCAL_USER_ID=$(id -u)"
    "-e" "INJECT_HELPERS=true"
    "-e" "TARGET_ARCHITECTURE=$TARGET_ARCH"
    "-e" "DEBIAN_FRONTEND=noninteractive"
    "-v" "$WORKSPACE:/workspace:rw"
    "-v" "$HELPERS_DIR:/opt/colcon-deb/helpers:ro"
    "-v" "$OUTPUT_DIR:/output:rw"
    "-w" "/workspace"
)

# Add interactive flags if needed
if [ "$INTERACTIVE" = true ] && [ ${#COMMAND_ARGS[@]} -eq 0 ]; then
    DOCKER_CMD+=("-it")
fi

# Add the image
DOCKER_CMD+=("$IMAGE_TAG")

# Add command if provided
if [ ${#COMMAND_ARGS[@]} -gt 0 ]; then
    DOCKER_CMD+=("${COMMAND_ARGS[@]}")
fi

# Show what we're running
echo -e "${GREEN}Running colcon-deb in Docker...${NC}"
echo -e "${GREEN}  ROS Distro: $ROS_DISTRO${NC}"
echo -e "${GREEN}  Workspace: $WORKSPACE${NC}"
echo -e "${GREEN}  Output: $OUTPUT_DIR${NC}"
echo -e "${GREEN}  Architecture: $TARGET_ARCH${NC}"

# Execute the Docker command
exec "${DOCKER_CMD[@]}"