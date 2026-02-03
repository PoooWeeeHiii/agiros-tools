#!/bin/bash
# Build colcon-deb Docker images for different ROS distributions

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_DIR="$SCRIPT_DIR/base"

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# ROS distributions to build
declare -A ROS_DISTROS=(
    ["loong"]="ubuntu:22.04"
    ["iron"]="ubuntu:22.04"
    ["rolling"]="ubuntu:22.04"
)

# Parse command line arguments
BUILD_DISTRO=""
PUSH_IMAGES=false
REGISTRY="ghcr.io/your-org"  # Change this to your registry

while [[ $# -gt 0 ]]; do
    case $1 in
        --distro)
            BUILD_DISTRO="$2"
            shift 2
            ;;
        --push)
            PUSH_IMAGES=true
            shift
            ;;
        --registry)
            REGISTRY="$2"
            shift 2
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo "Options:"
            echo "  --distro DISTRO   Build only specified ROS distro (loong, iron, rolling)"
            echo "  --push            Push images to registry after building"
            echo "  --registry REG    Container registry to use (default: ghcr.io/your-org)"
            echo "  --help            Show this help message"
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1
            ;;
    esac
done

# Build function
build_image() {
    local distro=$1
    local base_os=$2
    local tag="colcon-deb:${distro}"
    local full_tag="${REGISTRY}/${tag}"
    
    echo -e "${GREEN}Building image for ROS ${distro} (${base_os})...${NC}"
    
    # Build the image
    docker build \
        --build-arg BASE_IMAGE="ros:${distro}-ros-base" \
        --tag "$tag" \
        --tag "$full_tag" \
        --file "$BASE_DIR/Dockerfile" \
        "$SCRIPT_DIR"
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ Successfully built ${tag}${NC}"
        
        # Push if requested
        if [ "$PUSH_IMAGES" = true ]; then
            echo -e "${YELLOW}Pushing ${full_tag}...${NC}"
            docker push "$full_tag"
            if [ $? -eq 0 ]; then
                echo -e "${GREEN}✓ Successfully pushed ${full_tag}${NC}"
            else
                echo -e "${RED}✗ Failed to push ${full_tag}${NC}"
            fi
        fi
    else
        echo -e "${RED}✗ Failed to build ${tag}${NC}"
        return 1
    fi
}

# Main build process
echo -e "${GREEN}Starting colcon-deb Docker image builds...${NC}"

if [ -n "$BUILD_DISTRO" ]; then
    # Build specific distro
    if [ -z "${ROS_DISTROS[$BUILD_DISTRO]}" ]; then
        echo -e "${RED}Unknown ROS distro: $BUILD_DISTRO${NC}"
        exit 1
    fi
    build_image "$BUILD_DISTRO" "${ROS_DISTROS[$BUILD_DISTRO]}"
else
    # Build all distros
    for distro in "${!ROS_DISTROS[@]}"; do
        build_image "$distro" "${ROS_DISTROS[$distro]}"
    done
fi

echo -e "${GREEN}Build process complete!${NC}"

# Show built images
echo -e "\n${GREEN}Available colcon-deb images:${NC}"
docker images | grep -E "REPOSITORY|colcon-deb"