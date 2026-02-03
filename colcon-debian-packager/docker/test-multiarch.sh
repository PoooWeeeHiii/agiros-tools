#!/bin/bash
# Test multi-architecture Docker builds

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Architectures to test
ARCHITECTURES=("amd64" "arm64")

# Test function
test_arch() {
    local arch=$1
    local platform=""
    
    case $arch in
        amd64)
            platform="linux/amd64"
            ;;
        arm64)
            platform="linux/arm64"
            ;;
        *)
            echo -e "${RED}Unknown architecture: $arch${NC}"
            return 1
            ;;
    esac
    
    echo -e "${GREEN}Testing $arch architecture...${NC}"
    
    # Build test image for architecture
    docker buildx build \
        --platform "$platform" \
        --build-arg BASE_IMAGE="ros:loong-ros-base" \
        --tag "colcon-deb:loong-$arch" \
        --load \
        --file "$SCRIPT_DIR/base/Dockerfile" \
        "$SCRIPT_DIR"
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ Successfully built for $arch${NC}"
        
        # Test the image
        echo -e "${YELLOW}Testing image functionality...${NC}"
        docker run --rm --platform "$platform" "colcon-deb:loong-$arch" bash -c "
            echo 'Architecture: \$(uname -m)'
            echo 'Debian arch: \$(dpkg --print-architecture)'
            rust-script --version
            bloom-generate --version 2>/dev/null || echo 'bloom-generate not available'
            dpkg-buildpackage --version | head -n1
        "
        
        if [ $? -eq 0 ]; then
            echo -e "${GREEN}✓ Image test passed for $arch${NC}"
        else
            echo -e "${RED}✗ Image test failed for $arch${NC}"
            return 1
        fi
    else
        echo -e "${RED}✗ Failed to build for $arch${NC}"
        return 1
    fi
}

# Check if Docker buildx is available
if ! docker buildx version >/dev/null 2>&1; then
    echo -e "${RED}Docker buildx is required for multi-architecture builds${NC}"
    echo "Please install Docker Desktop or enable buildx"
    exit 1
fi

# Create and use buildx builder
BUILDER_NAME="colcon-deb-multiarch"
if ! docker buildx inspect "$BUILDER_NAME" >/dev/null 2>&1; then
    echo -e "${YELLOW}Creating buildx builder: $BUILDER_NAME${NC}"
    docker buildx create --name "$BUILDER_NAME" --driver docker-container --bootstrap
fi

docker buildx use "$BUILDER_NAME"

# Run tests
echo -e "${GREEN}Starting multi-architecture tests...${NC}"

for arch in "${ARCHITECTURES[@]}"; do
    test_arch "$arch"
    echo
done

echo -e "${GREEN}Multi-architecture test complete!${NC}"

# Show results
echo -e "\n${GREEN}Built images:${NC}"
docker images | grep -E "REPOSITORY|colcon-deb.*loong"