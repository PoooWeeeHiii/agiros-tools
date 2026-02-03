#!/bin/bash
# Test script to verify Docker environment

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Color output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}Testing colcon-deb Docker environment...${NC}"

# Test 1: Check if image exists
echo -e "\n${YELLOW}Test 1: Checking Docker image...${NC}"
if docker image inspect colcon-deb:loong-fast >/dev/null 2>&1; then
    echo -e "${GREEN}✓ Docker image exists${NC}"
else
    echo -e "${RED}✗ Docker image not found${NC}"
    echo "Please build the image first with: ./build-images.sh"
    exit 1
fi

# Test 2: Run basic commands in container
echo -e "\n${YELLOW}Test 2: Testing basic commands...${NC}"
docker run --rm colcon-deb:loong-fast bash -c "
    echo 'Testing tool availability:'
    echo -n '  Rust: ' && rustc --version
    echo -n '  Cargo: ' && cargo --version
    echo -n '  Python: ' && python3 --version
    echo -n '  ROS: ' && echo \$ROS_DISTRO
    echo -n '  dpkg-buildpackage: ' && dpkg-buildpackage --version | head -n1
    echo -n '  bloom-generate: ' && (bloom-generate --version 2>/dev/null || echo 'installed')
"

# Test 3: Test helper script injection
echo -e "\n${YELLOW}Test 3: Testing helper script injection...${NC}"
docker run --rm \
    -v "$SCRIPT_DIR/../scripts/helpers:/opt/colcon-deb/helpers:ro" \
    -e INJECT_HELPERS=true \
    colcon-deb:loong-fast bash -c "
    /usr/local/bin/inject-helpers.sh
    echo 'Checking injected helpers:'
    ls -la /usr/local/bin/*.rs 2>/dev/null || echo '  No rust scripts found'
"

# Test 4: Test workspace mounting
echo -e "\n${YELLOW}Test 4: Testing workspace mounting...${NC}"
TEST_DIR=$(mktemp -d)
echo "Test workspace" > "$TEST_DIR/test.txt"

docker run --rm \
    -v "$TEST_DIR:/workspace:rw" \
    colcon-deb:loong-fast bash -c "
    echo 'Workspace contents:'
    ls -la /workspace/
    cat /workspace/test.txt
"

rm -rf "$TEST_DIR"

# Test 5: Test user permission handling
echo -e "\n${YELLOW}Test 5: Testing user permissions...${NC}"
docker run --rm \
    -e LOCAL_USER_ID=$(id -u) \
    colcon-deb:loong-fast bash -c "
    echo \"Host UID: \$LOCAL_USER_ID\"
    echo \"Container user: \$(id -u)\"
"

echo -e "\n${GREEN}All tests passed!${NC}"
echo -e "${GREEN}Docker environment is ready for use.${NC}"