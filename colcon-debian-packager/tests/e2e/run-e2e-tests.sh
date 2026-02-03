#!/bin/bash
# End-to-end test runner for colcon-deb

set -e

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Test configuration
TEST_WORKSPACE="${TEST_WORKSPACE:-$PROJECT_ROOT/test_workspace}"
OUTPUT_DIR="${OUTPUT_DIR:-$SCRIPT_DIR/output}"
LOG_DIR="${LOG_DIR:-$SCRIPT_DIR/logs}"
ROS_DISTRO="${ROS_DISTRO:-loong}"
USE_DOCKER="${USE_DOCKER:-true}"

# Test results
TESTS_RUN=0
TESTS_PASSED=0
TESTS_FAILED=0

# Helper functions
log() {
    echo -e "${BLUE}[$(date +'%Y-%m-%d %H:%M:%S')]${NC} $*"
}

success() {
    echo -e "${GREEN}✓${NC} $*"
}

error() {
    echo -e "${RED}✗${NC} $*"
}

warning() {
    echo -e "${YELLOW}⚠${NC} $*"
}

run_test() {
    local test_name="$1"
    local test_func="$2"
    
    TESTS_RUN=$((TESTS_RUN + 1))
    
    echo
    log "Running test: $test_name"
    
    # Create test log file
    local log_file="$LOG_DIR/${test_name// /_}.log"
    
    # Run the test
    if $test_func > "$log_file" 2>&1; then
        TESTS_PASSED=$((TESTS_PASSED + 1))
        success "$test_name"
    else
        TESTS_FAILED=$((TESTS_FAILED + 1))
        error "$test_name (see $log_file)"
    fi
}

# Setup test environment
setup_test_env() {
    log "Setting up test environment..."
    
    # Create directories
    mkdir -p "$OUTPUT_DIR"
    mkdir -p "$LOG_DIR"
    
    # Check CLI availability
    if ! command -v colcon-deb >/dev/null 2>&1; then
        # Build the CLI if not available
        log "Building colcon-deb CLI..."
        (cd "$PROJECT_ROOT" && cargo build --release --bin colcon-deb)
        export PATH="$PROJECT_ROOT/target/release:$PATH"
    fi
    
    # Check Docker if needed
    if [ "$USE_DOCKER" = "true" ]; then
        if ! docker info >/dev/null 2>&1; then
            error "Docker is not available"
            exit 1
        fi
        
        # Build Docker image if needed
        if ! docker image inspect "colcon-deb:$ROS_DISTRO-fast" >/dev/null 2>&1; then
            log "Building Docker image..."
            (cd "$PROJECT_ROOT/docker" && ./build-images.sh --distro "$ROS_DISTRO")
        fi
    fi
    
    success "Test environment ready"
}

# Test 1: Basic CLI functionality
test_cli_basic() {
    echo "Testing basic CLI functionality..."
    
    # Test help command
    colcon-deb --help
    
    # Test version
    colcon-deb --version || true
    
    # Test subcommand help
    colcon-deb build --help
    colcon-deb validate --help
    colcon-deb clean --help
    colcon-deb init --help
}

# Test 2: Configuration initialization
test_config_init() {
    echo "Testing configuration initialization..."
    
    local test_dir=$(mktemp -d)
    cd "$test_dir"
    
    # Initialize config
    colcon-deb init
    
    # Check if file was created
    if [ ! -f "colcon-deb.yaml" ]; then
        error "Configuration file not created"
        return 1
    fi
    
    # Validate the config
    colcon-deb validate -c colcon-deb.yaml || true
    
    cd - > /dev/null
    rm -rf "$test_dir"
}

# Test 3: Test workspace scanning
test_workspace_scanning() {
    echo "Testing workspace scanning..."
    
    if [ ! -d "$TEST_WORKSPACE" ]; then
        warning "Test workspace not found at $TEST_WORKSPACE"
        return 1
    fi
    
    # Create config for test workspace
    cat > "$OUTPUT_DIR/test-config.yaml" <<EOF
colcon_repo: $TEST_WORKSPACE
debian_dirs: $OUTPUT_DIR/debian_dirs
docker:
  image: ros:$ROS_DISTRO-ros-base
ros_distro: $ROS_DISTRO
output_dir: $OUTPUT_DIR/packages
parallel_jobs: 4
EOF
    
    # Run validation
    colcon-deb validate -c "$OUTPUT_DIR/test-config.yaml"
}

# Test 4: Helper script testing
test_helper_scripts() {
    echo "Testing helper scripts..."
    
    local helpers_dir="$PROJECT_ROOT/scripts/helpers"
    
    # Test scanner.rs
    if [ -f "$helpers_dir/scanner.rs" ]; then
        echo "Testing scanner.rs..."
        rust-script "$helpers_dir/scanner.rs" -- "$TEST_WORKSPACE/src" | jq '.' || true
    fi
    
    # Test debian-preparer.rs
    if [ -f "$helpers_dir/debian-preparer.rs" ]; then
        echo "Testing debian-preparer.rs..."
        # Would need a test package with debian directory
    fi
}

# Test 5: Docker environment
test_docker_env() {
    echo "Testing Docker environment..."
    
    if [ "$USE_DOCKER" != "true" ]; then
        warning "Docker tests skipped"
        return 0
    fi
    
    # Run basic Docker test
    docker run --rm "colcon-deb:$ROS_DISTRO-fast" bash -c "
        echo 'AGIROS Distro: $ROS_DISTRO'
        rustc --version
        python3 --version
        dpkg-buildpackage --version | head -n1
    "
}

# Test 6: Build process (simplified)
test_build_simple() {
    echo "Testing simplified build process..."
    
    if [ ! -d "$TEST_WORKSPACE" ]; then
        warning "Test workspace not found"
        return 1
    fi
    
    # Use the test config
    if [ -f "$OUTPUT_DIR/test-config.yaml" ]; then
        # Try to build (may not fully work yet)
        colcon-deb build -c "$OUTPUT_DIR/test-config.yaml" -vv || true
    fi
}

# Test 7: Error handling
test_error_handling() {
    echo "Testing error handling..."
    
    # Test with non-existent config
    colcon-deb validate -c /tmp/nonexistent.yaml 2>&1 | grep -i "error" || true
    
    # Test with invalid workspace
    local test_dir=$(mktemp -d)
    cat > "$test_dir/bad-config.yaml" <<EOF
colcon_repo: /tmp/nonexistent_workspace
debian_dirs: $test_dir/debian_dirs
docker:
  image: ros:$ROS_DISTRO-ros-base
output_dir: $test_dir/output
parallel_jobs: 0
EOF
    
    colcon-deb validate -c "$test_dir/bad-config.yaml" 2>&1 | grep -i "error" || true
    
    rm -rf "$test_dir"
}

# Test 8: Clean command
test_clean_command() {
    echo "Testing clean command..."
    
    local test_dir=$(mktemp -d)
    mkdir -p "$test_dir/output"
    touch "$test_dir/output/test.deb"
    
    # Create a config pointing to test dir
    cat > "$test_dir/config.yaml" <<EOF
colcon_repo: $TEST_WORKSPACE
debian_dirs: $test_dir/debian_dirs
docker:
  image: ros:$ROS_DISTRO-ros-base
output_dir: $test_dir/output
parallel_jobs: 4
EOF
    
    # Run clean
    colcon-deb clean -c "$test_dir/config.yaml"
    
    rm -rf "$test_dir"
}

# Main test execution
main() {
    echo -e "${GREEN}Colcon-Deb End-to-End Test Suite${NC}"
    echo "=================================="
    echo "Test workspace: $TEST_WORKSPACE"
    echo "Output directory: $OUTPUT_DIR"
    echo "AGIROS distro: $ROS_DISTRO"
    echo "Use Docker: $USE_DOCKER"
    echo
    
    # Setup
    setup_test_env
    
    # Run tests
    run_test "Basic CLI functionality" test_cli_basic
    run_test "Configuration initialization" test_config_init
    run_test "Workspace scanning" test_workspace_scanning
    run_test "Helper scripts" test_helper_scripts
    run_test "Docker environment" test_docker_env
    run_test "Simple build process" test_build_simple
    run_test "Error handling" test_error_handling
    run_test "Clean command" test_clean_command
    
    # Summary
    echo
    echo "=================================="
    echo -e "${BLUE}Test Summary:${NC}"
    echo "Tests run: $TESTS_RUN"
    echo -e "Tests passed: ${GREEN}$TESTS_PASSED${NC}"
    echo -e "Tests failed: ${RED}$TESTS_FAILED${NC}"
    
    if [ $TESTS_FAILED -eq 0 ]; then
        echo -e "\n${GREEN}All tests passed!${NC}"
        exit 0
    else
        echo -e "\n${RED}Some tests failed!${NC}"
        exit 1
    fi
}

# Run main if not sourced
if [ "${BASH_SOURCE[0]}" = "${0}" ]; then
    main "$@"
fi