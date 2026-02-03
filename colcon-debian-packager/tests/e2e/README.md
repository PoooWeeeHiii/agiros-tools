# End-to-End Tests for Colcon-Deb

This directory contains comprehensive end-to-end tests for the colcon-deb tool.

## Test Scripts

### 1. `run-e2e-tests.sh`
Main test runner that executes all end-to-end tests:
- CLI functionality
- Configuration management
- Workspace scanning
- Helper scripts
- Docker environment
- Build process
- Error handling
- Clean command

**Usage:**
```bash
./run-e2e-tests.sh
```

**Environment Variables:**
- `TEST_WORKSPACE`: Path to test workspace (default: project test_workspace)
- `OUTPUT_DIR`: Test output directory (default: ./output)
- `LOG_DIR`: Test logs directory (default: ./logs)
- `ROS_DISTRO`: ROS distribution to test (default: loong)
- `USE_DOCKER`: Whether to test Docker functionality (default: true)

### 2. `benchmark.sh`
Performance benchmarking script:
- Workspace scanning speed
- Configuration validation time
- Container startup overhead
- rust-script compilation time
- Memory usage profiling

**Usage:**
```bash
./benchmark.sh
ITERATIONS=5 ./benchmark.sh  # Run 5 iterations
```

### 3. `test-ros-packages.sh`
Tests with real ROS packages:
- Clones actual ROS packages (std_msgs, example_interfaces)
- Creates custom test packages
- Validates package discovery
- Tests error scenarios

**Usage:**
```bash
./test-ros-packages.sh
WORK_DIR=/tmp/ros_test ./test-ros-packages.sh
```

### 4. `test-apt-repo.sh`
APT repository generation testing:
- Creates dummy .deb packages
- Generates repository metadata
- Validates repository structure
- Tests APT compatibility

**Usage:**
```bash
./test-apt-repo.sh
TEST_DIR=/tmp/apt_test ./test-apt-repo.sh
```

## Test Results

All test scripts generate detailed reports:
- Console output with colored pass/fail indicators
- Log files in `logs/` directory
- Markdown reports for detailed results
- Performance metrics in CSV format

## Directory Structure

```
e2e/
├── run-e2e-tests.sh      # Main test runner
├── benchmark.sh          # Performance benchmarks
├── test-ros-packages.sh  # Real ROS package tests
├── test-apt-repo.sh      # APT repository tests
├── test-summary.md       # Overall test summary
├── README.md            # This file
├── .gitignore           # Git ignore patterns
│
├── output/              # Test outputs (gitignored)
├── logs/                # Test logs (gitignored)
├── benchmark_results/   # Benchmark data (gitignored)
├── apt_repo_test/       # APT test files (gitignored)
└── ros_test_workspace/  # ROS test workspace (gitignored)
```

## Running All Tests

To run the complete test suite:

```bash
# Run all e2e tests
./run-e2e-tests.sh

# Run benchmarks
./benchmark.sh

# Test with real ROS packages
./test-ros-packages.sh

# Test APT repository generation
./test-apt-repo.sh
```

## CI Integration

These tests can be integrated into CI pipelines:

```yaml
# Example GitHub Actions
- name: Run E2E Tests
  run: |
    cd tests/e2e
    ./run-e2e-tests.sh
    
- name: Run Benchmarks
  run: |
    cd tests/e2e
    ITERATIONS=3 ./benchmark.sh
```

## Requirements

- Rust toolchain
- Docker (for container tests)
- rust-script
- Basic debian packaging tools (dpkg-deb, dpkg-scanpackages)
- jq (for JSON processing)
- bc (for benchmark calculations)