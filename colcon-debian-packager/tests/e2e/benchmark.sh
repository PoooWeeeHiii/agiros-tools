#!/bin/bash
# Performance benchmarking script for colcon-deb

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Color output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Benchmark configuration
WORKSPACE="${WORKSPACE:-$PROJECT_ROOT/test_workspace}"
OUTPUT_DIR="${OUTPUT_DIR:-$SCRIPT_DIR/benchmark_results}"
ITERATIONS="${ITERATIONS:-3}"
PARALLEL_JOBS="${PARALLEL_JOBS:-4}"

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Helper to measure time
measure_time() {
    local name="$1"
    shift
    local start_time=$(date +%s.%N)
    
    echo -e "${BLUE}Measuring: $name${NC}"
    
    # Run command
    "$@"
    
    local end_time=$(date +%s.%N)
    local duration=$(echo "$end_time - $start_time" | bc)
    
    echo "$name,$duration" >> "$OUTPUT_DIR/timings.csv"
    echo -e "${GREEN}Completed in ${duration}s${NC}\n"
}

# Benchmark 1: Workspace scanning
benchmark_scanning() {
    echo -e "${YELLOW}Benchmark: Workspace Scanning${NC}"
    
    for i in $(seq 1 $ITERATIONS); do
        measure_time "workspace_scan_$i" \
            rust-script "$PROJECT_ROOT/scripts/helpers/scanner.rs" -- "$WORKSPACE/src"
    done
}

# Benchmark 2: Configuration validation
benchmark_config_validation() {
    echo -e "${YELLOW}Benchmark: Configuration Validation${NC}"
    
    # Create test config
    cat > "$OUTPUT_DIR/bench-config.yaml" <<EOF
colcon_repo: $WORKSPACE
debian_dirs: $OUTPUT_DIR/debian_dirs
docker:
  image: ros:loong-ros-base
output_dir: $OUTPUT_DIR/packages
parallel_jobs: $PARALLEL_JOBS
EOF
    
    for i in $(seq 1 $ITERATIONS); do
        measure_time "config_validation_$i" \
            colcon-deb validate -c "$OUTPUT_DIR/bench-config.yaml"
    done
}

# Benchmark 3: Container startup
benchmark_container_startup() {
    echo -e "${YELLOW}Benchmark: Container Startup${NC}"
    
    if ! docker info >/dev/null 2>&1; then
        echo "Docker not available, skipping container benchmarks"
        return
    fi
    
    for i in $(seq 1 $ITERATIONS); do
        measure_time "container_startup_$i" \
            docker run --rm colcon-deb:loong-fast echo "Hello"
    done
}

# Benchmark 4: rust-script compilation
benchmark_rust_script() {
    echo -e "${YELLOW}Benchmark: rust-script Compilation${NC}"
    
    # Create a simple rust script
    cat > "$OUTPUT_DIR/test.rs" <<'EOF'
#!/usr/bin/env rust-script
//! ```cargo
//! [dependencies]
//! serde_json = "1"
//! ```

fn main() {
    println!("Test script");
}
EOF
    
    chmod +x "$OUTPUT_DIR/test.rs"
    
    # Clear rust-script cache
    rm -rf ~/.cargo/script-cache/test_* 2>/dev/null || true
    
    for i in $(seq 1 $ITERATIONS); do
        # First run (cold cache)
        measure_time "rust_script_cold_$i" \
            rust-script "$OUTPUT_DIR/test.rs"
        
        # Second run (warm cache)
        measure_time "rust_script_warm_$i" \
            rust-script "$OUTPUT_DIR/test.rs"
        
        # Clear cache for next iteration
        rm -rf ~/.cargo/script-cache/test_* 2>/dev/null || true
    done
}

# Benchmark 5: Memory usage
benchmark_memory() {
    echo -e "${YELLOW}Benchmark: Memory Usage${NC}"
    
    if command -v /usr/bin/time >/dev/null 2>&1; then
        # Measure memory for workspace scanning
        /usr/bin/time -v rust-script "$PROJECT_ROOT/scripts/helpers/scanner.rs" -- "$WORKSPACE/src" \
            2>&1 | grep -E "(Maximum resident set size|User time|System time)" \
            > "$OUTPUT_DIR/memory_scan.txt"
        
        echo "Memory usage saved to $OUTPUT_DIR/memory_scan.txt"
    else
        echo "GNU time not available, skipping memory benchmarks"
    fi
}

# Generate report
generate_report() {
    echo -e "\n${YELLOW}Benchmark Report${NC}"
    echo "=================="
    
    if [ -f "$OUTPUT_DIR/timings.csv" ]; then
        echo -e "\n${BLUE}Average Times:${NC}"
        
        # Calculate averages
        awk -F',' '
        {
            sum[$1] += $2
            count[$1]++
        }
        END {
            for (test in sum) {
                avg = sum[test] / count[test]
                printf "%-30s %.3fs\n", test, avg
            }
        }' "$OUTPUT_DIR/timings.csv" | sort
    fi
    
    if [ -f "$OUTPUT_DIR/memory_scan.txt" ]; then
        echo -e "\n${BLUE}Memory Usage:${NC}"
        cat "$OUTPUT_DIR/memory_scan.txt"
    fi
    
    echo -e "\nResults saved to: $OUTPUT_DIR"
}

# Main
main() {
    echo -e "${GREEN}Colcon-Deb Performance Benchmarks${NC}"
    echo "=================================="
    echo "Workspace: $WORKSPACE"
    echo "Iterations: $ITERATIONS"
    echo
    
    # Initialize CSV
    echo "test,duration_seconds" > "$OUTPUT_DIR/timings.csv"
    
    # Run benchmarks
    benchmark_scanning
    benchmark_config_validation
    benchmark_container_startup
    benchmark_rust_script
    benchmark_memory
    
    # Generate report
    generate_report
}

if [ "${BASH_SOURCE[0]}" = "${0}" ]; then
    main "$@"
fi