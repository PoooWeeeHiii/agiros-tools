# Makefile for Colcon Debian Packager (Rust)

# Variables
CARGO := cargo
CARGO_FLAGS :=
NEXTEST := cargo nextest
NEXTEST_FLAGS := --no-fail-fast
RELEASE_FLAGS := --release
DOCKER := docker
RUSTFMT := rustfmt
CLIPPY_FLAGS := -- -D warnings

# Colors for output
RED := \033[0;31m
GREEN := \033[0;32m
YELLOW := \033[0;33m
BLUE := \033[0;34m
NC := \033[0m # No Color

# Default target
.DEFAULT_GOAL := help

# Phony targets
.PHONY: help build build-release test \
        test-coverage format format-check clippy clippy-fix audit deny doc doc-open \
        clean clean-all install install-dev install-tools run benchmark \
        docker-build docker-test lint release version

## Help
help:
	@echo "$(BLUE)Colcon Debian Packager - Rust$(NC)"
	@echo ""
	@echo "$(GREEN)Available targets:$(NC)"
	@echo "  $(YELLOW)Building:$(NC)"
	@echo "    build          - Build debug version"
	@echo "    build-release  - Build release version"
	@echo "    install        - Install release binary"
	@echo ""
	@echo "  $(YELLOW)Testing:$(NC)"
	@echo "    test           - Run all tests with nextest"
	@echo ""
	@echo "  $(YELLOW)Code Quality:$(NC)"
	@echo "    format         - Format code"
	@echo "    format-check      - Check code formatting"
	@echo "    clippy         - Run clippy linter"
	@echo "    clippy-fix     - Fix clippy warnings"
	@echo "    audit          - Check for security vulnerabilities"
	@echo "    deny           - Check licenses and dependencies"
	@echo ""
	@echo "  $(YELLOW)Documentation:$(NC)"
	@echo "    doc            - Build documentation"
	@echo "    doc-open       - Build and open documentation"
	@echo ""
	@echo "  $(YELLOW)Development:$(NC)"
	@echo "    install-dev    - Install development dependencies"
	@echo "    install-tools  - Install required tools"
	@echo "    run            - Run CLI with arguments (ARGS=...)"
	@echo "    benchmark      - Run benchmarks"
	@echo ""
	@echo "  $(YELLOW)CI/CD:$(NC)"
	@echo "    test           - Run CI test suite"
	@echo "    lint           - Run CI linting"
	@echo "    release        - Create release build"
	@echo ""
	@echo "  $(YELLOW)Maintenance:$(NC)"
	@echo "    clean          - Clean build artifacts"
	@echo "    clean-all      - Clean all generated files"
	@echo "    version        - Show version information"

## Building
build:
	@echo "$(GREEN)Building debug version...$(NC)"
	$(CARGO) build $(CARGO_FLAGS) --all-targets

build-release:
	@echo "$(GREEN)Building release version...$(NC)"
	$(CARGO) build $(RELEASE_FLAGS) $(CARGO_FLAGS) --all-targets

install: build-release
	@echo "$(GREEN)Installing release binary...$(NC)"
	$(CARGO) install --path crates/colcon-deb-cli

## Testing
test:
	@echo "$(GREEN)Running all tests...$(NC)"
	$(NEXTEST) run $(NEXTEST_FLAGS)
#	@echo "$(GREEN)Running end-to-end tests...$(NC)"
#	@test -d tests/e2e && cd tests/e2e && ./run-all.sh || echo "No E2E tests found"

test-coverage:
	@echo "$(GREEN)Generating test coverage...$(NC)"
	$(CARGO) tarpaulin --out Html --output-dir coverage

## Code Quality
lint: format-check clippy audit
	@echo "$(GREEN)CI linting complete$(NC)"

format:
	@echo "$(GREEN)Formatting code...$(NC)"
	$(CARGO) +nightly fmt --all

format-check:
	@echo "$(GREEN)Checking code formatting...$(NC)"
	$(CARGO) +nightly fmt --all -- --check

clippy:
	@echo "$(GREEN)Running clippy...$(NC)"
	$(CARGO) clippy --all-targets --all-features $(CLIPPY_FLAGS)

clippy-fix:
	@echo "$(GREEN)Fixing clippy warnings...$(NC)"
	$(CARGO) clippy --fix --all-targets --all-features --allow-dirty --allow-staged

audit:
	@echo "$(GREEN)Checking for security vulnerabilities...$(NC)"
	$(CARGO) audit

deny:
	@echo "$(GREEN)Checking licenses and dependencies...$(NC)"
	$(CARGO) deny check

## Documentation
doc:
	@echo "$(GREEN)Building documentation...$(NC)"
	$(CARGO) doc --no-deps --all-features

doc-open:
	@echo "$(GREEN)Building and opening documentation...$(NC)"
	$(CARGO) doc --no-deps --all-features --open

## Development
install-dev:
	@echo "$(GREEN)Installing development dependencies...$(NC)"
	rustup update stable
	rustup component add clippy
	rustup install nightly
	rustup component add rustfmt --toolchain nightly

install-tools:
	@echo "$(GREEN)Installing required tools...$(NC)"
	@command -v cargo-nextest >/dev/null 2>&1 || cargo install cargo-nextest
	@command -v cargo-tarpaulin >/dev/null 2>&1 || cargo install cargo-tarpaulin
	@command -v cargo-audit >/dev/null 2>&1 || cargo install cargo-audit
	@command -v cargo-deny >/dev/null 2>&1 || cargo install cargo-deny
	@command -v cargo-watch >/dev/null 2>&1 || cargo install cargo-watch
	@command -v cargo-release >/dev/null 2>&1 || cargo install cargo-release

run:
	@echo "$(GREEN)Running colcon-deb...$(NC)"
	$(CARGO) run --bin colcon-deb -- $(ARGS)

benchmark:
	@echo "$(GREEN)Running benchmarks...$(NC)"
	$(CARGO) bench

## Docker
docker-build:
	@echo "$(GREEN)Building Docker image...$(NC)"
	$(DOCKER) build -t colcon-deb-rust:latest .

docker-test:
	@echo "$(GREEN)Running tests in Docker...$(NC)"
	$(DOCKER) run --rm -v $(PWD):/workspace colcon-deb-rust:latest make test

## CI/CD
release: clean build-release
	@echo "$(GREEN)Creating release build...$(NC)"
	mkdir -p release
	cp target/release/colcon-deb release/
	cd release && tar -czf colcon-deb-$(shell git describe --tags --always).tar.gz colcon-deb
	@echo "$(GREEN)Release created in release/$(NC)"

## Maintenance
clean:
	@echo "$(GREEN)Cleaning build artifacts...$(NC)"
	$(CARGO) clean
	rm -rf coverage/
	rm -rf release/

clean-all: clean
	@echo "$(GREEN)Cleaning all generated files...$(NC)"
	rm -rf target/
	rm -rf Cargo.lock
	find . -name "*.profraw" -delete
	find . -name "*.profdata" -delete

version:
	@echo "$(GREEN)Version information:$(NC)"
	@echo "Rust version: $(shell rustc --version)"
	@echo "Cargo version: $(shell cargo --version)"
	@echo "Project version: $(shell grep '^version' crates/colcon-deb-cli/Cargo.toml | head -1 | cut -d'"' -f2)"

## Watch targets (for development)
watch:
	@echo "$(GREEN)Watching for changes...$(NC)"
	$(CARGO) watch -x build

watch-test:
	@echo "$(GREEN)Watching for changes and running tests...$(NC)"
	$(CARGO) watch -x 'nextest run --no-fail-fast'

watch-clippy:
	@echo "$(GREEN)Watching for changes and running clippy...$(NC)"
	$(CARGO) watch -x 'clippy --all-features -- -D warnings'

## Utility functions
check-nextest:
	@command -v cargo-nextest >/dev/null 2>&1 || (echo "$(RED)cargo-nextest not found. Run 'make install-tools' to install it.$(NC)" && exit 1)

# Include guard for nextest
test test-unit test-integration: check-nextest

# Print cargo version for debugging
debug-info:
	@echo "PATH: $$PATH"
	@echo "CARGO: $(shell which cargo)"
	@echo "RUSTC: $(shell which rustc)"
	@echo "Working directory: $(shell pwd)"
