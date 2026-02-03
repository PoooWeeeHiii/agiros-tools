# Development Guide - Colcon Debian Packager (Rust)

This guide covers the development environment setup, build processes, testing, and contribution guidelines for the Rust implementation of Colcon Debian Packager.

## Prerequisites

### Required Software

- **Rust**: 1.75.0 or later (stable)
  ```bash
  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
  rustup update stable
  ```

- **Docker**: 24.0 or later
  ```bash
  # Ubuntu/Debian
  sudo apt-get update
  sudo apt-get install docker.io docker-compose
  sudo usermod -aG docker $USER
  ```

- **Build Tools**:
  ```bash
  sudo apt-get install build-essential pkg-config libssl-dev
  ```

- **Debian Packaging Tools**:
  ```bash
  sudo apt-get install dpkg-dev debhelper lintian
  ```

### Recommended Tools

Install all recommended tools at once:
```bash
make install-tools
```

This will install:
- **cargo-nextest**: Next-generation test runner (required)
- **cargo-watch**: Auto-rebuild on file changes
- **cargo-tarpaulin**: Code coverage tool
- **cargo-deny**: License and security checks
- **cargo-audit**: Security vulnerability scanner
- **cargo-release**: Release automation

## Development Environment Setup

### 1. Clone Repository

```bash
git clone https://github.com/your-org/colcon-deb.git
cd colcon-deb
```

### 2. Install Development Dependencies

```bash
# Install Rust toolchains and components
make install-dev
```

This will:
- Update stable toolchain
- Add clippy for stable
- Install nightly toolchain
- Add rustfmt for nightly (required for formatting)

### 3. Setup Pre-commit Hooks

```bash
# Install pre-commit
pip install pre-commit

# Setup hooks
pre-commit install
```

### 4. Configure IDE

#### VS Code
```json
// .vscode/settings.json
{
    "rust-analyzer.cargo.features": "all",
    "rust-analyzer.checkOnSave.command": "clippy",
    "editor.formatOnSave": true,
    "[rust]": {
        "editor.defaultFormatter": "rust-lang.rust-analyzer"
    }
}
```

#### IntelliJ IDEA / CLion
- Install Rust plugin
- Enable `cargo fmt` on save
- Configure Clippy as external tool

### 5. Environment Variables

```bash
# Optional: Set custom Docker socket
export DOCKER_HOST=unix:///var/run/docker.sock

# Enable debug logging
export RUST_LOG=colcon_deb=debug

# Set test workspace location
export COLCON_DEB_TEST_WORKSPACE=/path/to/test/workspace
```

## Building the Project

### Quick Start with Make

```bash
# Show all available commands
make help

# Build debug version
make build

# Build release version
make build-release

# Install the binary
make install
```

### Development Build

```bash
# Using Makefile
make build

# Using cargo directly
cargo build --all-features

# Build specific crate
cargo build -p colcon-deb-cli
```

### Watch Mode

```bash
# Watch and rebuild
make watch

# Watch and test
make watch-test

# Watch with clippy
make watch-clippy
```

### Feature Flags

```toml
# Available features
default = ["docker", "parallel"]
docker = ["bollard"]         # Docker support
parallel = ["rayon"]         # Parallel processing
experimental = []            # Experimental features
```

## Testing

### Running Tests with Nextest

The project uses `cargo nextest` as the primary test runner for better output and performance.

```bash
# Run all tests (recommended)
make test

# Run all tests with no-fail-fast to see all failures
make test-all

# Run unit tests only
make test-unit

# Run integration tests
make test-integration

# Run end-to-end tests
make test-e2e
```

### Using Nextest Directly

```bash
# Run all tests
cargo nextest run --no-fail-fast

# Run tests for specific crate
cargo nextest run -p colcon-deb-core --no-fail-fast

# Run specific test
cargo nextest run test_package_parsing

# Run with specific profile
cargo nextest run --profile ci
```

### Test Coverage

```bash
# Generate HTML coverage report
make test-coverage

# View coverage report
open coverage/tarpaulin-report.html
```

### Writing Tests with Eyre

For tests, use `eyre` for better error reporting:

```rust
use eyre::Result;

#[test]
fn test_with_eyre() -> Result<()> {
    color_eyre::install()?;
    
    let result = some_operation()
        .wrap_err("Failed to perform operation")?;
    
    assert_eq!(result, expected);
    Ok(())
}
```

## Code Quality

### Formatting

```bash
# Format all code (uses nightly rustfmt)
make format

# Check formatting (CI)
make format-check
```

**Note**: This project uses nightly rustfmt for code formatting to access advanced formatting features. The nightly toolchain is automatically installed by `make install-dev`.

### Linting

```bash
# Run clippy
make clippy

# Auto-fix clippy warnings
make clippy-fix
```

### Security and License Checks

```bash
# Check for vulnerabilities
make audit

# Check licenses and dependencies
make deny

# Run all CI checks
make lint
```

## Debugging

### Debug Builds

```bash
# Build with debug symbols
make build

# Run with debug logging
RUST_LOG=debug make run ARGS="build /workspace"
```

### Using GDB/LLDB

```bash
# Debug with GDB
rust-gdb target/debug/colcon-deb

# Debug with LLDB
rust-lldb target/debug/colcon-deb
```

### Error Handling with Eyre

In binaries and examples, use `eyre` for rich error reports:

```rust
use eyre::{Result, WrapErr};

fn main() -> Result<()> {
    // Install error hooks for better reports
    color_eyre::install()?;
    
    // Use wrap_err for context
    let config = load_config()
        .wrap_err("Failed to load configuration")?;
    
    Ok(())
}
```

### Logging

```rust
// Use tracing macros
use tracing::{debug, info, warn, error};

debug!("Package discovered: {}", package.name);
info!("Starting build for {} packages", count);
warn!("Cache miss for package: {}", name);
error!("Build failed: {}", err);
```

### Environment Variables for Debugging

```bash
# Enable all debug logs
export RUST_LOG=trace

# Enable specific module
export RUST_LOG=colcon_deb_docker=debug

# Enable backtraces
export RUST_BACKTRACE=1

# Enable full backtraces
export RUST_BACKTRACE=full
```

## Benchmarking

### Micro-benchmarks

```bash
# Run benchmarks
cargo bench

# Run specific benchmark
cargo bench package_parsing

# Save baseline
cargo bench -- --save-baseline main
```

### Performance Profiling

```bash
# Install flamegraph
cargo install flamegraph

# Generate flamegraph
cargo flamegraph --bin colcon-deb -- build /workspace
```

## Documentation

### Building Docs

```bash
# Build documentation
make doc

# Build and open in browser
make doc-open

# Using cargo directly
cargo doc --no-deps --document-private-items --all-features
```

### Writing Documentation

```rust
/// Brief description of the function.
///
/// # Arguments
///
/// * `path` - Path to the workspace
/// * `config` - Build configuration
///
/// # Returns
///
/// Returns `Ok(packages)` on success
///
/// # Errors
///
/// Returns `Err` if:
/// - Workspace doesn't exist
/// - No packages found
///
/// # Examples
///
/// ```
/// use colcon_deb_core::discover_packages;
///
/// let packages = discover_packages("/workspace")?;
/// ```
pub fn discover_packages(path: &Path) -> Result<Vec<Package>> {
    // Implementation
}
```

## Release Process

### Version Bumping

```bash
# Install cargo-release
cargo install cargo-release

# Bump version (dry run)
cargo release patch --dry-run

# Bump version
cargo release patch --execute
```

### Creating Release

1. Update CHANGELOG.md
2. Bump version numbers
3. Create git tag
4. Push to GitHub
5. GitHub Actions will build releases

### Publishing to crates.io

```bash
# Dry run
cargo publish --dry-run

# Publish
cargo publish
```

## CI/CD

### GitHub Actions Workflows

- **ci.yml**: Runs on every push/PR
  - Format check
  - Clippy linting
  - Unit tests with nextest
  - Integration tests
  
- **release.yml**: Runs on tags
  - Build binaries
  - Create GitHub release
  - Upload artifacts

### Local CI Testing

```bash
# Run CI test suite locally
make test

# Run CI linting locally
make lint

# Install act for GitHub Actions testing
sudo snap install act

# Run CI locally with act
act -j test
```

## Common Development Tasks

### Adding a New Crate

```bash
# Create new crate
cargo new --lib crates/colcon-deb-newfeature

# Add to workspace
echo 'members = ["crates/colcon-deb-newfeature"]' >> Cargo.toml

# Add dependencies
cd crates/colcon-deb-newfeature
cargo add tokio serde
```

### Adding a Dependency

```bash
# Add to specific crate
cd crates/colcon-deb-core
cargo add futures

# Add with features
cargo add tokio --features full

# Add dev dependency
cargo add --dev mockall
```

### Updating Dependencies

```bash
# Update all dependencies
cargo update

# Update specific dependency
cargo update tokio

# Check outdated
cargo install cargo-outdated
cargo outdated
```

## Troubleshooting

### Common Issues

1. **Docker permission denied**
   ```bash
   sudo usermod -aG docker $USER
   newgrp docker
   ```

2. **Rust toolchain issues**
   ```bash
   rustup update
   rustup default stable
   ```

3. **Out of memory during build**
   ```bash
   # Limit parallel jobs
   cargo build -j 2
   ```

4. **Test failures with Docker**
   ```bash
   # Ensure Docker is running
   sudo systemctl start docker
   
   # Clean Docker state
   docker system prune -a
   ```

5. **Nextest not found**
   ```bash
   # Install nextest
   make install-tools
   # Or directly:
   cargo install cargo-nextest
   ```

6. **Make command not working**
   ```bash
   # Check if in correct directory
   pwd
   # Verify Makefile exists
   ls -la Makefile
   ```

## Contributing Guidelines

### Code Style

- Follow Rust naming conventions
- Use `rustfmt` for formatting
- Address all `clippy` warnings
- Write descriptive commit messages

### Pull Request Process

1. Fork the repository
2. Create feature branch (`git checkout -b feature/amazing-feature`)
3. Write tests for new functionality
4. Ensure all tests pass
5. Update documentation
6. Submit PR with clear description

### Review Checklist

- [ ] Tests pass locally
- [ ] Code is formatted (`cargo fmt`)
- [ ] No clippy warnings (`cargo clippy`)
- [ ] Documentation updated
- [ ] CHANGELOG.md updated
- [ ] No security issues (`cargo audit`)