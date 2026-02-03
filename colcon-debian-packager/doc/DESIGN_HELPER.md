# Design Details - Container Helper Scripts (rust-script)

This document details the design of the helper scripts that run inside the container using rust-script for cross-architecture compatibility.

## Overview

We use `rust-script` to run Rust code directly inside containers without pre-compilation. This provides the performance and type safety of Rust while ensuring cross-architecture compatibility (ARM64 containers on AMD64 hosts).

## rust-script Approach

**Solution**: Use `rust-script` to compile and execute Rust code on-demand inside containers.

**Benefits**:
- Rust performance and type safety
- Cross-architecture compatibility  
- No pre-compilation required
- Consistent tooling across host and container

## Architecture

### Script Structure

```
scripts/helpers/
├── package-scanner.rs      # Rust script for package discovery
├── debian-preparer.rs      # Rust script for debian directory management
├── build-orchestrator.rs   # Rust script for build coordination
├── progress-reporter.rs    # Rust script for structured logging
└── common.rs              # Shared Rust utilities
```

### Dependencies

All scripts rely on tools available in ROS containers with Rust support:
- **rust-script**: For on-demand Rust compilation
- **Rust toolchain**: Installed in container or via rustup
- **System Tools**: Standard build tools (bloom, dpkg, etc.)

## Key Scripts

### 1. Package Scanner (rust-script)

**Purpose**: Scan workspace for ROS packages and output structured JSON.

```rust
#!/usr/bin/env rust-script
//! ```cargo
//! [dependencies]
//! serde = { version = "1.0", features = ["derive"] }
//! serde_json = "1.0"
//! quick-xml = { version = "0.31", features = ["serialize"] }
//! walkdir = "2.0"
//! clap = { version = "4.0", features = ["derive"] }
//! ```

// scripts/helpers/package-scanner.rs

use clap::Parser;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};
use walkdir::WalkDir;

#[derive(Parser)]
#[command(name = "package-scanner")]
#[command(about = "Scan ROS workspace for packages")]
struct Args {
    /// Workspace path to scan
    workspace: PathBuf,
}

#[derive(Debug, Serialize, Deserialize)]
struct Package {
    name: String,
    version: String,
    description: String,
    path: String,
    dependencies: Dependencies,
    build_type: String,
}

#[derive(Debug, Serialize, Deserialize)]
struct Dependencies {
    build: Vec<String>,
    build_export: Vec<String>,
    exec: Vec<String>,
    test: Vec<String>,
    build_tool: Vec<String>,
    doc: Vec<String>,
}

#[derive(Debug, Serialize, Deserialize)]
struct ScanResult {
    packages: Vec<Package>,
    count: usize,
    workspace: String,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    
    let packages = scan_workspace(&args.workspace)?;
    
    let result = ScanResult {
        count: packages.len(),
        workspace: args.workspace.to_string_lossy().to_string(),
        packages,
    };
    
    println!("{}", serde_json::to_string_pretty(&result)?);
    Ok(())
}

fn scan_workspace(workspace: &Path) -> Result<Vec<Package>, Box<dyn std::error::Error>> {
    let mut packages = Vec::new();
    
    for entry in WalkDir::new(workspace) {
        let entry = entry?;
        let path = entry.path();
        
        // Skip if COLCON_IGNORE exists
        if path.join("COLCON_IGNORE").exists() {
            continue;
        }
        
        // Check for package.xml
        let package_xml = path.join("package.xml");
        if package_xml.exists() {
            match parse_package_xml(&package_xml, workspace) {
                Ok(package) => packages.push(package),
                Err(e) => {
                    eprintln!("Warning: Failed to parse {:?}: {}", package_xml, e);
                }
            }
        }
    }
    
    Ok(packages)
}

fn parse_package_xml(package_xml: &Path, workspace: &Path) -> Result<Package, Box<dyn std::error::Error>> {
    let content = fs::read_to_string(package_xml)?;
    let doc = quick_xml::Reader::from_str(&content);
    
    // Parse XML to extract package information
    let mut package = Package {
        name: String::new(),
        version: String::new(),
        description: String::new(),
        path: package_xml.parent().unwrap()
            .strip_prefix(workspace)?
            .to_string_lossy()
            .to_string(),
        dependencies: Dependencies {
            build: Vec::new(),
            build_export: Vec::new(),
            exec: Vec::new(),
            test: Vec::new(),
            build_tool: Vec::new(),
            doc: Vec::new(),
        },
        build_type: String::new(),
    };
    
    // Simple XML parsing for demonstration
    // In practice, you'd use a proper XML parser
    for line in content.lines() {
        if line.trim().starts_with("<name>") {
            package.name = extract_xml_text(line);
        } else if line.trim().starts_with("<version>") {
            package.version = extract_xml_text(line);
        } else if line.trim().starts_with("<description>") {
            package.description = extract_xml_text(line);
        }
        // Extract dependencies based on tags
        // This is simplified - real implementation would use proper XML parsing
    }
    
    package.build_type = detect_build_type(&content);
    
    Ok(package)
}

fn extract_xml_text(line: &str) -> String {
    line.split('>').nth(1)
        .and_then(|s| s.split('<').next())
        .unwrap_or("")
        .to_string()
}

fn detect_build_type(content: &str) -> String {
    if content.contains("<buildtool_depend>ament_cmake</buildtool_depend>") {
        "ament_cmake".to_string()
    } else if content.contains("<buildtool_depend>ament_python</buildtool_depend>") {
        "ament_python".to_string()
    } else if content.contains("cmake") {
        "cmake".to_string()
    } else {
        "unknown".to_string()
    }
}
```

### 2. Progress Reporter (rust-script)

**Purpose**: Provide structured progress reporting functions.

```rust
#!/usr/bin/env rust-script
//! ```cargo
//! [dependencies]
//! clap = { version = "4.0", features = ["derive"] }
//! ```

// scripts/helpers/progress-reporter.rs

use clap::{Parser, Subcommand};
use std::io::{self, Write};

#[derive(Parser)]
#[command(name = "progress-reporter")]
#[command(about = "Report structured progress events")]
struct Args {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Report package start
    PackageStart {
        /// Package name
        name: String,
    },
    /// Report package completion
    PackageComplete {
        /// Package name
        name: String,
        /// Success status
        #[arg(long)]
        success: bool,
    },
    /// Report stage change
    Stage {
        /// Stage name
        value: String,
    },
    /// Report general progress
    Progress {
        /// Current step
        #[arg(long)]
        current: usize,
        /// Total steps
        #[arg(long)]
        total: usize,
        /// Progress message
        #[arg(long)]
        message: String,
    },
    /// Report log message
    Log {
        /// Log level
        #[arg(long)]
        level: String,
        /// Log message
        #[arg(long)]
        message: String,
    },
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    
    match args.command {
        Commands::PackageStart { name } => {
            report_progress("package_start", &format!("name={}", name))?;
            report_log("info", &format!("Starting package: {}", name))?;
        }
        Commands::PackageComplete { name, success } => {
            report_progress("package_complete", &format!("name={},success={}", name, success))?;
            let level = if success { "info" } else { "error" };
            let action = if success { "Completed" } else { "Failed" };
            report_log(level, &format!("{} package: {}", action, name))?;
        }
        Commands::Stage { value } => {
            report_progress("stage", &format!("value={}", value))?;
            report_log("info", &format!("Stage: {}", value))?;
        }
        Commands::Progress { current, total, message } => {
            report_progress("general", &format!("current={},total={},message={}", current, total, message))?;
        }
        Commands::Log { level, message } => {
            report_log(&level, &message)?;
        }
    }
    
    Ok(())
}

fn report_progress(event_type: &str, params: &str) -> io::Result<()> {
    writeln!(io::stderr(), "::progress::type={},{}", event_type, params)
}

fn report_log(level: &str, message: &str) -> io::Result<()> {
    writeln!(io::stderr(), "::log::level={},msg={}", level, message)
}
```

### 3. Debian Preparer (rust-script)

**Purpose**: Manage debian directories using bloom-generate or existing configs. This runs AFTER colcon build when all dependencies are already built.

```rust
#!/usr/bin/env rust-script
//! ```cargo
//! [dependencies]
//! clap = { version = "4.0", features = ["derive"] }
//! tokio = { version = "1.0", features = ["process", "fs"] }
//! ```

// scripts/helpers/debian-preparer.rs

use clap::Parser;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use tokio::fs;

#[derive(Parser)]
#[command(name = "debian-preparer")]
#[command(about = "Prepare debian directories for ROS packages")]
struct Args {
    /// Package name
    package_name: String,
    /// Package source path
    package_path: PathBuf,
    /// Debian directories collection path
    debian_dirs: PathBuf,
    /// ROS distribution
    ros_distro: String,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    
    prepare_debian_dir(
        &args.package_name,
        &args.package_path,
        &args.debian_dirs,
        &args.ros_distro,
    ).await?;
    
    Ok(())
}

async fn prepare_debian_dir(
    package_name: &str,
    package_path: &Path,
    debian_dirs: &Path,
    ros_distro: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    let debian_dir = debian_dirs.join(package_name).join("debian");
    let target_debian = package_path.join("debian");
    
    report_log("debug", &format!("Preparing debian dir for {}", package_name)).await?;
    
    if debian_dir.exists() {
        report_log("info", &format!("Using existing debian directory for {}", package_name)).await?;
        
        // Copy existing debian directory
        copy_dir_all(&debian_dir, &target_debian).await.map_err(|e| {
            format!("Failed to copy debian directory for {}: {}", package_name, e)
        })?;
    } else {
        report_log("info", &format!("Generating debian directory for {} with bloom-generate", package_name)).await?;
        
        // Generate with bloom
        let output = Command::new("bloom-generate")
            .args(&[
                "rosdebian",
                "--ros-distro", ros_distro,
                "--debian-inc", "0",
                "--os-name", "ubuntu",
                "--os-version", "jammy",
                "."
            ])
            .current_dir(package_path)
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .output()?;
            
        if !output.status.success() {
            let stderr = String::from_utf8_lossy(&output.stderr);
            report_log("error", &format!("bloom-generate failed for {}: {}", package_name, stderr)).await?;
            return Err(format!("bloom-generate failed for {}", package_name).into());
        }
        
        // Save generated debian dir to collection
        if target_debian.exists() {
            let collection_dir = debian_dirs.join(package_name);
            fs::create_dir_all(&collection_dir).await?;
            
            match copy_dir_all(&target_debian, &collection_dir.join("debian")).await {
                Ok(_) => {
                    report_log("info", &format!("Saved generated debian directory for {}", package_name)).await?;
                }
                Err(e) => {
                    report_log("warning", &format!("Failed to save generated debian dir for {}: {}", package_name, e)).await?;
                    // Continue anyway, we have the debian dir in place
                }
            }
        } else {
            report_log("error", &format!("bloom-generate did not create debian directory for {}", package_name)).await?;
            return Err(format!("bloom-generate did not create debian directory for {}", package_name).into());
        }
    }
    
    Ok(())
}

async fn copy_dir_all(src: &Path, dst: &Path) -> Result<(), Box<dyn std::error::Error>> {
    fs::create_dir_all(dst).await?;
    
    let mut entries = fs::read_dir(src).await?;
    while let Some(entry) = entries.next_entry().await? {
        let ty = entry.file_type().await?;
        if ty.is_dir() {
            copy_dir_all(&entry.path(), &dst.join(entry.file_name())).await?;
        } else {
            fs::copy(entry.path(), dst.join(entry.file_name())).await?;
        }
    }
    
    Ok(())
}

async fn report_log(level: &str, message: &str) -> Result<(), Box<dyn std::error::Error>> {
    let output = Command::new("/helpers/progress-reporter.rs")
        .args(&["log", "--level", level, "--message", message])
        .output()?;
        
    if !output.status.success() {
        eprintln!("Failed to report log: {}", String::from_utf8_lossy(&output.stderr));
    }
    
    Ok(())
}
```

### 4. Build Orchestrator (rust-script)

**Purpose**: Orchestrate the entire build process with progress reporting. The key insight is that colcon build handles all dependency ordering, so we can create .deb packages in parallel after that.

```rust
#!/usr/bin/env rust-script
//! ```cargo
//! [dependencies]
//! clap = { version = "4.0", features = ["derive"] }
//! tokio = { version = "1.0", features = ["process", "fs"] }
//! serde = { version = "1.0", features = ["derive"] }
//! serde_json = "1.0"
//! ```

// scripts/helpers/build-orchestrator.rs

use clap::Parser;
use serde::{Deserialize, Serialize};
use std::env;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use tokio::fs;
use tokio::io::{AsyncBufReadExt, BufReader};
use tokio::process::Command as TokioCommand;

#[derive(Parser)]
#[command(name = "build-orchestrator")]
#[command(about = "Orchestrate the build process for all packages")]
struct Args {
    /// Workspace path
    #[arg(long, default_value = "/workspace")]
    workspace: PathBuf,
    
    /// Debian directories path
    #[arg(long, default_value = "/workspace/debian_dirs")]
    debian_dirs: PathBuf,
    
    /// Output directory
    #[arg(long, default_value = "/workspace/output")]
    output_dir: PathBuf,
}

#[derive(Debug, Serialize, Deserialize)]
struct ScanResult {
    packages: Vec<Package>,
    count: usize,
    workspace: String,
}

#[derive(Debug, Serialize, Deserialize)]
struct Package {
    name: String,
    version: String,
    description: String,
    path: String,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    
    let ros_distro = env::var("ROS_DISTRO").unwrap_or_else(|_| "loong".to_string());
    let parallel_jobs = env::var("PARALLEL_JOBS").unwrap_or_else(|_| "4".to_string());
    
    build_all_packages(&args.workspace, &args.debian_dirs, &args.output_dir, &ros_distro).await?;
    
    Ok(())
}

async fn build_all_packages(
    workspace: &Path,
    debian_dirs: &Path,
    output_dir: &Path,
    ros_distro: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    // Stage: Scanning
    report_stage("scanning").await?;
    
    report_log("info", "Scanning workspace for ROS packages").await?;
    let scan_result = scan_packages(&workspace.join("src")).await?;
    
    let total_packages = scan_result.packages.len();
    report_log("info", &format!("Found {} packages to build", total_packages)).await?;
    
    // Stage: Building with colcon (handles all dependencies)
    report_stage("colcon_build").await?;
    
    report_log("info", "Building all packages with colcon").await?;
    run_colcon_build(workspace).await?;
    
    // Source the install/setup.bash
    report_log("info", "Sourcing install/setup.bash").await?;
    // Note: In practice, this would be handled by the shell script
    
    // Stage: Preparing Debian directories
    report_stage("preparing").await?;
    
    for (current, package) in scan_result.packages.iter().enumerate() {
        let current = current + 1;
        report_progress(current, total_packages, &format!("Preparing {}", package.name)).await?;
        
        let package_path = workspace.join("src").join(&package.name);
        prepare_debian_dir(&package.name, &package_path, debian_dirs, ros_distro).await?;
    }
    
    // Stage: Creating .deb packages (can be done in parallel)
    report_stage("packaging").await?;
    
    // Build all .deb packages in parallel (no dependency ordering needed)
    let mut handles = vec![];
    
    for package in scan_result.packages {
        let package_name = package.name.clone();
        let package_path = workspace.join("src").join(&package.name);
        let output_dir = output_dir.to_path_buf();
        
        let handle = tokio::spawn(async move {
            report_package_start(&package_name).await?;
            
            match build_single_package(&package_name, &package_path, &output_dir).await {
                Ok(_) => {
                    report_package_complete(&package_name, true).await?;
                    Ok(())
                }
                Err(e) => {
                    report_package_complete(&package_name, false).await?;
                    report_log("error", &format!("Build failed for {}: {}", package_name, e)).await?;
                    Err(e)
                }
            }
        });
        
        handles.push(handle);
    }
    
    // Wait for all packages to complete
    for handle in handles {
        handle.await??;
    }
    
    // Stage: Repository
    report_stage("repository").await?;
    generate_repository(output_dir).await?;
    
    // Stage: Complete
    report_stage("complete").await?;
    report_log("info", "Build completed successfully").await?;
    
    Ok(())
}

async fn scan_packages(src_path: &Path) -> Result<ScanResult, Box<dyn std::error::Error>> {
    let output = Command::new("/helpers/package-scanner.rs")
        .arg(src_path)
        .output()?;
        
    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        return Err(format!("Failed to scan packages: {}", stderr).into());
    }
    
    let stdout = String::from_utf8(output.stdout)?;
    let result: ScanResult = serde_json::from_str(&stdout)?;
    
    Ok(result)
}

async fn run_colcon_build(workspace: &Path) -> Result<(), Box<dyn std::error::Error>> {
    let parallel_jobs = env::var("PARALLEL_JOBS").unwrap_or_else(|_| "4".to_string());
    
    let mut child = TokioCommand::new("colcon")
        .args(&[
            "build",
            "--merge-install",
            "--parallel-workers", &parallel_jobs,
        ])
        .current_dir(workspace)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()?;
        
    // Stream output
    if let Some(stdout) = child.stdout.take() {
        let reader = BufReader::new(stdout);
        let mut lines = reader.lines();
        
        while let Some(line) = lines.next_line().await? {
            report_log("debug", &format!("colcon: {}", line)).await?;
        }
    }
    
    let status = child.wait().await?;
    
    if !status.success() {
        return Err("colcon build failed".into());
    }
    
    Ok(())
}

async fn prepare_debian_dir(
    package_name: &str,
    package_path: &Path,
    debian_dirs: &Path,
    ros_distro: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    let output = Command::new("/helpers/debian-preparer.rs")
        .args(&[
            package_name,
            &package_path.to_string_lossy(),
            &debian_dirs.to_string_lossy(),
            ros_distro,
        ])
        .output()?;
        
    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        return Err(format!("Failed to prepare debian dir for {}: {}", package_name, stderr).into());
    }
    
    Ok(())
}

async fn build_single_package(
    package_name: &str,
    package_path: &Path,
    output_dir: &Path,
) -> Result<(), Box<dyn std::error::Error>> {
    report_log("debug", &format!("Building package {} at {:?}", package_name, package_path)).await?;
    
    let mut child = TokioCommand::new("dpkg-buildpackage")
        .args(&["-b", "-uc", "-us"])
        .current_dir(package_path)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()?;
        
    // Stream output
    if let Some(stdout) = child.stdout.take() {
        let reader = BufReader::new(stdout);
        let mut lines = reader.lines();
        
        while let Some(line) = lines.next_line().await? {
            report_log("debug", &format!("{}: {}", package_name, line)).await?;
        }
    }
    
    let status = child.wait().await?;
    
    if !status.success() {
        return Err(format!("dpkg-buildpackage failed for {}", package_name).into());
    }
    
    // Move generated .deb files to output
    move_deb_files(package_path.parent().unwrap(), output_dir).await?;
    
    Ok(())
}

async fn move_deb_files(source_dir: &Path, output_dir: &Path) -> Result<(), Box<dyn std::error::Error>> {
    let mut entries = fs::read_dir(source_dir).await?;
    
    while let Some(entry) = entries.next_entry().await? {
        let path = entry.path();
        if let Some(extension) = path.extension() {
            if extension == "deb" {
                let dest = output_dir.join(entry.file_name());
                fs::rename(&path, &dest).await?;
            }
        }
    }
    
    Ok(())
}

async fn generate_repository(output_dir: &Path) -> Result<(), Box<dyn std::error::Error>> {
    report_log("info", "Generating APT repository metadata").await?;
    
    // Create Packages file
    let packages_output = Command::new("dpkg-scanpackages")
        .args(&[".", "/dev/null"])
        .current_dir(output_dir)
        .output()?;
        
    if !packages_output.status.success() {
        return Err("Failed to create Packages file".into());
    }
    
    // Compress Packages file
    let mut gzip_child = Command::new("gzip")
        .args(&["-9c"])
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .spawn()?;
        
    if let Some(stdin) = gzip_child.stdin.take() {
        use std::io::Write;
        let mut stdin = stdin;
        stdin.write_all(&packages_output.stdout)?;
    }
    
    let gzip_output = gzip_child.wait_with_output()?;
    fs::write(output_dir.join("Packages.gz"), gzip_output.stdout).await?;
    
    // Create Release file
    let release_output = Command::new("apt-ftparchive")
        .args(&["release", "."])
        .current_dir(output_dir)
        .output()?;
        
    if !release_output.status.success() {
        return Err("Failed to create Release file".into());
    }
    
    fs::write(output_dir.join("Release"), release_output.stdout).await?;
    
    Ok(())
}

// Progress reporting functions
async fn report_stage(stage: &str) -> Result<(), Box<dyn std::error::Error>> {
    Command::new("/helpers/progress-reporter.rs")
        .args(&["stage", stage])
        .output()?;
    Ok(())
}

async fn report_progress(current: usize, total: usize, message: &str) -> Result<(), Box<dyn std::error::Error>> {
    Command::new("/helpers/progress-reporter.rs")
        .args(&[
            "progress",
            "--current", &current.to_string(),
            "--total", &total.to_string(),
            "--message", message
        ])
        .output()?;
    Ok(())
}

async fn report_package_start(name: &str) -> Result<(), Box<dyn std::error::Error>> {
    Command::new("/helpers/progress-reporter.rs")
        .args(&["package-start", name])
        .output()?;
    Ok(())
}

async fn report_package_complete(name: &str, success: bool) -> Result<(), Box<dyn std::error::Error>> {
    Command::new("/helpers/progress-reporter.rs")
        .args(&[
            "package-complete",
            name,
            "--success", &success.to_string()
        ])
        .output()?;
    Ok(())
}

async fn report_log(level: &str, message: &str) -> Result<(), Box<dyn std::error::Error>> {
    Command::new("/helpers/progress-reporter.rs")
        .args(&["log", "--level", level, "--message", message])
        .output()?;
    Ok(())
}
```

### 5. Container Setup Requirements

**Purpose**: Ensure rust-script and dependencies are available in containers.

**Container Dockerfile additions**:
```dockerfile
# Install Rust and rust-script
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"
RUN cargo install rust-script

# Alternatively, use a base image with Rust pre-installed
# FROM rust:1.75-slim

# Install ROS and build dependencies
RUN apt-get update && apt-get install -y \
    python3-bloom \
    dpkg-dev \
    apt-utils \
    && rm -rf /var/lib/apt/lists/*
```

**Runtime Requirements Check**:
```rust
#!/usr/bin/env rust-script
//! ```cargo
//! [dependencies]
//! which = "4.0"
//! ```

// scripts/helpers/check-requirements.rs

use std::process::Command;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let required_tools = [
        "rust-script",
        "bloom-generate", 
        "dpkg-buildpackage",
        "dpkg-scanpackages",
        "apt-ftparchive",
        "rustc",
        "cargo"
    ];
    
    let mut missing_tools = Vec::new();
    
    for tool in &required_tools {
        if which::which(tool).is_err() {
            missing_tools.push(tool);
        }
    }
    
    if !missing_tools.is_empty() {
        eprintln!("Missing required tools: {:?}", missing_tools);
        std::process::exit(1);
    }
    
    println!("All required tools are available");
    Ok(())
}
```

## Integration with Host

### Volume Mount Structure

```
Container Mounts:
/helpers/                   # Helper scripts from host (rust-script)
├── package-scanner.rs
├── debian-preparer.rs  
├── build-orchestrator.rs
├── progress-reporter.rs
└── check-requirements.rs
```

### Container Entry Point Update

```bash
#!/bin/bash
# /scripts/entrypoint.sh - Updated to use rust-script helpers

# ... user setup code ...

# Make helper scripts executable
chmod +x /helpers/*.rs

# Check rust-script and dependencies
/helpers/check-requirements.rs || exit 1

# Run main build script with helper integration
exec su - builder -c "/scripts/main.sh"
```

### Main Script Update

```bash
#!/bin/bash
# /scripts/main.sh - Updated to use rust-script helpers

source /opt/agiros/$ROS_DISTRO/setup.bash

# Install dependencies
sudo agirosdep init || true
agirosdep update
sudo agirosdep install --from-paths src --ignore-src -y

# Build packages using rust-script orchestrator
# (The orchestrator will run colcon build first, then create .deb packages)
/helpers/build-orchestrator.rs
```

## Performance Considerations

### rust-script vs Pre-compiled Binary Performance

| Aspect            | rust-script                    | Pre-compiled Binary   |
|-------------------|--------------------------------|-----------------------|
| **Startup**       | Compilation overhead first run | Instant startup       |
| **Execution**     | Near-native performance        | Native performance    |
| **Compatibility** | Universal (any architecture)   | Architecture-specific |
| **Development**   | Edit-and-run workflow          | Requires rebuild      |
| **Dependencies**  | Rust toolchain required        | Self-contained        |
| **Caching**       | Cached after first compile     | No compilation needed |

### rust-script Optimizations

1. **Compilation Caching**: rust-script caches compiled versions
2. **Minimal Dependencies**: Keep Cargo.toml dependencies lightweight
3. **Single-File Scripts**: Avoid complex multi-file setups
4. **Async Operations**: Use tokio for I/O-bound operations
5. **Incremental Compilation**: Leverage Rust's incremental compilation

## Testing Strategy

### rust-script Testing

```bash
# Test individual scripts locally
rust-script scripts/helpers/package-scanner.rs /workspace/src

# Test in container with rust support
docker run --rm -it \
  -v $(pwd)/scripts/helpers:/helpers \
  rust:1.75-slim \
  bash -c "cargo install rust-script && /helpers/package-scanner.rs /workspace/src"

# Test full integration  
make test-e2e-cross-arch
```

### Cross-Architecture Testing

```yaml
# .github/workflows/test-cross-arch.yml
name: Cross-Architecture Tests with rust-script

on: [push, pull_request]

jobs:
  test-cross-arch:
    runs-on: ubuntu-latest
    strategy:
      matrix:
        arch: [amd64, arm64]
        rust-version: ["1.75", "1.76", "latest"]
    steps:
      - uses: actions/checkout@v3
      - name: Set up QEMU
        uses: docker/setup-qemu-action@v2
      - name: Test rust-script on ${{ matrix.arch }}
        run: |
          docker run --rm --platform=linux/${{ matrix.arch }} \
            -v $(pwd)/scripts/helpers:/helpers \
            rust:${{ matrix.rust-version }}-slim \
            bash -c "
              cargo install rust-script && \
              /helpers/check-requirements.rs && \
              /helpers/package-scanner.rs /workspace/src
            "
```

### Unit Testing rust-script Files

```rust
#!/usr/bin/env rust-script
//! ```cargo
//! [dependencies]
//! # ... deps for main functionality
//! 
//! [dev-dependencies]
//! tempfile = "3.0"
//! ```

// Include tests directly in rust-script files
#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::TempDir;
    
    #[test]
    fn test_package_scanning() {
        let temp_dir = TempDir::new().unwrap();
        // Create test package.xml
        std::fs::write(
            temp_dir.path().join("package.xml"),
            r#"<package><name>test_pkg</name><version>1.0.0</version></package>"#
        ).unwrap();
        
        let packages = scan_workspace(temp_dir.path()).unwrap();
        assert_eq!(packages.len(), 1);
        assert_eq!(packages[0].name, "test_pkg");
    }
}

fn main() {
    // Normal script execution
}

// Run tests with: rust-script --test script.rs
```

## Container Image Strategy

### Option 1: Custom ROS + Rust Image

```dockerfile
FROM ros:loong-ros-base

# Install Rust
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

# Install rust-script
RUN cargo install rust-script

# Install build dependencies
RUN apt-get update && apt-get install -y \
    python3-bloom \
    dpkg-dev \
    apt-utils \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Verify installation
RUN rust-script --version && bloom-generate --version
```

### Option 2: Multi-stage Build

```dockerfile
# Build stage with full Rust toolchain
FROM rust:1.75 as rust-builder
RUN cargo install rust-script

# Runtime stage with minimal footprint
FROM ros:loong-ros-base
COPY --from=rust-builder /usr/local/cargo/bin/rust-script /usr/local/bin/
COPY --from=rust-builder /usr/local/cargo/bin/rustc /usr/local/bin/
COPY --from=rust-builder /usr/local/cargo/bin/cargo /usr/local/bin/

# Install minimal Rust runtime
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- --minimal -y
ENV PATH="/root/.cargo/bin:${PATH}"
```

## Migration from Compiled Binary Approach

### 1. Remove Binary Compilation
- Remove `colcon-deb-helper` crate from workspace
- Update Docker integration to mount scripts instead of binaries
- Remove cross-compilation logic

### 2. Update Host Integration  
- Change helper preparation from binary compilation to script mounting
- Update container specs to require Rust toolchain
- Modify output parsing to handle rust-script execution

### 3. Container Requirements
- Ensure containers have Rust toolchain or use custom base images
- Install rust-script during container setup
- Verify compilation cache directories are writable

This approach provides the best of both worlds: Rust's performance and type safety with universal cross-architecture compatibility.
