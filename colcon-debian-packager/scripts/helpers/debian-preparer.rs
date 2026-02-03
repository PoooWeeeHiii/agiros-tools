#!/usr/bin/env rust-script
//! ```cargo
//! [dependencies]
//! clap = { version = "4.0", features = ["derive"] }
//! walkdir = "2.4"
//! ```

// scripts/helpers/debian-preparer.rs
// Prepare debian directories for ROS packages.

use clap::Parser;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use walkdir::WalkDir;

#[derive(Parser, Debug)]
#[command(name = "debian-preparer")]
#[command(about = "Prepare debian directories for ROS packages")]
struct Args {
    /// Package name
    #[arg(long)]
    package_name: String,

    /// Package source path
    #[arg(long)]
    package_path: PathBuf,

    /// Package version (unused but accepted for compatibility)
    #[arg(long)]
    package_version: String,

    /// Debian directories collection path
    #[arg(long)]
    debian_dirs: PathBuf,

    /// ROS distribution
    #[arg(long)]
    ros_distro: String,

    /// Use bloom-generate when no custom debian directory exists
    #[arg(long, default_value_t = true)]
    use_bloom: bool,

    /// Force regeneration of debian directory
    #[arg(long, default_value_t = false)]
    force_regenerate: bool,
}

fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let args = Args::parse();

    prepare_debian_dir(&args)?;

    Ok(())
}

fn prepare_debian_dir(args: &Args) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let package_name = &args.package_name;
    let package_path = &args.package_path;
    let target_debian = package_path.join("debian");
    let custom_debian = args.debian_dirs.join(package_name).join("debian");

    report_log("debug", &format!("Preparing debian dir for {}", package_name))?;

    if target_debian.exists() && !args.force_regenerate {
        report_log(
            "info",
            &format!(
                "Skipping debian-preparer, debian directory already exists for {}",
                package_name
            ),
        )?;
        return Ok(());
    }

    if target_debian.exists() {
        std::fs::remove_dir_all(&target_debian)?;
    }

    if custom_debian.exists() {
        report_log(
            "info",
            &format!("Using custom debian directory for {}", package_name),
        )?;
        copy_directory_recursive(&custom_debian, &target_debian)?;
        return Ok(());
    }

    if !args.use_bloom {
        return Err(format!(
            "No custom debian directory found for {} and bloom-generate disabled",
            package_name
        )
        .into());
    }

    report_log(
        "info",
        &format!(
            "Generating debian directory with bloom-generate for {}",
            package_name
        ),
    )?;
    let output = Command::new("bloom-generate")
        .arg("debian")
        .arg("--ros-distro")
        .arg(&args.ros_distro)
        .arg("--os-name")
        .arg("ubuntu")
        .arg("--os-version")
        .arg("jammy")
        .current_dir(package_path)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .output()?;

    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        return Err(format!("bloom-generate failed for {}: {}", package_name, stderr).into());
    }

    if !target_debian.exists() {
        return Err(format!(
            "bloom-generate did not create debian directory for {}",
            package_name
        )
        .into());
    }

    if let Some(parent) = custom_debian.parent() {
        std::fs::create_dir_all(parent)?;
    }
    if let Err(err) = copy_directory_recursive(&target_debian, &custom_debian) {
        report_log(
            "warning",
            &format!("Failed to save generated debian dir for {}: {}", package_name, err),
        )?;
    }

    Ok(())
}

fn copy_directory_recursive(
    source: &Path,
    target: &Path,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    std::fs::create_dir_all(target)?;

    for entry in WalkDir::new(source).into_iter().filter_map(|e| e.ok()) {
        let source_path = entry.path();
        let relative = source_path.strip_prefix(source)?;
        let target_path = target.join(relative);

        if source_path.is_dir() {
            std::fs::create_dir_all(&target_path)?;
        } else {
            if let Some(parent) = target_path.parent() {
                std::fs::create_dir_all(parent)?;
            }
            std::fs::copy(source_path, &target_path)?;
        }
    }

    Ok(())
}

fn report_log(level: &str, message: &str) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    eprintln!("::log::level={},msg={}", level, message);
    Ok(())
}
