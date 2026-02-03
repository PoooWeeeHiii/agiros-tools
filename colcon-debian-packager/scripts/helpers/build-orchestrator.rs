#!/usr/bin/env rust-script
//! ```cargo
//! [dependencies]
//! clap = { version = "4.0", features = ["derive"] }
//! tokio = { version = "1.0", features = ["process", "fs", "rt-multi-thread", "macros", "io-util"] }
//! serde = { version = "1.0", features = ["derive"] }
//! serde_json = "1.0"
//! futures = "0.3"
//! ```

// scripts/helpers/build-orchestrator.rs
// Orchestrates the entire build process with parallel .deb creation

use clap::Parser;
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
use std::env;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use tokio::fs;
use tokio::io::{AsyncBufReadExt, BufReader};
use tokio::process::Command as TokioCommand;

#[derive(Parser, Debug)]
#[command(name = "build-orchestrator")]
#[command(about = "Orchestrate the build process for all packages")]
struct Args {
    /// Workspace path
    #[arg(long, default_value = "/workspace")]
    workspace: PathBuf,
    
    /// Source directory (relative to workspace or absolute)
    #[arg(long)]
    src_dir: Option<PathBuf>,

    /// Build queue file (only build packages listed without #)
    #[arg(long)]
    queue_file: Option<PathBuf>,

    /// Debian directories path
    #[arg(long, default_value = "/workspace/debian_dirs")]
    debian_dirs: PathBuf,
    
    /// Output directory
    #[arg(long, default_value = "/workspace/output")]
    output_dir: PathBuf,

    /// APT source line for AGIROS repository (optional)
    #[arg(long)]
    apt_source: Option<String>,

    /// APT source list file path (optional)
    #[arg(long)]
    apt_source_file: Option<PathBuf>,

    /// Auto-fix missing dependencies with apt when possible
    #[arg(long)]
    auto_fix_deps: Option<bool>,
}

#[derive(Debug, Serialize, Deserialize)]
struct ScanResult {
    packages: Vec<Package>,
    total: usize,
    errors: Vec<String>,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
struct Package {
    name: String,
    version: String,
    path: PathBuf,
    build_type: String,
    description: String,
    maintainers: Vec<String>,
    dependencies: Vec<String>,
}

#[derive(Debug, Clone)]
struct AptConfig {
    source: Option<String>,
    source_file: PathBuf,
    auto_fix_deps: bool,
}

#[derive(Debug, Default)]
struct AptState {
    updated: bool,
    source_checked: bool,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let args = Args::parse();
    
    let ros_distro = env::var("ROS_DISTRO").unwrap_or_else(|_| "loong".to_string());
    let src_dir = args
        .src_dir
        .or_else(|| env::var("COLCON_SRC_DIR").ok().map(PathBuf::from))
        .unwrap_or_else(|| PathBuf::from("src"));
    let queue_file = args
        .queue_file
        .or_else(|| env::var("AGIROS_QUEUE_FILE").ok().map(PathBuf::from));
    let apt_source = args
        .apt_source
        .or_else(|| env::var("AGIROS_APT_SOURCE").ok())
        .and_then(|value| {
            let trimmed = value.trim().to_string();
            if trimmed.is_empty() {
                None
            } else {
                Some(trimmed)
            }
        });
    let apt_source_file = args
        .apt_source_file
        .or_else(|| env::var("AGIROS_APT_SOURCE_FILE").ok().map(PathBuf::from))
        .unwrap_or_else(|| PathBuf::from("/etc/apt/sources.list.d/agiros.list"));
    let auto_fix_deps = args
        .auto_fix_deps
        .or_else(|| env::var("AGIROS_AUTO_FIX_DEPS").ok().map(parse_bool_env))
        .unwrap_or(true);
    let apt_config = AptConfig {
        source: apt_source,
        source_file: apt_source_file,
        auto_fix_deps,
    };

    build_all_packages(
        &args.workspace,
        &src_dir,
        queue_file.as_deref(),
        &args.debian_dirs,
        &args.output_dir,
        &ros_distro,
        &apt_config,
    )
    .await?;
    
    Ok(())
}

async fn build_all_packages(
    workspace: &Path,
    src_dir: &Path,
    queue_file: Option<&Path>,
    debian_dirs: &Path,
    output_dir: &Path,
    ros_distro: &str,
    apt_config: &AptConfig,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let src_dir = resolve_src_dir(workspace, src_dir);
    report_log("info", &format!("Using source root: {}", src_dir.display())).await?;

    if !src_dir.exists() {
        return Err(format!("Source directory does not exist: {}", src_dir.display()).into());
    }
    // Stage: Scanning
    report_stage("scanning").await?;
    
    report_log("info", "Scanning workspace for ROS packages").await?;
    let scan_result = scan_packages(&src_dir).await?;
    let selected_packages = load_queue_packages(queue_file)?;
    let mut ordered_names_override: Option<Vec<String>> = None;
    if !selected_packages.is_empty() {
        report_log(
            "info",
            &format!(
                "Queue targets: {} package(s); resolving dependencies",
                selected_packages.len()
            ),
        )
        .await?;
        let ordered = get_topological_order(workspace, &src_dir, &selected_packages).await?;
        if ordered.is_empty() {
            return Err("Queue resolved to 0 packages; check workspace contents".into());
        }
        report_log(
            "info",
            &format!(
                "Resolved {} package(s) (queue + dependencies)",
                ordered.len()
            ),
        )
        .await?;
        let ordered_set: HashSet<String> = ordered.iter().cloned().collect();
        let mut missing = Vec::new();
        for name in &selected_packages {
            if !ordered_set.contains(name) {
                missing.push(name.clone());
            }
        }
        if !missing.is_empty() {
            report_log(
                "warning",
                &format!(
                    "Queue contains packages not found in workspace: {}",
                    missing.join(", ")
                ),
            )
            .await?;
        }
        ordered_names_override = Some(ordered);
    }
    
    let package_map: HashMap<String, Package> = scan_result
        .packages
        .iter()
        .cloned()
        .map(|pkg| (pkg.name.clone(), pkg))
        .collect();
    let mut ordered_packages: Vec<Package> = Vec::new();
    if let Some(ref ordered_names) = ordered_names_override {
        for name in ordered_names {
            if let Some(pkg) = package_map.get(name) {
                ordered_packages.push(pkg.clone());
            }
        }
    } else {
        ordered_packages = scan_result.packages.clone();
    }

    let total_packages = ordered_packages.len();
    report_log("info", &format!("Found {} packages to build", total_packages)).await?;
    if !scan_result.errors.is_empty() {
        report_log("warning", &format!("Package scan errors: {}", scan_result.errors.len())).await?;
        for error in scan_result.errors.iter().take(5) {
            report_log("warning", error).await?;
        }
    }
    
    if total_packages == 0 {
        report_log("warning", "No packages found to build").await?;
        if !scan_result.errors.is_empty() || !selected_packages.is_empty() {
            return Err("Package scan failed; no packages discovered".into());
        }
        return Ok(());
    }
    
    // Stage: Building with colcon (handles all dependencies)
    report_stage("colcon_build").await?;
    
    report_log("info", "Building all packages with colcon").await?;
    run_colcon_build(workspace, &src_dir, &selected_packages).await?;
    
    // Stage: Preparing Debian directories
    report_stage("preparing").await?;
    
    for (current, package) in ordered_packages.iter().enumerate() {
        let current = current + 1;
        report_progress(current, total_packages, &format!("Preparing {}", package.name)).await?;
        
        let package_path = src_dir.join(&package.path);
        prepare_debian_dir(&package.name, &package_path, debian_dirs, ros_distro, &package.version).await?;
    }
    
    // Refine package order based on Debian Build-Depends
    if let Ok(reordered) = order_by_debian_build_depends(&ordered_packages, &src_dir, ros_distro) {
        ordered_packages = reordered;
    }

    // Stage: Creating .deb packages (topological order + install)
    report_stage("packaging_and_installing").await?;
    
    // Create output directory
    fs::create_dir_all(output_dir).await?;
    
    if ordered_names_override.is_none() {
        let ordered_names = match get_topological_order(workspace, &src_dir, &selected_packages).await {
            Ok(names) if !names.is_empty() => names,
            Ok(_) => {
                report_log("warning", "colcon list returned empty order, falling back to scan order").await?;
                Vec::new()
            }
            Err(e) => {
                report_log("warning", &format!("Failed to get topological order: {}", e)).await?;
                Vec::new()
            }
        };
        if !ordered_names.is_empty() {
            let mut reordered = Vec::new();
            let mut seen = HashSet::new();
            for name in ordered_names {
                if let Some(pkg) = package_map.get(&name) {
                    reordered.push(pkg.clone());
                    seen.insert(name);
                } else {
                    report_log("warning", &format!("Topological list contains unknown package: {}", name)).await?;
                }
            }
            for pkg in &ordered_packages {
                if !seen.contains(&pkg.name) {
                    reordered.push(pkg.clone());
                }
            }
            ordered_packages = reordered;
        }
    }

    let mut apt_state = AptState::default();
    for package in ordered_packages {
        let package_name = package.name.clone();
        let package_path = src_dir.join(&package.path);

        report_package_start(&package_name).await?;

        match build_single_package(
            &package_name,
            &package_path,
            output_dir,
            apt_config,
            &mut apt_state,
        )
        .await
        {
            Ok(debs) => {
                if let Err(e) = install_deb_packages(
                    &package_name,
                    &debs,
                    apt_config,
                    &mut apt_state,
                )
                .await
                {
                    report_package_complete(&package_name, false).await?;
                    report_log("error", &format!("Install failed for {}: {}", package_name, e)).await?;
                    return Err(e);
                }
                if let Err(e) = mark_queue_completed(queue_file, &package_name) {
                    report_log("warning", &format!("Failed to update queue for {}: {}", package_name, e)).await?;
                }
                report_package_complete(&package_name, true).await?;
            }
            Err(e) => {
                report_package_complete(&package_name, false).await?;
                report_log("error", &format!("Build failed for {}: {}", package_name, e)).await?;
                return Err(e);
            }
        }
    }
    
    // Stage: Repository
    report_stage("repository").await?;
    generate_repository(output_dir).await?;

    // Create artifacts.tar for host tool to collect
    package_artifacts(output_dir).await?;
    
    // Stage: Complete
    report_stage("complete").await?;
    report_log("info", "Build completed successfully").await?;
    
    Ok(())
}

async fn scan_packages(src_path: &Path) -> Result<ScanResult, Box<dyn std::error::Error + Send + Sync>> {
    let helper = helper_script_path("package-scanner.rs");
    let output = Command::new("rust-script")
        .arg(&helper)
        .arg(src_path)
        .arg("--format")
        .arg("json")
        .output()?;
        
    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        return Err(format!("Failed to scan packages: {}", stderr).into());
    }
    
    let stdout = String::from_utf8(output.stdout)?;
    let result: ScanResult = serde_json::from_str(&stdout)?;
    
    Ok(result)
}

async fn run_colcon_build(
    workspace: &Path,
    src_dir: &Path,
    selected_packages: &[String],
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let parallel_jobs = env::var("PARALLEL_JOBS").unwrap_or_else(|_| "4".to_string());
    let build_type = env::var("BUILD_TYPE").unwrap_or_else(|_| "Release".to_string());
    
    let mut child = TokioCommand::new("colcon")
        .args(&[
            "build",
            "--merge-install",
            "--base-paths",
            &src_dir.to_string_lossy(),
        ])
        .args(if selected_packages.is_empty() {
            Vec::new()
        } else {
            let mut args = vec!["--packages-up-to".to_string()];
            args.extend(selected_packages.iter().cloned());
            args
        })
        .args(&[
            "--parallel-workers",
            &parallel_jobs,
            "--cmake-args",
            &format!("-DCMAKE_BUILD_TYPE={}", build_type),
        ])
        .current_dir(workspace)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()?;
        
    // Stream stdout
    if let Some(stdout) = child.stdout.take() {
        let reader = BufReader::new(stdout);
        let mut lines = reader.lines();
        
        tokio::spawn(async move {
            while let Ok(Some(line)) = lines.next_line().await {
                let _ = report_log("debug", &format!("colcon: {}", line)).await;
            }
        });
    }
    
    // Stream stderr
    if let Some(stderr) = child.stderr.take() {
        let reader = BufReader::new(stderr);
        let mut lines = reader.lines();
        
        tokio::spawn(async move {
            while let Ok(Some(line)) = lines.next_line().await {
                let _ = report_log("debug", &format!("colcon: {}", line)).await;
            }
        });
    }
    
    let status = child.wait().await?;
    
    if !status.success() {
        return Err("colcon build failed".into());
    }
    
    Ok(())
}

async fn get_topological_order(
    workspace: &Path,
    src_dir: &Path,
    selected_packages: &[String],
) -> Result<Vec<String>, Box<dyn std::error::Error + Send + Sync>> {
    let output = TokioCommand::new("colcon")
        .args(&[
            "list",
            "--topological-order",
            "--names-only",
            "--base-paths",
            &src_dir.to_string_lossy(),
        ])
        .args(if selected_packages.is_empty() {
            Vec::new()
        } else {
            let mut args = vec!["--packages-up-to".to_string()];
            args.extend(selected_packages.iter().cloned());
            args
        })
        .current_dir(workspace)
        .output()
        .await?;

    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        return Err(format!("colcon list failed: {}", stderr).into());
    }

    let stdout = String::from_utf8(output.stdout)?;
    let ordered = stdout
        .lines()
        .map(|line| line.trim())
        .filter(|line| !line.is_empty())
        .map(|line| line.to_string())
        .collect::<Vec<String>>();

    Ok(ordered)
}

fn resolve_src_dir(workspace: &Path, src_dir: &Path) -> PathBuf {
    if src_dir.is_absolute() {
        src_dir.to_path_buf()
    } else {
        workspace.join(src_dir)
    }
}

fn parse_bool_env(value: String) -> bool {
    matches!(
        value.trim().to_lowercase().as_str(),
        "1" | "true" | "yes" | "y" | "on"
    )
}

fn load_queue_packages(
    queue_file: Option<&Path>,
) -> Result<Vec<String>, Box<dyn std::error::Error + Send + Sync>> {
    let Some(queue_file) = queue_file else {
        return Ok(Vec::new());
    };
    if !queue_file.exists() {
        return Err(format!("Queue file does not exist: {}", queue_file.display()).into());
    }
    let content = std::fs::read_to_string(queue_file)?;
    let mut pending = Vec::new();
    for line in content.lines() {
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }
        let mut completed = false;
        let mut name: Option<String> = None;
        if trimmed.starts_with('{') && trimmed.ends_with('}') {
            if let Ok(value) = serde_json::from_str::<serde_json::Value>(trimmed) {
                if let Some(n) = value.get("name").and_then(|v| v.as_str()) {
                    name = Some(n.to_string());
                    completed = value.get("completed").and_then(|v| v.as_bool()).unwrap_or(false);
                }
            }
        } else {
            let mut raw = trimmed.to_string();
            if raw.ends_with('#') {
                completed = true;
                raw = raw.trim_end_matches('#').trim().to_string();
            }
            if !raw.is_empty() {
                name = Some(raw);
            }
        }
        let Some(mut pkg_name) = name else { continue };
        if let Some(base) = Path::new(&pkg_name).file_name() {
            pkg_name = base.to_string_lossy().to_string();
        }
        pkg_name = normalize_package_name(&pkg_name);
        if completed {
            continue;
        }
        if !pending.contains(&pkg_name) {
            pending.push(pkg_name);
        }
    }
    if pending.is_empty() {
        return Err("Queue file has no pending packages".into());
    }
    Ok(pending)
}

fn normalize_package_name(name: &str) -> String {
    name.replace('-', "_")
}

fn mark_queue_completed(
    queue_file: Option<&Path>,
    package_name: &str,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let Some(queue_file) = queue_file else {
        return Ok(());
    };
    if !queue_file.exists() {
        return Err(format!("Queue file does not exist: {}", queue_file.display()).into());
    }

    let target = normalize_package_name(package_name);
    let content = std::fs::read_to_string(queue_file)?;
    let mut changed = false;
    let mut output_lines = Vec::new();

    for raw_line in content.lines() {
        let trimmed = raw_line.trim();
        if trimmed.is_empty() {
            output_lines.push(raw_line.to_string());
            continue;
        }

        let mut line_out = raw_line.to_string();
        let mut name: Option<String> = None;
        let mut is_json = false;
        let mut completed = false;

        if trimmed.starts_with('{') && trimmed.ends_with('}') {
            if let Ok(mut value) = serde_json::from_str::<serde_json::Value>(trimmed) {
                is_json = true;
                if let Some(n) = value.get("name").and_then(|v| v.as_str()) {
                    name = Some(n.to_string());
                }
                completed = value.get("completed").and_then(|v| v.as_bool()).unwrap_or(false);
                if let Some(n) = &name {
                    if normalize_package_name(n) == target {
                        if !completed {
                            value["completed"] = serde_json::Value::Bool(true);
                            line_out = serde_json::to_string(&value)?;
                            changed = true;
                        }
                    }
                }
            }
        }

        if !is_json {
            let mut raw = trimmed.to_string();
            if raw.ends_with('#') {
                completed = true;
                raw = raw.trim_end_matches('#').trim().to_string();
            }
            if !raw.is_empty() {
                let base = Path::new(&raw)
                    .file_name()
                    .map(|v| v.to_string_lossy().to_string())
                    .unwrap_or(raw.clone());
                name = Some(base);
            }
            if let Some(n) = &name {
                if normalize_package_name(n) == target && !completed {
                    line_out = format!("{}#", raw);
                    changed = true;
                }
            }
        }

        output_lines.push(line_out);
    }

    if changed {
        std::fs::write(queue_file, output_lines.join("\n"))?;
    }

    Ok(())
}

fn order_by_debian_build_depends(
    packages: &[Package],
    src_dir: &Path,
    ros_distro: &str,
) -> Result<Vec<Package>, Box<dyn std::error::Error + Send + Sync>> {
    let name_map = build_debian_name_map(packages, ros_distro);
    let mut deps: HashMap<String, HashSet<String>> = HashMap::new();
    let package_names: HashSet<String> = packages.iter().map(|p| p.name.clone()).collect();

    for pkg in packages {
        let control_path = src_dir.join(&pkg.path).join("debian").join("control");
        if !control_path.exists() {
            continue;
        }
        if let Ok(dep_tokens) = parse_build_depends(&control_path) {
            let mut dep_set: HashSet<String> = HashSet::new();
            for token in dep_tokens {
                if let Some(dep_pkg) = resolve_dep_to_package(&token, &name_map) {
                    if dep_pkg != pkg.name && package_names.contains(&dep_pkg) {
                        dep_set.insert(dep_pkg);
                    }
                }
            }
            if !dep_set.is_empty() {
                deps.insert(pkg.name.clone(), dep_set);
            }
        }
    }

    let order_hint: HashMap<String, usize> = packages
        .iter()
        .enumerate()
        .map(|(idx, pkg)| (pkg.name.clone(), idx))
        .collect();

    let ordered_names = topo_sort_with_hint(packages, &deps, &order_hint)?;
    let mut ordered = Vec::new();
    let package_map: HashMap<String, Package> =
        packages.iter().cloned().map(|pkg| (pkg.name.clone(), pkg)).collect();
    for name in ordered_names {
        if let Some(pkg) = package_map.get(&name) {
            ordered.push(pkg.clone());
        }
    }
    Ok(ordered)
}

fn build_debian_name_map(
    packages: &[Package],
    ros_distro: &str,
) -> HashMap<String, String> {
    let mut map = HashMap::new();
    for pkg in packages {
        let ros_name = pkg.name.clone();
        let hyphen = ros_name.replace('_', "-");
        let candidates = [
            ros_name.clone(),
            hyphen.clone(),
            format!("ros-{}-{}", ros_distro, hyphen),
            format!("agiros-{}-{}", ros_distro, hyphen),
            format!("ros-{}", hyphen),
            format!("agiros-{}", hyphen),
        ];
        for candidate in candidates {
            map.insert(candidate.to_lowercase(), ros_name.clone());
        }
    }
    map
}

fn parse_build_depends(
    control_path: &Path,
) -> Result<Vec<String>, Box<dyn std::error::Error + Send + Sync>> {
    let content = std::fs::read_to_string(control_path)?;
    let mut deps_raw: Vec<String> = Vec::new();
    let mut current: Option<String> = None;

    for line in content.lines() {
        if line.starts_with("Build-Depends:") {
            if let Some(prev) = current.take() {
                deps_raw.push(prev);
            }
            current = Some(line["Build-Depends:".len()..].trim().to_string());
            continue;
        }
        if line.starts_with("Build-Depends-Indep:") {
            if let Some(prev) = current.take() {
                deps_raw.push(prev);
            }
            current = Some(line["Build-Depends-Indep:".len()..].trim().to_string());
            continue;
        }
        if line.starts_with(' ') || line.starts_with('\t') {
            if let Some(ref mut buf) = current {
                buf.push(' ');
                buf.push_str(line.trim());
            }
            continue;
        }
        if let Some(prev) = current.take() {
            deps_raw.push(prev);
        }
    }
    if let Some(prev) = current.take() {
        deps_raw.push(prev);
    }

    let mut deps = Vec::new();
    for raw in deps_raw {
        for item in raw.split(',') {
            let item = item.trim();
            if item.is_empty() {
                continue;
            }
            deps.push(item.to_string());
        }
    }
    Ok(deps)
}

fn resolve_dep_to_package(
    raw: &str,
    name_map: &HashMap<String, String>,
) -> Option<String> {
    for alt in raw.split('|') {
        let token = alt.trim();
        if token.is_empty() {
            continue;
        }
        let name = token
            .split_whitespace()
            .next()
            .unwrap_or(token)
            .trim()
            .trim_end_matches(')')
            .trim_end_matches('(')
            .to_string();
        let key = name.to_lowercase();
        if let Some(mapped) = name_map.get(&key) {
            return Some(mapped.clone());
        }
    }
    None
}

fn topo_sort_with_hint(
    packages: &[Package],
    deps: &HashMap<String, HashSet<String>>,
    order_hint: &HashMap<String, usize>,
) -> Result<Vec<String>, Box<dyn std::error::Error + Send + Sync>> {
    let mut indegree: HashMap<String, usize> = HashMap::new();
    let mut adj: HashMap<String, Vec<String>> = HashMap::new();
    let names: Vec<String> = packages.iter().map(|p| p.name.clone()).collect();

    for name in &names {
        indegree.entry(name.clone()).or_insert(0);
    }
    for (pkg, dep_set) in deps {
        for dep in dep_set {
            adj.entry(dep.clone()).or_default().push(pkg.clone());
            if let Some(count) = indegree.get_mut(pkg) {
                *count += 1;
            }
        }
    }

    let mut ready: Vec<String> = indegree
        .iter()
        .filter_map(|(name, count)| if *count == 0 { Some(name.clone()) } else { None })
        .collect();
    ready.sort_by_key(|name| order_hint.get(name).cloned().unwrap_or(usize::MAX));

    let mut ordered = Vec::new();
    while let Some(name) = ready.first().cloned() {
        ready.remove(0);
        ordered.push(name.clone());
        if let Some(neighbors) = adj.get(&name) {
            for neighbor in neighbors {
                if let Some(count) = indegree.get_mut(neighbor) {
                    *count = count.saturating_sub(1);
                    if *count == 0 {
                        ready.push(neighbor.clone());
                        ready.sort_by_key(|n| order_hint.get(n).cloned().unwrap_or(usize::MAX));
                    }
                }
            }
        }
    }

    if ordered.len() != names.len() {
        return Err("Detected dependency cycle in Debian Build-Depends".into());
    }

    Ok(ordered)
}

async fn prepare_debian_dir(
    package_name: &str,
    package_path: &Path,
    debian_dirs: &Path,
    ros_distro: &str,
    package_version: &str,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let helper = helper_script_path("debian-preparer.rs");
    let output = Command::new("rust-script")
        .arg(&helper)
        .args(&[
            "--package-name", package_name,
            "--package-path", &package_path.to_string_lossy(),
            "--package-version", package_version,
            "--debian-dirs", &debian_dirs.to_string_lossy(),
            "--ros-distro", ros_distro,
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
    apt_config: &AptConfig,
    apt_state: &mut AptState,
) -> Result<Vec<PathBuf>, Box<dyn std::error::Error + Send + Sync>> {
    report_log("debug", &format!("Building package {} at {:?}", package_name, package_path)).await?;
    
    // Check if debian directory exists
    if !package_path.join("debian").exists() {
        return Err(format!("No debian directory found for package {}", package_name).into());
    }

    cleanup_dh_build_dirs(package_path).await?;
    let (status, stderr_buf) = run_dpkg_buildpackage(package_name, package_path).await?;

    if !status.success() {
        // Log stderr on failure
        for line in &stderr_buf {
            let _ = report_log("error", &format!("{}: {}", package_name, line.trim())).await;
        }
        if apt_config.auto_fix_deps {
            let missing = parse_missing_build_deps(&stderr_buf);
            if !missing.is_empty() {
                let (alias_changed, resolved_missing) =
                    apply_build_dep_aliases(package_path, &missing)?;
                if alias_changed {
                    report_log(
                        "warning",
                        &format!(
                            "Adjusted Build-Depends for {} using alias map",
                            package_name
                        ),
                    )
                    .await?;
                }
                let missing = if alias_changed {
                    resolved_missing
                } else {
                    missing
                };
                report_log(
                    "warning",
                    &format!(
                        "Missing build deps for {}: {}",
                        package_name,
                        missing.join(", ")
                    ),
                )
                .await?;
                if install_build_deps(&missing, apt_config, apt_state).await? {
                    cleanup_dh_build_dirs(package_path).await?;
                    let (retry_status, retry_stderr) =
                        run_dpkg_buildpackage(package_name, package_path).await?;
                    if retry_status.success() {
                        let moved = move_deb_files(package_path.parent().unwrap(), output_dir).await?;
                        return Ok(moved);
                    }
                    for line in &retry_stderr {
                        let _ = report_log("error", &format!("{}: {}", package_name, line.trim())).await;
                    }
                }
            }
        }
        return Err(format!("dpkg-buildpackage failed for {}", package_name).into());
    }
    
    // Move generated .deb files to output
    let moved = move_deb_files(package_path.parent().unwrap(), output_dir).await?;

    Ok(moved)
}

async fn cleanup_dh_build_dirs(
    package_path: &Path,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let mut entries = match fs::read_dir(package_path).await {
        Ok(entries) => entries,
        Err(_) => return Ok(()),
    };
    while let Some(entry) = entries.next_entry().await? {
        let name = entry.file_name();
        let name_str = name.to_string_lossy();
        if !name_str.starts_with(".obj-") {
            continue;
        }
        let path = entry.path();
        let file_type = entry.file_type().await?;
        if file_type.is_dir() {
            let _ = fs::remove_dir_all(&path).await;
        } else {
            let _ = fs::remove_file(&path).await;
        }
    }
    Ok(())
}

async fn run_dpkg_buildpackage(
    package_name: &str,
    package_path: &Path,
) -> Result<(std::process::ExitStatus, Vec<String>), Box<dyn std::error::Error + Send + Sync>> {
    let mut child = TokioCommand::new("dpkg-buildpackage")
        .args(&["-b", "-uc", "-us"])
        .current_dir(package_path)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()?;

    let mut stderr_buf = Vec::new();

    if let Some(stdout) = child.stdout.take() {
        let mut reader = BufReader::new(stdout);
        let mut line = String::new();
        while reader.read_line(&mut line).await? > 0 {
            let _ = report_log("debug", &format!("{}: {}", package_name, line.trim())).await;
            line.clear();
        }
    }

    if let Some(stderr) = child.stderr.take() {
        let mut reader = BufReader::new(stderr);
        let mut line = String::new();
        while reader.read_line(&mut line).await? > 0 {
            stderr_buf.push(line.clone());
            line.clear();
        }
    }

    let status = child.wait().await?;
    Ok((status, stderr_buf))
}

async fn move_deb_files(
    source_dir: &Path,
    output_dir: &Path,
) -> Result<Vec<PathBuf>, Box<dyn std::error::Error + Send + Sync>> {
    let mut entries = fs::read_dir(source_dir).await?;
    let mut moved_files = Vec::new();
    
    while let Some(entry) = entries.next_entry().await? {
        let path = entry.path();
        if let Some(extension) = path.extension() {
            if extension == "deb" {
                let name = entry.file_name();
                let name_str = name.to_string_lossy();
                if name_str.contains("-dbgsym_") {
                    continue;
                }
                let dest = output_dir.join(entry.file_name());
                fs::copy(&path, &dest).await?;
                // fs::remove_file(&path).await?; // Optional: cleanup source
                moved_files.push(dest);
            }
        }
    }
    
    if moved_files.is_empty() {
        return Err("No .deb files generated".into());
    }
    
    Ok(moved_files)
}

async fn install_deb_packages(
    package_name: &str,
    debs: &[PathBuf],
    apt_config: &AptConfig,
    apt_state: &mut AptState,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    if debs.is_empty() {
        return Err(format!("No .deb files to install for {}", package_name).into());
    }

    report_log("info", &format!("Installing built packages for {}", package_name)).await?;

    let mut args = Vec::new();
    for deb in debs {
        args.push(deb.to_string_lossy().to_string());
    }
    args.sort();

    let output = TokioCommand::new("dpkg")
        .arg("-i")
        .args(&args)
        .output()
        .await?;

    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        if apt_config.auto_fix_deps {
            report_log(
                "warning",
                &format!(
                    "dpkg -i failed for {}, attempting apt-get -f install",
                    package_name
                ),
            )
            .await?;
            if apt_fix_dependencies(apt_config, apt_state).await? {
                let retry = TokioCommand::new("dpkg")
                    .arg("-i")
                    .args(&args)
                    .output()
                    .await?;
                if retry.status.success() {
                    return Ok(());
                }
                let retry_err = String::from_utf8_lossy(&retry.stderr);
                return Err(format!(
                    "dpkg -i failed for {} after apt-get -f install: {}",
                    package_name, retry_err
                )
                .into());
            }
        }
        return Err(format!("dpkg -i failed for {}: {}", package_name, stderr).into());
    }

    Ok(())
}

fn parse_missing_build_deps(stderr_lines: &[String]) -> Vec<String> {
    let mut deps = Vec::new();
    let marker = "Unmet build dependencies:";
    for line in stderr_lines {
        if let Some(idx) = line.find(marker) {
            let raw = line[idx + marker.len()..].trim();
            deps.extend(extract_dep_names(raw));
        }
    }
    deps.sort();
    deps.dedup();
    deps
}

fn apply_build_dep_aliases(
    package_path: &Path,
    missing: &[String],
) -> Result<(bool, Vec<String>), Box<dyn std::error::Error + Send + Sync>> {
    const DEP_ALIASES: &[(&str, &str)] = &[("python3-catkin-pkg-modules", "python3-catkin-pkg")];
    let mut missing_set: HashSet<String> = missing.iter().cloned().collect();
    let mut replacements: Vec<(&str, &str)> = Vec::new();

    for (from, to) in DEP_ALIASES {
        if missing_set.contains(*from) {
            replacements.push((*from, *to));
            missing_set.remove(*from);
            missing_set.insert((*to).to_string());
        }
    }

    if replacements.is_empty() {
        let mut out: Vec<String> = missing_set.into_iter().collect();
        out.sort();
        return Ok((false, out));
    }

    let control_path = package_path.join("debian").join("control");
    if !control_path.exists() {
        let mut out: Vec<String> = missing_set.into_iter().collect();
        out.sort();
        return Ok((false, out));
    }

    let mut content = std::fs::read_to_string(&control_path)?;
    let mut changed = false;
    for (from, to) in replacements {
        if content.contains(from) {
            content = content.replace(from, to);
            changed = true;
        }
    }
    if changed {
        std::fs::write(&control_path, content)?;
    }

    let mut out: Vec<String> = missing_set.into_iter().collect();
    out.sort();
    Ok((changed, out))
}

fn extract_dep_names(raw: &str) -> Vec<String> {
    let mut names = Vec::new();
    let normalized = raw.replace(',', " ").replace('|', " ");
    for token in normalized.split_whitespace() {
        if let Some(name) = clean_dep_token(token) {
            names.push(name);
        }
    }
    names
}

fn clean_dep_token(token: &str) -> Option<String> {
    let cleaned = token.trim_matches(|c: char| c == ',' || c == '(' || c == ')');
    if cleaned.is_empty() {
        return None;
    }
    if cleaned.starts_with('[') {
        return None;
    }
    if cleaned == "|" {
        return None;
    }
    if cleaned.starts_with(">") || cleaned.starts_with("<") || cleaned.starts_with("=") {
        return None;
    }
    if cleaned.chars().all(|c| c.is_ascii_digit() || c == '.') {
        return None;
    }
    if !cleaned.chars().any(|c| c.is_ascii_alphabetic()) {
        return None;
    }
    let base = cleaned.split(':').next().unwrap_or(cleaned);
    Some(base.to_string())
}

async fn install_build_deps(
    packages: &[String],
    apt_config: &AptConfig,
    apt_state: &mut AptState,
) -> Result<bool, Box<dyn std::error::Error + Send + Sync>> {
    if packages.is_empty() {
        return Ok(false);
    }
    ensure_apt_ready(apt_config, apt_state).await?;
    let mut cmd = TokioCommand::new("apt-get");
    cmd.args(&["install", "-y", "--no-install-recommends"]);
    for pkg in packages {
        cmd.arg(pkg);
    }
    let output = cmd.output().await?;
    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        report_log("warning", &format!("apt-get install failed: {}", stderr)).await?;
        return Ok(false);
    }
    Ok(true)
}

async fn apt_fix_dependencies(
    apt_config: &AptConfig,
    apt_state: &mut AptState,
) -> Result<bool, Box<dyn std::error::Error + Send + Sync>> {
    ensure_apt_ready(apt_config, apt_state).await?;
    let output = TokioCommand::new("apt-get")
        .args(&[
            "-o",
            "Debug::pkgProblemResolver=yes",
            "-f",
            "install",
            "-y",
        ])
        .output()
        .await?;
    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        report_log("warning", &format!("apt-get -f install failed: {}", stderr)).await?;
        return Ok(false);
    }
    Ok(true)
}

async fn ensure_apt_ready(
    apt_config: &AptConfig,
    apt_state: &mut AptState,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    if !apt_config.auto_fix_deps {
        return Ok(());
    }
    if !apt_state.source_checked {
        ensure_apt_source(apt_config).await?;
        apt_state.source_checked = true;
    }
    if !apt_state.updated {
        let output = TokioCommand::new("apt-get").arg("update").output().await?;
        if !output.status.success() {
            let stderr = String::from_utf8_lossy(&output.stderr);
            report_log("warning", &format!("apt-get update failed: {}", stderr)).await?;
        }
        apt_state.updated = true;
    }
    Ok(())
}

async fn ensure_apt_source(
    apt_config: &AptConfig,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let Some(source_line) = &apt_config.source else {
        return Ok(());
    };
    let list_path = &apt_config.source_file;
    if let Some(parent) = list_path.parent() {
        if !parent.exists() {
            fs::create_dir_all(parent).await?;
        }
    }
    let existing = fs::read_to_string(list_path).await.unwrap_or_default();
    if existing.lines().any(|line| line.trim() == source_line) {
        return Ok(());
    }
    let mut new_content = existing;
    if !new_content.is_empty() && !new_content.ends_with('\n') {
        new_content.push('\n');
    }
    new_content.push_str(source_line);
    new_content.push('\n');
    fs::write(list_path, new_content).await?;
    report_log(
        "info",
        &format!("Added APT source line to {}", list_path.display()),
    )
    .await?;
    Ok(())
}

async fn generate_repository(output_dir: &Path) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    report_log("info", "Generating APT repository metadata").await?;
    
    // Call the create-repo.sh script
    let output = Command::new(script_path("create-repo.sh"))
        .arg(output_dir)
        .output()?;
        
    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        return Err(format!("Failed to create repository: {}", stderr).into());
    }
    
    Ok(())
}

async fn package_artifacts(output_dir: &Path) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    report_log("info", "Packing artifacts into tar archive").await?;
    
    // Create artifacts.tar from all files in output_dir
    // We use std::process::Command to call tar directly for simplicity
    let tar_path = output_dir.join("artifacts.tar");
    let output = Command::new("tar")
        // Create archive
        .arg("cvf")
        .arg(&tar_path)
        // Change to output dir so paths are relative
        .arg("-C")
        .arg(output_dir)
        // Archive everything in current dir (which is output_dir due to -C)
        .arg(".") 
        .output()?;

    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        return Err(format!("Failed to create artifacts.tar: {}", stderr).into());
    }
    
    Ok(())
}

// Progress reporting functions
async fn report_stage(stage: &str) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    eprintln!("::progress::type=stage,value={}", stage);
    Ok(())
}

async fn report_progress(current: usize, total: usize, message: &str) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    eprintln!("::progress::type=general,current={},total={},message={}", current, total, message);
    Ok(())
}

async fn report_package_start(name: &str) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    eprintln!("::progress::type=package_start,name={}", name);
    Ok(())
}

async fn report_package_complete(name: &str, success: bool) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    eprintln!("::progress::type=package_complete,name={},success={}", name, success);
    Ok(())
}

async fn report_log(level: &str, message: &str) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    eprintln!("::log::level={},msg={}", level, message);
    Ok(())
}

fn helper_script_path(name: &str) -> PathBuf {
    let dir = env::var("COLCON_HELPERS_DIR").unwrap_or_else(|_| "/helpers".to_string());
    PathBuf::from(dir).join(name)
}

fn script_path(name: &str) -> PathBuf {
    let dir = env::var("COLCON_SCRIPTS_DIR").unwrap_or_else(|_| "/scripts".to_string());
    PathBuf::from(dir).join(name)
}
