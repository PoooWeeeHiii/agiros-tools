#!/usr/bin/env rust-script
//! ```cargo
//! [dependencies]
//! serde = { version = "1.0", features = ["derive"] }
//! serde_json = "1.0"
//! walkdir = "2.4"
//! quick-xml = "0.31"
//! clap = { version = "4.0", features = ["derive"] }
//! ```

// scripts/helpers/package-scanner.rs
// Scans a workspace for ROS packages and outputs JSON

use clap::Parser;
use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};
use std::process::Command;
use std::thread::sleep;
use std::time::Duration;
use walkdir::WalkDir;

#[derive(Parser, Debug)]
#[command(name = "package-scanner")]
#[command(about = "Scan ROS workspace for packages")]
struct Args {
    /// Path to scan for packages
    #[arg(default_value = "/workspace/src")]
    path: PathBuf,
    
    /// Output format
    #[arg(short, long, default_value = "json")]
    format: String,
}

#[derive(Debug, Serialize, Deserialize)]
struct PackageInfo {
    name: String,
    version: String,
    path: PathBuf,
    build_type: String,
    description: String,
    maintainers: Vec<String>,
    dependencies: Vec<String>,
}

#[derive(Debug, Serialize)]
struct ScanResult {
    packages: Vec<PackageInfo>,
    total: usize,
    errors: Vec<String>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    
    let mut result = ScanResult {
        packages: Vec::new(),
        total: 0,
        errors: Vec::new(),
    };
    
    // Walk the directory tree
    for entry in WalkDir::new(&args.path)
        .follow_links(true)
        .into_iter()
        .filter_entry(|e| {
            // Skip directories that contain COLCON_IGNORE or are build artifacts
            if e.path().is_dir() {
                let name = e.file_name().to_string_lossy();
                if name == "build" || name == "install" || name == "log" || name == "target" || name == "debian" {
                    return false;
                }
                if e.path().join("COLCON_IGNORE").exists() {
                    return false;
                }
            }
            true
        })
        .filter_map(|e| e.ok())
    {
        let path = entry.path();
        
        // Look for package.xml
        if path.file_name() == Some(std::ffi::OsStr::new("package.xml")) {
            match parse_package_xml(path) {
                Ok(package) => {
                    result.packages.push(package);
                    result.total += 1;
                }
                Err(e) => {
                    result.errors.push(format!("{}: {}", path.display(), e));
                }
            }
        }
    }
    
    // Sort packages by name
    result.packages.sort_by(|a, b| a.name.cmp(&b.name));
    
    // Output result
    match args.format.as_str() {
        "json" => {
            println!("{}", serde_json::to_string_pretty(&result)?);
        }
        "names" => {
            for pkg in &result.packages {
                println!("{}", pkg.name);
            }
        }
        "paths" => {
            for pkg in &result.packages {
                println!("{}", pkg.path.display());
            }
        }
        _ => {
            eprintln!("Unknown format: {}", args.format);
            std::process::exit(1);
        }
    }
    
    Ok(())
}

fn parse_package_xml(path: &Path) -> Result<PackageInfo, Box<dyn std::error::Error>> {
    let content = read_package_xml(path)?;
    let mut package = PackageInfo {
        name: String::new(),
        version: String::new(),
        path: path.parent().unwrap_or(path).to_path_buf(),
        build_type: "unknown".to_string(),
        description: String::new(),
        maintainers: Vec::new(),
        dependencies: Vec::new(),
    };
    
    use quick_xml::events::Event;
    use quick_xml::Reader;
    
    let mut reader = Reader::from_str(&content);
    reader.trim_text(true);
    
    let mut buf = Vec::new();
    let mut current_element = String::new();
    let mut in_export = false;
    
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let name_bytes = e.name();
                let name = std::str::from_utf8(name_bytes.as_ref())?;
                current_element = name.to_string();
                if name == "export" {
                    in_export = true;
                }
            }
            Ok(Event::End(ref e)) => {
                let name_bytes = e.name();
                let name = std::str::from_utf8(name_bytes.as_ref())?;
                if name == "export" {
                    in_export = false;
                }
            }
            Ok(Event::Text(e)) => {
                let text = e.unescape()?.trim().to_string();
                if !text.is_empty() {
                    match current_element.as_str() {
                        "name" => package.name = text,
                        "version" => package.version = text,
                        "description" => package.description = text,
                        "maintainer" => package.maintainers.push(text),
                        "build_depend" | "exec_depend" | "depend" => {
                            if !package.dependencies.contains(&text) {
                                package.dependencies.push(text);
                            }
                        }
                        _ => {}
                    }
                }
            }
            Ok(Event::Empty(ref e)) => {
                let name_bytes = e.name();
                let name = std::str::from_utf8(name_bytes.as_ref())?;
                if in_export && name == "build_type" {
                    if let Some(attr) = e.attributes()
                        .filter_map(|a| a.ok())
                        .find(|a| a.key.as_ref() == b"value")
                    {
                        package.build_type = std::str::from_utf8(&attr.value)?.to_string();
                    }
                }
            }
            Ok(Event::Eof) => break,
            Err(e) => return Err(Box::new(e)),
            _ => {}
        }
        buf.clear();
    }
    
    // Detect build type from dependencies if not explicitly set
    if package.build_type == "unknown" {
        package.build_type = detect_build_type(&content);
    }
    
    Ok(package)
}

fn read_package_xml(path: &Path) -> Result<String, Box<dyn std::error::Error>> {
    let mut last_error: Option<std::io::Error> = None;
    for attempt in 0..6 {
        match std::fs::read_to_string(path) {
            Ok(content) => return Ok(content),
            Err(err) => {
                if err.raw_os_error() == Some(35)
                    || err.kind() == std::io::ErrorKind::WouldBlock
                {
                    last_error = Some(err);
                    sleep(Duration::from_millis(250 * (attempt + 1) as u64));
                    continue;
                }
                return Err(Box::new(err));
            }
        }
    }

    if let Some(err) = last_error {
        for attempt in 0..3 {
            if let Ok(output) = Command::new("cat").arg(path).output() {
                if !output.stdout.is_empty() {
                    let content = String::from_utf8_lossy(&output.stdout).to_string();
                    return Ok(content);
                }
            }
            sleep(Duration::from_millis(250 * (attempt + 1) as u64));
        }
        return Err(Box::new(err));
    }

    Err("Failed to read package.xml".into())
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
