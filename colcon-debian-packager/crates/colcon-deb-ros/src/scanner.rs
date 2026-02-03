//! Workspace scanner

use std::path::{Path, PathBuf};

use colcon_deb_core::{Package, Result};
use walkdir::WalkDir;

use crate::parser::parse_package_xml;

/// Scan a workspace for ROS packages
pub fn scan_workspace(workspace_path: &Path) -> Result<Vec<Package>> {
    let mut packages = Vec::new();

    // Walk through the workspace looking for package.xml files
    let walker = WalkDir::new(workspace_path)
        .follow_links(true)
        .into_iter()
        .filter_entry(|e| {
            // Skip directories that contain COLCON_IGNORE
            !(e.path().is_dir() && e.path().join("COLCON_IGNORE").exists())
        });

    for entry in walker.filter_map(|e| e.ok()) {
        let path = entry.path();

        // Look for package.xml
        if path.file_name() == Some(std::ffi::OsStr::new("package.xml")) {
            match parse_package_xml(path) {
                Ok(package) => packages.push(package),
                Err(e) => {
                    // Log error but continue scanning
                    eprintln!("Warning: Failed to parse {}: {}", path.display(), e);
                }
            }
        }
    }

    // Sort packages by name for consistent ordering
    packages.sort_by(|a, b| a.name.cmp(&b.name));

    Ok(packages)
}

/// Find all package paths in a workspace (returns paths to package directories)
pub fn find_package_paths(workspace_path: &Path) -> Result<Vec<PathBuf>> {
    let mut package_paths = Vec::new();

    let walker = WalkDir::new(workspace_path)
        .follow_links(true)
        .into_iter()
        .filter_entry(|e| {
            // Skip directories that contain COLCON_IGNORE
            !(e.path().is_dir() && e.path().join("COLCON_IGNORE").exists())
        });

    for entry in walker.filter_map(|e| e.ok()) {
        let path = entry.path();

        // Look for package.xml
        if path.file_name() == Some(std::ffi::OsStr::new("package.xml")) {
            if let Some(parent) = path.parent() {
                package_paths.push(parent.to_path_buf());
            }
        }
    }

    package_paths.sort();
    Ok(package_paths)
}
