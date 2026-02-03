#!/usr/bin/env rust-script
//! ```cargo
//! [dependencies]
//! which = "5.0"
//! ```

//! Check if all required tools are available in the container

use std::process;
use which::which;

fn main() {
    let required_tools = [
        ("rust-script", "Rust script runner"),
        ("bloom-generate", "Debian directory generator for ROS packages"),
        ("dpkg-buildpackage", "Debian package builder"),
        ("dpkg-scanpackages", "Debian package scanner"),
        ("apt-ftparchive", "APT repository metadata generator"),
        ("rustc", "Rust compiler"),
        ("cargo", "Rust package manager"),
        ("colcon", "AGIROS build tool"),
        ("agirosdep", "AGIROS dependency manager"),
    ];
    
    let mut missing_tools = Vec::new();
    let mut found_tools = Vec::new();
    
    for (tool, description) in &required_tools {
        match which(tool) {
            Ok(path) => {
                found_tools.push((tool, path));
            }
            Err(_) => {
                missing_tools.push((tool, description));
            }
        }
    }
    
    // Print found tools
    if !found_tools.is_empty() {
        println!("Found required tools:");
        for (tool, path) in &found_tools {
            println!("  ✓ {} -> {}", tool, path.display());
        }
    }
    
    // Check for missing tools
    if !missing_tools.is_empty() {
        eprintln!("\nMissing required tools:");
        for (tool, description) in &missing_tools {
            eprintln!("  ✗ {} - {}", tool, description);
        }
        eprintln!("\nPlease install the missing tools before proceeding.");
        process::exit(1);
    }
    
    println!("\nAll required tools are available!");
    
    // Check ROS environment
    match std::env::var("ROS_DISTRO") {
        Ok(distro) => println!("AGIROS distribution: {}", distro),
        Err(_) => {
            eprintln!("\nWarning: ROS_DISTRO environment variable not set");
            eprintln!("Make sure to source the ROS setup script");
        }
    }
}