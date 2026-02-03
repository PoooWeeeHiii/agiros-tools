//! ROS package handling for Colcon Debian Packager
//!
//! This crate provides functionality for discovering and parsing
//! ROS packages in a workspace.

pub mod package_xml;
pub mod parser;
pub mod scanner;

pub use package_xml::{DependencySpec, PackageDependencies, PackageManifest, Person, Url};
pub use parser::{parse_package_manifest, parse_package_xml};
pub use scanner::{find_package_paths, scan_workspace};
