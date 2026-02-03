//! Debian packaging support for Colcon Debian Packager
//!
//! This crate provides comprehensive functionality for managing Debian package
//! directories, validating package structure, and generating APT repositories
//! for ROS packages.
//!
//! # Features
//!
//! - **Debian Directory Management**: Validate and prepare debian/ directories
//! - **Version Conversion**: Convert ROS versions to Debian package format
//! - **Dependency Mapping**: Map ROS package dependencies to Debian packages
//! - **Repository Generation**: Create APT repositories with proper metadata
//! - **Package Validation**: Ensure debian directories meet standards
//!
//! # Example Usage
//!
//! ```rust,no_run
//! use std::path::PathBuf;
//!
//! # async fn example() -> Result<(), Box<dyn std::error::Error>> {
//! use colcon_deb_core::package::{BuildType, Dependencies, Maintainer, Package};
//! use colcon_deb_debian::{
//!     DebianManager, DebianValidator, DependencyMapper, RepositoryGenerator, VersionHandler,
//! };
//!
//! // Create a debian manager
//! let manager = DebianManager::new("/path/to/debian/dirs");
//!
//! // Prepare debian directory for a package
//! let package = Package {
//!     name: "example_package".to_string(),
//!     version: "1.0.0".to_string(),
//!     path: PathBuf::from("/path/to/package"),
//!     build_type: BuildType::AmentCmake,
//!     dependencies: Dependencies {
//!         build: vec!["std_msgs".to_string()],
//!         build_export: vec![],
//!         exec: vec!["std_msgs".to_string()],
//!         test: vec![],
//!         build_tool: vec![],
//!         doc: vec![],
//!     },
//!     description: "Example ROS package".to_string(),
//!     maintainers: vec![Maintainer {
//!         name: "Maintainer".to_string(),
//!         email: "email@example.com".to_string(),
//!     }],
//!     license: "MIT".to_string(),
//! };
//!
//! let result = manager
//!     .prepare_debian_directory(
//!         &package,
//!         &PathBuf::from("/tmp/build"),
//!         true, // use bloom-generate if needed
//!     )
//!     .await?;
//!
//! println!("Preparation result: {:?}", result);
//! # Ok(())
//! # }
//! ```
//!
//! # Modules
//!
//! - [`error`]: Error types and result handling
//! - [`manager`]: High-level debian directory management
//! - [`validation`]: Debian directory structure validation
//! - [`version`]: ROS to Debian version conversion
//! - [`dependency`]: ROS to Debian dependency mapping
//! - [`repository`]: APT repository generation

pub mod dependency;
pub mod error;
pub mod manager;
pub mod repository;
pub mod validation;
pub mod version;

// Re-export commonly used types
pub use dependency::{ControlDependencies, DependencyMapper, RosDistro};
pub use error::{DebianError, Result};
pub use manager::{DebianManager, DebianPreparationResult, PackageSummary};
pub use repository::{DebPackageInfo, RepositoryGenerator, RepositoryMetadata};
pub use validation::{DebianValidator, ValidationResult};
pub use version::VersionHandler;
