//! Core types and traits for Colcon Debian Packager
//!
//! This crate provides the fundamental data structures and error types
//! used throughout the colcon-deb project.

pub mod build;
pub mod dependency;
pub mod error;
pub mod package;

pub use build::{BuildResult, BuildStatus};
pub use dependency::Dependency;
pub use error::{Error, Result};
pub use package::Package;
