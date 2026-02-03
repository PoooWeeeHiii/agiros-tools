//! Docker integration for Colcon Debian Packager
//!
//! This crate provides Docker container management for building
//! Debian packages in isolated environments.

pub mod arch;
pub mod client;
pub mod container;
pub mod error;
pub mod image;
pub mod progress;
pub mod service;
pub mod types;

pub use arch::{check_architecture_compatibility, ArchitectureInfo};
pub use client::{DockerConfig, DockerService};
pub use container::{ContainerSpec, VolumeMount};
pub use error::{DockerError, Result};
pub use image::ImageManager;
pub use progress::{Progress, ProgressEvent, ProgressParser, ProgressStream};
pub use service::{ContainerOutput, DockerServiceTrait, LogOutput};
pub use types::{
    BuildContext, ContainerInfo, ContainerState, HealthCheck, ImageInfo, NetworkMode, RegistryAuth,
    ResourceLimits,
};
