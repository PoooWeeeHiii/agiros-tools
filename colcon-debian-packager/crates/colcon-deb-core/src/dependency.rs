//! Dependency representation and version handling

use std::fmt;

use serde::{Deserialize, Serialize};

/// Represents a package dependency with optional version constraints
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct Dependency {
    /// Package name
    pub name: String,

    /// Optional version constraint
    pub version_constraint: Option<VersionConstraint>,

    /// Optional condition (e.g., "$ROS_VERSION == 2")
    pub condition: Option<String>,
}

/// Version constraint for dependencies
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum VersionConstraint {
    /// Exact version match
    Exact(String),

    /// Minimum version (>=)
    GreaterOrEqual(String),

    /// Maximum version (<=)
    LessOrEqual(String),

    /// Version range
    Range {
        min: Option<String>,
        max: Option<String>,
    },
}

impl Dependency {
    /// Create a simple dependency without version constraints
    pub fn new(name: impl Into<String>) -> Self {
        Self { name: name.into(), version_constraint: None, condition: None }
    }

    /// Create a dependency with a minimum version requirement
    pub fn with_min_version(name: impl Into<String>, version: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            version_constraint: Some(VersionConstraint::GreaterOrEqual(version.into())),
            condition: None,
        }
    }

    /// Convert to Debian package name format (e.g., agiros-loong-foo)
    pub fn to_debian_name(&self, ros_distro: &str) -> String {
        // System packages (agirosdep keys) typically have hyphens and don't follow ROS
        // naming
        if self.name.contains('-') && !self.name.starts_with("ros-") {
            // Handle agirosdep keys like "python3-numpy", "libboost-dev", etc.
            self.name.clone()
        } else {
            // ROS package names become agiros-${distro}-${name}
            format!("agiros-{}-{}", ros_distro, self.name.replace('_', "-"))
        }
    }

    /// Convert to Debian dependency string with version constraints
    pub fn to_debian_dependency(&self, ros_distro: &str) -> String {
        let package_name = self.to_debian_name(ros_distro);

        match &self.version_constraint {
            None => package_name,
            Some(VersionConstraint::Exact(v)) => format!("{package_name} (= {v})"),
            Some(VersionConstraint::GreaterOrEqual(v)) => format!("{package_name} (>= {v})"),
            Some(VersionConstraint::LessOrEqual(v)) => format!("{package_name} (<= {v})"),
            Some(VersionConstraint::Range { min, max }) => match (min, max) {
                (Some(min), Some(max)) => {
                    format!("{package_name} (>= {min}), {package_name} (<= {max})")
                }
                (Some(min), None) => format!("{package_name} (>= {min})"),
                (None, Some(max)) => format!("{package_name} (<= {max})"),
                (None, None) => package_name,
            },
        }
    }
}

impl fmt::Display for Dependency {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.name)?;
        if let Some(constraint) = &self.version_constraint {
            write!(f, " {constraint}")?;
        }
        if let Some(condition) = &self.condition {
            write!(f, " [{condition}]")?;
        }
        Ok(())
    }
}

impl fmt::Display for VersionConstraint {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Exact(v) => write!(f, "= {v}"),
            Self::GreaterOrEqual(v) => write!(f, ">= {v}"),
            Self::LessOrEqual(v) => write!(f, "<= {v}"),
            Self::Range { min, max } => match (min, max) {
                (Some(min), Some(max)) => write!(f, ">= {min} <= {max}"),
                (Some(min), None) => write!(f, ">= {min}"),
                (None, Some(max)) => write!(f, "<= {max}"),
                (None, None) => Ok(()),
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dependency_to_debian_name() {
        let dep = Dependency::new("std_msgs");
        assert_eq!(dep.to_debian_name("loong"), "agiros-loong-std-msgs");

        let dep = Dependency::new("python3-numpy");
        assert_eq!(dep.to_debian_name("loong"), "python3-numpy");
    }

    #[test]
    fn test_dependency_to_debian_string() {
        let dep = Dependency::new("rclcpp");
        assert_eq!(dep.to_debian_dependency("loong"), "agiros-loong-rclcpp");

        let dep = Dependency::with_min_version("rclcpp", "1.0.0");
        assert_eq!(dep.to_debian_dependency("loong"), "agiros-loong-rclcpp (>= 1.0.0)");
    }
}
