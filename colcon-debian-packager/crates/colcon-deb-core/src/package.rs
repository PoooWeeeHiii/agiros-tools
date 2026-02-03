//! ROS package representation

use std::path::PathBuf;
use std::str::FromStr;

use serde::{Deserialize, Serialize};

/// Represents a ROS package
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct Package {
    /// Package name
    pub name: String,

    /// Package version
    pub version: String,

    /// Package description
    pub description: String,

    /// Path to the package relative to workspace
    pub path: PathBuf,

    /// Package maintainers
    pub maintainers: Vec<Maintainer>,

    /// Package license
    pub license: String,

    /// Build type (ament_cmake, ament_python, cmake, etc.)
    pub build_type: BuildType,

    /// Package dependencies
    pub dependencies: Dependencies,
}

/// Package maintainer information
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct Maintainer {
    /// Maintainer name
    pub name: String,

    /// Maintainer email
    pub email: String,
}

/// Build system type
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum BuildType {
    /// Ament CMake build system (ROS 2)
    AmentCmake,

    /// Ament Python build system (ROS 2)
    AmentPython,

    /// Plain CMake
    Cmake,

    /// Unknown build system
    Unknown,
}

/// Package dependencies categorized by type
#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq, Eq)]
pub struct Dependencies {
    /// Build dependencies
    pub build: Vec<String>,

    /// Build export dependencies
    pub build_export: Vec<String>,

    /// Execution dependencies
    pub exec: Vec<String>,

    /// Test dependencies
    pub test: Vec<String>,

    /// Build tool dependencies
    pub build_tool: Vec<String>,

    /// Documentation dependencies
    pub doc: Vec<String>,
}

impl Dependencies {
    /// Get all unique dependencies across all categories
    pub fn all_unique(&self) -> Vec<String> {
        let mut deps = Vec::new();
        deps.extend(self.build.clone());
        deps.extend(self.build_export.clone());
        deps.extend(self.exec.clone());
        deps.extend(self.test.clone());
        deps.extend(self.build_tool.clone());
        deps.extend(self.doc.clone());

        deps.sort();
        deps.dedup();
        deps
    }

    /// Get runtime dependencies (exec + build_export)
    pub fn runtime(&self) -> Vec<String> {
        let mut deps = self.exec.clone();
        deps.extend(self.build_export.clone());
        deps.sort();
        deps.dedup();
        deps
    }
}

impl FromStr for BuildType {
    type Err = std::convert::Infallible;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Ok(match s.to_lowercase().as_str() {
            "ament_cmake" => Self::AmentCmake,
            "ament_python" => Self::AmentPython,
            "cmake" => Self::Cmake,
            _ => Self::Unknown,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dependencies_all_unique() {
        let deps = Dependencies {
            build: vec!["foo".to_string(), "bar".to_string()],
            exec: vec!["bar".to_string(), "baz".to_string()],
            ..Default::default()
        };

        let all = deps.all_unique();
        assert_eq!(all, vec!["bar".to_string(), "baz".to_string(), "foo".to_string()]);
    }

    #[test]
    fn test_build_type_from_str() {
        assert_eq!(BuildType::from_str("ament_cmake").unwrap(), BuildType::AmentCmake);
        assert_eq!(BuildType::from_str("AMENT_PYTHON").unwrap(), BuildType::AmentPython);
        assert_eq!(BuildType::from_str("unknown").unwrap(), BuildType::Unknown);
    }
}
