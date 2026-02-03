//! Dependency mapping for converting ROS dependencies to Debian format

use std::collections::HashMap;

use serde::{Deserialize, Serialize};

use crate::error::{DebianError, Result};

/// ROS distribution information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RosDistro {
    /// Distribution name (loong, iron, etc.)
    pub name: String,
    /// Ubuntu codename
    pub ubuntu_codename: String,
    /// ROS version (2 for ROS 2)
    pub ros_version: u32,
}

impl RosDistro {
    /// Create ROS loong distribution
    pub fn loong() -> Self {
        Self {
            name: "loong".to_string(),
            ubuntu_codename: "jammy".to_string(),
            ros_version: 2,
        }
    }

    /// Create ROS Iron distribution
    pub fn iron() -> Self {
        Self {
            name: "iron".to_string(),
            ubuntu_codename: "jammy".to_string(),
            ros_version: 2,
        }
    }

    /// Create ROS pixiu distribution
    pub fn pixiu() -> Self {
        Self {
            name: "pixiu".to_string(),
            ubuntu_codename: "noble".to_string(),
            ros_version: 2,
        }
    }
}

/// Dependency mapper for ROS to Debian package names
#[derive(Debug)]
pub struct DependencyMapper {
    /// ROS distribution information
    ros_distro: RosDistro,
    /// System package mappings (agirosdep-style)
    system_mappings: HashMap<String, String>,
    /// Custom package mappings
    custom_mappings: HashMap<String, String>,
}

impl DependencyMapper {
    /// Create a new dependency mapper
    pub fn new(ros_distro: RosDistro) -> Self {
        Self {
            ros_distro,
            system_mappings: Self::default_system_mappings(),
            custom_mappings: HashMap::new(),
        }
    }

    /// Create mapper for ROS loong
    pub fn for_loong() -> Self {
        Self::new(RosDistro::loong())
    }

    /// Create mapper for ROS Iron
    pub fn for_iron() -> Self {
        Self::new(RosDistro::iron())
    }

    /// Create mapper for ROS pixiu
    pub fn for_pixiu() -> Self {
        Self::new(RosDistro::pixiu())
    }

    /// Add custom mapping
    pub fn add_custom_mapping(&mut self, ros_name: String, debian_name: String) {
        self.custom_mappings.insert(ros_name, debian_name);
    }

    /// Map ROS package name to Debian package name
    pub fn map_package(&self, ros_package: &str) -> Result<String> {
        // Check custom mappings first
        if let Some(debian_name) = self.custom_mappings.get(ros_package) {
            return Ok(debian_name.clone());
        }

        // Check system mappings
        if let Some(debian_name) = self.system_mappings.get(ros_package) {
            return Ok(debian_name.clone());
        }

        // Default ROS package mapping
        self.map_ros_package(ros_package)
    }

    /// Map ROS package to ros-{distro}-{package} format
    fn map_ros_package(&self, ros_package: &str) -> Result<String> {
        // Validate package name
        if !self.is_valid_ros_package_name(ros_package) {
            return Err(DebianError::dependency_mapping_failed(
                ros_package,
                "Invalid ROS package name",
            ));
        }

        // Convert to Debian package name format
        let debian_name = self.ros_package_to_debian(ros_package);
        Ok(debian_name)
    }

    /// Convert ROS package name to Debian format
    fn ros_package_to_debian(&self, ros_package: &str) -> String {
        // Convert underscores to hyphens for Debian naming
        let debian_package = ros_package.replace('_', "-");
        format!("agiros-{}-{}", self.ros_distro.name, debian_package)
    }

    /// Check if ROS package name is valid
    fn is_valid_ros_package_name(&self, name: &str) -> bool {
        if name.is_empty() {
            return false;
        }

        // ROS package names should only contain lowercase letters, numbers, and
        // underscores and should not start with a number
        name.chars()
            .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '_')
            && !name.chars().next().unwrap().is_ascii_digit()
    }

    /// Map system dependencies to Debian package names
    pub fn map_system_dependencies(&self, dependencies: &[String]) -> Result<Vec<String>> {
        dependencies
            .iter()
            .map(|dep| self.map_package(dep))
            .collect()
    }

    /// Get default system mappings (similar to agirosdep)
    fn default_system_mappings() -> HashMap<String, String> {
        let mut mappings = HashMap::new();

        // Common system dependencies
        mappings.insert("cmake".to_string(), "cmake".to_string());
        mappings.insert("pkg-config".to_string(), "pkg-config".to_string());
        mappings.insert("python3".to_string(), "python3".to_string());
        mappings.insert("python3-dev".to_string(), "python3-dev".to_string());
        mappings.insert("python3-numpy".to_string(), "python3-numpy".to_string());
        mappings.insert("python3-setuptools".to_string(), "python3-setuptools".to_string());
        mappings.insert("python3-pip".to_string(), "python3-pip".to_string());

        // C++ build tools
        mappings.insert("build-essential".to_string(), "build-essential".to_string());
        mappings.insert("gcc".to_string(), "gcc".to_string());
        mappings.insert("g++".to_string(), "g++".to_string());

        // Common libraries
        mappings.insert("libeigen3-dev".to_string(), "libeigen3-dev".to_string());
        mappings.insert("libboost-all-dev".to_string(), "libboost-all-dev".to_string());
        mappings.insert("libopencv-dev".to_string(), "libopencv-dev".to_string());
        mappings.insert("libpcl-dev".to_string(), "libpcl-dev".to_string());

        // Version control
        mappings.insert("git".to_string(), "git".to_string());

        mappings
    }

    /// Get ROS distribution name
    pub fn ros_distro_name(&self) -> &str {
        &self.ros_distro.name
    }

    /// Generate dependency list for debian/control
    pub fn generate_control_dependencies(
        &self,
        build_deps: &[String],
        exec_deps: &[String],
    ) -> Result<ControlDependencies> {
        let build_depends = self.map_system_dependencies(build_deps)?;
        let depends = self.map_system_dependencies(exec_deps)?;

        Ok(ControlDependencies {
            build_depends,
            depends,
            build_depends_indep: Vec::new(),
            depends_indep: Vec::new(),
        })
    }
}

/// Dependencies for debian/control file
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ControlDependencies {
    /// Build dependencies (Build-Depends)
    pub build_depends: Vec<String>,
    /// Runtime dependencies (Depends)
    pub depends: Vec<String>,
    /// Architecture-independent build dependencies (Build-Depends-Indep)
    pub build_depends_indep: Vec<String>,
    /// Architecture-independent runtime dependencies (Depends)
    pub depends_indep: Vec<String>,
}

impl ControlDependencies {
    /// Create empty dependencies
    pub fn new() -> Self {
        Self {
            build_depends: Vec::new(),
            depends: Vec::new(),
            build_depends_indep: Vec::new(),
            depends_indep: Vec::new(),
        }
    }

    /// Add build dependency
    pub fn add_build_depend(&mut self, package: String) {
        if !self.build_depends.contains(&package) {
            self.build_depends.push(package);
        }
    }

    /// Add runtime dependency
    pub fn add_depend(&mut self, package: String) {
        if !self.depends.contains(&package) {
            self.depends.push(package);
        }
    }

    /// Format as debian/control entries
    pub fn format_for_control(&self) -> String {
        let mut lines = Vec::new();

        if !self.build_depends.is_empty() {
            lines.push(format!("Build-Depends: {}", self.build_depends.join(", ")));
        }

        if !self.build_depends_indep.is_empty() {
            lines.push(format!("Build-Depends-Indep: {}", self.build_depends_indep.join(", ")));
        }

        if !self.depends.is_empty() {
            lines.push(format!("Depends: {}", self.depends.join(", ")));
        }

        if !self.depends_indep.is_empty() {
            lines.push(format!("Depends: {}", self.depends_indep.join(", ")));
        }

        lines.join("\n")
    }
}

impl Default for ControlDependencies {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ros_package_mapping() {
        let mapper = DependencyMapper::for_loong();

        assert_eq!(mapper.map_package("std_msgs").unwrap(), "agiros-loong-std-msgs");

        assert_eq!(mapper.map_package("geometry_msgs").unwrap(), "agiros-loong-geometry-msgs");

        assert_eq!(mapper.map_package("sensor_msgs").unwrap(), "agiros-loong-sensor-msgs");
    }

    #[test]
    fn test_system_package_mapping() {
        let mapper = DependencyMapper::for_loong();

        assert_eq!(mapper.map_package("cmake").unwrap(), "cmake");
        assert_eq!(mapper.map_package("python3").unwrap(), "python3");
        assert_eq!(mapper.map_package("build-essential").unwrap(), "build-essential");
    }

    #[test]
    fn test_custom_mappings() {
        let mut mapper = DependencyMapper::for_loong();
        mapper.add_custom_mapping("custom_pkg".to_string(), "my-custom-package".to_string());

        assert_eq!(mapper.map_package("custom_pkg").unwrap(), "my-custom-package");
    }

    #[test]
    fn test_invalid_package_names() {
        let mapper = DependencyMapper::for_loong();

        // Package name starting with number
        assert!(mapper.map_package("123invalid").is_err());

        // Package name with invalid characters
        assert!(mapper.map_package("invalid-name").is_err());

        // Empty package name
        assert!(mapper.map_package("").is_err());
    }

    #[test]
    fn test_control_dependencies() {
        let mapper = DependencyMapper::for_loong();

        let build_deps = vec!["cmake".to_string(), "std_msgs".to_string()];
        let exec_deps = vec!["geometry_msgs".to_string()];

        let deps = mapper
            .generate_control_dependencies(&build_deps, &exec_deps)
            .unwrap();

        assert!(deps.build_depends.contains(&"cmake".to_string()));
        assert!(deps
            .build_depends
            .contains(&"agiros-loong-std-msgs".to_string()));
        assert!(deps
            .depends
            .contains(&"agiros-loong-geometry-msgs".to_string()));
    }

    #[test]
    fn test_different_distros() {
        let loong_mapper = DependencyMapper::for_loong();
        let iron_mapper = DependencyMapper::for_iron();

        assert_eq!(loong_mapper.map_package("std_msgs").unwrap(), "agiros-loong-std-msgs");

        assert_eq!(iron_mapper.map_package("std_msgs").unwrap(), "agiros-iron-std-msgs");
    }

    #[test]
    fn test_control_dependencies_formatting() {
        let mut deps = ControlDependencies::new();
        deps.add_build_depend("cmake".to_string());
        deps.add_build_depend("agiros-loong-std-msgs".to_string());
        deps.add_depend("agiros-loong-geometry-msgs".to_string());

        let formatted = deps.format_for_control();
        assert!(formatted.contains("Build-Depends: cmake, agiros-loong-std-msgs"));
        assert!(formatted.contains("Depends: agiros-loong-geometry-msgs"));
    }
}
