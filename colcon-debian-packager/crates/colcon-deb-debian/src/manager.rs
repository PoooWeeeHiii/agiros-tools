//! Debian directory manager

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::process::Command;

use colcon_deb_core::Package;
use serde::{Deserialize, Serialize};
use tracing::{debug, info, warn};
use walkdir::WalkDir;

use crate::dependency::{DependencyMapper, RosDistro};
use crate::error::{DebianError, Result};
use crate::repository::{RepositoryGenerator, RepositoryMetadata};
use crate::validation::{DebianValidator, ValidationResult};
use crate::version::VersionHandler;

/// Manages Debian directories and packaging operations
#[derive(Debug)]
pub struct DebianManager {
    /// Directory containing custom debian directories
    debian_dirs: PathBuf,
    /// Version handler for ROS to Debian conversion
    version_handler: VersionHandler,
    /// Dependency mapper
    dependency_mapper: DependencyMapper,
    /// Debian directory validator
    validator: DebianValidator,
    /// Whether to use strict validation
    strict_validation: bool,
}

impl DebianManager {
    /// Create a new Debian manager
    pub fn new(debian_dirs: impl Into<PathBuf>) -> Self {
        Self {
            debian_dirs: debian_dirs.into(),
            version_handler: VersionHandler::new(),
            dependency_mapper: DependencyMapper::for_loong(), // Default to loong
            validator: DebianValidator::new(false),
            strict_validation: false,
        }
    }

    /// Create a new Debian manager with ROS distribution
    pub fn with_ros_distro(debian_dirs: impl Into<PathBuf>, ros_distro: RosDistro) -> Self {
        Self {
            debian_dirs: debian_dirs.into(),
            version_handler: VersionHandler::new(),
            dependency_mapper: DependencyMapper::new(ros_distro),
            validator: DebianValidator::new(false),
            strict_validation: false,
        }
    }

    /// Set strict validation mode
    pub fn with_strict_validation(mut self, strict: bool) -> Self {
        self.strict_validation = strict;
        self.validator = DebianValidator::new(strict);
        self
    }

    /// Check if a package has a custom Debian directory
    pub fn has_custom_debian(&self, package_name: &str) -> bool {
        self.get_custom_debian_path(package_name).exists()
    }

    /// Get path to custom debian directory for a package
    pub fn get_custom_debian_path(&self, package_name: &str) -> PathBuf {
        self.debian_dirs.join(package_name).join("debian")
    }

    /// List all packages with custom debian directories
    pub fn list_custom_packages(&self) -> Result<Vec<String>> {
        let mut packages = Vec::new();

        if !self.debian_dirs.exists() {
            debug!("Debian directories path does not exist: {}", self.debian_dirs.display());
            return Ok(packages);
        }

        for entry in WalkDir::new(&self.debian_dirs)
            .min_depth(1)
            .max_depth(2)
            .into_iter()
            .filter_map(|e| e.ok())
        {
            let path = entry.path();
            if path.is_dir() && path.file_name() == Some(std::ffi::OsStr::new("debian")) {
                if let Some(parent) = path.parent() {
                    if let Some(package_name) = parent.file_name() {
                        packages.push(package_name.to_string_lossy().to_string());
                    }
                }
            }
        }

        packages.sort();
        Ok(packages)
    }

    /// Validate debian directory for a package
    pub fn validate_debian_directory(&self, package_name: &str) -> Result<ValidationResult> {
        let debian_path = self.get_custom_debian_path(package_name);
        self.validator
            .validate_debian_directory(&debian_path, package_name)
    }

    /// Validate all custom debian directories
    pub fn validate_all_debian_directories(&self) -> Result<HashMap<String, ValidationResult>> {
        let packages = self.list_custom_packages()?;
        let mut results = HashMap::new();

        for package in packages {
            match self.validate_debian_directory(&package) {
                Ok(result) => {
                    results.insert(package, result);
                }
                Err(e) => {
                    warn!("Failed to validate debian directory for {}: {}", package, e);
                    // Continue with other packages in non-strict mode
                    if self.strict_validation {
                        return Err(e);
                    }
                }
            }
        }

        Ok(results)
    }

    /// Convert ROS version to Debian version
    pub fn convert_version(&self, ros_version: &str, debian_revision: u32) -> Result<String> {
        self.version_handler
            .ros_to_debian(ros_version, debian_revision)
    }

    /// Map ROS dependencies to Debian package names
    pub fn map_dependencies(&self, ros_dependencies: &[String]) -> Result<Vec<String>> {
        self.dependency_mapper
            .map_system_dependencies(ros_dependencies)
    }

    /// Prepare debian directory for a package
    pub async fn prepare_debian_directory(
        &self,
        package: &Package,
        target_dir: &Path,
        use_bloom: bool,
    ) -> Result<DebianPreparationResult> {
        info!("Preparing debian directory for package: {}", package.name);

        let custom_debian_path = self.get_custom_debian_path(&package.name);
        let target_debian_path = target_dir.join("debian");

        let mut result = DebianPreparationResult {
            package_name: package.name.clone(),
            used_custom: false,
            used_bloom: false,
            debian_path: target_debian_path.clone(),
            validation_result: None,
        };

        // Check if custom debian directory exists
        if custom_debian_path.exists() {
            info!("Using custom debian directory for {}", package.name);
            self.copy_debian_directory(&custom_debian_path, &target_debian_path)?;
            result.used_custom = true;

            // Validate custom directory
            result.validation_result = Some(
                self.validator
                    .validate_debian_directory(&target_debian_path, &package.name)?,
            );
        } else if use_bloom {
            info!("Generating debian directory with bloom-generate for {}", package.name);
            self.generate_with_bloom(package, target_dir).await?;
            result.used_bloom = true;

            // Validate generated directory
            result.validation_result = Some(
                self.validator
                    .validate_debian_directory(&target_debian_path, &package.name)?,
            );
        } else {
            return Err(DebianError::missing_required_file(
                &package.name,
                "debian directory (no custom directory and bloom disabled)",
            ));
        }

        Ok(result)
    }

    /// Copy debian directory
    fn copy_debian_directory(&self, source: &Path, target: &Path) -> Result<()> {
        if target.exists() {
            std::fs::remove_dir_all(target)?;
        }

        Self::copy_directory_recursive(source, target)?;
        debug!("Copied debian directory from {} to {}", source.display(), target.display());
        Ok(())
    }

    /// Copy directory recursively
    fn copy_directory_recursive(source: &Path, target: &Path) -> Result<()> {
        std::fs::create_dir_all(target)?;

        for entry in std::fs::read_dir(source)? {
            let entry = entry?;
            let source_path = entry.path();
            let target_path = target.join(entry.file_name());

            if source_path.is_dir() {
                Self::copy_directory_recursive(&source_path, &target_path)?;
            } else {
                std::fs::copy(&source_path, &target_path)?;
            }
        }

        Ok(())
    }

    /// Generate debian directory using bloom-generate
    async fn generate_with_bloom(&self, package: &Package, target_dir: &Path) -> Result<()> {
        let debian_version = self.convert_version(&package.version, 1)?;

        // Build bloom-generate command
        let mut cmd = Command::new("bloom-generate");
        cmd.arg("debian")
            .arg("--package-name")
            .arg(&package.name)
            .arg("--package-version")
            .arg(&debian_version)
            .arg("--ros-distro")
            .arg(self.dependency_mapper.ros_distro_name())
            .arg("--output-dir")
            .arg(target_dir)
            .current_dir(&package.path);

        // Add maintainer information if available
        if !package.maintainers.is_empty() {
            let maintainer = &package.maintainers[0];
            cmd.arg("--maintainer")
                .arg(format!("{} <{}>", maintainer.name, maintainer.email));
        }

        debug!("Running bloom-generate command: {cmd:?}");

        let output = cmd.output().map_err(|e| {
            DebianError::bloom_generate_failed(
                &package.name,
                format!("Failed to execute bloom-generate: {e}"),
            )
        })?;

        if !output.status.success() {
            let stderr = String::from_utf8_lossy(&output.stderr);
            return Err(DebianError::bloom_generate_failed(
                &package.name,
                format!("bloom-generate failed: {stderr}"),
            ));
        }

        // Post-process debian/control to enforce agiros naming
        self.post_process_control_file(target_dir, &self.dependency_mapper.ros_distro_name())?;

        info!("Successfully generated debian directory for {} using bloom", package.name);
        Ok(())
    }

    /// Post-process debian/control to replace ros- prefix with agiros-
    fn post_process_control_file(&self, target_dir: &Path, distro: &str) -> Result<()> {
        let control_path = target_dir.join("debian/control");
        if !control_path.exists() {
            return Ok(());
        }

        let content = std::fs::read_to_string(&control_path)?;
        let ros_prefix = format!("ros-{}-", distro);
        let agiros_prefix = format!("agiros-{}-", distro);

        // Replace Package: ros-distro-name and Source: ros-distro-name
        let new_content = content
            .replace(&format!("Package: {}", ros_prefix), &format!("Package: {}", agiros_prefix))
            .replace(&format!("Source: {}", ros_prefix), &format!("Source: {}", agiros_prefix));

        std::fs::write(control_path, new_content)?;
        debug!("Post-processed debian/control to use agiros naming");
        Ok(())
    }

    /// Generate APT repository from .deb files
    pub async fn generate_repository(
        &self,
        deb_files: &[PathBuf],
        repository_root: &Path,
        architecture: &str,
    ) -> Result<RepositoryMetadata> {
        let generator = RepositoryGenerator::for_ros(
            repository_root.to_path_buf(),
            self.dependency_mapper.ros_distro_name(),
            architecture.to_string(),
        );

        generator.generate_repository(deb_files).await
    }

    /// Get summary of all custom packages and their status
    pub fn get_package_summary(&self) -> Result<PackageSummary> {
        let packages = self.list_custom_packages()?;
        let mut valid_packages = Vec::new();
        let mut invalid_packages = Vec::new();
        let mut total_warnings = 0;

        for package in packages {
            match self.validate_debian_directory(&package) {
                Ok(validation) => {
                    total_warnings += validation.warnings.len();
                    if validation.is_valid {
                        valid_packages.push(package);
                    } else {
                        invalid_packages.push(package);
                    }
                }
                Err(_) => {
                    invalid_packages.push(package);
                }
            }
        }

        Ok(PackageSummary {
            total_packages: valid_packages.len() + invalid_packages.len(),
            valid_packages,
            invalid_packages,
            total_warnings,
        })
    }

    /// Add custom dependency mapping
    pub fn add_dependency_mapping(&mut self, ros_name: String, debian_name: String) {
        self.dependency_mapper
            .add_custom_mapping(ros_name, debian_name);
    }

    /// Get ROS distribution name
    pub fn ros_distro_name(&self) -> &str {
        self.dependency_mapper.ros_distro_name()
    }
}

/// Result of debian directory preparation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DebianPreparationResult {
    /// Package name
    pub package_name: String,
    /// Whether custom debian directory was used
    pub used_custom: bool,
    /// Whether bloom-generate was used
    pub used_bloom: bool,
    /// Path to prepared debian directory
    pub debian_path: PathBuf,
    /// Validation result
    pub validation_result: Option<ValidationResult>,
}

/// Summary of custom packages
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PackageSummary {
    /// Total number of packages
    pub total_packages: usize,
    /// Valid packages
    pub valid_packages: Vec<String>,
    /// Invalid packages
    pub invalid_packages: Vec<String>,
    /// Total number of warnings
    pub total_warnings: usize,
}

#[cfg(test)]
mod tests {
    use colcon_deb_core::package::{BuildType, Dependencies, Maintainer};
    use tempfile::TempDir;

    use super::*;

    #[allow(dead_code)]
    fn create_test_package() -> Package {
        Package {
            name: "test_package".to_string(),
            path: PathBuf::from("/test/path"),
            version: "1.0.0".to_string(),
            description: "Test package".to_string(),
            license: "MIT".to_string(),
            build_type: BuildType::Cmake,
            dependencies: Dependencies::default(),
            maintainers: vec![Maintainer {
                name: "Test Maintainer".to_string(),
                email: "test@example.com".to_string(),
            }],
        }
    }

    #[test]
    fn test_debian_manager_creation() {
        let temp_dir = TempDir::new().unwrap();
        let manager = DebianManager::new(temp_dir.path());
        assert!(!manager.has_custom_debian("nonexistent"));
    }

    #[test]
    fn test_custom_debian_detection() {
        let temp_dir = TempDir::new().unwrap();
        let manager = DebianManager::new(temp_dir.path());

        // Create custom debian directory
        let custom_debian = temp_dir.path().join("test_package").join("debian");
        std::fs::create_dir_all(&custom_debian).unwrap();
        std::fs::write(custom_debian.join("control"), "test control").unwrap();

        assert!(manager.has_custom_debian("test_package"));
        assert!(!manager.has_custom_debian("nonexistent"));
    }

    #[test]
    fn test_list_custom_packages() {
        let temp_dir = TempDir::new().unwrap();
        let manager = DebianManager::new(temp_dir.path());

        // Create multiple custom debian directories
        for package in ["pkg1", "pkg2", "pkg3"] {
            let debian_dir = temp_dir.path().join(package).join("debian");
            std::fs::create_dir_all(&debian_dir).unwrap();
            std::fs::write(debian_dir.join("control"), "test").unwrap();
        }

        let packages = manager.list_custom_packages().unwrap();
        assert_eq!(packages.len(), 3);
        assert!(packages.contains(&"pkg1".to_string()));
        assert!(packages.contains(&"pkg2".to_string()));
        assert!(packages.contains(&"pkg3".to_string()));
    }

    #[test]
    fn test_version_conversion() {
        let manager = DebianManager::new("/tmp");

        assert_eq!(manager.convert_version("1.2.3", 1).unwrap(), "1.2.3-1");

        assert_eq!(manager.convert_version("1.2.3-alpha1", 1).unwrap(), "1.2.3~alpha1-1");
    }

    #[test]
    fn test_dependency_mapping() {
        let manager = DebianManager::new("/tmp");

        let ros_deps = vec!["std_msgs".to_string(), "cmake".to_string()];
        let debian_deps = manager.map_dependencies(&ros_deps).unwrap();

        assert!(debian_deps.contains(&"agiros-loong-std-msgs".to_string()));
        assert!(debian_deps.contains(&"cmake".to_string()));
    }

    #[test]
    fn test_package_summary() {
        let temp_dir = TempDir::new().unwrap();
        let manager = DebianManager::new(temp_dir.path());

        // Create one valid and one invalid debian directory
        let valid_debian = temp_dir.path().join("valid_pkg").join("debian");
        std::fs::create_dir_all(&valid_debian).unwrap();
        std::fs::write(
            valid_debian.join("control"),
            "Package: valid\nArchitecture: any\nMaintainer: Test <test@example.com>\nDescription: \
             Test",
        )
        .unwrap();
        std::fs::write(valid_debian.join("changelog"), "test changelog").unwrap();
        std::fs::write(valid_debian.join("copyright"), "test copyright").unwrap();
        std::fs::write(valid_debian.join("rules"), "#!/usr/bin/make -f\nbuild:\nbinary:\nclean:")
            .unwrap();

        let invalid_debian = temp_dir.path().join("invalid_pkg").join("debian");
        std::fs::create_dir_all(&invalid_debian).unwrap();
        // Only create control file, missing other required files

        let summary = manager.get_package_summary().unwrap();
        assert_eq!(summary.total_packages, 2);
    }
}
