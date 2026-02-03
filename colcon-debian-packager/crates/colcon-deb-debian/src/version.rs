//! Version handling for converting ROS versions to Debian format

use regex::Regex;

use crate::error::{DebianError, Result};

/// Version handler for ROS to Debian conversion
#[derive(Debug, Clone)]
pub struct VersionHandler {
    /// Regex for parsing ROS version strings
    version_regex: Regex,
    /// Regex for pre-release versions
    prerelease_regex: Regex,
}

impl Default for VersionHandler {
    fn default() -> Self {
        Self::new()
    }
}

impl VersionHandler {
    /// Create a new version handler
    pub fn new() -> Self {
        Self {
            // Matches version strings like "1.2.3", "1.2.3-alpha1", "1.2.3.dev4"
            version_regex: Regex::new(
                r"^(\d+)\.(\d+)\.(\d+)(?:\.(\d+))?(?:[-.]?(alpha|beta|rc|dev)(\d+)?)?$",
            )
            .expect("Valid regex"),
            // Matches pre-release identifiers
            prerelease_regex: Regex::new(r"(alpha|beta|rc|dev)(\d+)?").expect("Valid regex"),
        }
    }

    /// Convert ROS version to Debian version format
    pub fn ros_to_debian(&self, ros_version: &str, debian_revision: u32) -> Result<String> {
        if ros_version.is_empty() {
            return Err(DebianError::invalid_version(ros_version, "Version cannot be empty"));
        }

        // Parse the version using regex
        let captures = self
            .version_regex
            .captures(ros_version)
            .ok_or_else(|| DebianError::invalid_version(ros_version, "Invalid version format"))?;

        let major = captures.get(1).unwrap().as_str();
        let minor = captures.get(2).unwrap().as_str();
        let patch = captures.get(3).unwrap().as_str();
        let build = captures.get(4).map(|m| m.as_str());
        let prerelease_type = captures.get(5).map(|m| m.as_str());
        let prerelease_num = captures.get(6).map(|m| m.as_str());

        // Build base version
        let mut debian_version = if let Some(build) = build {
            format!("{major}.{minor}.{patch}.{build}")
        } else {
            format!("{major}.{minor}.{patch}")
        };

        // Handle pre-release versions
        if let Some(pre_type) = prerelease_type {
            let pre_suffix = match pre_type {
                "alpha" => "~alpha",
                "beta" => "~beta",
                "rc" => "~rc",
                "dev" => "~dev",
                _ => {
                    return Err(DebianError::invalid_version(
                        ros_version,
                        format!("Unknown pre-release type: {pre_type}"),
                    ))
                }
            };

            if let Some(pre_num) = prerelease_num {
                debian_version.push_str(&format!("{pre_suffix}{pre_num}"));
            } else {
                debian_version.push_str(pre_suffix);
            }
        }

        // Add Debian revision
        debian_version.push_str(&format!("-{debian_revision}"));

        Ok(debian_version)
    }

    /// Convert Debian version back to ROS version (mainly for validation)
    pub fn debian_to_ros(&self, debian_version: &str) -> Result<String> {
        // Remove Debian revision (everything after last -)
        let version_part = debian_version.rsplit('-').nth(1).unwrap_or(debian_version);

        // Convert pre-release markers back
        let ros_version = version_part
            .replace("~alpha", "-alpha")
            .replace("~beta", "-beta")
            .replace("~rc", "-rc")
            .replace("~dev", ".dev");

        Ok(ros_version)
    }

    /// Compare two Debian versions (basic implementation)
    pub fn compare_debian_versions(&self, version1: &str, version2: &str) -> std::cmp::Ordering {
        // This is a simplified version comparison
        // In production, you might want to use a proper Debian version comparison
        // library
        version1.cmp(version2)
    }

    /// Validate version format
    pub fn validate_ros_version(&self, version: &str) -> Result<()> {
        if self.version_regex.is_match(version) {
            Ok(())
        } else {
            Err(DebianError::invalid_version(
                version,
                "Does not match expected ROS version format",
            ))
        }
    }

    /// Extract major.minor from version
    pub fn get_major_minor(&self, version: &str) -> Result<(u32, u32)> {
        let captures = self
            .version_regex
            .captures(version)
            .ok_or_else(|| DebianError::invalid_version(version, "Invalid version format"))?;

        let major =
            captures.get(1).unwrap().as_str().parse().map_err(|_| {
                DebianError::invalid_version(version, "Invalid major version number")
            })?;

        let minor =
            captures.get(2).unwrap().as_str().parse().map_err(|_| {
                DebianError::invalid_version(version, "Invalid minor version number")
            })?;

        Ok((major, minor))
    }

    /// Check if version is a pre-release
    pub fn is_prerelease(&self, version: &str) -> bool {
        self.prerelease_regex.is_match(version)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ros_to_debian_basic() {
        let handler = VersionHandler::new();

        // Basic version
        assert_eq!(handler.ros_to_debian("1.2.3", 1).unwrap(), "1.2.3-1");

        // Version with build number
        assert_eq!(handler.ros_to_debian("1.2.3.4", 1).unwrap(), "1.2.3.4-1");
    }

    #[test]
    fn test_ros_to_debian_prerelease() {
        let handler = VersionHandler::new();

        // Alpha version
        assert_eq!(handler.ros_to_debian("1.2.3-alpha1", 1).unwrap(), "1.2.3~alpha1-1");

        // Beta version
        assert_eq!(handler.ros_to_debian("1.2.3-beta2", 1).unwrap(), "1.2.3~beta2-1");

        // RC version
        assert_eq!(handler.ros_to_debian("1.2.3-rc1", 1).unwrap(), "1.2.3~rc1-1");

        // Dev version
        assert_eq!(handler.ros_to_debian("1.2.3.dev4", 1).unwrap(), "1.2.3~dev4-1");
    }

    #[test]
    fn test_debian_revision() {
        let handler = VersionHandler::new();

        // Different revisions
        assert_eq!(handler.ros_to_debian("1.2.3", 1).unwrap(), "1.2.3-1");

        assert_eq!(handler.ros_to_debian("1.2.3", 5).unwrap(), "1.2.3-5");
    }

    #[test]
    fn test_invalid_versions() {
        let handler = VersionHandler::new();

        // Empty version
        assert!(handler.ros_to_debian("", 1).is_err());

        // Invalid format
        assert!(handler.ros_to_debian("not.a.version", 1).is_err());

        // Invalid characters
        assert!(handler.ros_to_debian("1.2.3-gamma", 1).is_err());
    }

    #[test]
    fn test_debian_to_ros() {
        let handler = VersionHandler::new();

        assert_eq!(handler.debian_to_ros("1.2.3-1").unwrap(), "1.2.3");

        assert_eq!(handler.debian_to_ros("1.2.3~alpha1-1").unwrap(), "1.2.3-alpha1");

        assert_eq!(handler.debian_to_ros("1.2.3~dev4-1").unwrap(), "1.2.3.dev4");
    }

    #[test]
    fn test_version_validation() {
        let handler = VersionHandler::new();

        // Valid versions
        assert!(handler.validate_ros_version("1.2.3").is_ok());
        assert!(handler.validate_ros_version("1.2.3.4").is_ok());
        assert!(handler.validate_ros_version("1.2.3-alpha1").is_ok());

        // Invalid versions
        assert!(handler.validate_ros_version("").is_err());
        assert!(handler.validate_ros_version("1.2").is_err());
        assert!(handler.validate_ros_version("1.2.3.4.5").is_err());
    }

    #[test]
    fn test_major_minor_extraction() {
        let handler = VersionHandler::new();

        assert_eq!(handler.get_major_minor("1.2.3").unwrap(), (1, 2));
        assert_eq!(handler.get_major_minor("10.25.3").unwrap(), (10, 25));
        assert_eq!(handler.get_major_minor("1.2.3-alpha1").unwrap(), (1, 2));
    }

    #[test]
    fn test_prerelease_detection() {
        let handler = VersionHandler::new();

        assert!(handler.is_prerelease("1.2.3-alpha1"));
        assert!(handler.is_prerelease("1.2.3-beta2"));
        assert!(handler.is_prerelease("1.2.3.dev4"));
        assert!(!handler.is_prerelease("1.2.3"));
    }
}
