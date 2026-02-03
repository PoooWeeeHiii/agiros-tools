//! Architecture detection and compatibility checking
//!
//! This module provides functionality to detect host and container
//! architectures and determine if emulation is needed.

use std::env;

use bollard::Docker;
use serde::{Deserialize, Serialize};

use crate::error::{DockerError, Result};

/// Architecture information for a system
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ArchitectureInfo {
    /// The architecture name (e.g., "amd64", "arm64", "armv7")
    pub arch: String,
    /// Alternative names for this architecture
    pub aliases: Vec<String>,
    /// Whether this is a 64-bit architecture
    pub is_64bit: bool,
}

impl ArchitectureInfo {
    /// Create architecture info from a string
    pub fn from_arch(arch: &str) -> Self {
        match arch.to_lowercase().as_str() {
            "x86_64" | "amd64" => Self {
                arch: "amd64".to_string(),
                aliases: vec!["x86_64".to_string()],
                is_64bit: true,
            },
            "aarch64" | "arm64" => Self {
                arch: "arm64".to_string(),
                aliases: vec!["aarch64".to_string()],
                is_64bit: true,
            },
            "armv7l" | "armv7" | "armhf" => Self {
                arch: "armv7".to_string(),
                aliases: vec!["armv7l".to_string(), "armhf".to_string()],
                is_64bit: false,
            },
            "i386" | "i686" => Self {
                arch: "i386".to_string(),
                aliases: vec!["i686".to_string()],
                is_64bit: false,
            },
            "ppc64le" => Self { arch: "ppc64le".to_string(), aliases: vec![], is_64bit: true },
            "s390x" => Self { arch: "s390x".to_string(), aliases: vec![], is_64bit: true },
            _ => Self {
                arch: arch.to_string(),
                aliases: vec![],
                is_64bit: arch.contains("64"),
            },
        }
    }

    /// Check if this architecture is compatible with another
    pub fn is_compatible_with(&self, other: &ArchitectureInfo) -> bool {
        // Same architecture is always compatible
        if self.arch == other.arch {
            return true;
        }

        // Check aliases
        if self.aliases.contains(&other.arch) || other.aliases.contains(&self.arch) {
            return true;
        }

        // Check specific compatibility rules
        match (self.arch.as_str(), other.arch.as_str()) {
            // 64-bit x86 can run 32-bit x86
            ("amd64", "i386") => true,
            // 64-bit ARM can run 32-bit ARM (with proper kernel support)
            ("arm64", "armv7") => true,
            _ => false,
        }
    }

    /// Check if emulation is needed to run another architecture
    pub fn needs_emulation(&self, other: &ArchitectureInfo) -> bool {
        !self.is_compatible_with(other)
    }
}

/// Detect the host architecture
pub fn detect_host_architecture() -> ArchitectureInfo {
    let arch = env::consts::ARCH;
    ArchitectureInfo::from_arch(arch)
}

/// Get architecture information from a Docker image
pub async fn get_image_architecture(docker: &Docker, image: &str) -> Result<ArchitectureInfo> {
    let image_info = docker
        .inspect_image(image)
        .await
        .map_err(DockerError::Client)?;

    let arch = image_info.architecture.as_deref().unwrap_or("unknown");

    Ok(ArchitectureInfo::from_arch(arch))
}

/// Check architecture compatibility between host and image
pub async fn check_architecture_compatibility(
    docker: &Docker,
    image: &str,
) -> Result<ArchitectureCompatibility> {
    let host_arch = detect_host_architecture();
    let image_arch = get_image_architecture(docker, image).await?;

    let needs_emulation = host_arch.needs_emulation(&image_arch);
    let emulation_available = if needs_emulation {
        check_emulation_available(&image_arch).await
    } else {
        false
    };

    Ok(ArchitectureCompatibility {
        host: host_arch,
        image: image_arch,
        needs_emulation,
        emulation_available,
        warnings: generate_compatibility_warnings(needs_emulation, emulation_available),
    })
}

/// Architecture compatibility result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ArchitectureCompatibility {
    /// Host architecture
    pub host: ArchitectureInfo,
    /// Image architecture
    pub image: ArchitectureInfo,
    /// Whether emulation is needed
    pub needs_emulation: bool,
    /// Whether emulation is available (e.g., via QEMU)
    pub emulation_available: bool,
    /// Any warnings about compatibility
    pub warnings: Vec<String>,
}

impl ArchitectureCompatibility {
    /// Check if the container can run
    pub fn can_run(&self) -> bool {
        !self.needs_emulation || self.emulation_available
    }

    /// Get a summary message about compatibility
    pub fn summary(&self) -> String {
        if !self.needs_emulation {
            format!(
                "Native execution: {} host can run {} containers",
                self.host.arch, self.image.arch
            )
        } else if self.emulation_available {
            format!(
                "Emulation required: {} host running {} containers (QEMU available)",
                self.host.arch, self.image.arch
            )
        } else {
            format!(
                "Cannot run: {} host cannot run {} containers (emulation not available)",
                self.host.arch, self.image.arch
            )
        }
    }
}

/// Check if emulation is available for a given architecture
async fn check_emulation_available(target_arch: &ArchitectureInfo) -> bool {
    // Check for QEMU user static binaries
    let qemu_binary = format!("/usr/bin/qemu-{}-static", get_qemu_arch(&target_arch.arch));
    std::path::Path::new(&qemu_binary).exists()
}

/// Get the QEMU architecture name
fn get_qemu_arch(arch: &str) -> &str {
    match arch {
        "arm64" | "aarch64" => "aarch64",
        "armv7" | "armv7l" | "armhf" => "arm",
        "i386" | "i686" => "i386",
        "amd64" | "x86_64" => "x86_64",
        "ppc64le" => "ppc64le",
        "s390x" => "s390x",
        _ => arch,
    }
}

/// Generate compatibility warnings
fn generate_compatibility_warnings(
    needs_emulation: bool,
    emulation_available: bool,
) -> Vec<String> {
    let mut warnings = Vec::new();

    if needs_emulation {
        if emulation_available {
            warnings.push(
                "Container will run under emulation, which may be significantly slower".to_string(),
            );
            warnings.push(
                "Consider using native architecture images for better performance".to_string(),
            );
        } else {
            warnings.push(
                "Container architecture is not compatible with host and emulation is not available"
                    .to_string(),
            );
            warnings.push(
                "Install qemu-user-static or use containers matching your host architecture"
                    .to_string(),
            );
        }
    }

    warnings
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_architecture_detection() {
        let amd64 = ArchitectureInfo::from_arch("x86_64");
        assert_eq!(amd64.arch, "amd64");
        assert!(amd64.aliases.contains(&"x86_64".to_string()));
        assert!(amd64.is_64bit);

        let arm64 = ArchitectureInfo::from_arch("aarch64");
        assert_eq!(arm64.arch, "arm64");
        assert!(arm64.aliases.contains(&"aarch64".to_string()));
        assert!(arm64.is_64bit);
    }

    #[test]
    fn test_compatibility() {
        let amd64 = ArchitectureInfo::from_arch("amd64");
        let i386 = ArchitectureInfo::from_arch("i386");
        let arm64 = ArchitectureInfo::from_arch("arm64");

        // AMD64 can run i386
        assert!(amd64.is_compatible_with(&i386));
        assert!(!amd64.needs_emulation(&i386));

        // AMD64 cannot run ARM64 without emulation
        assert!(!amd64.is_compatible_with(&arm64));
        assert!(amd64.needs_emulation(&arm64));

        // Same architecture is compatible
        assert!(amd64.is_compatible_with(&amd64));
    }
}
