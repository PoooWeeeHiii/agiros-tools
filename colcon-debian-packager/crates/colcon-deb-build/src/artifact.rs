//! Build artifact collection and organization

use std::fs;
use std::io;
use std::path::{Path, PathBuf};

use tar::Archive;
use tracing::{debug, info};

use crate::{
    context::BuildContext,
    error::{BuildError, Result},
    executor::BuildExecutor,
};

/// Build artifact information
#[derive(Debug, Clone)]
pub struct BuildArtifact {
    /// Package name
    pub package_name: String,
    /// Artifact file path
    pub path: PathBuf,
    /// File size in bytes
    pub size: u64,
    /// Artifact type (e.g., "deb", "dbgsym")
    pub artifact_type: String,
    /// Architecture
    pub architecture: String,
    /// Version
    pub version: String,
}

impl BuildArtifact {
    /// Parse artifact information from filename
    pub fn from_path(path: &Path) -> Option<Self> {
        let filename = path.file_name()?.to_str()?;

        // Parse debian package filename format: package_version_arch.deb
        if filename.ends_with(".deb") {
            let parts: Vec<&str> = filename.trim_end_matches(".deb").split('_').collect();
            if parts.len() >= 3 {
                let package_name = parts[0].to_string();
                let version = parts[1].to_string();
                let architecture = parts[2].to_string();

                let artifact_type = if filename.contains("-dbgsym") {
                    "dbgsym".to_string()
                } else {
                    "deb".to_string()
                };

                let size = fs::metadata(path).ok()?.len();

                return Some(BuildArtifact {
                    package_name,
                    path: path.to_path_buf(),
                    size,
                    artifact_type,
                    architecture,
                    version,
                });
            }
        }

        None
    }

    /// Get the artifact filename
    pub fn filename(&self) -> Option<&str> {
        self.path.file_name()?.to_str()
    }
}

/// Artifact collector for organizing build outputs
pub struct ArtifactCollector {
    /// Output directory
    output_dir: PathBuf,
}

impl ArtifactCollector {
    /// Create a new artifact collector
    pub fn new(output_dir: PathBuf) -> Self {
        Self { output_dir }
    }

    /// Collect artifacts from build executor
    pub async fn collect_from_executor(
        &self,
        executor: &BuildExecutor,
        context: &BuildContext,
    ) -> Result<Vec<PathBuf>> {
        info!("Collecting build artifacts from container");

        // Get artifacts from container
        let artifacts_data = executor
            .copy_from_container("/output/artifacts.tar")
            .await?;

        // Extract artifacts
        let artifacts = self.extract_artifacts(&artifacts_data).await?;

        // Organize by package
        self.organize_artifacts(&artifacts, context)?;

        Ok(artifacts.into_iter().map(|a| a.path).collect())
    }

    /// Extract artifacts from tar archive
    async fn extract_artifacts(&self, tar_data: &[u8]) -> Result<Vec<BuildArtifact>> {
        let mut artifacts = Vec::new();
        let mut archive = Archive::new(io::Cursor::new(tar_data));

        // Create temporary extraction directory
        let temp_dir = tempfile::tempdir().map_err(|e| BuildError::ArtifactCollection {
            reason: format!("Failed to create temp dir: {e}"),
        })?;

        // Extract files
        archive
            .unpack(temp_dir.path())
            .map_err(|e| BuildError::ArtifactCollection {
                reason: format!("Failed to extract artifacts: {e}"),
            })?;

        // Find all .deb files
        for entry in walkdir::WalkDir::new(temp_dir.path()) {
            let entry = entry.map_err(|e| BuildError::ArtifactCollection {
                reason: format!("Failed to walk directory: {e}"),
            })?;

            if entry.file_type().is_file() {
                let path = entry.path();
                if path.extension().and_then(|s| s.to_str()) == Some("deb") {
                    // Copy to output directory
                    let filename =
                        path.file_name()
                            .ok_or_else(|| BuildError::ArtifactCollection {
                                reason: "Invalid artifact filename".to_string(),
                            })?;

                    let dest_path = self.output_dir.join(filename);
                    fs::copy(path, &dest_path).map_err(|e| BuildError::ArtifactCollection {
                        reason: format!("Failed to copy artifact: {e}"),
                    })?;

                    // Parse artifact info
                    if let Some(artifact) = BuildArtifact::from_path(&dest_path) {
                        debug!(
                            "Found artifact: {} ({} bytes)",
                            artifact.filename().unwrap_or("unknown"),
                            artifact.size
                        );
                        artifacts.push(artifact);
                    }
                }
            }
        }

        info!("Extracted {} artifacts", artifacts.len());
        Ok(artifacts)
    }

    /// Organize artifacts by package
    fn organize_artifacts(
        &self,
        artifacts: &[BuildArtifact],
        _context: &BuildContext,
    ) -> Result<()> {
        // Group artifacts by package
        let mut by_package: std::collections::HashMap<String, Vec<&BuildArtifact>> =
            std::collections::HashMap::new();

        for artifact in artifacts {
            by_package
                .entry(artifact.package_name.clone())
                .or_default()
                .push(artifact);
        }

        // Create package subdirectories
        for (package_name, package_artifacts) in by_package {
            let package_dir = self.output_dir.join(&package_name);

            // Always organize by package if multiple artifacts
            if package_artifacts.len() > 1 {
                // Create package directory
                fs::create_dir_all(&package_dir).map_err(|e| BuildError::ArtifactCollection {
                    reason: format!("Failed to create package directory: {e}"),
                })?;

                // Move artifacts to package directory
                for artifact in package_artifacts {
                    let filename =
                        artifact
                            .filename()
                            .ok_or_else(|| BuildError::ArtifactCollection {
                                reason: "Invalid artifact filename".to_string(),
                            })?;

                    let new_path = package_dir.join(filename);

                    if artifact.path != new_path {
                        fs::rename(&artifact.path, &new_path).map_err(|e| {
                            BuildError::ArtifactCollection {
                                reason: format!("Failed to move artifact: {e}"),
                            }
                        })?;

                        debug!("Moved {} to package directory", filename);
                    }
                }
            }
        }

        Ok(())
    }

    /// Collect artifacts from directory
    pub fn collect_from_directory(&self, dir: &Path) -> Result<Vec<BuildArtifact>> {
        let mut artifacts = Vec::new();

        for entry in walkdir::WalkDir::new(dir) {
            let entry = entry.map_err(|e| BuildError::ArtifactCollection {
                reason: format!("Failed to walk directory: {e}"),
            })?;

            if entry.file_type().is_file() {
                let path = entry.path();
                if path.extension().and_then(|s| s.to_str()) == Some("deb") {
                    if let Some(artifact) = BuildArtifact::from_path(path) {
                        artifacts.push(artifact);
                    }
                }
            }
        }

        Ok(artifacts)
    }

    /// Create artifact summary
    pub fn create_summary(&self, artifacts: &[BuildArtifact]) -> String {
        let mut summary = String::new();
        summary.push_str("Build Artifacts:\n");
        summary.push_str("================\n\n");

        // Group by package
        let mut by_package: std::collections::HashMap<String, Vec<&BuildArtifact>> =
            std::collections::HashMap::new();
        for artifact in artifacts {
            by_package
                .entry(artifact.package_name.clone())
                .or_default()
                .push(artifact);
        }

        // Format summary
        for (package, artifacts) in by_package {
            summary.push_str(&format!("Package: {package}\n"));
            for artifact in artifacts {
                let size_mb = artifact.size as f64 / 1_048_576.0;
                summary.push_str(&format!(
                    "  - {} ({:.2} MB) [{}]\n",
                    artifact.filename().unwrap_or("unknown"),
                    size_mb,
                    artifact.artifact_type
                ));
            }
            summary.push('\n');
        }

        let total_size: u64 = artifacts.iter().map(|a| a.size).sum();
        let total_size_mb = total_size as f64 / 1_048_576.0;
        summary.push_str(&format!(
            "Total: {} artifacts, {:.2} MB\n",
            artifacts.len(),
            total_size_mb
        ));

        summary
    }
}
