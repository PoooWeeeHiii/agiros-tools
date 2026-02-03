//! APT repository generation for Debian packages

use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

use serde::{Deserialize, Serialize};
use tracing::{debug, info, warn};

use crate::error::{DebianError, Result};

/// APT repository generator
#[derive(Debug)]
pub struct RepositoryGenerator {
    /// Repository root directory
    repository_root: PathBuf,
    /// Distribution name (e.g., "stable", "unstable")
    distribution: String,
    /// Component name (e.g., "main", "contrib")
    component: String,
    /// Architecture (e.g., "amd64", "arm64")
    architecture: String,
}

impl RepositoryGenerator {
    /// Create a new repository generator
    pub fn new(
        repository_root: PathBuf,
        distribution: String,
        component: String,
        architecture: String,
    ) -> Self {
        Self { repository_root, distribution, component, architecture }
    }

    /// Create a repository generator for ROS packages
    pub fn for_ros(repository_root: PathBuf, ros_distro: &str, architecture: String) -> Self {
        Self::new(repository_root, ros_distro.to_string(), "main".to_string(), architecture)
    }

    /// Generate complete APT repository
    pub async fn generate_repository(&self, deb_files: &[PathBuf]) -> Result<RepositoryMetadata> {
        info!("Generating APT repository at {}", self.repository_root.display());

        // Create repository directory structure
        self.create_repository_structure()?;

        // Copy .deb files to repository
        let pool_dir = self.get_pool_directory();
        self.copy_deb_files(deb_files, &pool_dir)?;

        // Generate package index
        let packages_file = self.generate_packages_index(&pool_dir).await?;

        // Generate release file
        let release_file = self.generate_release_file(&packages_file).await?;

        // Create repository metadata
        let metadata = RepositoryMetadata {
            repository_root: self.repository_root.clone(),
            distribution: self.distribution.clone(),
            component: self.component.clone(),
            architecture: self.architecture.clone(),
            packages_file,
            release_file,
            package_count: deb_files.len(),
        };

        info!("Repository generation completed with {} packages", deb_files.len());
        Ok(metadata)
    }

    /// Create repository directory structure
    fn create_repository_structure(&self) -> Result<()> {
        debug!("Creating repository directory structure");

        // Create dists/{distribution}/{component}/binary-{architecture}
        let binary_dir = self
            .repository_root
            .join("dists")
            .join(&self.distribution)
            .join(&self.component)
            .join(format!("binary-{}", self.architecture));

        fs::create_dir_all(&binary_dir)?;

        // Create pool directory
        let pool_dir = self.get_pool_directory();
        fs::create_dir_all(&pool_dir)?;

        debug!("Repository structure created");
        Ok(())
    }

    /// Get pool directory path
    fn get_pool_directory(&self) -> PathBuf {
        self.repository_root.join("pool").join(&self.component)
    }

    /// Copy .deb files to repository pool
    fn copy_deb_files(&self, deb_files: &[PathBuf], pool_dir: &Path) -> Result<()> {
        info!("Copying {} .deb files to repository", deb_files.len());

        for deb_file in deb_files {
            if !deb_file.exists() {
                warn!("Skipping non-existent file: {}", deb_file.display());
                continue;
            }

            let file_name = deb_file.file_name().ok_or_else(|| {
                DebianError::repository_generation_failed(format!(
                    "Invalid file path: {}",
                    deb_file.display()
                ))
            })?;

            let dest_path = pool_dir.join(file_name);
            fs::copy(deb_file, &dest_path)?;
            debug!("Copied {} to pool", file_name.to_string_lossy());
        }

        Ok(())
    }

    /// Generate Packages index using dpkg-scanpackages
    async fn generate_packages_index(&self, pool_dir: &Path) -> Result<PathBuf> {
        info!("Generating Packages index");

        let binary_dir = self
            .repository_root
            .join("dists")
            .join(&self.distribution)
            .join(&self.component)
            .join(format!("binary-{}", self.architecture));

        let packages_file = binary_dir.join("Packages");

        // Run dpkg-scanpackages
        let output = Command::new("dpkg-scanpackages")
            .arg("--multiversion")
            .arg(pool_dir)
            .current_dir(&self.repository_root)
            .output()
            .map_err(|e| {
                DebianError::repository_generation_failed(format!(
                    "Failed to run dpkg-scanpackages: {e}"
                ))
            })?;

        if !output.status.success() {
            return Err(DebianError::repository_generation_failed(format!(
                "dpkg-scanpackages failed: {}",
                String::from_utf8_lossy(&output.stderr)
            )));
        }

        // Write Packages file
        fs::write(&packages_file, &output.stdout)?;

        // Create compressed version
        let packages_gz = binary_dir.join("Packages.gz");
        self.compress_file(&packages_file, &packages_gz)?;

        debug!("Packages index generated at {}", packages_file.display());
        Ok(packages_file)
    }

    /// Generate Release file
    async fn generate_release_file(&self, packages_file: &Path) -> Result<PathBuf> {
        info!("Generating Release file");

        let release_dir = self.repository_root.join("dists").join(&self.distribution);

        let release_file = release_dir.join("Release");

        // Calculate checksums
        let packages_info = self.calculate_file_info(packages_file)?;
        let packages_gz_info =
            self.calculate_file_info(&packages_file.with_extension("Packages.gz"))?;

        // Generate release content
        let release_content = self.generate_release_content(&packages_info, &packages_gz_info)?;

        // Write Release file
        fs::write(&release_file, release_content)?;

        debug!("Release file generated at {}", release_file.display());
        Ok(release_file)
    }

    /// Generate Release file content
    fn generate_release_content(
        &self,
        packages_info: &FileInfo,
        packages_gz_info: &FileInfo,
    ) -> Result<String> {
        let date = chrono::Utc::now().format("%a, %d %b %Y %H:%M:%S UTC");

        let component_path = format!("{}/binary-{}", self.component, self.architecture);

        let content = format!(
            "Origin: Agiros {}\nLabel: Agiros {}\nSuite: {}\nCodename: {}\nArchitectures: \
             {}\nComponents: {}\nDescription: Agiros {} packages\nDate: {}\nMD5Sum:\n{} {} \
             {}/Packages\n{} {} {}/Packages.gz\nSHA1:\n{} {} {}/Packages\n{} {} \
             {}/Packages.gz\nSHA256:\n{} {} {}/Packages\n{} {} {}/Packages.gz\n",
            self.distribution,
            self.distribution,
            self.distribution,
            self.distribution,
            self.architecture,
            self.component,
            self.distribution,
            date,
            packages_info.md5,
            packages_info.size,
            component_path,
            packages_gz_info.md5,
            packages_gz_info.size,
            component_path,
            packages_info.sha1,
            packages_info.size,
            component_path,
            packages_gz_info.sha1,
            packages_gz_info.size,
            component_path,
            packages_info.sha256,
            packages_info.size,
            component_path,
            packages_gz_info.sha256,
            packages_gz_info.size,
            component_path,
        );

        Ok(content)
    }

    /// Calculate file checksums and size
    fn calculate_file_info(&self, file_path: &Path) -> Result<FileInfo> {
        let content = fs::read(file_path)?;

        let md5 = format!("{:x}", md5::compute(&content));
        let sha1 = hex::encode(sha1_smol::Sha1::from(&content).digest().bytes());
        let sha256 = format!("{:x}", sha2::Sha256::digest(&content));
        let size = content.len();

        Ok(FileInfo { md5, sha1, sha256, size })
    }

    /// Compress file using gzip
    fn compress_file(&self, input_path: &Path, output_path: &Path) -> Result<()> {
        let output = Command::new("gzip")
            .arg("-c")
            .arg(input_path)
            .output()
            .map_err(|e| {
                DebianError::repository_generation_failed(format!("Failed to run gzip: {e}"))
            })?;

        if !output.status.success() {
            return Err(DebianError::repository_generation_failed(format!(
                "gzip failed: {}",
                String::from_utf8_lossy(&output.stderr)
            )));
        }

        fs::write(output_path, &output.stdout)?;
        Ok(())
    }

    /// Validate .deb file using dpkg
    pub fn validate_deb_file(&self, deb_path: &Path) -> Result<DebPackageInfo> {
        let output = Command::new("dpkg-deb")
            .arg("--info")
            .arg(deb_path)
            .output()
            .map_err(|e| {
                DebianError::repository_generation_failed(format!("Failed to run dpkg-deb: {e}"))
            })?;

        if !output.status.success() {
            return Err(DebianError::repository_generation_failed(format!(
                "dpkg-deb validation failed for {}: {}",
                deb_path.display(),
                String::from_utf8_lossy(&output.stderr)
            )));
        }

        let info_text = String::from_utf8_lossy(&output.stdout);
        self.parse_deb_info(&info_text)
    }

    /// Parse dpkg-deb --info output
    fn parse_deb_info(&self, info_text: &str) -> Result<DebPackageInfo> {
        let mut fields = HashMap::new();

        for line in info_text.lines() {
            if let Some((key, value)) = line.split_once(':') {
                fields.insert(key.trim().to_string(), value.trim().to_string());
            }
        }

        let package = fields.get("Package").cloned().ok_or_else(|| {
            DebianError::repository_generation_failed("Missing Package field in .deb")
        })?;

        let version = fields.get("Version").cloned().ok_or_else(|| {
            DebianError::repository_generation_failed("Missing Version field in .deb")
        })?;

        let architecture = fields.get("Architecture").cloned().ok_or_else(|| {
            DebianError::repository_generation_failed("Missing Architecture field in .deb")
        })?;

        Ok(DebPackageInfo { package, version, architecture, fields })
    }

    /// Get repository URL for sources.list
    pub fn get_sources_list_entry(&self, base_url: &str) -> String {
        format!("deb {base_url} {}", self.distribution)
    }
}

/// File information for Release file
#[derive(Debug, Clone)]
struct FileInfo {
    md5: String,
    sha1: String,
    sha256: String,
    size: usize,
}

/// Repository metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RepositoryMetadata {
    /// Repository root directory
    pub repository_root: PathBuf,
    /// Distribution name
    pub distribution: String,
    /// Component name
    pub component: String,
    /// Architecture
    pub architecture: String,
    /// Packages file path
    pub packages_file: PathBuf,
    /// Release file path
    pub release_file: PathBuf,
    /// Number of packages
    pub package_count: usize,
}

/// Information about a .deb package
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DebPackageInfo {
    /// Package name
    pub package: String,
    /// Version
    pub version: String,
    /// Architecture
    pub architecture: String,
    /// All control fields
    pub fields: HashMap<String, String>,
}

// Add required dependencies to Cargo.toml
use md5;
use sha1_smol;
use sha2::Digest;

#[cfg(test)]
mod tests {
    use tempfile::TempDir;

    use super::*;

    #[test]
    fn test_repository_structure_creation() {
        let temp_dir = TempDir::new().unwrap();
        let generator = RepositoryGenerator::for_ros(
            temp_dir.path().to_path_buf(),
            "loong",
            "amd64".to_string(),
        );

        generator.create_repository_structure().unwrap();

        let binary_dir = temp_dir
            .path()
            .join("dists")
            .join("loong")
            .join("main")
            .join("binary-amd64");

        assert!(binary_dir.exists());

        let pool_dir = temp_dir.path().join("pool").join("main");
        assert!(pool_dir.exists());
    }

    #[test]
    fn test_sources_list_entry() {
        let temp_dir = TempDir::new().unwrap();
        let generator = RepositoryGenerator::for_ros(
            temp_dir.path().to_path_buf(),
            "loong",
            "amd64".to_string(),
        );

        let entry = generator.get_sources_list_entry("https://example.com/repo");
        assert_eq!(entry, "deb https://example.com/repo loong");
    }

    #[test]
    fn test_file_info_calculation() {
        let temp_dir = TempDir::new().unwrap();
        let test_file = temp_dir.path().join("test.txt");
        fs::write(&test_file, "test content").unwrap();

        let generator = RepositoryGenerator::for_ros(
            temp_dir.path().to_path_buf(),
            "loong",
            "amd64".to_string(),
        );

        let info = generator.calculate_file_info(&test_file).unwrap();
        assert!(!info.md5.is_empty());
        assert!(!info.sha1.is_empty());
        assert!(!info.sha256.is_empty());
        assert_eq!(info.size, 12); // "test content" is 12 bytes
    }
}
