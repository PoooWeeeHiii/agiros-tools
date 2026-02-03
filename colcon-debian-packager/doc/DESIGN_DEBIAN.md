# Design Details - Debian Package Generation

This document details the design of the Debian package generation component (`colcon-deb-debian`).

## Overview

The Debian packaging layer manages the debian directory collection and coordinates package building:
- **Host Side**: Manages debian directory collection, validates structure
- **Container Side**: Uses bloom-generate for missing dirs, dpkg-deb for building

This crate **does not generate debian directories** - it uses bloom-generate or user-provided directories.

## Debian Directory Management

### 1. Debian Directory Manager

```rust
pub struct DebianDirManager {
    debian_dirs_path: PathBuf,
    ros_distro: String,
}

impl DebianDirManager {
    /// Check if a custom debian directory exists for a package
    pub fn has_debian_dir(&self, package_name: &str) -> bool {
        self.debian_dirs_path
            .join(package_name)
            .join("debian")
            .exists()
    }
    
    /// Get the path to a package's debian directory
    pub fn get_debian_dir(&self, package_name: &str) -> PathBuf {
        self.debian_dirs_path.join(package_name).join("debian")
    }
    
    /// List all packages with custom debian directories
    pub fn list_custom_packages(&self) -> Result<Vec<String>> {
        let mut packages = Vec::new();
        
        for entry in fs::read_dir(&self.debian_dirs_path)? {
            let entry = entry?;
            let path = entry.path();
            
            if path.is_dir() && path.join("debian").exists() {
                if let Some(name) = path.file_name() {
                    packages.push(name.to_string_lossy().to_string());
                }
            }
        }
        
        Ok(packages)
    }
    
    /// Validate debian directory structure
    pub fn validate_debian_dir(&self, package_name: &str) -> Result<()> {
        let debian_dir = self.get_debian_dir(package_name);
        
        // Check required files
        let required_files = ["control", "rules", "changelog", "compat"];
        
        for file in &required_files {
            if !debian_dir.join(file).exists() {
                eyre::bail!("Missing required file: debian/{}", file);
            }
        }
        
        Ok(())
    }
}
```

### 2. Bloom Integration

```rust
pub struct BloomGenerator {
    ros_distro: String,
    container_id: String,
}

impl BloomGenerator {
    /// Generate debian directory for a package using bloom-generate
    pub async fn generate_debian_dir(
        &self,
        package_path: &Path,
        output_path: &Path,
    ) -> Result<()> {
        // This is called from container scripts, not from Rust
        // The Rust side only manages the directory structure
        unreachable!("bloom-generate is called from container scripts")
    }
}
```

### 3. Container-Side Debian Generation Script

```bash
#!/bin/bash
# /scripts/generate-debian.sh - Generate debian directory if missing
# This script runs as the non-root 'builder' user

PACKAGE_NAME=$1
PACKAGE_PATH=$2
DEBIAN_DIRS=$3

# Check if custom debian dir exists
if [ -d "$DEBIAN_DIRS/$PACKAGE_NAME/debian" ]; then
    echo "Using custom debian directory for $PACKAGE_NAME"
    cp -r "$DEBIAN_DIRS/$PACKAGE_NAME/debian" "$PACKAGE_PATH/"
else
    echo "Generating debian directory for $PACKAGE_NAME with bloom-generate"
    cd "$PACKAGE_PATH"
    
    # bloom-generate runs as user (doesn't need root)
    bloom-generate rosdebian \
        --ros-distro "$ROS_DISTRO" \
        --debian-inc 0 \
        --os-name ubuntu \
        --os-version jammy \
        .
    
    # Save generated debian dir back to collection (as user)
    if [ -d "$PACKAGE_PATH/debian" ]; then
        # Post-process debian/control to enforce agiros naming
        if [ -f "$PACKAGE_PATH/debian/control" ]; then
            sed -i "s/Package: ros-$ROS_DISTRO-/Package: agiros-$ROS_DISTRO-/g" "$PACKAGE_PATH/debian/control"
            sed -i "s/Source: ros-$ROS_DISTRO-/Source: agiros-$ROS_DISTRO-/g" "$PACKAGE_PATH/debian/control"
        fi

        mkdir -p "$DEBIAN_DIRS/$PACKAGE_NAME"
        cp -r "$PACKAGE_PATH/debian" "$DEBIAN_DIRS/$PACKAGE_NAME/"
        echo "Saved generated debian directory to collection"
    else
        echo "ERROR: bloom-generate failed to create debian directory"
        exit 1
    fi
fi
```

### 4. Container-Side Build Script

```bash
#!/bin/bash
# /scripts/build-deb.sh - Build debian package
# This script runs as the non-root 'builder' user

PACKAGE_PATH=$1
PACKAGE_NAME=$(basename "$PACKAGE_PATH")

cd "$PACKAGE_PATH"

# Build the package (runs as user, dpkg-buildpackage handles sudo internally if needed)
dpkg-buildpackage -b -uc -us

# Move generated .deb files to output
mv ../*.deb /workspace/output/

# Alternative: use dpkg-deb directly (may need sudo)
# sudo dpkg-deb --build --root-owner-group debian /workspace/output/${PACKAGE_NAME}.deb
```

## Version Handling

### 1. Version Conversion

```rust
pub struct VersionConverter {
    ros_version: String,
    revision: u32,
}

impl VersionConverter {
    pub fn to_debian_version(&self) -> DebianVersion {
        // Handle ROS version formats
        // Add debian revision
        // Handle pre-release versions
        DebianVersion {
            upstream: self.normalize_upstream(),
            debian_revision: self.revision,
            epoch: None,
        }
    }
    
    fn normalize_upstream(&self) -> String {
        // Convert _ to .
        // Handle special cases
        // Ensure valid Debian version
    }
}
```

### 2. Version Comparison

```rust
impl PartialOrd for DebianVersion {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        // Implement Debian version comparison algorithm
        // Handle epochs
        // Compare upstream versions
        // Compare debian revisions
    }
}
```

## Dependency Mapping

### 1. Dependency Resolver

```rust
pub struct DependencyMapper {
    agirosdep_db: agirosdepDatabase,
    package_map: HashMap<String, String>,
}

impl DependencyMapper {
    pub fn map_dependencies(&self, deps: &[Dependency]) -> Vec<DebianDependency> {
        deps.iter()
            .filter_map(|dep| self.map_single_dependency(dep))
            .collect()
    }
    
    fn map_single_dependency(&self, dep: &Dependency) -> Option<DebianDependency> {
        // Check if it's another ROS package
        if let Some(deb_name) = self.package_map.get(&dep.name) {
            return Some(DebianDependency {
                package: deb_name.clone(),
                version_constraint: self.map_version_constraint(&dep.version_range),
            });
        }
        
        // Look up in agirosdep
        if let Ok(system_dep) = self.agirosdep_db.lookup(&dep.name) {
            return Some(DebianDependency {
                package: system_dep.debian_package,
                version_constraint: None,
            });
        }
        
        None
    }
}
```

### 2. Virtual Packages

```rust
pub struct VirtualPackageProvider {
    mappings: HashMap<String, Vec<String>>,
}

impl VirtualPackageProvider {
    pub fn get_providers(&self, virtual_package: &str) -> Option<&Vec<String>> {
        self.mappings.get(virtual_package)
    }
    
    pub fn is_virtual(&self, package: &str) -> bool {
        self.mappings.contains_key(package)
    }
}
```

## Package Building

### 1. Dpkg Integration

```rust
use tokio::process::Command;

pub struct DpkgBuilder {
    work_dir: TempDir,
    progress_tx: mpsc::Sender<BuildProgress>,
}

impl DpkgBuilder {
    pub async fn build_package(&self, source_dir: &Path) -> Result<PathBuf> {
        // Prepare build directory
        self.prepare_build_tree(source_dir).await?;
        
        // Use dpkg-deb directly for better control
        let deb_path = self.work_dir.path().join("package.deb");
        
        let status = Command::new("dpkg-deb")
            .args(&[
                "--build",
                "--root-owner-group",
                source_dir.to_str().unwrap(),
                deb_path.to_str().unwrap(),
            ])
            .status()
            .await?;
        
        if !status.success() {
            eyre::bail!("dpkg-deb failed to build package");
        }
        
        Ok(deb_path)
    }
    
    pub async fn validate_package(&self, deb_path: &Path) -> Result<()> {
        // Use lintian for validation
        let output = Command::new("lintian")
            .args(&[
                "--no-tag-display-limit",
                "--pedantic",
                deb_path.to_str().unwrap(),
            ])
            .output()
            .await?;
        
        // Parse lintian output for warnings/errors
        self.parse_lintian_output(&output.stdout)
    }
    
    pub async fn extract_control(&self, deb_path: &Path) -> Result<String> {
        // Use dpkg-deb to extract control information
        let output = Command::new("dpkg-deb")
            .args(&["-f", deb_path.to_str().unwrap()])
            .output()
            .await?;
        
        Ok(String::from_utf8(output.stdout)?)
    }
}
```

### 2. File Installation

```rust
pub struct FileInstaller {
    install_rules: Vec<InstallRule>,
}

pub struct InstallRule {
    pub source: PathBuf,
    pub destination: PathBuf,
    pub permissions: u32,
}

impl FileInstaller {
    pub async fn install_files(&self, build_dir: &Path, install_dir: &Path) -> Result<()> {
        for rule in &self.install_rules {
            let src = build_dir.join(&rule.source);
            let dst = install_dir.join(&rule.destination);
            
            // Create parent directories
            if let Some(parent) = dst.parent() {
                fs::create_dir_all(parent).await?;
            }
            
            // Copy file with permissions
            fs::copy(&src, &dst).await?;
            fs::set_permissions(&dst, Permissions::from_mode(rule.permissions)).await?;
        }
        Ok(())
    }
}
```

## Repository Generation

### 1. APT Repository Builder

```rust
pub struct AptRepositoryBuilder {
    root: PathBuf,
    distribution: String,
    component: String,
    architectures: Vec<String>,
}

impl AptRepositoryBuilder {
    pub async fn add_package(&self, deb_path: &Path) -> Result<()> {
        // Copy to pool directory
        let pool_dir = self.get_pool_dir(&deb_path).await?;
        fs::copy(deb_path, pool_dir.join(deb_path.file_name().unwrap())).await?;
        
        // Update package indices
        self.update_indices().await?;
        
        Ok(())
    }
    
    pub async fn generate_release(&self) -> Result<()> {
        // Generate Release file
        let release = ReleaseFile {
            origin: "Colcon Debian Packager".to_string(),
            label: self.distribution.clone(),
            suite: self.distribution.clone(),
            codename: self.distribution.clone(),
            architectures: self.architectures.clone(),
            components: vec![self.component.clone()],
            description: "Local ROS packages".to_string(),
        };
        
        release.write(&self.root.join("dists").join(&self.distribution)).await
    }
}
```

### 2. Package Indices

```rust
pub struct PackageIndex {
    packages: Vec<PackageEntry>,
}

pub struct PackageEntry {
    pub package: String,
    pub version: String,
    pub architecture: String,
    pub filename: String,
    pub size: u64,
    pub md5sum: String,
    pub sha256: String,
    pub description: String,
    pub depends: Vec<String>,
}

impl PackageIndex {
    pub fn add_from_deb(&mut self, deb_path: &Path) -> Result<()> {
        // Extract control information
        let control = extract_control(deb_path)?;
        
        // Calculate checksums
        let checksums = calculate_checksums(deb_path)?;
        
        // Add entry
        self.packages.push(PackageEntry {
            // ... populate fields
        });
        
        Ok(())
    }
    
    pub async fn write_packages_file(&self, path: &Path) -> Result<()> {
        // Write in correct format
        // Handle compression
    }
}
```

## Signing Support

### 1. GPG Integration

```rust
pub struct GpgSigner {
    key_id: String,
    gpg_home: Option<PathBuf>,
}

impl GpgSigner {
    pub async fn sign_file(&self, file_path: &Path) -> Result<PathBuf> {
        let sig_path = file_path.with_extension("gpg");
        
        let mut cmd = Command::new("gpg");
        
        if let Some(home) = &self.gpg_home {
            cmd.arg("--homedir").arg(home);
        }
        
        cmd.args(&[
            "--default-key", &self.key_id,
            "--armor",
            "--detach-sign",
            "--output", sig_path.to_str().unwrap(),
            file_path.to_str().unwrap(),
        ]);
        
        cmd.output().await?;
        
        Ok(sig_path)
    }
}
```

### 2. Repository Signing

```rust
pub struct SecureRepository {
    repository: AptRepositoryBuilder,
    signer: GpgSigner,
}

impl SecureRepository {
    pub async fn sign_repository(&self) -> Result<()> {
        // Sign Release file
        let release_path = self.repository.root
            .join("dists")
            .join(&self.repository.distribution)
            .join("Release");
            
        self.signer.sign_file(&release_path).await?;
        
        // Create InRelease file
        self.create_in_release(&release_path).await?;
        
        Ok(())
    }
}
```

## Meta-Package Support

### 1. Meta-Package Generator

```rust
pub struct MetaPackageBuilder {
    name: String,
    dependencies: Vec<String>,
    description: String,
}

impl MetaPackageBuilder {
    pub async fn build(&self, output_dir: &Path) -> Result<PathBuf> {
        // Create minimal debian directory
        let debian_dir = output_dir.join("debian");
        fs::create_dir_all(&debian_dir).await?;
        
        // Write control file
        self.write_control(&debian_dir).await?;
        
        // Write other required files
        self.write_rules(&debian_dir).await?;
        self.write_changelog(&debian_dir).await?;
        
        // Build package
        DpkgBuilder::new().build_package(output_dir).await
    }
}
```

## Testing Utilities

### 1. Package Validation

```rust
pub struct DebianPackageValidator {
    lintian_enabled: bool,
}

impl DebianPackageValidator {
    pub async fn validate(&self, deb_path: &Path) -> Result<ValidationReport> {
        let mut report = ValidationReport::new();
        
        // Check package structure
        report.add_results(self.check_structure(deb_path).await?);
        
        // Validate control fields
        report.add_results(self.validate_control(deb_path).await?);
        
        // Run lintian if enabled
        if self.lintian_enabled {
            report.add_results(self.run_lintian(deb_path).await?);
        }
        
        Ok(report)
    }
}
```