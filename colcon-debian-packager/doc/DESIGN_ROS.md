# Design Details - ROS Package Handling

This document details the design of the ROS package handling component (`colcon-deb-ros`).

## Overview

The ROS package handling layer provides data structures and parsing logic used by both host and container sides:
- **Host Side**: Parses package.xml for planning and validation
- **Container Side**: Uses the actual ROS tools (agirosdep, colcon) for building

This crate primarily defines the **data models** and **parsing logic**, while actual ROS operations happen inside the container.

## Package Discovery

### 1. Workspace Scanner

```rust
pub struct WorkspaceScanner {
    root: PathBuf,
    ignore_patterns: Vec<String>,
}

impl WorkspaceScanner {
    pub async fn scan(&self) -> Result<Vec<PackagePath>> {
        // Recursively find package.xml files
        // Apply ignore patterns
        // Return sorted list of package paths
    }
}
```

### 2. Package Detection Strategy

**Algorithm**:
1. Look for `package.xml` in each directory
2. Validate it's a valid ROS package
3. Skip COLCON_IGNORE directories
4. Handle nested packages correctly

## Package Parsing

### 1. Package.xml Parser

```rust
pub struct PackageXmlParser {
    strict_mode: bool,
}

impl PackageXmlParser {
    pub fn parse(&self, content: &str) -> Result<PackageManifest> {
        // Parse XML with quick-xml
        // Handle format version 3 (ROS 2)
        // Extract all metadata
    }
}
```

### 2. Package Manifest Structure

```rust
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PackageManifest {
    pub name: String,
    pub version: Version,
    pub description: String,
    pub maintainers: Vec<Person>,
    pub authors: Vec<Person>,
    pub license: Vec<String>,
    pub dependencies: Dependencies,
    pub exports: HashMap<String, Value>,
}

#[derive(Debug, Clone)]
pub struct Dependencies {
    pub build: Vec<Dependency>,
    pub build_export: Vec<Dependency>,
    pub buildtool: Vec<Dependency>,
    pub buildtool_export: Vec<Dependency>,
    pub exec: Vec<Dependency>,
    pub test: Vec<Dependency>,
    pub doc: Vec<Dependency>,
}

#[derive(Debug, Clone)]
pub struct Dependency {
    pub name: String,
    pub version_range: Option<VersionReq>,
    pub condition: Option<String>,
}
```

### 3. Format Version Handling

```rust
pub enum PackageFormat {
    Format3, // ROS 2 standard format
}

impl PackageFormat {
    pub fn from_xml(root: &Element) -> Result<Self>;
    pub fn parse_dependencies(&self, root: &Element) -> Result<Dependencies>;
}
```

## Build System Detection

### 1. Build Type Identification

```rust
pub enum BuildType {
    AmentCmake,
    AmentPython,
    CmakeOnly,
    PythonOnly,
}

impl BuildType {
    pub fn detect(manifest: &PackageManifest, path: &Path) -> Result<Self> {
        // Check build tool dependencies
        // Examine CMakeLists.txt for ament_cmake
        // Check for setup.py with ament_python
        // Default to cmake for plain packages
    }
}
```

### 2. Build System Handlers

```rust
pub trait BuildSystemHandler {
    fn prepare_build(&self, package: &Package) -> Result<BuildCommands>;
    fn install_commands(&self, package: &Package) -> Vec<String>;
    fn clean_commands(&self, package: &Package) -> Vec<String>;
}

pub struct AmentCmakeHandler;
pub struct AmentPythonHandler;
pub struct CmakeHandler;
```

## Dependency Resolution

### 1. Dependency Graph

```rust
pub struct DependencyGraph {
    packages: HashMap<String, Package>,
    edges: HashMap<String, HashSet<String>>,
}

impl DependencyGraph {
    pub fn add_package(&mut self, package: Package);
    pub fn add_dependency(&mut self, from: &str, to: &str, dep_type: DependencyType);
    pub fn topological_sort(&self) -> Result<Vec<String>>;
    pub fn find_cycles(&self) -> Vec<Vec<String>>;
}
```

### 2. Dependency Types

```rust
pub enum DependencyType {
    Build,
    BuildExport,
    Buildtool,
    BuildtoolExport,
    Exec,
    Test,
    Doc,
}

impl DependencyType {
    pub fn is_runtime(&self) -> bool {
        matches!(self, Self::Exec | Self::BuildExport | Self::BuildtoolExport)
    }
    
    pub fn is_build_time(&self) -> bool {
        matches!(self, Self::Build | Self::Buildtool)
    }
}
```

### 3. agirosdep Integration

```rust
use tokio::process::Command;

pub struct agirosdepResolver {
    ros_distro: String,
    os_name: String,
    os_version: String,
}

impl agirosdepResolver {
    pub async fn resolve(&self, ros_dep: &str) -> Result<SystemDependency> {
        // Use agirosdep command directly
        let output = Command::new("agirosdep")
            .args(&[
                "resolve",
                ros_dep,
                "--rosdistro", &self.ros_distro,
                "--os", &format!("{}:{}", self.os_name, self.os_version),
            ])
            .output()
            .await?;
        
        // Parse agirosdep output
        self.parse_agirosdep_output(&output.stdout)
    }
    
    pub async fn install_dependencies(&self, workspace: &Path) -> Result<()> {
        // Let agirosdep handle the complexity
        Command::new("agirosdep")
            .args(&[
                "install",
                "--from-paths", workspace.to_str().unwrap(),
                "--ignore-src",
                "--rosdistro", &self.ros_distro,
                "-y",
            ])
            .status()
            .await?;
        
        Ok(())
    }
}
```

## Package Analysis

### 1. Package Metadata Extraction

```rust
pub struct PackageAnalyzer {
    package: Package,
}

impl PackageAnalyzer {
    pub fn debian_name(&self, ros_distro: &str) -> String {
        format!("ros-{}-{}", ros_distro, self.package.name.replace('_', "-"))
    }
    
    pub fn debian_version(&self) -> String {
        // Convert ROS version to Debian version
        // Handle pre-release versions
    }
    
    pub fn debian_dependencies(&self) -> Vec<String> {
        // Convert ROS deps to Debian deps
        // Handle system dependencies
    }
}
```

### 2. License Detection

```rust
pub struct LicenseDetector {
    spdx_licenses: HashSet<String>,
}

impl LicenseDetector {
    pub fn detect_license(&self, package: &Package) -> Result<Vec<License>> {
        // Parse license field
        // Check LICENSE files
        // Validate SPDX identifiers
    }
}
```

## Colcon Integration

### 1. Colcon Metadata

```rust
pub struct ColconPackage {
    pub name: String,
    pub path: PathBuf,
    pub build_type: String,
    pub dependencies: HashMap<String, Vec<String>>,
}

impl From<Package> for ColconPackage {
    fn from(package: Package) -> Self {
        // Convert ROS package to Colcon format
    }
}
```

### 2. Build Order Computation

```rust
pub struct BuildOrderComputer {
    graph: DependencyGraph,
}

impl BuildOrderComputer {
    pub fn compute_build_order(&self) -> Result<Vec<BuildWave>> {
        // Topological sort with parallelism
        // Group independent packages
    }
}

pub struct BuildWave {
    pub wave_number: usize,
    pub packages: Vec<String>,
}
```

## Testing Utilities

### 1. Test Package Generator

```rust
pub struct TestPackageBuilder {
    name: String,
    version: String,
    dependencies: Vec<String>,
}

impl TestPackageBuilder {
    pub fn build(&self) -> Package {
        // Generate test package
    }
    
    pub fn write_to_disk(&self, path: &Path) -> Result<()> {
        // Create package.xml
        // Create minimal CMakeLists.txt
    }
}
```

### 2. Fixture Management

```rust
pub struct RosWorkspaceFixture {
    temp_dir: TempDir,
    packages: Vec<Package>,
}

impl RosWorkspaceFixture {
    pub fn new() -> Self;
    pub fn add_package(&mut self, package: Package);
    pub fn path(&self) -> &Path;
}
```

## Performance Considerations

### 1. Parallel Package Discovery

```rust
pub async fn discover_packages_parallel(root: &Path) -> Result<Vec<Package>> {
    let scanner = WorkspaceScanner::new(root);
    let paths = scanner.scan().await?;
    
    // Parse packages in parallel
    let packages = futures::stream::iter(paths)
        .map(|path| async move {
            parse_package(&path).await
        })
        .buffer_unordered(num_cpus::get())
        .try_collect()
        .await?;
    
    Ok(packages)
}
```

### 2. Caching Parsed Packages

```rust
pub struct PackageCache {
    cache: DashMap<PathBuf, (SystemTime, Package)>,
}

impl PackageCache {
    pub async fn get_or_parse(&self, path: &Path) -> Result<Package> {
        // Check cache with mtime
        // Parse if needed
        // Update cache
    }
}
```

## Error Handling

```rust
use thiserror::Error;

#[derive(Error, Debug)]
pub enum RosPackageError {
    #[error("Invalid package.xml: {0}")]
    InvalidManifest(String),
    
    #[error("Unsupported package format: {0}")]
    UnsupportedFormat(u8),
    
    #[error("Cyclic dependency detected: {}", .0.join(" -> "))]
    CyclicDependency(Vec<String>),
    
    #[error("Missing required field: {0}")]
    MissingField(String),
    
    #[error("XML parsing error")]
    XmlParse(#[from] quick_xml::Error),
    
    #[error("Package not found at path: {0}")]
    PackageNotFound(PathBuf),
    
    #[error("Invalid version specification: {0}")]
    InvalidVersion(#[source] semver::Error),
}

#[derive(Error, Debug)]
pub enum DependencyError {
    #[error("Unresolved dependency: {package}")]
    Unresolved { package: String },
    
    #[error("Version conflict: {package} requires {required}, but {found} is available")]
    VersionConflict {
        package: String,
        required: String,
        found: String,
    },
    
    #[error("agirosdep lookup failed")]
    agirosdepLookup(#[from] agirosdepError),
}
```