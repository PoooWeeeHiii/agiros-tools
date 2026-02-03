//! Package.xml parsing types and structures

use serde::{Deserialize, Serialize};

/// ROS package manifest from package.xml
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct PackageManifest {
    /// Package name
    pub name: String,

    /// Package version
    pub version: String,

    /// Package description
    pub description: String,

    /// Package maintainers
    pub maintainers: Vec<Person>,

    /// Package authors
    pub authors: Vec<Person>,

    /// Package license(s)
    pub licenses: Vec<String>,

    /// Package URL(s)
    pub urls: Vec<Url>,

    /// Build type (ament_cmake, ament_python, etc.)
    pub build_type: Option<String>,

    /// Package dependencies
    pub dependencies: PackageDependencies,
}

/// Person information (maintainer or author)
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Person {
    /// Person's name
    pub name: String,

    /// Person's email (optional)
    pub email: Option<String>,
}

/// URL with type
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Url {
    /// URL type (website, repository, bugtracker, etc.)
    #[serde(rename = "type")]
    pub url_type: Option<String>,

    /// The URL itself
    pub url: String,
}

/// All dependency types in package.xml
#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq)]
pub struct PackageDependencies {
    /// Build dependencies
    pub build_depend: Vec<DependencySpec>,

    /// Build export dependencies
    pub build_export_depend: Vec<DependencySpec>,

    /// Build tool dependencies
    pub buildtool_depend: Vec<DependencySpec>,

    /// Build tool export dependencies
    pub buildtool_export_depend: Vec<DependencySpec>,

    /// Execution dependencies
    pub exec_depend: Vec<DependencySpec>,

    /// Test dependencies
    pub test_depend: Vec<DependencySpec>,

    /// Documentation dependencies
    pub doc_depend: Vec<DependencySpec>,

    /// Generic dependencies (maps to build, build_export, and exec)
    pub depend: Vec<DependencySpec>,
}

/// A dependency specification
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct DependencySpec {
    /// Package name
    pub name: String,

    /// Optional version constraint
    pub version_eq: Option<String>,
    pub version_gte: Option<String>,
    pub version_lte: Option<String>,
    pub version_gt: Option<String>,
    pub version_lt: Option<String>,

    /// Optional condition
    pub condition: Option<String>,
}

impl PackageDependencies {
    /// Get all unique dependency names
    pub fn all_unique_names(&self) -> Vec<String> {
        let mut names = Vec::new();

        names.extend(self.build_depend.iter().map(|d| d.name.clone()));
        names.extend(self.build_export_depend.iter().map(|d| d.name.clone()));
        names.extend(self.buildtool_depend.iter().map(|d| d.name.clone()));
        names.extend(self.buildtool_export_depend.iter().map(|d| d.name.clone()));
        names.extend(self.exec_depend.iter().map(|d| d.name.clone()));
        names.extend(self.test_depend.iter().map(|d| d.name.clone()));
        names.extend(self.doc_depend.iter().map(|d| d.name.clone()));
        names.extend(self.depend.iter().map(|d| d.name.clone()));

        names.sort();
        names.dedup();
        names
    }

    /// Expand generic 'depend' tags into specific dependency types
    pub fn expand_generic_depends(&mut self) {
        // Generic 'depend' expands to build_depend, build_export_depend, and
        // exec_depend
        for dep in &self.depend {
            self.build_depend.push(dep.clone());
            self.build_export_depend.push(dep.clone());
            self.exec_depend.push(dep.clone());
        }
    }
}
