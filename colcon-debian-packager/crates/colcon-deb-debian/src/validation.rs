//! Validation logic for Debian package directories and files

use std::fs;
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};
use tracing::debug;

use crate::error::{DebianError, Result};

/// Required files in a debian directory
const REQUIRED_FILES: &[&str] = &["control", "changelog", "copyright", "rules"];

/// Optional files that are commonly present
const OPTIONAL_FILES: &[&str] = &[
    "compat", "install", "postinst", "prerm", "postrm", "preinst", "triggers", "watch",
];

/// Debian directory validator
#[derive(Debug)]
pub struct DebianValidator {
    /// Strict validation mode
    strict_mode: bool,
}

impl Default for DebianValidator {
    fn default() -> Self {
        Self::new(false)
    }
}

impl DebianValidator {
    /// Create a new validator
    pub fn new(strict_mode: bool) -> Self {
        Self { strict_mode }
    }

    /// Validate a debian directory
    pub fn validate_debian_directory(
        &self,
        debian_dir: &Path,
        package_name: &str,
    ) -> Result<ValidationResult> {
        if !debian_dir.exists() {
            return Err(DebianError::missing_required_file(package_name, "debian directory"));
        }

        if !debian_dir.is_dir() {
            return Err(DebianError::invalid_control_file(
                package_name,
                "debian path is not a directory",
            ));
        }

        let mut result = ValidationResult::new();

        // Check required files
        for &required_file in REQUIRED_FILES {
            let file_path = debian_dir.join(required_file);
            if file_path.exists() {
                result.found_files.push(required_file.to_string());

                // Validate specific files
                match required_file {
                    "control" => {
                        self.validate_control_file(&file_path, package_name, &mut result)?
                    }
                    "changelog" => {
                        self.validate_changelog_file(&file_path, package_name, &mut result)?
                    }
                    "copyright" => {
                        self.validate_copyright_file(&file_path, package_name, &mut result)?
                    }
                    "rules" => self.validate_rules_file(&file_path, package_name, &mut result)?,
                    _ => {}
                }
            } else {
                result.missing_files.push(required_file.to_string());
                if self.strict_mode {
                    return Err(DebianError::missing_required_file(package_name, required_file));
                }
            }
        }

        // Check optional files
        for &optional_file in OPTIONAL_FILES {
            let file_path = debian_dir.join(optional_file);
            if file_path.exists() {
                result.found_files.push(optional_file.to_string());
            }
        }

        // Check source directory
        let source_dir = debian_dir.join("source");
        if source_dir.exists() {
            self.validate_source_directory(&source_dir, package_name, &mut result)?;
        }

        // Overall validation result
        result.is_valid = result.missing_files.is_empty();

        Ok(result)
    }

    /// Validate debian/control file
    fn validate_control_file(
        &self,
        control_path: &Path,
        package_name: &str,
        result: &mut ValidationResult,
    ) -> Result<()> {
        let content = fs::read_to_string(control_path).map_err(|e| {
            DebianError::invalid_control_file(
                package_name,
                format!("Cannot read control file: {e}"),
            )
        })?;

        // Check for required fields in control file
        let required_fields = ["Package", "Architecture", "Maintainer", "Description"];
        let mut found_fields = Vec::new();
        let mut missing_fields = Vec::new();

        for &field in &required_fields {
            if content.contains(&format!("{field}:")) {
                found_fields.push(field.to_string());
            } else {
                missing_fields.push(field.to_string());
                result
                    .warnings
                    .push(format!("Missing required field in control: {field}"));
            }
        }

        // Check for source package section
        if !content.contains("Source:") {
            result
                .warnings
                .push("Missing Source field in control file".to_string());
        }

        // Check for proper formatting
        if !content.contains("Priority:") {
            result
                .warnings
                .push("Missing Priority field in control file".to_string());
        }

        if !content.contains("Section:") {
            result
                .warnings
                .push("Missing Section field in control file".to_string());
        }

        debug!(
            "Control file validation: found fields: {found_fields:?}, missing: {missing_fields:?}"
        );

        if !missing_fields.is_empty() && self.strict_mode {
            return Err(DebianError::invalid_control_file(
                package_name,
                format!("Missing required fields: {}", missing_fields.join(", ")),
            ));
        }

        Ok(())
    }

    /// Validate debian/changelog file
    fn validate_changelog_file(
        &self,
        changelog_path: &Path,
        package_name: &str,
        result: &mut ValidationResult,
    ) -> Result<()> {
        let content = fs::read_to_string(changelog_path).map_err(|e| {
            DebianError::invalid_control_file(
                package_name,
                format!("Cannot read changelog file: {e}"),
            )
        })?;

        // Basic changelog format validation
        if content.is_empty() {
            result.warnings.push("Changelog file is empty".to_string());
            return Ok(());
        }

        // Check if it starts with package name
        let first_line = content.lines().next().unwrap_or("");
        if !first_line.starts_with(package_name) && !first_line.contains('(') {
            result
                .warnings
                .push("Changelog doesn't start with proper package entry".to_string());
        }

        // Check for basic debian changelog format
        if !content.contains(" -- ") {
            result
                .warnings
                .push("Changelog missing proper signature line".to_string());
        }

        debug!("Changelog validation completed for {package_name}");
        Ok(())
    }

    /// Validate debian/copyright file
    fn validate_copyright_file(
        &self,
        copyright_path: &Path,
        package_name: &str,
        result: &mut ValidationResult,
    ) -> Result<()> {
        let content = fs::read_to_string(copyright_path).map_err(|e| {
            DebianError::invalid_control_file(
                package_name,
                format!("Cannot read copyright file: {e}"),
            )
        })?;

        if content.is_empty() {
            result.warnings.push("Copyright file is empty".to_string());
            return Ok(());
        }

        // Check for common copyright indicators
        let copyright_indicators = ["Copyright", "License", "Source"];
        let mut found_indicators = 0;

        for indicator in &copyright_indicators {
            if content.contains(indicator) {
                found_indicators += 1;
            }
        }

        if found_indicators == 0 {
            result
                .warnings
                .push("Copyright file lacks common copyright information".to_string());
        }

        debug!("Copyright validation completed for {package_name}");
        Ok(())
    }

    /// Validate debian/rules file
    fn validate_rules_file(
        &self,
        rules_path: &Path,
        package_name: &str,
        result: &mut ValidationResult,
    ) -> Result<()> {
        let content = fs::read_to_string(rules_path).map_err(|e| {
            DebianError::invalid_control_file(package_name, format!("Cannot read rules file: {e}"))
        })?;

        // Check if rules file is executable
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            let metadata = fs::metadata(rules_path).map_err(|e| {
                DebianError::invalid_control_file(
                    package_name,
                    format!("Cannot read rules file metadata: {e}"),
                )
            })?;
            let permissions = metadata.permissions();
            if permissions.mode() & 0o111 == 0 {
                result
                    .warnings
                    .push("Rules file is not executable".to_string());
            }
        }

        // Check for shebang
        if !content.starts_with("#!/") {
            result
                .warnings
                .push("Rules file missing shebang".to_string());
        }

        // Check for required targets
        let required_targets = ["build", "binary", "clean"];
        for target in &required_targets {
            if !content.contains(&format!("{target}:")) {
                result
                    .warnings
                    .push(format!("Rules file missing {target} target"));
            }
        }

        debug!("Rules validation completed for {package_name}");
        Ok(())
    }

    /// Validate debian/source directory
    fn validate_source_directory(
        &self,
        source_dir: &Path,
        package_name: &str,
        result: &mut ValidationResult,
    ) -> Result<()> {
        let format_file = source_dir.join("format");
        if !format_file.exists() {
            result
                .warnings
                .push("Missing debian/source/format file".to_string());
            return Ok(());
        }

        let content = fs::read_to_string(&format_file).map_err(|e| {
            DebianError::invalid_control_file(
                package_name,
                format!("Cannot read source/format file: {e}"),
            )
        })?;

        let format_version = content.trim();
        if !["3.0 (native)", "3.0 (quilt)"].contains(&format_version) {
            result
                .warnings
                .push(format!("Unknown source format: {format_version}"));
        }

        debug!("Source directory validation completed for {package_name}");
        Ok(())
    }

    /// List all files in debian directory
    pub fn list_debian_files(&self, debian_dir: &Path) -> Result<Vec<PathBuf>> {
        if !debian_dir.exists() {
            return Ok(Vec::new());
        }

        let mut files = Vec::new();
        for entry in fs::read_dir(debian_dir)? {
            let entry = entry?;
            let path = entry.path();
            if path.is_file() {
                if let Some(file_name) = path.file_name() {
                    files.push(PathBuf::from(file_name));
                }
            }
        }

        files.sort();
        Ok(files)
    }

    /// Check if debian directory is complete
    pub fn is_complete_debian_directory(&self, debian_dir: &Path) -> bool {
        REQUIRED_FILES
            .iter()
            .all(|&file| debian_dir.join(file).exists())
    }
}

/// Result of debian directory validation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ValidationResult {
    /// Whether the directory is valid
    pub is_valid: bool,
    /// Files that were found
    pub found_files: Vec<String>,
    /// Files that are missing
    pub missing_files: Vec<String>,
    /// Validation warnings
    pub warnings: Vec<String>,
    /// Validation errors
    pub errors: Vec<String>,
}

impl ValidationResult {
    /// Create a new validation result
    pub fn new() -> Self {
        Self {
            is_valid: false,
            found_files: Vec::new(),
            missing_files: Vec::new(),
            warnings: Vec::new(),
            errors: Vec::new(),
        }
    }

    /// Check if validation passed without warnings
    pub fn is_clean(&self) -> bool {
        self.is_valid && self.warnings.is_empty() && self.errors.is_empty()
    }

    /// Get summary of validation
    pub fn summary(&self) -> String {
        format!(
            "Valid: {}, Found: {}, Missing: {}, Warnings: {}, Errors: {}",
            self.is_valid,
            self.found_files.len(),
            self.missing_files.len(),
            self.warnings.len(),
            self.errors.len()
        )
    }
}

impl Default for ValidationResult {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use std::fs;

    use tempfile::TempDir;

    use super::*;

    fn create_test_debian_dir(temp_dir: &TempDir) -> PathBuf {
        let debian_dir = temp_dir.path().join("debian");
        fs::create_dir(&debian_dir).unwrap();
        debian_dir
    }

    #[test]
    fn test_missing_debian_directory() {
        let temp_dir = TempDir::new().unwrap();
        let validator = DebianValidator::new(false);

        let result = validator
            .validate_debian_directory(&temp_dir.path().join("nonexistent"), "test_package");

        assert!(result.is_err());
    }

    #[test]
    fn test_empty_debian_directory() {
        let temp_dir = TempDir::new().unwrap();
        let debian_dir = create_test_debian_dir(&temp_dir);
        let validator = DebianValidator::new(false);

        let result = validator
            .validate_debian_directory(&debian_dir, "test_package")
            .unwrap();

        assert!(!result.is_valid);
        assert_eq!(result.missing_files.len(), REQUIRED_FILES.len());
    }

    #[test]
    fn test_complete_debian_directory() {
        let temp_dir = TempDir::new().unwrap();
        let debian_dir = create_test_debian_dir(&temp_dir);

        // Create all required files
        for &file in REQUIRED_FILES {
            let file_path = debian_dir.join(file);
            match file {
                "control" => fs::write(
                    &file_path,
                    "Package: test\nArchitecture: any\nMaintainer: Test \
                     <test@example.com>\nDescription: Test package",
                )
                .unwrap(),
                "changelog" => fs::write(
                    &file_path,
                    "test (1.0.0-1) unstable; urgency=medium\n\n  * Initial release\n\n -- Test \
                     <test@example.com>  Mon, 01 Jan 2024 00:00:00 +0000",
                )
                .unwrap(),
                "copyright" => fs::write(&file_path, "Copyright 2024 Test\nLicense: MIT").unwrap(),
                "rules" => {
                    fs::write(&file_path, "#!/usr/bin/make -f\n\nbuild:\n\nbinary:\n\nclean:\n")
                        .unwrap();
                    #[cfg(unix)]
                    {
                        use std::os::unix::fs::PermissionsExt;
                        let mut perms = fs::metadata(&file_path).unwrap().permissions();
                        perms.set_mode(0o755);
                        fs::set_permissions(&file_path, perms).unwrap();
                    }
                }
                _ => fs::write(&file_path, "test content").unwrap(),
            }
        }

        let validator = DebianValidator::new(false);
        let result = validator
            .validate_debian_directory(&debian_dir, "test_package")
            .unwrap();

        assert!(result.is_valid);
        assert_eq!(result.found_files.len(), REQUIRED_FILES.len());
        assert!(result.missing_files.is_empty());
    }

    #[test]
    fn test_strict_mode() {
        let temp_dir = TempDir::new().unwrap();
        let debian_dir = create_test_debian_dir(&temp_dir);

        let validator = DebianValidator::new(true);
        let result = validator.validate_debian_directory(&debian_dir, "test_package");

        // Should fail in strict mode due to missing files
        assert!(result.is_err());
    }

    #[test]
    fn test_list_debian_files() {
        let temp_dir = TempDir::new().unwrap();
        let debian_dir = create_test_debian_dir(&temp_dir);

        // Create some files
        fs::write(debian_dir.join("control"), "test").unwrap();
        fs::write(debian_dir.join("rules"), "test").unwrap();

        let validator = DebianValidator::new(false);
        let files = validator.list_debian_files(&debian_dir).unwrap();

        assert!(files.contains(&PathBuf::from("control")));
        assert!(files.contains(&PathBuf::from("rules")));
    }

    #[test]
    fn test_is_complete_debian_directory() {
        let temp_dir = TempDir::new().unwrap();
        let debian_dir = create_test_debian_dir(&temp_dir);

        let validator = DebianValidator::new(false);

        // Initially incomplete
        assert!(!validator.is_complete_debian_directory(&debian_dir));

        // Create all required files
        for &file in REQUIRED_FILES {
            fs::write(debian_dir.join(file), "test").unwrap();
        }

        // Now complete
        assert!(validator.is_complete_debian_directory(&debian_dir));
    }
}
