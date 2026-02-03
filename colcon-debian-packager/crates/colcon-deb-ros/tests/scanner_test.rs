use std::path::PathBuf;

use colcon_deb_core::package::BuildType;
use colcon_deb_ros::{find_package_paths, scan_workspace};

#[test]
fn test_scan_test_workspace() {
    let workspace_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .join("tests/test_workspace/src");

    if !workspace_path.exists() {
        eprintln!("Test workspace not found at {workspace_path:?}, skipping test");
        return;
    }

    let packages = scan_workspace(&workspace_path).expect("Failed to scan workspace");

    // Should find our test packages
    assert!(packages.len() >= 3, "Expected at least 3 packages, found {}", packages.len());

    // Check specific packages
    let simple_pub = packages.iter().find(|p| p.name == "simple_publisher");
    assert!(simple_pub.is_some(), "simple_publisher not found");
    let simple_pub = simple_pub.unwrap();
    assert_eq!(simple_pub.version, "0.1.0");
    assert_eq!(simple_pub.build_type, BuildType::AmentCmake);

    let simple_sub = packages.iter().find(|p| p.name == "simple_subscriber");
    assert!(simple_sub.is_some(), "simple_subscriber not found");

    let python_node = packages.iter().find(|p| p.name == "python_node");
    assert!(python_node.is_some(), "python_node not found");
    let python_node = python_node.unwrap();
    assert_eq!(python_node.version, "0.2.0");
    assert_eq!(python_node.build_type, BuildType::AmentPython);
}

#[test]
fn test_find_package_paths() {
    let workspace_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .join("tests/test_workspace/src");

    if !workspace_path.exists() {
        eprintln!("Test workspace not found at {workspace_path:?}, skipping test");
        return;
    }

    let paths = find_package_paths(&workspace_path).expect("Failed to find package paths");

    assert!(paths.len() >= 3, "Expected at least 3 package paths, found {}", paths.len());

    // Check that paths point to package directories
    for path in &paths {
        assert!(path.join("package.xml").exists(), "No package.xml in {path:?}");
    }
}

#[test]
fn test_colcon_ignore() {
    use std::fs;

    use tempfile::TempDir;

    let temp_dir = TempDir::new().unwrap();
    let src_dir = temp_dir.path().join("src");
    fs::create_dir(&src_dir).unwrap();

    // Create a normal package
    let pkg1_dir = src_dir.join("package1");
    fs::create_dir(&pkg1_dir).unwrap();
    fs::write(
        pkg1_dir.join("package.xml"),
        r#"<?xml version="1.0"?>
<package format="3">
  <name>package1</name>
  <version>1.0.0</version>
  <description>Test package 1</description>
  <maintainer email="test@test.com">Test</maintainer>
  <license>MIT</license>
</package>"#,
    )
    .unwrap();

    // Create an ignored package
    let pkg2_dir = src_dir.join("package2");
    fs::create_dir(&pkg2_dir).unwrap();
    fs::write(pkg2_dir.join("COLCON_IGNORE"), "").unwrap();
    fs::write(
        pkg2_dir.join("package.xml"),
        r#"<?xml version="1.0"?>
<package format="3">
  <name>package2</name>
  <version>1.0.0</version>
  <description>Test package 2 (ignored)</description>
  <maintainer email="test@test.com">Test</maintainer>
  <license>MIT</license>
</package>"#,
    )
    .unwrap();

    // Scan workspace
    let packages = scan_workspace(&src_dir).expect("Failed to scan workspace");

    // Should only find package1
    assert_eq!(packages.len(), 1);
    assert_eq!(packages[0].name, "package1");
}
