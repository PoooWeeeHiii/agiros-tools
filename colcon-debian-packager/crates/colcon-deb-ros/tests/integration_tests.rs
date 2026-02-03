use std::path::Path;

use colcon_deb_core::package::BuildType;
use colcon_deb_ros::{find_package_paths, scan_workspace};

#[test]
fn test_scan_real_workspace() {
    let workspace_path = Path::new("../../test_workspace/src");
    assert!(
        workspace_path.exists(),
        "test_workspace/src must exist. Please create it with: make create-test-workspace"
    );

    let packages = scan_workspace(workspace_path).expect("Failed to scan workspace");

    // Should find 4 packages (ignored_package should be skipped)
    assert_eq!(packages.len(), 4);

    // Check packages are sorted by name
    let names: Vec<&str> = packages.iter().map(|p| p.name.as_str()).collect();
    assert_eq!(
        names,
        vec![
            "edge_cases",
            "python_node",
            "simple_publisher",
            "simple_subscriber"
        ]
    );

    // Verify simple_publisher
    let publisher = packages
        .iter()
        .find(|p| p.name == "simple_publisher")
        .unwrap();
    assert_eq!(publisher.version, "0.1.0");
    assert_eq!(publisher.build_type, BuildType::AmentCmake);
    assert_eq!(publisher.maintainers.len(), 1);
    assert_eq!(publisher.maintainers[0].email, "maintainer@example.com");
    assert!(publisher.dependencies.build.contains(&"rclcpp".to_string()));
    assert!(publisher
        .dependencies
        .build
        .contains(&"std_msgs".to_string()));

    // Verify simple_subscriber with version constraints
    let subscriber = packages
        .iter()
        .find(|p| p.name == "simple_subscriber")
        .unwrap();
    assert_eq!(subscriber.version, "0.2.1");
    assert_eq!(subscriber.license, "MIT, BSD");
    assert!(subscriber
        .dependencies
        .build
        .contains(&"simple_publisher".to_string()));
    assert!(subscriber
        .dependencies
        .build
        .contains(&"libboost-system-dev".to_string()));
    assert!(subscriber
        .dependencies
        .build
        .contains(&"python3-numpy".to_string()));

    // Verify python_node
    let python = packages.iter().find(|p| p.name == "python_node").unwrap();
    assert_eq!(python.version, "1.0.0");
    assert_eq!(python.build_type, BuildType::AmentPython);
    // Check that generic 'depend' was expanded
    assert!(python.dependencies.build.contains(&"rclpy".to_string()));
    assert!(python.dependencies.exec.contains(&"rclpy".to_string()));
    assert!(python
        .dependencies
        .build_export
        .contains(&"python3-yaml".to_string()));

    // Verify edge_cases
    let edge = packages.iter().find(|p| p.name == "edge_cases").unwrap();
    assert_eq!(edge.version, "0.0.1-alpha");
    assert_eq!(edge.build_type, BuildType::Cmake); // Inferred from cmake buildtool
                                                   // Check generic depend expansion
    assert!(edge
        .dependencies
        .build
        .contains(&"visualization_msgs".to_string()));
    assert!(edge
        .dependencies
        .exec
        .contains(&"visualization_msgs".to_string()));
}

#[test]
fn test_find_package_paths_integration() {
    let workspace_path = Path::new("../../test_workspace/src");
    assert!(
        workspace_path.exists(),
        "test_workspace/src must exist. Please create it with: make create-test-workspace"
    );

    let paths = find_package_paths(workspace_path).expect("Failed to find package paths");

    // Should find 4 packages
    assert_eq!(paths.len(), 4);

    // Check that ignored_package is not included
    let path_names: Vec<String> = paths
        .iter()
        .map(|p| p.file_name().unwrap().to_string_lossy().to_string())
        .collect();
    assert!(!path_names.contains(&"ignored_package".to_string()));
    assert!(path_names.contains(&"simple_publisher".to_string()));
    assert!(path_names.contains(&"simple_subscriber".to_string()));
    assert!(path_names.contains(&"python_node".to_string()));
    assert!(path_names.contains(&"edge_cases".to_string()));
}

#[test]
fn test_rust_script_scanner() {
    use std::process::Command;

    let output = Command::new("rust-script")
        .arg("../../scripts/helpers/package-scanner.rs")
        .arg("--format")
        .arg("json")
        .arg("../../test_workspace/src")
        .output();

    let output = output.expect("rust-script must be installed. Run: cargo install rust-script");
    assert!(
        output.status.success(),
        "rust-script failed: {:?}",
        String::from_utf8_lossy(&output.stderr)
    );

    let json = String::from_utf8(output.stdout).expect("Invalid UTF-8");

    // Try to parse JSON, print debug info if it fails
    let packages: serde_json::Value = match serde_json::from_str(&json) {
        Ok(v) => v,
        Err(e) => {
            eprintln!("Failed to parse JSON: {e}");
            eprintln!("JSON output: {json}");
            panic!("Invalid JSON output from rust-script");
        }
    };

    assert!(packages.is_object(), "Expected object, got: {packages:?}");
    let packages_obj = packages.as_object().unwrap();

    let packages_array = packages_obj
        .get("packages")
        .expect("Missing 'packages' field")
        .as_array()
        .expect("'packages' field is not an array");

    assert_eq!(packages_array.len(), 4);
}

#[test]
fn test_dependency_mapping() {
    use colcon_deb_core::dependency::Dependency;

    // Test ROS package name mapping
    let ros_dep = Dependency::new("simple_publisher");
    assert_eq!(ros_dep.to_debian_name("loong"), "agiros-loong-simple-publisher");

    // Test system package name (with hyphen)
    let system_dep = Dependency::new("libboost-system-dev");
    assert_eq!(system_dep.to_debian_name("loong"), "libboost-system-dev");

    let python_dep = Dependency::new("python3-numpy");
    assert_eq!(python_dep.to_debian_name("loong"), "python3-numpy");

    // Test underscores in ROS packages
    let underscore_dep = Dependency::new("sensor_msgs");
    assert_eq!(underscore_dep.to_debian_name("loong"), "agiros-loong-sensor-msgs");
}
