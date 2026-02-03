use colcon_deb_ros::parse_package_manifest;

#[test]
fn test_parse_simple_publisher() {
    let xml = r#"<?xml version="1.0"?>
<package format="3">
  <name>simple_publisher</name>
  <version>0.1.0</version>
  <description>A simple ROS 2 publisher example</description>
  <maintainer email="test@example.com">Test Maintainer</maintainer>
  <license>Apache-2.0</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>"#;

    let manifest = parse_package_manifest(xml).expect("Failed to parse package.xml");

    assert_eq!(manifest.name, "simple_publisher");
    assert_eq!(manifest.version, "0.1.0");
    assert_eq!(manifest.description, "A simple ROS 2 publisher example");
    assert_eq!(manifest.licenses, vec!["Apache-2.0"]);
    assert_eq!(manifest.maintainers.len(), 1);
    assert_eq!(manifest.maintainers[0].name, "Test Maintainer");
    assert_eq!(manifest.maintainers[0].email, Some("test@example.com".to_string()));
    assert_eq!(manifest.build_type, Some("ament_cmake".to_string()));

    // Check dependencies
    assert_eq!(manifest.dependencies.buildtool_depend.len(), 1);
    assert_eq!(manifest.dependencies.buildtool_depend[0].name, "ament_cmake");

    // Generic 'depend' should be expanded
    assert!(manifest
        .dependencies
        .build_depend
        .iter()
        .any(|d| d.name == "rclcpp"));
    assert!(manifest
        .dependencies
        .exec_depend
        .iter()
        .any(|d| d.name == "std_msgs"));

    assert_eq!(manifest.dependencies.test_depend.len(), 2);
}

#[test]
fn test_parse_python_package() {
    let xml = r#"<?xml version="1.0"?>
<package format="3">
  <name>python_node</name>
  <version>0.2.0</version>
  <description>A Python ROS 2 node example</description>
  <maintainer email="python@example.com">Python Maintainer</maintainer>
  <license>MIT</license>
  
  <buildtool_depend>ament_python</buildtool_depend>
  
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>python3-numpy</exec_depend>
  
  <test_depend>python3-pytest</test_depend>
  
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>"#;

    let manifest = parse_package_manifest(xml).expect("Failed to parse package.xml");

    assert_eq!(manifest.name, "python_node");
    assert_eq!(manifest.version, "0.2.0");
    assert_eq!(manifest.build_type, Some("ament_python".to_string()));

    // Check dependencies
    assert_eq!(manifest.dependencies.buildtool_depend.len(), 1);
    assert_eq!(manifest.dependencies.buildtool_depend[0].name, "ament_python");

    assert_eq!(manifest.dependencies.exec_depend.len(), 3);
    assert!(manifest
        .dependencies
        .exec_depend
        .iter()
        .any(|d| d.name == "python3-numpy"));
}

#[test]
fn test_parse_with_version_constraints() {
    let xml = r#"<?xml version="1.0"?>
<package format="3">
  <name>test_pkg</name>
  <version>1.0.0</version>
  <description>Test package with version constraints</description>
  <maintainer email="test@test.com">Tester</maintainer>
  <license>MIT</license>
  
  <depend version_gte="2.0.0" version_lt="3.0.0">some_package</depend>
  <build_depend version_eq="1.2.3">exact_version</build_depend>
  <exec_depend condition="$ROS_VERSION == 2">conditional_dep</exec_depend>
</package>"#;

    let manifest = parse_package_manifest(xml).expect("Failed to parse package.xml");

    // Check version constraints
    let some_pkg_deps: Vec<_> = manifest
        .dependencies
        .build_depend
        .iter()
        .filter(|d| d.name == "some_package")
        .collect();
    assert!(!some_pkg_deps.is_empty());
    assert_eq!(some_pkg_deps[0].version_gte, Some("2.0.0".to_string()));
    assert_eq!(some_pkg_deps[0].version_lt, Some("3.0.0".to_string()));

    // Check exact version
    let exact_deps: Vec<_> = manifest
        .dependencies
        .build_depend
        .iter()
        .filter(|d| d.name == "exact_version")
        .collect();
    assert!(!exact_deps.is_empty());
    assert_eq!(exact_deps[0].version_eq, Some("1.2.3".to_string()));

    // Check conditional dependency
    let cond_deps: Vec<_> = manifest
        .dependencies
        .exec_depend
        .iter()
        .filter(|d| d.name == "conditional_dep")
        .collect();
    assert!(!cond_deps.is_empty());
    assert_eq!(cond_deps[0].condition, Some("$ROS_VERSION == 2".to_string()));
}

#[test]
fn test_missing_required_fields() {
    let xml = r#"<?xml version="1.0"?>
<package format="3">
  <name>test_pkg</name>
  <version>1.0.0</version>
  <license>MIT</license>
</package>"#;

    // Should fail due to missing description and maintainer
    let result = parse_package_manifest(xml);
    assert!(result.is_err());
}

#[test]
fn test_parse_urls() {
    let xml = r#"<?xml version="1.0"?>
<package format="3">
  <name>test_pkg</name>
  <version>1.0.0</version>
  <description>Test package</description>
  <maintainer email="test@test.com">Tester</maintainer>
  <license>MIT</license>
  
  <url type="website">https://example.com</url>
  <url type="repository">https://github.com/example/repo</url>
  <url type="bugtracker">https://github.com/example/repo/issues</url>
</package>"#;

    let manifest = parse_package_manifest(xml).expect("Failed to parse package.xml");

    assert_eq!(manifest.urls.len(), 3);

    let website = manifest
        .urls
        .iter()
        .find(|u| u.url_type == Some("website".to_string()));
    assert!(website.is_some());
    assert_eq!(website.unwrap().url, "https://example.com");
}

#[test]
fn test_multiple_licenses() {
    let xml = r#"<?xml version="1.0"?>
<package format="3">
  <name>test_pkg</name>
  <version>1.0.0</version>
  <description>Test package</description>
  <maintainer email="test@test.com">Tester</maintainer>
  <license>MIT</license>
  <license>Apache-2.0</license>
</package>"#;

    let manifest = parse_package_manifest(xml).expect("Failed to parse package.xml");

    assert_eq!(manifest.licenses.len(), 2);
    assert!(manifest.licenses.contains(&"MIT".to_string()));
    assert!(manifest.licenses.contains(&"Apache-2.0".to_string()));
}
