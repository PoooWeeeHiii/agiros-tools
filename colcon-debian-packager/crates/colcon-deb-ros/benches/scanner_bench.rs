use std::path::Path;

use colcon_deb_ros::{find_package_paths, scan_workspace};
use criterion::{black_box, criterion_group, criterion_main, Criterion};
use tempfile::TempDir;

fn create_test_package(dir: &Path, name: &str, deps: Vec<&str>) {
    let pkg_dir = dir.join(name);
    std::fs::create_dir_all(&pkg_dir).unwrap();

    let package_xml = format!(
        r#"<?xml version="1.0"?>
<package format="3">
  <name>{}</name>
  <version>1.0.0</version>
  <description>Test package {}</description>
  <maintainer email="test@example.com">Test</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  {}
</package>"#,
        name,
        name,
        deps.iter()
            .map(|d| format!("  <depend>{d}</depend>"))
            .collect::<Vec<_>>()
            .join("\n")
    );

    std::fs::write(pkg_dir.join("package.xml"), package_xml).unwrap();
}

fn create_large_workspace(num_packages: usize) -> TempDir {
    let temp_dir = TempDir::new().unwrap();
    let src_dir = temp_dir.path().join("src");
    std::fs::create_dir(&src_dir).unwrap();

    // Create packages with some dependencies
    for i in 0..num_packages {
        if i > 0 {
            let dep_name = format!("package_{}", i - 1);
            create_test_package(&src_dir, &format!("package_{i}"), vec![&dep_name]);
        } else {
            create_test_package(&src_dir, &format!("package_{i}"), vec![]);
        }
    }

    // Add some packages with COLCON_IGNORE
    for i in 0..5 {
        let pkg_name = format!("ignored_package_{i}");
        create_test_package(&src_dir, &pkg_name, vec![]);
        std::fs::write(src_dir.join(&pkg_name).join("COLCON_IGNORE"), "").unwrap();
    }

    temp_dir
}

fn benchmark_scan_workspace(c: &mut Criterion) {
    let small_workspace = create_large_workspace(10);
    let medium_workspace = create_large_workspace(50);
    let large_workspace = create_large_workspace(100);

    let mut group = c.benchmark_group("scan_workspace");

    group.bench_function("10_packages", |b| {
        b.iter(|| {
            let packages = scan_workspace(black_box(small_workspace.path().join("src").as_path()))
                .expect("Failed to scan");
            assert_eq!(packages.len(), 10);
        });
    });

    group.bench_function("50_packages", |b| {
        b.iter(|| {
            let packages = scan_workspace(black_box(medium_workspace.path().join("src").as_path()))
                .expect("Failed to scan");
            assert_eq!(packages.len(), 50);
        });
    });

    group.bench_function("100_packages", |b| {
        b.iter(|| {
            let packages = scan_workspace(black_box(large_workspace.path().join("src").as_path()))
                .expect("Failed to scan");
            assert_eq!(packages.len(), 100);
        });
    });

    group.finish();
}

fn benchmark_find_package_paths(c: &mut Criterion) {
    let small_workspace = create_large_workspace(10);
    let medium_workspace = create_large_workspace(50);
    let large_workspace = create_large_workspace(100);

    let mut group = c.benchmark_group("find_package_paths");

    group.bench_function("10_packages", |b| {
        b.iter(|| {
            let paths = find_package_paths(black_box(small_workspace.path().join("src").as_path()))
                .expect("Failed to find paths");
            assert_eq!(paths.len(), 10);
        });
    });

    group.bench_function("50_packages", |b| {
        b.iter(|| {
            let paths =
                find_package_paths(black_box(medium_workspace.path().join("src").as_path()))
                    .expect("Failed to find paths");
            assert_eq!(paths.len(), 50);
        });
    });

    group.bench_function("100_packages", |b| {
        b.iter(|| {
            let paths = find_package_paths(black_box(large_workspace.path().join("src").as_path()))
                .expect("Failed to find paths");
            assert_eq!(paths.len(), 100);
        });
    });

    group.finish();
}

fn benchmark_xml_parsing(c: &mut Criterion) {
    use colcon_deb_ros::parse_package_xml;

    // Create a complex package.xml for parsing benchmark
    let temp_dir = TempDir::new().unwrap();
    let complex_xml_path = temp_dir.path().join("package.xml");

    let complex_xml = r#"<?xml version="1.0"?>
<package format="3">
  <name>complex_package</name>
  <version>2.1.0-beta</version>
  <description>A complex package with many dependencies and metadata</description>
  
  <maintainer email="maintainer1@example.com">Main Tainer</maintainer>
  <maintainer email="maintainer2@example.com">Second Maintainer</maintainer>
  
  <author email="author1@example.com">First Author</author>
  <author email="author2@example.com">Second Author</author>
  <author>Third Author</author>
  
  <license>Apache-2.0</license>
  <license>MIT</license>
  
  <url type="website">https://example.com/complex_package</url>
  <url type="repository">https://github.com/example/complex_package</url>
  <url type="bugtracker">https://github.com/example/complex_package/issues</url>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_export_depend>ament_cmake_core</buildtool_export_depend>
  
  <build_depend version_gte="0.9.0">rclcpp</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>sensor_msgs</build_depend>
  <build_depend version_eq="1.0.0">geometry_msgs</build_depend>
  <build_depend>tf2_ros</build_depend>
  <build_depend>pcl_ros</build_depend>
  
  <build_export_depend>rclcpp</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  
  <exec_depend>rclcpp</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend condition="$ROS_DISTRO == loong">tf2_ros</exec_depend>
  
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <test_depend>ament_cmake_gtest</test_depend>
  
  <doc_depend>doxygen</doc_depend>
  <doc_depend>sphinx</doc_depend>
  
  <depend>nav_msgs</depend>
  <depend version_lte="2.0.0">visualization_msgs</depend>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>"#;

    std::fs::write(&complex_xml_path, complex_xml).unwrap();

    c.bench_function("parse_complex_package_xml", |b| {
        b.iter(|| {
            let package = parse_package_xml(black_box(&complex_xml_path)).expect("Failed to parse");
            assert_eq!(package.name, "complex_package");
        });
    });
}

criterion_group!(
    benches,
    benchmark_scan_workspace,
    benchmark_find_package_paths,
    benchmark_xml_parsing
);
criterion_main!(benches);
