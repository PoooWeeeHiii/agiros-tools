use std::path::Path;
use std::time::Instant;

use colcon_deb_ros::{find_package_paths, scan_workspace};

#[test]
fn test_scanning_performance_baseline() {
    let workspace_path = Path::new("../../test_workspace/src");
    assert!(
        workspace_path.exists(),
        "test_workspace/src must exist. Please create it with: make create-test-workspace"
    );

    // Warm up
    let _ = scan_workspace(workspace_path);

    // Measure scan_workspace performance
    let start = Instant::now();
    let packages = scan_workspace(workspace_path).expect("Failed to scan workspace");
    let scan_duration = start.elapsed();

    println!("scan_workspace took: {:?} for {} packages", scan_duration, packages.len());

    // Basic performance assertion - scanning 4 packages should be very fast
    assert!(
        scan_duration.as_millis() < 100,
        "Scanning {} packages took too long: {:?}",
        packages.len(),
        scan_duration
    );

    // Measure find_package_paths performance
    let start = Instant::now();
    let paths = find_package_paths(workspace_path).expect("Failed to find paths");
    let find_duration = start.elapsed();

    println!("find_package_paths took: {:?} for {} packages", find_duration, paths.len());

    // find_package_paths should be faster than full scan
    assert!(
        find_duration < scan_duration,
        "find_package_paths ({find_duration:?}) should be faster than scan_workspace \
         ({scan_duration:?})"
    );
}

#[test]
fn test_parallel_parsing_performance() {
    use std::fs;

    use rayon::prelude::*;

    let workspace_path = Path::new("../../test_workspace/src");
    assert!(
        workspace_path.exists(),
        "test_workspace/src must exist. Please create it with: make create-test-workspace"
    );

    // Find all package.xml files
    let package_files: Vec<_> = find_package_paths(workspace_path)
        .expect("Failed to find paths")
        .into_iter()
        .map(|p| p.join("package.xml"))
        .collect();

    // Sequential parsing
    let start = Instant::now();
    let sequential_results: Vec<_> = package_files
        .iter()
        .map(|path| {
            let content = fs::read_to_string(path).unwrap();
            colcon_deb_ros::parse_package_manifest(&content)
        })
        .collect();
    let sequential_duration = start.elapsed();

    // Parallel parsing
    let start = Instant::now();
    let parallel_results: Vec<_> = package_files
        .par_iter()
        .map(|path| {
            let content = fs::read_to_string(path).unwrap();
            colcon_deb_ros::parse_package_manifest(&content)
        })
        .collect();
    let parallel_duration = start.elapsed();

    println!("Sequential parsing: {sequential_duration:?}");
    println!("Parallel parsing: {parallel_duration:?}");
    println!("Number of packages: {}", package_files.len());

    // Verify same results
    assert_eq!(sequential_results.len(), parallel_results.len());

    // For small workspaces, parallel might not be faster due to overhead
    // but it should not be significantly slower (allow up to 20x slower for tiny
    // workspaces due to thread spawning overhead)
    assert!(
        parallel_duration.as_secs_f64() < sequential_duration.as_secs_f64() * 20.0,
        "Parallel parsing is too slow compared to sequential: {:?} vs {:?}. For {} packages, \
         parallel overhead is expected.",
        parallel_duration,
        sequential_duration,
        package_files.len()
    );
}

#[test]
fn test_colcon_ignore_performance() {
    use std::fs;

    use tempfile::TempDir;

    // Create a workspace with many ignored directories
    let temp_dir = TempDir::new().unwrap();
    let src_dir = temp_dir.path().join("src");
    fs::create_dir(&src_dir).unwrap();

    // Create some normal packages
    for i in 0..5 {
        let pkg_dir = src_dir.join(format!("package_{i}"));
        fs::create_dir(&pkg_dir).unwrap();
        fs::write(
            pkg_dir.join("package.xml"),
            format!(
                r#"<?xml version="1.0"?>
<package format="3">
  <name>package_{i}</name>
  <version>1.0.0</version>
  <description>Test</description>
  <maintainer email="test@example.com">Test</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
</package>"#
            ),
        )
        .unwrap();
    }

    // Create many ignored directories with deep nesting
    for i in 0..10 {
        let ignored_dir = src_dir.join(format!("ignored_{i}"));
        fs::create_dir(&ignored_dir).unwrap();
        fs::write(ignored_dir.join("COLCON_IGNORE"), "").unwrap();

        // Create deep directory structure that should be skipped
        let mut current = ignored_dir;
        for j in 0..10 {
            current = current.join(format!("subdir_{j}"));
            fs::create_dir(&current).unwrap();
            fs::write(
                current.join("package.xml"),
                "<package><name>should_not_be_found</name></package>",
            )
            .unwrap();
        }
    }

    // Measure scanning performance with COLCON_IGNORE
    let start = Instant::now();
    let packages = scan_workspace(&src_dir).expect("Failed to scan");
    let duration = start.elapsed();

    println!("Scanning with {count} ignored directories took: {duration:?}", count = 10);

    // Should only find the 5 non-ignored packages
    assert_eq!(packages.len(), 5);

    // Even with many ignored directories, scanning should be fast
    assert!(
        duration.as_millis() < 200,
        "Scanning with ignored directories took too long: {duration:?}"
    );
}

#[test]
fn test_large_package_xml_parsing() {
    use std::fmt::Write;

    // Create a large package.xml with many dependencies
    let mut xml = String::from(
        r#"<?xml version="1.0"?>
<package format="3">
  <name>large_package</name>
  <version>1.0.0</version>
  <description>Package with many dependencies</description>
  <maintainer email="test@example.com">Test</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
"#,
    );

    // Add many dependencies
    for i in 0..100 {
        writeln!(xml, "  <depend>dependency_{i}</depend>").unwrap();
    }

    xml.push_str("</package>");

    // Measure parsing performance
    let start = Instant::now();
    let manifest = colcon_deb_ros::parse_package_manifest(&xml).expect("Failed to parse");
    let duration = start.elapsed();

    println!("Parsing large package.xml with 100 dependencies took: {duration:?}");

    // Verify correct parsing
    assert_eq!(manifest.name, "large_package");

    // Parsing even large package.xml should be fast
    assert!(
        duration.as_millis() < 50,
        "Parsing large package.xml took too long: {duration:?}"
    );
}
