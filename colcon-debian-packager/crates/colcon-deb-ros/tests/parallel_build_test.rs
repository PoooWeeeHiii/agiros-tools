use std::path::Path;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use colcon_deb_ros::scan_workspace;
use rayon::prelude::*;

/// Simulate building a .deb package
fn simulate_deb_build(package_name: &str, build_time_ms: u64) -> Result<String, String> {
    // Simulate build time
    std::thread::sleep(Duration::from_millis(build_time_ms));

    // Simulate occasional failures
    if package_name.contains("fail") {
        Err(format!("Failed to build {package_name}"))
    } else {
        Ok(format!("{package_name}_0.1.0-1_amd64.deb"))
    }
}

#[test]
fn test_parallel_deb_creation() {
    let workspace_path = Path::new("../../test_workspace/src");
    assert!(
        workspace_path.exists(),
        "test_workspace/src must exist. Please create it with: make create-test-workspace"
    );

    let packages = scan_workspace(workspace_path).expect("Failed to scan workspace");
    assert!(!packages.is_empty(), "No packages found in workspace");

    // Track results
    let results = Arc::new(Mutex::new(Vec::new()));
    let errors = Arc::new(Mutex::new(Vec::new()));

    let start = Instant::now();

    // Use rayon for parallel execution
    packages.par_iter().for_each(|package| {
        let package_name = &package.name;
        let build_time = match package_name.as_str() {
            "simple_publisher" => 100,
            "simple_subscriber" => 150,
            "python_node" => 80,
            "edge_cases" => 120,
            _ => 100,
        };

        match simulate_deb_build(package_name, build_time) {
            Ok(deb_file) => {
                results
                    .lock()
                    .unwrap()
                    .push((package_name.clone(), deb_file));
            }
            Err(e) => {
                errors.lock().unwrap().push((package_name.clone(), e));
            }
        }
    });

    let elapsed = start.elapsed();

    // Verify results
    let results = results.lock().unwrap();
    let errors = errors.lock().unwrap();

    println!("Parallel build completed in {elapsed:?}");
    println!("Successfully built {} packages", results.len());
    println!("Failed to build {} packages", errors.len());

    // All packages should build successfully
    assert_eq!(results.len(), packages.len());
    assert_eq!(errors.len(), 0);

    // Verify parallel execution - should be faster than sequential
    // Sequential would take 100+150+80+120 = 450ms minimum
    // Parallel should take ~150ms (the longest build)
    assert!(elapsed < Duration::from_millis(300), "Parallel build too slow: {elapsed:?}");

    // Verify all expected .deb files
    for (pkg_name, deb_file) in results.iter() {
        assert!(deb_file.ends_with("_amd64.deb"));
        assert!(deb_file.contains(pkg_name));
    }
}

#[test]
fn test_parallel_build_with_dependencies() {
    use std::collections::HashMap;

    // Simulate a dependency graph where some packages must wait for others
    let mut dep_graph: HashMap<&str, Vec<&str>> = HashMap::new();
    dep_graph.insert("simple_subscriber", vec!["simple_publisher"]);

    // Track build order
    let build_order = Arc::new(Mutex::new(Vec::new()));
    let completed = Arc::new(Mutex::new(Vec::new()));

    // Simulate parallel build with dependency checking
    let packages = vec![
        "simple_publisher",
        "simple_subscriber",
        "python_node",
        "edge_cases",
    ];

    packages.par_iter().for_each(|&package| {
        // Check if dependencies are built
        if let Some(deps) = dep_graph.get(package) {
            // Wait for dependencies (in real implementation, this would be more
            // sophisticated)
            while !deps
                .iter()
                .all(|dep| completed.lock().unwrap().contains(&dep.to_string()))
            {
                std::thread::sleep(Duration::from_millis(10));
            }
        }

        build_order.lock().unwrap().push(package.to_string());

        // Simulate build
        let _ = simulate_deb_build(package, 50);

        completed.lock().unwrap().push(package.to_string());
    });

    let build_order = build_order.lock().unwrap();

    // Verify that simple_publisher was built before simple_subscriber
    let publisher_idx = build_order
        .iter()
        .position(|p| p == "simple_publisher")
        .unwrap();
    let subscriber_idx = build_order
        .iter()
        .position(|p| p == "simple_subscriber")
        .unwrap();

    assert!(
        publisher_idx < subscriber_idx,
        "simple_publisher should be built before simple_subscriber"
    );
}

#[test]
fn test_concurrent_build_limits() {
    use rayon::ThreadPoolBuilder;

    let workspace_path = Path::new("../../test_workspace/src");
    assert!(
        workspace_path.exists(),
        "test_workspace/src must exist. Please create it with: make create-test-workspace"
    );

    let packages = scan_workspace(workspace_path).expect("Failed to scan workspace");

    // Test with limited concurrency (2 threads)
    let pool = ThreadPoolBuilder::new().num_threads(2).build().unwrap();

    let active_builds = Arc::new(Mutex::new(0));
    let max_concurrent = Arc::new(Mutex::new(0));

    pool.install(|| {
        packages.par_iter().for_each(|package| {
            // Track concurrent builds
            {
                let mut active = active_builds.lock().unwrap();
                *active += 1;
                let mut max = max_concurrent.lock().unwrap();
                if *active > *max {
                    *max = *active;
                }
            }

            // Simulate build
            let _ = simulate_deb_build(&package.name, 100);

            // Decrement active builds
            {
                let mut active = active_builds.lock().unwrap();
                *active -= 1;
            }
        });
    });

    let max_concurrent = *max_concurrent.lock().unwrap();

    // With 2 threads, we should never have more than 2 concurrent builds
    assert!(max_concurrent <= 2, "Too many concurrent builds: {max_concurrent}");
    assert!(max_concurrent >= 1, "No concurrent builds detected");
}
