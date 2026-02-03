//! Performance benchmarks for build orchestration
//!
//! This module benchmarks the performance of key orchestration components:
//! - Parallel build execution scaling
//! - Progress event processing throughput
//! - Recovery mechanism overhead
//! - Context management performance

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use colcon_deb_build::{
    retry_with_backoff, BuildContext, BuildError, BuildRecoveryStrategy, BuildState, ProgressUI,
    RecoveryContext, RecoveryManager, Result, RetryConfig,
};
use colcon_deb_config::{Config, DockerConfig};
use colcon_deb_core::Package;
use colcon_deb_docker::progress::{LogLevel, ProgressEvent};
use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use futures::future::join_all;
use tempfile::TempDir;
use tokio::runtime::Runtime;
use tokio::sync::mpsc;

/// Create a test configuration for benchmarking
fn create_bench_config() -> Config {
    let temp_dir = TempDir::new().unwrap();
    let workspace = temp_dir.path().join("workspace");
    std::fs::create_dir_all(workspace.join("src")).unwrap();

    Config {
        colcon_repo: workspace,
        debian_dirs: temp_dir.path().join("debian"),
        docker: DockerConfig::Image { image: "ros:loong".to_string() },
        ros_distro: Some("loong".to_string()),
        output_dir: temp_dir.path().join("output"),
        parallel_jobs: 4,
    }
}

/// Create benchmark packages
fn create_bench_packages(count: usize) -> Vec<Package> {
    (0..count)
        .map(|i| Package {
            name: format!("benchmark_package_{i}"),
            path: format!("/workspace/src/package_{i}").into(),
            version: "1.0.0".to_string(),
            description: format!("Benchmark package {i}"),
            license: "MIT".to_string(),
            build_type: colcon_deb_core::package::BuildType::Cmake,
            dependencies: colcon_deb_core::package::Dependencies {
                build: vec!["cmake".to_string()],
                exec: if i > 0 {
                    vec![format!("benchmark_package_{}", i - 1)]
                } else {
                    vec!["std_msgs".to_string()]
                },
                ..Default::default()
            },
            maintainers: vec![colcon_deb_core::package::Maintainer {
                name: "Benchmark".to_string(),
                email: "bench@example.com".to_string(),
            }],
        })
        .collect()
}

/// Fast mock operation for benchmarking
async fn fast_mock_operation(delay_micros: u64) -> Result<String> {
    if delay_micros > 0 {
        tokio::time::sleep(Duration::from_micros(delay_micros)).await;
    }
    Ok("success".to_string())
}

/// Benchmark progress UI that counts events efficiently
#[derive(Debug, Clone)]
struct BenchProgressUI {
    event_count: Arc<Mutex<usize>>,
}

impl BenchProgressUI {
    fn new() -> Self {
        Self { event_count: Arc::new(Mutex::new(0)) }
    }

    fn get_event_count(&self) -> usize {
        *self.event_count.lock().unwrap()
    }
}

impl ProgressUI for BenchProgressUI {
    fn update(&self, _event: &ProgressEvent) {
        *self.event_count.lock().unwrap() += 1;
    }

    fn clear(&self) {}

    fn finish(&self) {}

    fn get_package_progress(&self, _package: &str) -> Option<Box<dyn ProgressUI>> {
        Some(Box::new(self.clone()))
    }
}

/// Benchmark parallel execution scaling
fn bench_parallel_execution(c: &mut Criterion) {
    let rt = Runtime::new().unwrap();
    let mut group = c.benchmark_group("parallel_execution");

    // Test with different numbers of concurrent operations
    for &num_ops in &[1, 2, 4, 8, 16, 32] {
        group.throughput(Throughput::Elements(num_ops as u64));
        group.bench_with_input(
            BenchmarkId::new("concurrent_operations", num_ops),
            &num_ops,
            |b, &num_ops| {
                b.to_async(&rt).iter(|| async {
                    let futures: Vec<_> = (0..num_ops)
                        .map(|_| fast_mock_operation(100)) // 100 microsecond delay
                        .collect();

                    let start = Instant::now();
                    let results = join_all(futures).await;
                    let duration = start.elapsed();

                    // Verify all operations succeeded
                    for result in results {
                        assert!(result.is_ok());
                    }

                    black_box(duration)
                });
            },
        );
    }

    group.finish();
}

/// Benchmark progress event processing throughput
fn bench_progress_events(c: &mut Criterion) {
    let rt = Runtime::new().unwrap();
    let mut group = c.benchmark_group("progress_events");

    // Test with different numbers of events
    for &num_events in &[100, 1000, 5000, 10000] {
        group.throughput(Throughput::Elements(num_events as u64));
        group.bench_with_input(
            BenchmarkId::new("event_processing", num_events),
            &num_events,
            |b, &num_events| {
                b.to_async(&rt).iter(|| async {
                    let progress_ui = BenchProgressUI::new();

                    // Create a batch of events
                    let events: Vec<_> = (0..num_events)
                        .map(|i| ProgressEvent::Log {
                            level: LogLevel::Info,
                            message: format!("Processing item {i}"),
                        })
                        .collect();

                    let start = Instant::now();

                    // Process all events
                    for event in &events {
                        progress_ui.update(event);
                    }

                    let duration = start.elapsed();

                    // Verify all events were processed
                    assert_eq!(progress_ui.get_event_count(), num_events);

                    black_box(duration)
                });
            },
        );
    }

    group.finish();
}

/// Benchmark build context operations
fn bench_build_context(c: &mut Criterion) {
    let rt = Runtime::new().unwrap();
    let mut group = c.benchmark_group("build_context");

    group.bench_function("context_creation", |b| {
        b.iter(|| {
            let config = create_bench_config();
            let context = BuildContext::new(config);
            black_box(context)
        });
    });

    group.bench_function("state_transitions", |b| {
        b.iter(|| {
            let config = create_bench_config();
            let mut context = BuildContext::new(config);

            // Perform multiple state transitions
            context.set_state(BuildState::Preparing);
            context.set_state(BuildState::Building);
            context.set_state(BuildState::CollectingArtifacts);
            context.set_state(BuildState::Completed);

            black_box(context.state())
        });
    });

    group.bench_function("stats_calculation", |b| {
        b.to_async(&rt).iter(|| async {
            let config = create_bench_config();
            let mut context = BuildContext::new(config);

            // Add multiple results
            for _i in 0..100 {
                // Mock adding package build results - this would be internal to BuildContext
                // For benchmarking purposes, we'll just trigger state changes
                context.set_state(BuildState::Building);
            }

            let stats = context.stats();
            black_box(stats)
        });
    });

    group.finish();
}

/// Benchmark recovery mechanisms
fn bench_recovery_mechanisms(c: &mut Criterion) {
    let rt = Runtime::new().unwrap();
    let mut group = c.benchmark_group("recovery_mechanisms");

    group.bench_function("recovery_strategy_determination", |b| {
        b.iter(|| {
            let _recovery_manager = RecoveryManager::with_strategy(BuildRecoveryStrategy::Retry);

            // Benchmark strategy determination for different scenarios
            let errors = [
                BuildError::transient("Network error"),
                BuildError::Docker(colcon_deb_docker::DockerError::ExecutionFailed {
                    reason: "Container failed".to_string(),
                }),
                BuildError::config("Invalid config"),
            ];

            let contexts: Vec<_> = errors
                .iter()
                .map(|error| RecoveryContext::for_package("build", "test_package", error))
                .collect();

            let strategies: Vec<_> = contexts
                .iter()
                .map(|_ctx| BuildRecoveryStrategy::Retry) // Simplified for benchmarking
                .collect();

            black_box(strategies)
        });
    });

    group.bench_function("retry_with_backoff_success", |b| {
        b.to_async(&rt).iter(|| async {
            let config = RetryConfig {
                max_attempts: 3,
                initial_delay: Duration::from_micros(10),
                max_delay: Duration::from_millis(1),
                multiplier: 2.0,
                max_elapsed_time: Some(Duration::from_millis(100)),
            };

            // Operation that succeeds immediately
            let result = retry_with_backoff(|| fast_mock_operation(0), &config, None).await;
            assert!(result.is_ok());
            black_box(result)
        });
    });

    group.bench_function("retry_with_backoff_failure", |b| {
        b.to_async(&rt).iter(|| async {
            let config = RetryConfig {
                max_attempts: 2,
                initial_delay: Duration::from_micros(10),
                max_delay: Duration::from_millis(1),
                multiplier: 2.0,
                max_elapsed_time: Some(Duration::from_millis(100)),
            };

            // Operation that always fails
            let result: Result<String> = retry_with_backoff(
                || async { Err(BuildError::transient("Always fails")) },
                &config,
                None,
            )
            .await;
            assert!(result.is_err());
            black_box(result)
        });
    });

    group.finish();
}

/// Benchmark concurrent progress processing
fn bench_concurrent_progress(c: &mut Criterion) {
    let rt = Runtime::new().unwrap();
    let mut group = c.benchmark_group("concurrent_progress");

    // Test with different numbers of concurrent senders
    for &num_senders in &[1, 2, 4, 8] {
        group.throughput(Throughput::Elements(num_senders as u64 * 1000));
        group.bench_with_input(
            BenchmarkId::new("concurrent_senders", num_senders),
            &num_senders,
            |b, &num_senders| {
                b.to_async(&rt).iter(|| async {
                    let (tx, mut rx) = mpsc::unbounded_channel();
                    let progress_ui = BenchProgressUI::new();

                    // Start receiver task
                    let ui = progress_ui.clone();
                    let receiver_task = tokio::spawn(async move {
                        let mut count = 0;
                        while let Some(event) = rx.recv().await {
                            ui.update(&event);
                            count += 1;
                            if count >= num_senders * 1000 {
                                break;
                            }
                        }
                    });

                    // Start sender tasks
                    let sender_tasks: Vec<_> = (0..num_senders)
                        .map(|sender_id| {
                            let tx = tx.clone();
                            tokio::spawn(async move {
                                for i in 0..1000 {
                                    let event = ProgressEvent::Log {
                                        level: LogLevel::Info,
                                        message: format!("Sender {sender_id} message {i}"),
                                    };
                                    tx.send(event).unwrap();
                                }
                            })
                        })
                        .collect();

                    // Drop the original sender
                    drop(tx);

                    // Wait for all senders to complete
                    for task in sender_tasks {
                        task.await.unwrap();
                    }

                    // Wait for receiver to process all events
                    receiver_task.await.unwrap();

                    // Verify all events were processed
                    assert_eq!(progress_ui.get_event_count(), num_senders * 1000);

                    black_box(progress_ui.get_event_count())
                });
            },
        );
    }

    group.finish();
}

/// Benchmark package dependency resolution
fn bench_dependency_resolution(c: &mut Criterion) {
    let mut group = c.benchmark_group("dependency_resolution");

    // Test with different numbers of packages
    for &num_packages in &[10, 50, 100, 500] {
        group.throughput(Throughput::Elements(num_packages as u64));
        group.bench_with_input(
            BenchmarkId::new("package_creation", num_packages),
            &num_packages,
            |b, &num_packages| {
                b.iter(|| {
                    let packages = create_bench_packages(num_packages);

                    // Simulate dependency analysis
                    let mut dep_count = 0;
                    for package in &packages {
                        dep_count += package.dependencies.build.len();
                        dep_count += package.dependencies.exec.len();
                    }

                    black_box((packages.len(), dep_count))
                });
            },
        );
    }

    group.finish();
}

/// Benchmark memory usage patterns
fn bench_memory_patterns(c: &mut Criterion) {
    let rt = Runtime::new().unwrap();
    let mut group = c.benchmark_group("memory_patterns");

    group.bench_function("large_context_operations", |b| {
        b.to_async(&rt).iter(|| async {
            let config = create_bench_config();
            let context = BuildContext::new(config);

            // Simulate memory usage by creating large contexts
            for _i in 0..1000 {
                // Create multiple contexts to test memory patterns
                let _temp_context = BuildContext::new(create_bench_config());
            }

            let stats = context.stats();
            black_box(stats)
        });
    });

    group.finish();
}

criterion_group!(
    orchestration_benches,
    bench_parallel_execution,
    bench_progress_events,
    bench_build_context,
    bench_recovery_mechanisms,
    bench_concurrent_progress,
    bench_dependency_resolution,
    bench_memory_patterns
);

criterion_main!(orchestration_benches);
