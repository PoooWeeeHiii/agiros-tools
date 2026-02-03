# Design Details - Build Orchestration

This document details the design of the build orchestration component (`colcon-deb-build`).

## Overview

The build orchestration layer runs on the **host system** and coordinates the entire build process. It manages container lifecycle, monitors progress, and collects artifacts, but **never directly builds packages**.

## Host-Side Architecture

The orchestrator follows a clear separation:
- **Host Orchestrator**: Manages containers, monitors progress, handles errors
- **Container Scripts**: Execute agirosdep, colcon, dpkg-deb commands

This ensures:
1. The host system remains clean (no ROS dependencies needed)
2. Builds are reproducible (consistent container environment)
3. Multiple ROS distros can be supported (different containers)

## Core Components

### 1. Build Orchestrator (Host-Side)

```rust
pub struct BuildOrchestrator {
    config: BuildConfig,
    docker: Arc<DockerService>,
    progress: ProgressUI,
    event_bus: EventBus,
}

impl BuildOrchestrator {
    pub async fn build_workspace(
        &self,
        source_path: &Path,
        output_path: &Path,
    ) -> Result<BuildResult> {
        // 1. Host-side validation
        self.validate_environment().await?;
        
        // 2. Prepare container configuration
        let container_spec = self.prepare_container(source_path, output_path)?;
        
        // 3. Launch build container
        let container = self.docker.create_and_start(container_spec).await?;
        
        // 4. Monitor container execution
        let result = self.monitor_build(container).await?;
        
        // 5. Collect artifacts from container
        let artifacts = self.collect_artifacts(output_path).await?;
        
        Ok(BuildResult {
            artifacts,
            logs: result.logs,
            duration: result.duration,
        })
    }
    
    async fn monitor_build(&self, container: Container) -> Result<ContainerResult> {
        // Stream logs to progress UI
        let log_stream = self.docker.attach_logs(&container).await?;
        
        // Parse progress from container output
        while let Some(line) = log_stream.next().await {
            self.parse_and_display_progress(line)?;
        }
        
        // Wait for container to finish
        let exit_code = self.docker.wait(&container).await?;
        
        if exit_code != 0 {
            eyre::bail!("Build failed with exit code {}", exit_code);
        }
        
        Ok(ContainerResult::success())
    }
}
```

### 2. Build Plan

```rust
pub struct BuildPlan {
    pub waves: Vec<BuildWave>,
    pub total_packages: usize,
    pub dependency_graph: DependencyGraph,
    pub cache_strategy: CacheStrategy,
}

pub struct BuildWave {
    pub wave_number: usize,
    pub packages: Vec<PackageBuildTask>,
    pub dependencies_met: HashSet<String>,
}

pub struct PackageBuildTask {
    pub package: Package,
    pub container_spec: ContainerSpec,
    pub cache_key: String,
    pub timeout: Duration,
}
```

### 3. Build Execution Engine

```rust
pub struct BuildExecutor {
    docker_pool: ContainerPool,
    max_parallel: usize,
    event_tx: mpsc::Sender<BuildEvent>,
}

impl BuildExecutor {
    pub async fn execute_wave(
        &self,
        wave: BuildWave,
    ) -> Result<Vec<PackageBuildResult>> {
        // Create futures for all packages in wave
        let build_futures: Vec<_> = wave.packages
            .into_iter()
            .map(|task| self.build_package(task))
            .collect();
        
        // Execute with concurrency limit
        let results = futures::stream::iter(build_futures)
            .buffer_unordered(self.max_parallel)
            .collect::<Vec<_>>()
            .await;
        
        // Process results
        self.process_wave_results(results).await
    }
    
    async fn build_package(
        &self,
        task: PackageBuildTask,
    ) -> Result<PackageBuildResult> {
        // Check cache
        if let Some(cached) = self.check_cache(&task).await? {
            return Ok(cached);
        }
        
        // Run build in container
        let result = self.docker_pool
            .run_container(task.container_spec)
            .await?;
        
        // Process output
        self.process_build_output(task.package, result).await
    }
}
```

## State Management

### 1. Build State Machine

```rust
pub enum BuildState {
    Initialized,
    Discovering,
    Planning,
    Building { current_wave: usize, total_waves: usize },
    Packaging,
    CreatingRepository,
    Completed,
    Failed { error: String },
}

pub struct BuildStateMachine {
    state: Arc<RwLock<BuildState>>,
    transitions: Vec<StateTransition>,
}

impl BuildStateMachine {
    pub async fn transition(&self, event: BuildEvent) -> Result<()> {
        // Validate transition
        // Update state
        // Emit state change event
    }
}
```

### 2. Progress Tracking

```rust
use indicatif::{MultiProgress, ProgressBar, ProgressStyle};
use tokio::sync::mpsc;

pub struct ProgressUI {
    multi: MultiProgress,
    overall_bar: ProgressBar,
    package_bars: DashMap<String, ProgressBar>,
    event_rx: mpsc::Receiver<ProgressEvent>,
}

impl ProgressUI {
    pub fn new(total_packages: usize) -> (Self, mpsc::Sender<ProgressEvent>) {
        let multi = MultiProgress::new();
        let overall_bar = multi.add(ProgressBar::new(total_packages as u64));
        
        overall_bar.set_style(
            ProgressStyle::with_template(
                "{spinner:.green} [{elapsed_precise}] [{bar:40.cyan/blue}] {pos}/{len} packages"
            )?
            .progress_chars("#>-")
        );
        
        let (tx, rx) = mpsc::channel(100);
        
        (Self {
            multi,
            overall_bar,
            package_bars: DashMap::new(),
            event_rx: rx,
        }, tx)
    }
    
    pub async fn run(mut self) {
        while let Some(event) = self.event_rx.recv().await {
            match event {
                ProgressEvent::PackageStarted { name } => {
                    let pb = self.multi.add(ProgressBar::new_spinner());
                    pb.set_style(
                        ProgressStyle::with_template("{spinner:.yellow} {msg}")
                            .unwrap()
                    );
                    pb.set_message(format!("Building {}", name));
                    self.package_bars.insert(name, pb);
                }
                ProgressEvent::PackageCompleted { name, success } => {
                    if let Some((_, pb)) = self.package_bars.remove(&name) {
                        if success {
                            pb.finish_with_message(format!("✓ {} built", name));
                        } else {
                            pb.finish_with_message(format!("✗ {} failed", name));
                        }
                    }
                    self.overall_bar.inc(1);
                }
                ProgressEvent::LogMessage { package, message } => {
                    if let Some(pb) = self.package_bars.get(&package) {
                        pb.set_message(format!("{}: {}", package, message));
                    }
                }
            }
        }
    }
}
```

## Caching System

### 1. Cache Strategy

```rust
pub enum CacheStrategy {
    None,
    Conservative, // Only cache if inputs unchanged
    Aggressive,   // Cache all successful builds
}

pub struct BuildCache {
    strategy: CacheStrategy,
    storage: CacheStorage,
}

impl BuildCache {
    pub async fn get(&self, key: &CacheKey) -> Option<CachedBuild> {
        match self.strategy {
            CacheStrategy::None => None,
            _ => self.storage.get(key).await,
        }
    }
    
    pub async fn put(&self, key: CacheKey, result: BuildResult) -> Result<()> {
        if self.should_cache(&result) {
            self.storage.put(key, result).await?;
        }
        Ok(())
    }
}
```

### 2. Cache Key Generation

```rust
pub struct CacheKey {
    package_name: String,
    package_version: String,
    source_hash: String,
    dependencies_hash: String,
    build_config_hash: String,
}

impl CacheKey {
    pub fn generate(package: &Package, config: &BuildConfig) -> Self {
        // Hash package source
        // Hash dependencies
        // Hash build configuration
    }
}
```

## Error Handling and Recovery

### 1. Error Recovery Strategy

```rust
pub struct ErrorRecovery {
    max_retries: usize,
    retry_delay: Duration,
    continue_on_error: bool,
}

impl ErrorRecovery {
    pub async fn handle_build_failure(
        &self,
        task: &PackageBuildTask,
        error: BuildError,
        attempt: usize,
    ) -> Result<RecoveryAction> {
        match error {
            BuildError::Timeout => {
                if attempt < self.max_retries {
                    Ok(RecoveryAction::Retry { delay: self.retry_delay })
                } else {
                    Ok(RecoveryAction::Fail)
                }
            }
            BuildError::DependencyMissing(_) => {
                Ok(RecoveryAction::Skip)
            }
            _ => {
                if self.continue_on_error {
                    Ok(RecoveryAction::Continue)
                } else {
                    Ok(RecoveryAction::Fail)
                }
            }
        }
    }
}
```

### 2. Rollback Mechanism

```rust
pub struct BuildRollback {
    checkpoints: Vec<BuildCheckpoint>,
}

impl BuildRollback {
    pub async fn create_checkpoint(&mut self, state: BuildState) -> Result<()>;
    pub async fn rollback_to(&self, checkpoint: &BuildCheckpoint) -> Result<()>;
}
```

## Event System

### 1. Event Bus

```rust
pub struct EventBus {
    subscribers: Arc<RwLock<HashMap<EventType, Vec<EventHandler>>>>,
}

#[async_trait]
pub trait EventHandler: Send + Sync {
    async fn handle(&self, event: BuildEvent);
}

pub enum BuildEvent {
    StateChanged { from: BuildState, to: BuildState },
    PackageStarted { name: String },
    PackageCompleted { name: String, success: bool, duration: Duration },
    OutputReceived { package: String, line: String },
    Error { package: Option<String>, error: String },
}
```

### 2. Event Handlers

```rust
pub struct LoggingHandler {
    logger: slog::Logger,
}

pub struct MetricsHandler {
    metrics: Arc<Metrics>,
}

pub struct UIHandler {
    ui: Arc<Mutex<UI>>,
}
```

## Resource Management

### 1. Resource Limits

```rust
pub struct ResourceLimits {
    max_memory_per_container: usize,
    max_cpu_per_container: f64,
    max_concurrent_builds: usize,
    disk_space_threshold: usize,
}

pub struct ResourceManager {
    limits: ResourceLimits,
    current_usage: Arc<RwLock<ResourceUsage>>,
}

impl ResourceManager {
    pub async fn acquire_resources(&self, task: &PackageBuildTask) -> Result<ResourceGrant>;
    pub async fn release_resources(&self, grant: ResourceGrant);
}
```

### 2. Cleanup

```rust
pub struct CleanupManager {
    temp_dirs: Arc<Mutex<Vec<TempDir>>>,
    containers: Arc<Mutex<Vec<String>>>,
}

impl CleanupManager {
    pub async fn register_temp_dir(&self, dir: TempDir);
    pub async fn register_container(&self, id: String);
    pub async fn cleanup_all(&self);
}

impl Drop for CleanupManager {
    fn drop(&mut self) {
        // Ensure cleanup even on panic
    }
}
```

## Metrics and Monitoring

### 1. Build Metrics

```rust
pub struct BuildMetrics {
    start_time: Instant,
    package_times: DashMap<String, Duration>,
    cache_hits: AtomicUsize,
    cache_misses: AtomicUsize,
    total_size: AtomicU64,
}

impl BuildMetrics {
    pub fn summary(&self) -> MetricsSummary {
        MetricsSummary {
            total_duration: self.start_time.elapsed(),
            average_package_time: self.calculate_average(),
            cache_hit_rate: self.calculate_hit_rate(),
            total_size_mb: self.total_size.load(Ordering::SeqCst) / 1_048_576,
        }
    }
}
```

### 2. Performance Profiling

```rust
pub struct BuildProfiler {
    spans: Arc<Mutex<Vec<ProfileSpan>>>,
}

pub struct ProfileSpan {
    name: String,
    start: Instant,
    end: Option<Instant>,
    metadata: HashMap<String, String>,
}

impl BuildProfiler {
    pub fn start_span(&self, name: &str) -> SpanGuard;
    pub fn export_flamegraph(&self, path: &Path) -> Result<()>;
}
```

## Testing Support

### 1. Build Mocking

```rust
pub struct MockBuildOrchestrator {
    responses: HashMap<String, BuildResult>,
}

impl MockBuildOrchestrator {
    pub fn expect_package(&mut self, name: &str, result: BuildResult);
}
```

### 2. Test Scenarios

```rust
pub struct BuildScenario {
    packages: Vec<TestPackage>,
    expected_order: Vec<Vec<String>>,
    failures: HashMap<String, BuildError>,
}

pub async fn run_scenario(scenario: BuildScenario) -> Result<()> {
    // Setup test workspace
    // Run build
    // Verify results
}
```