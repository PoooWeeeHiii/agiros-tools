//! Progress UI components for build visualization
//!
//! This module provides visual progress indicators for the build process,
//! integrating with the Docker progress events.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Instant;

use colcon_deb_docker::progress::{LogLevel, ProgressEvent};
use indicatif::{MultiProgress, ProgressBar, ProgressStyle};
use tracing::{debug, error, info, warn};

/// Trait for progress UI implementations
pub trait ProgressUI: Send + Sync {
    /// Update the UI with a progress event
    fn update(&self, event: &ProgressEvent);

    /// Clear the progress display
    fn clear(&self);

    /// Finish the progress display
    fn finish(&self);

    /// Get a handle for package-specific progress
    fn get_package_progress(&self, package: &str) -> Option<Box<dyn ProgressUI>>;
}

/// Indicatif-based progress UI implementation
pub struct IndicatifProgressUI {
    /// Multi-progress container for parallel builds
    multi_progress: MultiProgress,
    /// Main progress bar
    main_bar: ProgressBar,
    /// Package progress bars
    package_bars: Arc<Mutex<HashMap<String, ProgressBar>>>,
    /// Overall progress tracking
    total_packages: Arc<Mutex<Option<usize>>>,
    /// Completed packages count
    completed_packages: Arc<Mutex<usize>>,
    /// Start time for duration tracking
    start_time: Instant,
}

impl IndicatifProgressUI {
    /// Create a new indicatif progress UI
    pub fn new() -> Self {
        let multi_progress = MultiProgress::new();

        // Create main progress bar
        let main_bar = multi_progress.add(ProgressBar::new(100));
        main_bar.set_style(
            ProgressStyle::default_bar()
                .template(
                    "{prefix:.bold.dim} {spinner:.green} [{elapsed_precise}] \
                     [{wide_bar:.cyan/blue}] {pos}/{len} ({eta})",
                )
                .expect("Valid template")
                .progress_chars("#>-"),
        );
        main_bar.set_prefix("Building packages");

        Self {
            multi_progress,
            main_bar,
            package_bars: Arc::new(Mutex::new(HashMap::new())),
            total_packages: Arc::new(Mutex::new(None)),
            completed_packages: Arc::new(Mutex::new(0)),
            start_time: Instant::now(),
        }
    }

    /// Create a package-specific progress bar
    fn create_package_bar(&self, package: &str) -> ProgressBar {
        let pb = self.multi_progress.add(ProgressBar::new_spinner());
        pb.set_style(
            ProgressStyle::default_spinner()
                .template("{prefix:.bold.dim} {spinner:.green} {wide_msg}")
                .expect("Valid template"),
        );
        pb.set_prefix(format!("  {package}"));
        pb.enable_steady_tick(std::time::Duration::from_millis(100));
        pb
    }

    /// Update main progress bar
    fn update_main_progress(&self) {
        if let (Ok(total), Ok(completed)) =
            (self.total_packages.lock(), self.completed_packages.lock())
        {
            if let Some(total) = *total {
                if total > 0 {
                    self.main_bar.set_length(total as u64);
                    self.main_bar.set_position(*completed as u64);

                    let percentage = (*completed as f64 / total as f64) * 100.0;
                    self.main_bar
                        .set_message(format!("{percentage:.1}% complete"));
                }
            }
        }
    }

    /// Handle package start event
    fn handle_package_start(&self, name: &str, current: Option<usize>, total: Option<usize>) {
        // Update total packages if provided
        if let Some(total) = total {
            if let Ok(mut total_packages) = self.total_packages.lock() {
                *total_packages = Some(total);
            }
        }

        // Create or update package bar
        let pb = self.create_package_bar(name);
        pb.set_message("Building...");

        if let Ok(mut bars) = self.package_bars.lock() {
            bars.insert(name.to_string(), pb);
        }

        // Update main progress
        if let Some(current) = current {
            self.main_bar.set_message(format!(
                "Building {} [{}/{}]",
                name,
                current,
                total.unwrap_or(0)
            ));
        }

        self.update_main_progress();
    }

    /// Handle package complete event
    fn handle_package_complete(&self, name: &str, success: bool, error: Option<&str>) {
        // Update package bar
        if let Ok(bars) = self.package_bars.lock() {
            if let Some(pb) = bars.get(name) {
                if success {
                    pb.finish_with_message("✓ Complete");
                } else {
                    let msg = error.unwrap_or("Failed");
                    pb.finish_with_message(format!("✗ {msg}"));
                }
            }
        }

        // Update completed count
        if let Ok(mut completed) = self.completed_packages.lock() {
            *completed += 1;
        }

        self.update_main_progress();
    }

    /// Handle stage change event
    fn handle_stage(&self, name: &str, description: Option<&str>) {
        let msg = if let Some(desc) = description {
            format!("Stage: {name} - {desc}")
        } else {
            format!("Stage: {name}")
        };

        self.main_bar.set_prefix(msg);
    }

    /// Handle progress event
    fn handle_progress(&self, current: usize, total: usize, message: Option<&str>) {
        self.main_bar.set_length(total as u64);
        self.main_bar.set_position(current as u64);

        if let Some(msg) = message {
            self.main_bar.set_message(msg.to_string());
        }
    }

    /// Handle log event
    fn handle_log(&self, level: LogLevel, message: &str) {
        // For important logs, we might want to display them
        match level {
            LogLevel::Error => {
                error!("{}", message);
                self.multi_progress
                    .println(format!("ERROR: {message}"))
                    .ok();
            }
            LogLevel::Warn => {
                warn!("{}", message);
                self.multi_progress.println(format!("WARN: {message}")).ok();
            }
            LogLevel::Info => {
                info!("{}", message);
            }
            LogLevel::Debug => {
                debug!("{}", message);
            }
        }
    }
}

impl Default for IndicatifProgressUI {
    fn default() -> Self {
        Self::new()
    }
}

impl ProgressUI for IndicatifProgressUI {
    fn update(&self, event: &ProgressEvent) {
        match event {
            ProgressEvent::PackageStart { name, current, total } => {
                self.handle_package_start(name, *current, *total);
            }
            ProgressEvent::PackageComplete { name, success, error } => {
                self.handle_package_complete(name, *success, error.as_deref());
            }
            ProgressEvent::Stage { name, description } => {
                self.handle_stage(name, description.as_deref());
            }
            ProgressEvent::Progress { current, total, message } => {
                self.handle_progress(*current, *total, message.as_deref());
            }
            ProgressEvent::Log { level, message } => {
                self.handle_log(*level, message);
            }
        }
    }

    fn clear(&self) {
        self.multi_progress.clear().ok();
    }

    fn finish(&self) {
        // Finish all package bars
        if let Ok(bars) = self.package_bars.lock() {
            for pb in bars.values() {
                if !pb.is_finished() {
                    pb.finish();
                }
            }
        }

        // Finish main bar
        let elapsed = self.start_time.elapsed();
        self.main_bar
            .finish_with_message(format!("Build completed in {elapsed:?}"));
    }

    fn get_package_progress(&self, _package: &str) -> Option<Box<dyn ProgressUI>> {
        // For now, we'll use the same UI for all packages
        // In the future, we could return package-specific UI instances
        None
    }
}

/// No-op progress UI for when visual progress is not needed
pub struct NoOpProgressUI;

impl ProgressUI for NoOpProgressUI {
    fn update(&self, event: &ProgressEvent) {
        // Just log the events
        match event {
            ProgressEvent::PackageStart { name, current, total } => {
                if let (Some(current), Some(total)) = (current, total) {
                    info!("Building package {} [{}/{}]", name, current, total);
                } else {
                    info!("Building package {}", name);
                }
            }
            ProgressEvent::PackageComplete { name, success, error } => {
                if *success {
                    info!("Package {} built successfully", name);
                } else {
                    error!(
                        "Package {} failed: {}",
                        name,
                        error.as_deref().unwrap_or("Unknown error")
                    );
                }
            }
            ProgressEvent::Stage { name, description } => {
                if let Some(desc) = description {
                    info!("Stage: {} - {}", name, desc);
                } else {
                    info!("Stage: {}", name);
                }
            }
            ProgressEvent::Progress { current, total, message } => {
                if let Some(msg) = message {
                    info!("Progress [{}/{}]: {}", current, total, msg);
                } else {
                    info!("Progress [{}/{}]", current, total);
                }
            }
            ProgressEvent::Log { level, message } => match level {
                LogLevel::Error => error!("{}", message),
                LogLevel::Warn => warn!("{}", message),
                LogLevel::Info => info!("{}", message),
                LogLevel::Debug => debug!("{}", message),
            },
        }
    }

    fn clear(&self) {
        // No-op
    }

    fn finish(&self) {
        info!("Build process completed");
    }

    fn get_package_progress(&self, _package: &str) -> Option<Box<dyn ProgressUI>> {
        None
    }
}

/// Progress UI factory
pub struct ProgressUIFactory;

impl ProgressUIFactory {
    /// Create a progress UI based on environment
    pub fn create(interactive: bool) -> Box<dyn ProgressUI> {
        if interactive && atty::is(atty::Stream::Stdout) {
            Box::new(IndicatifProgressUI::new())
        } else {
            Box::new(NoOpProgressUI)
        }
    }

    /// Create an indicatif progress UI
    pub fn create_indicatif() -> Box<dyn ProgressUI> {
        Box::new(IndicatifProgressUI::new())
    }

    /// Create a no-op progress UI
    pub fn create_noop() -> Box<dyn ProgressUI> {
        Box::new(NoOpProgressUI)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_indicatif_progress_ui() {
        let ui = IndicatifProgressUI::new();

        // Test package start
        ui.update(&ProgressEvent::PackageStart {
            name: "test_package".to_string(),
            current: Some(1),
            total: Some(5),
        });

        // Test progress update
        ui.update(&ProgressEvent::Progress {
            current: 2,
            total: 5,
            message: Some("Processing...".to_string()),
        });

        // Test package complete
        ui.update(&ProgressEvent::PackageComplete {
            name: "test_package".to_string(),
            success: true,
            error: None,
        });

        // Test stage
        ui.update(&ProgressEvent::Stage {
            name: "build".to_string(),
            description: Some("Building packages".to_string()),
        });

        // Test log
        ui.update(&ProgressEvent::Log {
            level: LogLevel::Info,
            message: "Test log message".to_string(),
        });

        ui.finish();
    }

    #[test]
    fn test_noop_progress_ui() {
        let ui = NoOpProgressUI;

        // Should not panic on any event
        ui.update(&ProgressEvent::PackageStart {
            name: "test".to_string(),
            current: None,
            total: None,
        });

        ui.update(&ProgressEvent::PackageComplete {
            name: "test".to_string(),
            success: false,
            error: Some("Test error".to_string()),
        });

        ui.clear();
        ui.finish();
    }

    #[test]
    fn test_progress_factory() {
        // Test creating different UI types
        let noop_ui = ProgressUIFactory::create_noop();
        noop_ui.update(&ProgressEvent::Log { level: LogLevel::Debug, message: "Test".to_string() });

        let indicatif_ui = ProgressUIFactory::create_indicatif();
        indicatif_ui.update(&ProgressEvent::Stage { name: "test".to_string(), description: None });
    }
}
