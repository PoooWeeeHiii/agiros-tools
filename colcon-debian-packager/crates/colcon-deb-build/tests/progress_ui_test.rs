//! Comprehensive tests for progress UI functionality
//!
//! This module tests the progress UI system including:
//! - Progress UI with simulated events
//! - Multi-progress scenarios
//! - Error display and handling
//! - Different progress UI implementations

use std::sync::{Arc, Mutex};
use std::time::Duration;

use colcon_deb_build::{NoOpProgressUI, ProgressUI, ProgressUIFactory};
use colcon_deb_docker::progress::{LogLevel, ProgressEvent};
use tokio::time::sleep;

/// Mock progress UI for testing that captures all interactions
#[derive(Debug, Clone)]
pub struct TestProgressUI {
    /// Events received by this UI
    events: Arc<Mutex<Vec<ProgressEvent>>>,
    /// Number of times clear was called
    clear_calls: Arc<Mutex<usize>>,
    /// Number of times finish was called
    finish_calls: Arc<Mutex<usize>>,
    /// Package-specific progress UIs created
    package_progress_created: Arc<Mutex<Vec<String>>>,
    /// Whether this UI is enabled
    enabled: bool,
}

impl TestProgressUI {
    pub fn new(enabled: bool) -> Self {
        Self {
            events: Arc::new(Mutex::new(Vec::new())),
            clear_calls: Arc::new(Mutex::new(0)),
            finish_calls: Arc::new(Mutex::new(0)),
            package_progress_created: Arc::new(Mutex::new(Vec::new())),
            enabled,
        }
    }

    pub fn get_events(&self) -> Vec<ProgressEvent> {
        self.events.lock().unwrap().clone()
    }

    pub fn get_clear_calls(&self) -> usize {
        *self.clear_calls.lock().unwrap()
    }

    pub fn get_finish_calls(&self) -> usize {
        *self.finish_calls.lock().unwrap()
    }

    pub fn get_package_progress_created(&self) -> Vec<String> {
        self.package_progress_created.lock().unwrap().clone()
    }

    pub fn get_events_by_package(&self, package: &str) -> Vec<ProgressEvent> {
        self.events
            .lock()
            .unwrap()
            .iter()
            .filter(|event| match event {
                ProgressEvent::PackageStart { name, .. } => name == package,
                ProgressEvent::PackageComplete { name, .. } => name == package,
                _ => false,
            })
            .cloned()
            .collect()
    }

    pub fn get_log_events(&self) -> Vec<ProgressEvent> {
        self.events
            .lock()
            .unwrap()
            .iter()
            .filter(|event| matches!(event, ProgressEvent::Log { .. }))
            .cloned()
            .collect()
    }

    pub fn get_package_start_events(&self) -> Vec<ProgressEvent> {
        self.events
            .lock()
            .unwrap()
            .iter()
            .filter(|event| matches!(event, ProgressEvent::PackageStart { .. }))
            .cloned()
            .collect()
    }
}

impl ProgressUI for TestProgressUI {
    fn update(&self, event: &ProgressEvent) {
        if self.enabled {
            self.events.lock().unwrap().push(event.clone());
        }
    }

    fn clear(&self) {
        if self.enabled {
            *self.clear_calls.lock().unwrap() += 1;
        }
    }

    fn finish(&self) {
        if self.enabled {
            *self.finish_calls.lock().unwrap() += 1;
        }
    }

    fn get_package_progress(&self, package: &str) -> Option<Box<dyn ProgressUI>> {
        if self.enabled {
            self.package_progress_created
                .lock()
                .unwrap()
                .push(package.to_string());
            Some(Box::new(self.clone()))
        } else {
            None
        }
    }
}

/// Creates a test log event
fn create_test_log_event(level: LogLevel, message: &str) -> ProgressEvent {
    ProgressEvent::Log { level, message: message.to_string() }
}

/// Creates a test package start event
fn create_package_start_event(
    name: &str,
    current: Option<usize>,
    total: Option<usize>,
) -> ProgressEvent {
    ProgressEvent::PackageStart { name: name.to_string(), current, total }
}

/// Creates a test package complete event
fn create_package_complete_event(name: &str, success: bool, error: Option<&str>) -> ProgressEvent {
    ProgressEvent::PackageComplete {
        name: name.to_string(),
        success,
        error: error.map(|s| s.to_string()),
    }
}

#[tokio::test]
async fn test_basic_progress_ui_functionality() {
    let progress_ui = TestProgressUI::new(true);

    // Test basic event handling
    let event = create_package_start_event("test_package", Some(1), Some(5));

    progress_ui.update(&event);

    // Verify event was recorded
    let events = progress_ui.get_events();
    assert_eq!(events.len(), 1);

    match &events[0] {
        ProgressEvent::PackageStart { name, current, total } => {
            assert_eq!(name, "test_package");
            assert_eq!(*current, Some(1));
            assert_eq!(*total, Some(5));
        }
        _ => panic!("Expected PackageStart event"),
    }

    // Test lifecycle methods
    progress_ui.clear();
    assert_eq!(progress_ui.get_clear_calls(), 1);

    progress_ui.finish();
    assert_eq!(progress_ui.get_finish_calls(), 1);
}

#[tokio::test]
async fn test_multi_package_progress() {
    let progress_ui = TestProgressUI::new(true);

    let packages = vec!["package_a", "package_b", "package_c"];

    // Send progress events for multiple packages
    for (i, package) in packages.iter().enumerate() {
        let event = create_package_start_event(package, Some(i + 1), Some(packages.len()));
        progress_ui.update(&event);
    }

    // Verify all events were recorded
    let events = progress_ui.get_events();
    assert_eq!(events.len(), 3);

    // Check events by package
    for package in &packages {
        let package_events = progress_ui.get_events_by_package(package);
        assert_eq!(package_events.len(), 1);

        match &package_events[0] {
            ProgressEvent::PackageStart { name, .. } => {
                assert_eq!(name, package);
            }
            _ => panic!("Expected PackageStart event"),
        }
    }

    // Test package-specific progress UI creation
    for package in &packages {
        let package_ui = progress_ui.get_package_progress(package);
        assert!(package_ui.is_some());
    }

    let created_packages = progress_ui.get_package_progress_created();
    assert_eq!(created_packages.len(), 3);
    for package in &packages {
        assert!(created_packages.contains(&package.to_string()));
    }
}

#[tokio::test]
async fn test_error_display_and_handling() {
    let progress_ui = TestProgressUI::new(true);

    // Test various error scenarios
    let error_events = vec![
        create_package_complete_event(
            "failing_package",
            false,
            Some("Build failed: compilation error"),
        ),
        create_test_log_event(LogLevel::Warn, "Deprecated API usage detected"),
        create_test_log_event(LogLevel::Error, "Docker connection lost"),
    ];

    // Send error events
    for event in &error_events {
        progress_ui.update(event);
    }

    // Verify error events were recorded
    let all_events = progress_ui.get_events();
    assert_eq!(all_events.len(), 3);

    // Check the package complete event (error)
    match &all_events[0] {
        ProgressEvent::PackageComplete { name, success, error } => {
            assert_eq!(name, "failing_package");
            assert!(!success);
            assert!(error.as_ref().unwrap().contains("compilation error"));
        }
        _ => panic!("Expected PackageComplete event"),
    }

    // Check the warning log event
    match &all_events[1] {
        ProgressEvent::Log { level, message } => {
            assert_eq!(*level, LogLevel::Warn);
            assert!(message.contains("Deprecated API"));
        }
        _ => panic!("Expected Log event"),
    }

    // Check the error log event
    match &all_events[2] {
        ProgressEvent::Log { level, message } => {
            assert_eq!(*level, LogLevel::Error);
            assert!(message.contains("Docker connection"));
        }
        _ => panic!("Expected Log event"),
    }
}

#[tokio::test]
async fn test_progress_event_types() {
    let progress_ui = TestProgressUI::new(true);

    // Test different event types
    let events = vec![
        create_package_start_event("package_0", Some(1), Some(3)),
        create_test_log_event(LogLevel::Info, "Building package"),
        create_package_complete_event("package_0", true, None),
    ];

    for event in &events {
        progress_ui.update(event);
    }

    // Verify all event types were recorded
    let recorded_events = progress_ui.get_events();
    assert_eq!(recorded_events.len(), 3);

    // Check each event type
    match &recorded_events[0] {
        ProgressEvent::PackageStart { .. } => (),
        _ => panic!("Expected PackageStart event"),
    }

    match &recorded_events[1] {
        ProgressEvent::Log { .. } => (),
        _ => panic!("Expected Log event"),
    }

    match &recorded_events[2] {
        ProgressEvent::PackageComplete { .. } => (),
        _ => panic!("Expected PackageComplete event"),
    }
}

#[tokio::test]
async fn test_concurrent_progress_updates() {
    let progress_ui = Arc::new(TestProgressUI::new(true));

    // Simulate concurrent progress updates from multiple tasks
    let num_tasks = 5;
    let updates_per_task = 10;

    let mut handles = Vec::new();

    for task_id in 0..num_tasks {
        let ui = progress_ui.clone();
        let handle = tokio::spawn(async move {
            for update in 0..updates_per_task {
                let event = create_test_log_event(
                    LogLevel::Info,
                    &format!("Task {task_id} Update {update}"),
                );
                ui.update(&event);

                // Small delay to simulate real work
                sleep(Duration::from_millis(1)).await;
            }
        });
        handles.push(handle);
    }

    // Wait for all tasks to complete
    for handle in handles {
        handle.await.unwrap();
    }

    // Verify all updates were recorded
    let events = progress_ui.get_events();
    assert_eq!(events.len(), num_tasks * updates_per_task);

    // Verify all log events were received
    let log_events = progress_ui.get_log_events();
    assert_eq!(log_events.len(), num_tasks * updates_per_task);
}

#[tokio::test]
async fn test_progress_ui_factory() {
    // Test creating no-op UI
    let noop_ui = ProgressUIFactory::create_noop();

    // Test the no-op UI
    let event = create_test_log_event(LogLevel::Info, "Test message");

    // These should not panic
    noop_ui.update(&event);
    noop_ui.clear();
    noop_ui.finish();

    let package_ui = noop_ui.get_package_progress("test");
    assert!(package_ui.is_none());
}

#[tokio::test]
async fn test_disabled_progress_ui() {
    let progress_ui = TestProgressUI::new(false);

    // Send events to disabled UI
    let event = create_test_log_event(LogLevel::Info, "This should be ignored");

    progress_ui.update(&event);
    progress_ui.clear();
    progress_ui.finish();

    // Verify nothing was recorded
    assert_eq!(progress_ui.get_events().len(), 0);
    assert_eq!(progress_ui.get_clear_calls(), 0);
    assert_eq!(progress_ui.get_finish_calls(), 0);

    // Package progress should return None for disabled UI
    let package_ui = progress_ui.get_package_progress("test");
    assert!(package_ui.is_none());
}

#[tokio::test]
async fn test_stage_events() {
    let progress_ui = TestProgressUI::new(true);

    // Create stage event
    let event = ProgressEvent::Stage {
        name: "Compilation".to_string(),
        description: Some("Compiling C++ sources".to_string()),
    };

    progress_ui.update(&event);

    // Verify event was preserved
    let events = progress_ui.get_events();
    assert_eq!(events.len(), 1);

    match &events[0] {
        ProgressEvent::Stage { name, description } => {
            assert_eq!(name, "Compilation");
            assert_eq!(description.as_ref().unwrap(), "Compiling C++ sources");
        }
        _ => panic!("Expected Stage event"),
    }
}

#[tokio::test]
async fn test_progress_sequence() {
    let progress_ui = TestProgressUI::new(true);

    // Simulate a complete build sequence for a package
    let package = "sequence_test";
    let sequence = vec![
        create_package_start_event(package, Some(1), Some(1)),
        create_test_log_event(LogLevel::Info, "Configuring"),
        create_test_log_event(LogLevel::Info, "Compiling"),
        create_test_log_event(LogLevel::Info, "Linking"),
        create_test_log_event(LogLevel::Info, "Packaging"),
        create_package_complete_event(package, true, None),
    ];

    for event in &sequence {
        progress_ui.update(event);
    }

    // Verify sequence was recorded correctly
    let all_events = progress_ui.get_events();
    assert_eq!(all_events.len(), 6);

    // Verify sequence of event types
    match &all_events[0] {
        ProgressEvent::PackageStart { .. } => (),
        _ => panic!("Expected PackageStart"),
    }

    for event in &all_events[1..5] {
        match event {
            ProgressEvent::Log { .. } => (),
            _ => panic!("Expected Log event"),
        }
    }

    match &all_events[5] {
        ProgressEvent::PackageComplete { .. } => (),
        _ => panic!("Expected PackageComplete"),
    }
}

#[tokio::test]
async fn test_log_level_filtering() {
    let progress_ui = TestProgressUI::new(true);

    // Send events with different log levels
    let log_levels = vec![
        LogLevel::Debug,
        LogLevel::Info,
        LogLevel::Warn,
        LogLevel::Error,
    ];

    for log_level in &log_levels {
        let event = create_test_log_event(*log_level, &format!("Message with {log_level:?} level"));
        progress_ui.update(&event);
    }

    // Verify all events were recorded
    let events = progress_ui.get_events();
    assert_eq!(events.len(), 4);

    // Count events by log level
    let mut debug_count = 0;
    let mut info_count = 0;
    let mut warn_count = 0;
    let mut error_count = 0;

    for event in &events {
        if let ProgressEvent::Log { level, .. } = event {
            match level {
                LogLevel::Debug => debug_count += 1,
                LogLevel::Info => info_count += 1,
                LogLevel::Warn => warn_count += 1,
                LogLevel::Error => error_count += 1,
            }
        }
    }

    assert_eq!(debug_count, 1);
    assert_eq!(info_count, 1);
    assert_eq!(warn_count, 1);
    assert_eq!(error_count, 1);
}

#[tokio::test]
async fn test_event_ordering() {
    let progress_ui = TestProgressUI::new(true);

    // Send events with small delays to maintain order
    for i in 0..5 {
        let event = create_test_log_event(LogLevel::Info, &format!("Event {i}"));
        progress_ui.update(&event);

        // Small delay to simulate real processing
        sleep(Duration::from_millis(1)).await;
    }

    // Verify events are in order
    let events = progress_ui.get_events();
    assert_eq!(events.len(), 5);

    for (i, event) in events.iter().enumerate() {
        match event {
            ProgressEvent::Log { message, .. } => {
                assert_eq!(message, &format!("Event {i}"));
            }
            _ => panic!("Expected Log event"),
        }
    }
}

#[tokio::test]
async fn test_no_op_progress_ui() {
    let noop_ui = NoOpProgressUI;

    // Test that NoOpProgressUI doesn't panic and handles all operations
    let event = create_test_log_event(LogLevel::Info, "Test message");

    // These should all be no-ops and not panic
    noop_ui.update(&event);
    noop_ui.clear();
    noop_ui.finish();

    // NoOpProgressUI returns None for package progress, which is expected
    let package_ui = noop_ui.get_package_progress("test_package");
    assert!(package_ui.is_none());
}
