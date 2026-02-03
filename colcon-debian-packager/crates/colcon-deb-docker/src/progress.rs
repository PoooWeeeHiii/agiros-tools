//! Progress monitoring and parsing for container output
//!
//! This module provides functionality to parse structured progress output
//! from containers using the ::progress:: and ::log:: format.

use std::pin::Pin;
use std::task::{Context, Poll};

use futures::Stream;
use serde::{Deserialize, Serialize};

use crate::error::Result;

/// Progress event types
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ProgressEvent {
    /// Package build started
    PackageStart {
        /// Package name
        name: String,
        /// Current package number
        current: Option<usize>,
        /// Total number of packages
        total: Option<usize>,
    },
    /// Package build completed
    PackageComplete {
        /// Package name
        name: String,
        /// Whether the build succeeded
        success: bool,
        /// Error message if failed
        error: Option<String>,
    },
    /// Build stage changed
    Stage {
        /// Stage name
        name: String,
        /// Stage description
        description: Option<String>,
    },
    /// General progress update
    Progress {
        /// Current step
        current: usize,
        /// Total steps
        total: usize,
        /// Progress message
        message: Option<String>,
    },
    /// Log message
    Log {
        /// Log level
        level: LogLevel,
        /// Log message
        message: String,
    },
}

/// Log levels for progress events
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum LogLevel {
    /// Debug level
    Debug,
    /// Info level
    Info,
    /// Warning level
    Warn,
    /// Error level
    Error,
}

impl LogLevel {
    /// Parse log level from string
    pub fn from_level_str(s: &str) -> Self {
        match s.to_lowercase().as_str() {
            "debug" => Self::Debug,
            "info" => Self::Info,
            "warn" | "warning" => Self::Warn,
            "error" => Self::Error,
            _ => Self::Info,
        }
    }
}

/// Progress parser trait for parsing structured output
pub trait ProgressParser {
    /// Parse a line of output into a progress event
    fn parse_line(&self, line: &str) -> Option<ProgressEvent>;
}

/// Default progress parser implementation
#[derive(Debug, Default)]
pub struct DefaultProgressParser;

impl ProgressParser for DefaultProgressParser {
    fn parse_line(&self, line: &str) -> Option<ProgressEvent> {
        if line.starts_with("::progress::") {
            parse_progress_line(line)
        } else if line.starts_with("::log::") {
            parse_log_line(line)
        } else {
            None
        }
    }
}

/// Parse a ::progress:: line
fn parse_progress_line(line: &str) -> Option<ProgressEvent> {
    let content = line.strip_prefix("::progress::")?;
    let parts: Vec<&str> = content.split("::").collect();

    match parts.as_slice() {
        ["package_start", name_with_progress] => {
            // Check if name has progress appended like "my_package [2/10]"
            if let Some(space_pos) = name_with_progress.rfind(' ') {
                let name = &name_with_progress[..space_pos];
                let progress = &name_with_progress[space_pos + 1..];
                let (current, total) = parse_progress_numbers(progress);
                Some(ProgressEvent::PackageStart { name: name.to_string(), current, total })
            } else {
                Some(ProgressEvent::PackageStart {
                    name: name_with_progress.to_string(),
                    current: None,
                    total: None,
                })
            }
        }
        ["package_complete", name, "success"] => Some(ProgressEvent::PackageComplete {
            name: name.to_string(),
            success: true,
            error: None,
        }),
        ["package_complete", name, "failed", error] => Some(ProgressEvent::PackageComplete {
            name: name.to_string(),
            success: false,
            error: Some(error.to_string()),
        }),
        ["stage", name] => Some(ProgressEvent::Stage { name: name.to_string(), description: None }),
        ["stage", name, description] => Some(ProgressEvent::Stage {
            name: name.to_string(),
            description: Some(description.to_string()),
        }),
        ["overall", progress] => {
            if let Some((current, total)) = parse_fraction(progress) {
                Some(ProgressEvent::Progress { current, total, message: None })
            } else {
                None
            }
        }
        ["overall", progress, message] => {
            if let Some((current, total)) = parse_fraction(progress) {
                Some(ProgressEvent::Progress { current, total, message: Some(message.to_string()) })
            } else {
                None
            }
        }
        _ => None,
    }
}

/// Parse a ::log:: line
fn parse_log_line(line: &str) -> Option<ProgressEvent> {
    let content = line.strip_prefix("::log::")?;
    let parts: Vec<&str> = content.splitn(3, "::").collect();

    match parts.as_slice() {
        [level, message] => Some(ProgressEvent::Log {
            level: LogLevel::from_level_str(level),
            message: message.to_string(),
        }),
        _ => None,
    }
}

/// Parse progress numbers from "[2/10]" format
fn parse_progress_numbers(s: &str) -> (Option<usize>, Option<usize>) {
    let s = s.trim();
    if s.starts_with('[') && s.ends_with(']') {
        let inner = &s[1..s.len() - 1];
        if let Some((current, total)) = parse_fraction(inner) {
            return (Some(current), Some(total));
        }
    }
    (None, None)
}

/// Parse fraction from "2/10" format
fn parse_fraction(s: &str) -> Option<(usize, usize)> {
    let parts: Vec<&str> = s.split('/').collect();
    if parts.len() == 2 {
        let current = parts[0].parse().ok()?;
        let total = parts[1].parse().ok()?;
        Some((current, total))
    } else {
        None
    }
}

/// Progress tracking state
#[derive(Debug, Clone, Default)]
pub struct Progress {
    /// Current stage
    pub stage: Option<String>,
    /// Packages in progress
    pub packages_in_progress: Vec<String>,
    /// Completed packages
    pub completed_packages: Vec<(String, bool)>,
    /// Total packages
    pub total_packages: Option<usize>,
    /// Current overall progress
    pub current_progress: Option<usize>,
    /// Total progress steps
    pub total_progress: Option<usize>,
}

impl Progress {
    /// Update progress with an event
    pub fn update(&mut self, event: &ProgressEvent) {
        match event {
            ProgressEvent::PackageStart { name, total, .. } => {
                self.packages_in_progress.push(name.clone());
                if let Some(total) = total {
                    self.total_packages = Some(*total);
                }
            }
            ProgressEvent::PackageComplete { name, success, .. } => {
                self.packages_in_progress.retain(|n| n != name);
                self.completed_packages.push((name.clone(), *success));
            }
            ProgressEvent::Stage { name, .. } => {
                self.stage = Some(name.clone());
            }
            ProgressEvent::Progress { current, total, .. } => {
                self.current_progress = Some(*current);
                self.total_progress = Some(*total);
            }
            _ => {}
        }
    }

    /// Get completion percentage
    pub fn completion_percentage(&self) -> Option<f64> {
        match (self.current_progress, self.total_progress) {
            (Some(current), Some(total)) if total > 0 => {
                Some((current as f64 / total as f64) * 100.0)
            }
            _ => match (self.completed_packages.len(), self.total_packages) {
                (completed, Some(total)) if total > 0 => {
                    Some((completed as f64 / total as f64) * 100.0)
                }
                _ => None,
            },
        }
    }

    /// Get number of successful packages
    pub fn successful_packages(&self) -> usize {
        self.completed_packages
            .iter()
            .filter(|(_, success)| *success)
            .count()
    }

    /// Get number of failed packages
    pub fn failed_packages(&self) -> usize {
        self.completed_packages
            .iter()
            .filter(|(_, success)| !*success)
            .count()
    }
}

/// Stream wrapper that parses progress events
pub struct ProgressStream<S> {
    inner: S,
    parser: Box<dyn ProgressParser + Send>,
    buffer: String,
}

impl<S> ProgressStream<S> {
    /// Create a new progress stream with the default parser
    pub fn new(stream: S) -> Self {
        Self::with_parser(stream, Box::new(DefaultProgressParser))
    }

    /// Create a new progress stream with a custom parser
    pub fn with_parser(stream: S, parser: Box<dyn ProgressParser + Send>) -> Self {
        Self { inner: stream, parser, buffer: String::new() }
    }
}

impl<S> Stream for ProgressStream<S>
where
    S: Stream<Item = Result<String>> + Unpin,
{
    type Item = Result<ProgressEvent>;

    fn poll_next(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        loop {
            match Pin::new(&mut self.inner).poll_next(cx) {
                Poll::Ready(Some(Ok(chunk))) => {
                    self.buffer.push_str(&chunk);

                    // Process complete lines
                    while let Some(newline_pos) = self.buffer.find('\n') {
                        let line = self.buffer[..newline_pos].to_string();
                        self.buffer.drain(..=newline_pos);

                        if let Some(event) = self.parser.parse_line(&line) {
                            return Poll::Ready(Some(Ok(event)));
                        }
                    }
                }
                Poll::Ready(Some(Err(e))) => return Poll::Ready(Some(Err(e))),
                Poll::Ready(None) => {
                    // Process any remaining buffer
                    if !self.buffer.is_empty() {
                        let line = self.buffer.clone();
                        self.buffer.clear();
                        if let Some(event) = self.parser.parse_line(&line) {
                            return Poll::Ready(Some(Ok(event)));
                        }
                    }
                    return Poll::Ready(None);
                }
                Poll::Pending => return Poll::Pending,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_progress_events() {
        let parser = DefaultProgressParser;

        // Test package start
        let event = parser
            .parse_line("::progress::package_start::my_package")
            .unwrap();
        match event {
            ProgressEvent::PackageStart { name, .. } => assert_eq!(name, "my_package"),
            _ => panic!("Wrong event type"),
        }

        // Test package start with progress
        let event = parser
            .parse_line("::progress::package_start::my_package [2/10]")
            .unwrap();
        match event {
            ProgressEvent::PackageStart { name, current, total } => {
                assert_eq!(name, "my_package");
                assert_eq!(current, Some(2));
                assert_eq!(total, Some(10));
            }
            _ => panic!("Wrong event type"),
        }

        // Test package complete success
        let event = parser
            .parse_line("::progress::package_complete::my_package::success")
            .unwrap();
        match event {
            ProgressEvent::PackageComplete { name, success, .. } => {
                assert_eq!(name, "my_package");
                assert!(success);
            }
            _ => panic!("Wrong event type"),
        }

        // Test stage
        let event = parser.parse_line("::progress::stage::building").unwrap();
        match event {
            ProgressEvent::Stage { name, .. } => assert_eq!(name, "building"),
            _ => panic!("Wrong event type"),
        }

        // Test log
        let event = parser.parse_line("::log::info::Starting build").unwrap();
        match event {
            ProgressEvent::Log { level, message } => {
                assert_eq!(level, LogLevel::Info);
                assert_eq!(message, "Starting build");
            }
            _ => panic!("Wrong event type"),
        }
    }

    #[test]
    fn test_progress_tracking() {
        let mut progress = Progress::default();

        progress.update(&ProgressEvent::PackageStart {
            name: "pkg1".to_string(),
            current: Some(1),
            total: Some(3),
        });
        assert_eq!(progress.packages_in_progress.len(), 1);
        assert_eq!(progress.total_packages, Some(3));

        progress.update(&ProgressEvent::PackageComplete {
            name: "pkg1".to_string(),
            success: true,
            error: None,
        });
        assert_eq!(progress.packages_in_progress.len(), 0);
        assert_eq!(progress.completed_packages.len(), 1);
        assert_eq!(progress.successful_packages(), 1);

        progress.update(&ProgressEvent::Progress { current: 2, total: 3, message: None });
        assert!((progress.completion_percentage().unwrap() - 66.66666666666667).abs() < 0.0001);
    }
}
