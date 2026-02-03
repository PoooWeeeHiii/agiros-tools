//! Build orchestration for Colcon Debian Packager
//!
//! This crate coordinates the build process, managing Docker containers
//! and tracking build progress.

pub mod artifact;
pub mod context;
pub mod error;
pub mod executor;
pub mod graceful_shutdown;
pub mod orchestrator;
pub mod progress_ui;
pub mod recovery;

pub use artifact::{ArtifactCollector, BuildArtifact};
pub use context::{BuildContext, BuildState};
pub use error::{BuildError, Result};
pub use executor::{BuildExecutor, ExecutorConfig};
pub use graceful_shutdown::{
    setup_signal_handlers, ShutdownGuard, ShutdownManager, ShutdownReason, ShutdownSignal,
};
pub use orchestrator::{BuildOrchestrator, BuildOrchestratorTrait, ColconDebBuilder};
pub use progress_ui::{IndicatifProgressUI, NoOpProgressUI, ProgressUI, ProgressUIFactory};
pub use recovery::{
    retry_with_backoff, BuildRecoveryStrategy, RecoveryContext, RecoveryManager, RetryConfig,
};
