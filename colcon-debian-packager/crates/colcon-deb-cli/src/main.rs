//! Main CLI entry point for Colcon Debian Packager

use std::path::PathBuf;

use clap::{Parser, Subcommand};
use color_eyre::eyre::Result;
use tracing_subscriber::EnvFilter;

mod commands;

/// Colcon Debian Packager - Build .deb packages for ROS workspaces
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
#[command(propagate_version = true)]
struct Cli {
    /// Increase logging verbosity
    #[arg(short, long, action = clap::ArgAction::Count)]
    verbose: u8,

    /// Decrease logging verbosity
    #[arg(short, long, action = clap::ArgAction::Count)]
    quiet: u8,

    /// Configuration file path (global option)
    #[arg(short, long, global = true, value_name = "FILE")]
    config: Option<PathBuf>,

    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand, Debug)]
enum Commands {
    /// Build Debian packages from a ROS workspace
    Build {
        /// Override output directory
        #[arg(short, long, value_name = "DIR")]
        output: Option<PathBuf>,

        /// Number of parallel jobs
        #[arg(short = 'j', long, value_name = "N")]
        jobs: Option<usize>,

        /// Target architecture
        #[arg(long, value_name = "ARCH")]
        arch: Option<String>,

        /// Skip Docker and build natively
        #[arg(long)]
        no_docker: bool,

        /// Additional packages to install in container
        #[arg(long, value_name = "PACKAGE")]
        extra_packages: Vec<String>,

        /// Override AGIROS distribution (e.g. "loong", "pixiu")
        #[arg(long, value_name = "DISTRO")]
        agiros_distro: Option<String>,
    },

    /// Validate configuration file and workspace
    Validate {
        /// Check Docker availability
        #[arg(long)]
        check_docker: bool,
    },

    /// Clean build artifacts
    Clean {
        /// Clean all artifacts (including cache)
        #[arg(long)]
        all: bool,

        /// Also clean Docker containers and images
        #[arg(long)]
        docker: bool,
    },

    /// Initialize a new configuration file
    Init {
        /// Output path for configuration file
        #[arg(short, long, value_name = "FILE", default_value = "colcon-deb.yaml")]
        output: PathBuf,

        /// Force overwrite existing file
        #[arg(short, long)]
        force: bool,

        /// Initialize for specific ROS distro
        #[arg(long, value_name = "DISTRO", default_value = "loong")]
        ros_distro: String,
    },
}

#[tokio::main]
async fn main() -> Result<()> {
    // Install color-eyre for better error reports
    color_eyre::install()?;

    // Parse command line arguments
    let cli = Cli::parse();

    // Set up logging
    setup_logging(cli.verbose, cli.quiet)?;

    // Determine config path
    let config_path = cli
        .config
        .unwrap_or_else(|| PathBuf::from("colcon-deb.yaml"));

    // Handle commands
    let result = match cli.command {
        Commands::Build { output, jobs, arch: _, no_docker: _, extra_packages: _, agiros_distro } => {
            let command = commands::BuildCommand::new(config_path, output, jobs, agiros_distro);
            command.execute().await
        }

        Commands::Validate { check_docker: _ } => {
            let command = commands::ValidateCommand::new(config_path);
            command.execute().await
        }

        Commands::Clean { all, docker: _ } => {
            let command = commands::CleanCommand::new(Some(config_path), all);
            command.execute().await
        }

        Commands::Init { output, force, ros_distro: _ } => {
            let command = commands::InitCommand::new(output, force);
            command.execute().await
        }
    };

    // Handle command execution result
    if let Err(e) = result {
        tracing::error!("Command failed: {}", e);
        std::process::exit(1);
    }

    Ok(())
}

fn setup_logging(verbose: u8, quiet: u8) -> Result<()> {
    let log_level = match (verbose, quiet) {
        (0, 0) => "info",
        (1, 0) => "debug",
        (2, 0) => "trace",
        (v, 0) if v > 2 => "trace",
        (0, 1) => "warn",
        (0, 2) => "error",
        (0, q) if q > 2 => "off",
        _ => "info", // If both are set, default to info
    };

    let filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new(log_level));

    tracing_subscriber::fmt()
        .with_env_filter(filter)
        .with_target(false)
        .init();

    Ok(())
}
