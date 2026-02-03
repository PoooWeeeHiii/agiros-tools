#!/usr/bin/env rust-script
//! ```cargo
//! [dependencies]
//! clap = { version = "4.0", features = ["derive"] }
//! serde_json = "1.0"
//! ```

// scripts/helpers/progress-reporter.rs
// Reports structured progress events for build monitoring

use clap::{Parser, Subcommand};
use std::io::{self, Write};

#[derive(Parser)]
#[command(name = "progress-reporter")]
#[command(about = "Report structured progress events")]
struct Args {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Report package start
    PackageStart {
        /// Package name
        name: String,
        /// Current package number
        #[arg(short, long)]
        current: Option<usize>,
        /// Total packages
        #[arg(short, long)]
        total: Option<usize>,
    },
    /// Report package completion
    PackageComplete {
        /// Package name
        name: String,
        /// Success status
        #[arg(short, long)]
        success: bool,
        /// Error message if failed
        #[arg(short, long)]
        error: Option<String>,
    },
    /// Report build stage
    Stage {
        /// Stage name
        name: String,
        /// Stage description
        #[arg(short, long)]
        description: Option<String>,
    },
    /// Report log message
    Log {
        /// Log level (info, warn, error)
        #[arg(short, long, default_value = "info")]
        level: String,
        /// Log message
        message: String,
    },
    /// Report overall progress
    Progress {
        /// Current step
        current: usize,
        /// Total steps
        total: usize,
        /// Progress message
        #[arg(short, long)]
        message: Option<String>,
    },
}

fn main() -> io::Result<()> {
    let args = Args::parse();
    
    match args.command {
        Commands::PackageStart { name, current, total } => {
            let progress = match (current, total) {
                (Some(c), Some(t)) => format!(" [{}/{}]", c, t),
                _ => String::new(),
            };
            println!("::progress::package_start::{}{}", name, progress);
        }
        
        Commands::PackageComplete { name, success, error } => {
            if success {
                println!("::progress::package_complete::{}::success", name);
            } else {
                let error_msg = error.unwrap_or_else(|| "Unknown error".to_string());
                println!("::progress::package_complete::{}::failed::{}", name, error_msg);
            }
        }
        
        Commands::Stage { name, description } => {
            if let Some(desc) = description {
                println!("::progress::stage::{}::{}", name, desc);
            } else {
                println!("::progress::stage::{}", name);
            }
        }
        
        Commands::Log { level, message } => {
            println!("::log::{}::{}", level, message);
        }
        
        Commands::Progress { current, total, message } => {
            if let Some(msg) = message {
                println!("::progress::overall::{}/{}::{}", current, total, msg);
            } else {
                println!("::progress::overall::{}/{}", current, total);
            }
        }
    }
    
    // Flush output immediately
    io::stdout().flush()?;
    
    Ok(())
}