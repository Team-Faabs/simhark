//! # RoboCup SSL Simulator
//!
//! A high-performance, massively parallel RoboCup Small Size League simulator
//! inspired by grSim. Designed for AI training with support for thousands of
//! simultaneous worlds.
//!
//! ## Key Features
//! - Run 512, 1024, 2048+ worlds in parallel via rayon
//! - Pure Rust API (no protobuf needed)
//! - Domain randomization: per-world physics constant mutations for sim-to-real transfer
//! - Deterministic simulation with seeded RNG
//! - Headless mode for maximum throughput
//! - Optional visualization for debugging
//!
//! ## Quick Start
//! ```rust
//! use simhark::{SimulationEngine, WorldConfig, RobotCommand};
//!
//! // Create engine with 1024 parallel worlds
//! let mut engine = SimulationEngine::new(1024, WorldConfig::division_a());
//!
//! // Step all worlds
//! let states = engine.step_all();
//! ```

pub mod command;
pub mod config;
pub mod controller;
pub mod domain_randomization;
pub mod engine;
pub mod geometry;
pub mod grsim;
#[cfg(feature = "motion-audit")]
pub mod motion_audit;
pub mod physics;
pub mod proto;
pub mod robot;
pub mod state;
#[cfg(feature = "viewer")]
pub mod viewer;
pub mod world;

// Re-export main types
pub use command::{
  MoveCommand, RobotCommand, TeamCommand, TeleportBall, TeleportRobot, WorldCommand,
};
pub use config::{
  BallConfig, DEFAULT_KICK_SPEED, FieldConfig, MAX_BALL_SPEED, RobotConfig, WorldConfig,
};
pub use controller::{ControlledTeams, FnTeamController, NoopController, TeamController};
pub use domain_randomization::{DomainRandomizer, RandomizationConfig};
pub use engine::SimulationEngine;
pub use grsim::{GrSimCompatConfig, GrSimCompatServer};
pub use state::{BallState, RobotState, TeamColor, WorldState};
pub use world::World;
