//! Opt-in scheduled simulator. The normal `World` and `SimulationEngine` have
//! no realism fields, branches, queues, sensor generation, or allocations.
//!
//! Free-3D hardware, sensing, transport, processing and observation-only tracking.
//! Profiles remain provisional until calibrated. The default drive controller is
//! a surrogate; native firmware and production fusion use explicit adapters.
pub mod calibration;
pub mod calibration_fit;
pub mod clocks;
pub mod config;
mod contact3d;
pub mod controller;
mod devices;
mod drive_noise;
pub mod hardware;
pub mod live;
pub mod localization;
pub mod magnetic;
pub mod network;
pub mod power;
pub mod processing;
pub mod profile;
pub mod profiles;
pub mod protocol;
pub mod radio;
pub mod render;
mod runtime;
pub mod scene;
pub mod sensors;
pub mod time;
pub mod transport;
pub mod vision;
pub mod vision_fixture;

pub use config::RealisticConfig;
pub use runtime::{
  ActionIdentity, Diagnostics, RealisticWorld, RecordedInput, Recording, RobotDiagnostics,
  TraceEvent,
};
