//! Frozen SHREPL01 layout from before robot mass/wheel and floor configuration.
//! Bincode is positional: these fields must never be reordered or extended.
use super::{ReplayEvent, ReplayFrame, ReplayLog, ReplayMetadata};
use crate::config::{
  BallConfig, FieldConfig, PhysicsConfig, RobotConfig, WHEEL_COUNT, WorldConfig,
};
use serde::{Deserialize, Serialize};
#[derive(Serialize, Deserialize)]
pub(super) struct LegacyLog {
  pub metadata: LegacyMetadata,
  pub frames: Vec<ReplayFrame>,
  pub events: Vec<ReplayEvent>,
}
#[derive(Serialize, Deserialize)]
pub(super) struct LegacyMetadata {
  pub world_count: usize,
  pub world_config: LegacyWorld,
  pub tick_hz: f64,
  pub source: String,
}
#[derive(Serialize, Deserialize)]
pub(super) struct LegacyWorld {
  pub robots_per_team: usize,
  pub field: FieldConfig,
  pub ball: BallConfig,
  pub blue_robots: LegacyRobot,
  pub yellow_robots: LegacyRobot,
  pub physics: LegacyPhysics,
  pub seed: u64,
}
#[derive(Serialize, Deserialize)]
pub(super) struct LegacyPhysics {
  pub delta_time: f64,
  pub gravity: f64,
}
#[derive(Serialize, Deserialize)]
pub(super) struct LegacyRobot {
  // Geometry (meters)
  pub center_from_kicker: f64,
  pub radius: f64,
  pub height: f64,
  pub bottom_height: f64,
  pub kicker_z: f64,
  pub kicker_thickness: f64,
  pub kicker_width: f64,
  pub kicker_height: f64,
  pub wheel_radius: f64,
  pub wheel_thickness: f64,
  /// Wheel angles in degrees (front-right, front-left, back-left, back-right).
  pub wheel_angles: [f64; WHEEL_COUNT],

  // Physics
  pub body_mass: f64,
  pub wheel_mass: f64,
  pub kicker_mass: f64,
  pub kicker_damp_factor: f64,
  pub roller_torque_factor: f64,
  pub roller_perpendicular_torque_factor: f64,
  pub kicker_friction: f64,
  pub wheel_tangent_friction: f64,
  pub wheel_perpendicular_friction: f64,
  pub wheel_motor_fmax: f64,
  pub max_linear_kick_speed: f64,
  pub max_chip_kick_speed: f64,

  // Velocity/acceleration limits
  pub acc_speedup_absolute_max: f64,
  pub acc_speedup_angular_max: f64,
  pub acc_brake_absolute_max: f64,
  pub acc_brake_angular_max: f64,
  pub vel_absolute_max: f64,
  pub vel_angular_max: f64,
}
impl From<LegacyRobot> for RobotConfig {
  fn from(r: LegacyRobot) -> Self {
    Self {
      center_from_kicker: r.center_from_kicker,
      radius: r.radius,
      height: r.height,
      bottom_height: r.bottom_height,
      kicker_z: r.kicker_z,
      kicker_thickness: r.kicker_thickness,
      kicker_width: r.kicker_width,
      kicker_height: r.kicker_height,
      wheel_radius: r.wheel_radius,
      wheel_thickness: r.wheel_thickness,
      wheel_angles: r.wheel_angles,
      body_mass: r.body_mass,
      wheel_mass: r.wheel_mass,
      kicker_mass: r.kicker_mass,
      kicker_damp_factor: r.kicker_damp_factor,
      roller_torque_factor: r.roller_torque_factor,
      roller_perpendicular_torque_factor: r.roller_perpendicular_torque_factor,
      kicker_friction: r.kicker_friction,
      wheel_tangent_friction: r.wheel_tangent_friction,
      wheel_perpendicular_friction: r.wheel_perpendicular_friction,
      wheel_motor_fmax: r.wheel_motor_fmax,
      max_linear_kick_speed: r.max_linear_kick_speed,
      max_chip_kick_speed: r.max_chip_kick_speed,
      acc_speedup_absolute_max: r.acc_speedup_absolute_max,
      acc_speedup_angular_max: r.acc_speedup_angular_max,
      acc_brake_absolute_max: r.acc_brake_absolute_max,
      acc_brake_angular_max: r.acc_brake_angular_max,
      vel_absolute_max: r.vel_absolute_max,
      vel_angular_max: r.vel_angular_max,
      ..Self::default()
    }
  }
}
impl From<LegacyLog> for ReplayLog {
  fn from(log: LegacyLog) -> Self {
    let m = log.metadata;
    let w = m.world_config;
    Self {
      metadata: ReplayMetadata {
        world_count: m.world_count,
        tick_hz: m.tick_hz,
        source: m.source,
        world_config: WorldConfig {
          robots_per_team: w.robots_per_team,
          field: w.field,
          ball: w.ball,
          blue_robots: w.blue_robots.into(),
          yellow_robots: w.yellow_robots.into(),
          physics: PhysicsConfig {
            delta_time: w.physics.delta_time,
            gravity: w.physics.gravity,
            ..PhysicsConfig::default()
          },
          seed: w.seed,
        },
      },
      frames: log.frames,
      events: log.events,
    }
  }
}
