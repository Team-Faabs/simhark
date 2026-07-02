//! Configuration types mirroring grSim's ConfigWidget and RobotSettings.
//!
//! All values use SI units (meters, kg, seconds, radians) unless noted.

use serde::{Deserialize, Serialize};

/// Maximum robots per team (matches grSim's MAX_ROBOT_COUNT).
pub const MAX_ROBOTS_PER_TEAM: usize = 16;

/// Number of teams.
pub const TEAM_COUNT: usize = 2;

/// Number of wheels per robot.
pub const WHEEL_COUNT: usize = 4;

/// Number of bounding walls.
pub const WALL_COUNT: usize = 10;

/// Ball collision sub-steps (grSim uses 5).
pub const BALL_COLLISION_SUBSTEPS: usize = 5;

/// RoboCup SSL ball speed safety limit.
pub const MAX_BALL_SPEED: f64 = 10.0;

/// Default kick/chip launch speed. This stays below the card threshold.
pub const DEFAULT_KICK_SPEED: f64 = 3.5;

/// Complete world configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WorldConfig {
  pub robots_per_team: usize,
  pub field: FieldConfig,
  pub ball: BallConfig,
  pub blue_robots: RobotConfig,
  pub yellow_robots: RobotConfig,
  pub physics: PhysicsConfig,
  /// Seed for deterministic RNG. Each world in a parallel batch gets `seed + world_index`.
  pub seed: u64,
}

impl WorldConfig {
  /// Division A defaults (12m x 9m field, 11 robots).
  pub fn division_a() -> Self {
    Self {
      robots_per_team: 11,
      field: FieldConfig::division_a(),
      ball: BallConfig::default(),
      blue_robots: RobotConfig::default(),
      yellow_robots: RobotConfig::default(),
      physics: PhysicsConfig::default(),
      seed: 42,
    }
  }

  /// Division B defaults (9m x 6m field, 6 robots).
  pub fn division_b() -> Self {
    Self {
      robots_per_team: 6,
      field: FieldConfig::division_b(),
      ball: BallConfig::default(),
      blue_robots: RobotConfig::default(),
      yellow_robots: RobotConfig::default(),
      physics: PhysicsConfig::default(),
      seed: 42,
    }
  }
}

/// Field geometry configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FieldConfig {
  pub field_length: f64,
  pub field_width: f64,
  pub field_line_width: f64,
  pub field_center_radius: f64,
  pub field_free_kick: f64,
  pub penalty_width: f64,
  pub penalty_depth: f64,
  pub penalty_point: f64,
  pub margin_touch_line: f64,
  pub margin_goal_line: f64,
  pub goal_substitution_area_width: f64,
  pub referee_margin: f64,
  pub wall_thickness: f64,
  pub goal_thickness: f64,
  pub goal_depth: f64,
  pub goal_width: f64,
  pub goal_height: f64,
}

impl FieldConfig {
  pub fn division_a() -> Self {
    Self {
      field_length: 12.0,
      field_width: 9.0,
      field_line_width: 0.010,
      field_center_radius: 0.5,
      field_free_kick: 0.7,
      penalty_width: 3.6,
      penalty_depth: 1.8,
      penalty_point: 8.0,
      margin_touch_line: 0.3,
      margin_goal_line: 0.6,
      goal_substitution_area_width: 0.3,
      referee_margin: 0.0,
      wall_thickness: 0.05,
      goal_thickness: 0.02,
      goal_depth: 0.18,
      goal_width: 1.8,
      goal_height: 0.16,
    }
  }

  pub fn division_b() -> Self {
    Self {
      field_length: 9.0,
      field_width: 6.0,
      field_line_width: 0.010,
      field_center_radius: 0.5,
      field_free_kick: 0.7,
      penalty_width: 2.0,
      penalty_depth: 1.0,
      penalty_point: 6.0,
      margin_touch_line: 0.3,
      margin_goal_line: 0.3,
      goal_substitution_area_width: 0.0,
      referee_margin: 0.0,
      wall_thickness: 0.05,
      goal_thickness: 0.02,
      goal_depth: 0.18,
      goal_width: 1.0,
      goal_height: 0.16,
    }
  }
}

/// Ball physics configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BallConfig {
  pub radius: f64,
  pub mass: f64,
  pub friction: f64,
  pub slip: f64,
  pub bounce: f64,
  pub bounce_velocity: f64,
  pub linear_damping: f64,
  pub angular_damping: f64,
}

impl Default for BallConfig {
  fn default() -> Self {
    Self {
      radius: 0.0215,
      mass: 0.043,
      friction: 0.05,
      slip: 1.0,
      bounce: 0.5,
      bounce_velocity: 0.1,
      linear_damping: 0.004,
      angular_damping: 0.004,
    }
  }
}

/// Robot physical and geometric configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotConfig {
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

impl Default for RobotConfig {
  fn default() -> Self {
    Self {
      center_from_kicker: 0.073,
      radius: 0.09,
      height: 0.147,
      bottom_height: 0.02,
      kicker_z: 0.005,
      kicker_thickness: 0.005,
      kicker_width: 0.08,
      kicker_height: 0.04,
      wheel_radius: 0.027,
      wheel_thickness: 0.005,
      wheel_angles: [30.0, 150.0, 225.0, 315.0],
      body_mass: 2.0,
      wheel_mass: 0.2,
      kicker_mass: 0.02,
      kicker_damp_factor: 0.2,
      roller_torque_factor: 0.06,
      roller_perpendicular_torque_factor: 0.005,
      kicker_friction: 0.8,
      wheel_tangent_friction: 0.8,
      wheel_perpendicular_friction: 0.05,
      wheel_motor_fmax: 0.2,
      max_linear_kick_speed: DEFAULT_KICK_SPEED,
      max_chip_kick_speed: DEFAULT_KICK_SPEED,
      acc_speedup_absolute_max: 4.0,
      acc_speedup_angular_max: 50.0,
      acc_brake_absolute_max: 4.0,
      acc_brake_angular_max: 50.0,
      vel_absolute_max: 5.0,
      vel_angular_max: 20.0,
    }
  }
}

impl RobotConfig {
  /// Compute robot start Z position (same formula as grSim's ROBOT_START_Z macro).
  pub fn start_z(&self) -> f64 {
    self.height * 0.5 + self.wheel_radius
  }
}

/// Physics engine settings.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PhysicsConfig {
  pub delta_time: f64,
  pub gravity: f64,
}

impl Default for PhysicsConfig {
  fn default() -> Self {
    Self {
      delta_time: 1.0 / 60.0,
      gravity: 9.81,
    }
  }
}
