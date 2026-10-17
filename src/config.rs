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

/// Sanity cap for externally injected ball velocity (teleport commands).
/// Gameplay ball speed is never clamped; the SSL excessive-ball-speed rule
/// (6.5 m/s) is enforced by the autoref, not the physics.
pub const MAX_BALL_SPEED: f64 = 10.0;

/// Default kick/chip launch speed. This stays below the 6.5 m/s ball speed
/// rule limit.
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
  /// Shared simulator defaults in the repository config file.
  pub fn load_default_simulation_config(&mut self) -> Result<(), String> {
    self.load_simulation_config(concat!(env!("CARGO_MANIFEST_DIR"), "/config.toml"))
  }
  /// Load the `[simulation]` table while preserving division defaults and unrelated app settings.
  pub fn load_simulation_config(
    &mut self,
    path: impl AsRef<std::path::Path>,
  ) -> Result<(), String> {
    let path = path.as_ref();
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    self
      .apply_simulation_toml(&text)
      .map_err(|e| format!("{}: {e}", path.display()))
  }

  pub fn apply_simulation_toml(&mut self, text: &str) -> Result<(), String> {
    let mut document: toml::Table = toml::from_str(text).map_err(|e| e.to_string())?;
    let Some(simulation) = document.remove("simulation") else {
      return Ok(());
    };
    // JSON represents optional fields as null, allowing new optional tables to
    // be patched while omitted values retain their current defaults.
    fn overlay(base: &mut serde_json::Value, patch: &serde_json::Value) -> Result<(), String> {
      if let (Some(a), Some(b)) = (base.as_object_mut(), patch.as_object()) {
        for (key, value) in b {
          let old = a
            .get_mut(key)
            .ok_or_else(|| format!("unknown simulation setting: {key}"))?;
          if old.is_null() {
            *old = value.clone();
          } else {
            overlay(old, value)?;
          }
        }
      } else {
        *base = patch.clone();
      }
      Ok(())
    }
    if !simulation.is_table() {
      return Err("simulation must be a table".into());
    }
    fn finite(value: &toml::Value) -> bool {
      match value {
        toml::Value::Float(value) => value.is_finite(),
        toml::Value::Array(values) => values.iter().all(finite),
        toml::Value::Table(values) => values.values().all(finite),
        _ => true,
      }
    }
    if !finite(&simulation) {
      return Err("simulation settings must be finite".into());
    }
    let patch = serde_json::to_value(simulation).map_err(|e| e.to_string())?;
    let mut base = serde_json::to_value(&*self).map_err(|e| e.to_string())?;
    overlay(&mut base, &patch)?;
    let next: Self = serde_json::from_value(base).map_err(|e| e.to_string())?;
    next.validate_robot_physics()?;
    *self = next;
    Ok(())
  }

  pub fn validate_robot_physics(&self) -> Result<(), String> {
    self.physics.floor.validate()?;
    if !self.physics.delta_time.is_finite()
      || self.physics.delta_time <= 0.0
      || !self.physics.gravity.is_finite()
      || self.physics.gravity < 0.0
    {
      return Err("invalid simulation time step/gravity".into());
    }
    for robot in [&self.blue_robots, &self.yellow_robots] {
      for value in [
        robot.body_mass,
        robot.kicker_mass,
        robot.wheel_mass,
        robot.radius,
        robot.height,
        robot.wheel_radius,
        robot.wheel_motor_fmax,
      ] {
        if !value.is_finite() || value <= 0.0 {
          return Err("robot masses and geometry must be finite and positive".into());
        }
      }
      if robot.mass_kg.is_some_and(|v| !v.is_finite() || v <= 0.0)
        || robot
          .center_of_mass_body_m
          .is_some_and(|p| p.iter().any(|v| !v.is_finite()))
      {
        return Err("invalid robot mass/CoM".into());
      }
      if let Some(wheel) = &robot.wheel {
        wheel.validate()?;
      }
      if robot.wheel_angles.iter().any(|angle| !angle.is_finite()) {
        return Err("wheel angles must be finite".into());
      }
      let geometry = nalgebra::SMatrix::<f64, 3, WHEEL_COUNT>::from_fn(|row, col| {
        let angle = robot.wheel_angles[col].to_radians();
        match row {
          0 => 1.0,
          1 => robot.radius * angle.cos(),
          _ => robot.radius * angle.sin(),
        }
      });
      if (geometry * geometry.transpose()).try_inverse().is_none() {
        return Err("wheel support geometry must be invertible".into());
      }
    }
    Ok(())
  }

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
#[serde(default)]
pub struct RobotConfig {
  /// Optional total robot mass (kg), overriding body_mass + kicker_mass.
  pub mass_kg: Option<f64>,
  /// CoM relative to chassis origin: x forward, y left, z up (meters).
  pub center_of_mass_body_m: Option<[f64; 3]>,
  /// Enable directional wheel/floor slip physics in the standard planar simulator.
  pub wheel: Option<WheelProfile>,
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
      mass_kg: None,
      center_of_mass_body_m: None,
      wheel: None,
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
#[serde(default)]
pub struct PhysicsConfig {
  pub floor: FloorConfig,
  pub delta_time: f64,
  pub gravity: f64,
}

impl Default for PhysicsConfig {
  fn default() -> Self {
    Self {
      floor: FloorConfig::default(),
      delta_time: 1.0 / 60.0,
      gravity: 9.81,
    }
  }
}

/// Wheel material parameters. None inherits the corresponding legacy contact/hardware value.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct WheelProfile {
  pub longitudinal_friction: Option<f64>,
  pub transverse_friction: Option<f64>,
  pub longitudinal_slip_stiffness_n_per_m_s: Option<f64>,
  pub transverse_slip_stiffness_n_per_m_s: Option<f64>,
  pub rolling_resistance: Option<f64>,
}

impl WheelProfile {
  pub(crate) fn validate(&self) -> Result<(), String> {
    for value in [
      self.longitudinal_friction,
      self.transverse_friction,
      self.longitudinal_slip_stiffness_n_per_m_s,
      self.transverse_slip_stiffness_n_per_m_s,
      self.rolling_resistance,
    ]
    .into_iter()
    .flatten()
    {
      if !value.is_finite() || value < 0.0 {
        return Err(
          "wheel friction, stiffness, and rolling resistance must be finite and nonnegative".into(),
        );
      }
    }
    Ok(())
  }
}

/// Venue properties applied to the robot's wheel material. Axes follow each wheel.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct FloorConfig {
  pub longitudinal_grip_scale: f64,
  pub transverse_grip_scale: f64,
  /// Additional slip compliance (m/s per N), in series with the wheel compliance.
  pub longitudinal_slip_compliance_m_s_per_n: f64,
  pub transverse_slip_compliance_m_s_per_n: f64,
  pub rolling_resistance_scale: f64,
}

impl Default for FloorConfig {
  fn default() -> Self {
    Self {
      longitudinal_grip_scale: 1.0,
      transverse_grip_scale: 1.0,
      longitudinal_slip_compliance_m_s_per_n: 0.0,
      transverse_slip_compliance_m_s_per_n: 0.0,
      rolling_resistance_scale: 1.0,
    }
  }
}

impl FloorConfig {
  pub(crate) fn validate(&self) -> Result<(), String> {
    for value in [
      self.longitudinal_grip_scale,
      self.transverse_grip_scale,
      self.longitudinal_slip_compliance_m_s_per_n,
      self.transverse_slip_compliance_m_s_per_n,
      self.rolling_resistance_scale,
    ] {
      if !value.is_finite() || value < 0.0 {
        return Err(
          "floor grip, slip compliance, and rolling resistance must be finite and nonnegative"
            .into(),
        );
      }
    }
    Ok(())
  }
}
