//! Commands sent to the simulation each step.

use serde::{Deserialize, Serialize};

use crate::state::TeamColor;

/// Velocity command for a single robot.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum MoveCommand {
  /// Local velocity: forward (m/s), left (m/s), angular (rad/s).
  LocalVelocity {
    forward: f64,
    left: f64,
    angular: f64,
  },
  /// Global velocity: vx (m/s), vy (m/s), angular (rad/s).
  GlobalVelocity { vx: f64, vy: f64, angular: f64 },
  /// Individual wheel angular speeds in rad/s:
  /// [front_right, front_left, back_left, back_right].
  WheelVelocity([f64; 4]),
  /// Opaque host instruction for an installed realistic onboard controller.
  /// Transport delivers these bytes before onboard logic reads local sensors.
  Onboard(Vec<u8>),
}

/// Command for a single robot.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotCommand {
  pub id: usize,
  /// Movement command.
  pub move_command: Option<MoveCommand>,
  /// Kick speed (m/s). Only fires if touching ball.
  pub kick_speed: f64,
  /// Kick angle in degrees (0 = flat, >0 = chip).
  pub kick_angle: f64,
  /// Whether the dribbler/spinner is on.
  pub dribbler_on: bool,
}

impl RobotCommand {
  /// Create a no-op command for the given robot.
  pub fn noop(id: usize) -> Self {
    Self {
      id,
      move_command: None,
      kick_speed: 0.0,
      kick_angle: 0.0,
      dribbler_on: false,
    }
  }
}

/// Commands for an entire team.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TeamCommand {
  pub team: TeamColor,
  pub commands: Vec<RobotCommand>,
}

/// Teleport a robot to a specific position.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TeleportRobot {
  pub id: usize,
  pub team: TeamColor,
  pub x: Option<f64>,
  pub y: Option<f64>,
  pub orientation: Option<f64>,
  /// Optional rigid-body linear velocity in world frame (m/s).
  pub vx: Option<f64>,
  /// Optional rigid-body linear velocity in world frame (m/s).
  pub vy: Option<f64>,
  /// Optional rigid-body angular velocity around +Z (rad/s).
  pub v_angular: Option<f64>,
  pub present: Option<bool>,
}

/// Teleport the ball.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TeleportBall {
  pub x: Option<f64>,
  pub y: Option<f64>,
  pub z: Option<f64>,
  pub vx: Option<f64>,
  pub vy: Option<f64>,
  pub vz: Option<f64>,
}

/// Commands for a single world in a given step.
/// Per-robot motion and dribbler commands are latched: omitted robots keep
/// their previous command until explicitly overridden.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct WorldCommand {
  pub blue: Vec<RobotCommand>,
  pub yellow: Vec<RobotCommand>,
  pub teleport_ball: Option<TeleportBall>,
  pub teleport_robots: Vec<TeleportRobot>,
}
