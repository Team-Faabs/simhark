//! Controllers receive only delivered observations and commands, never World.
use super::{config::MotorConfig, sensors::Observations, time::TimeNs};
use crate::{MoveCommand, RobotCommand, RobotConfig};

pub trait RobotController: Send {
  /// Returning None opens the driver output (coasting). Some(0) actively brakes.
  fn tick(
    &mut self,
    now: TimeNs,
    command: Option<&RobotCommand>,
    observations: &Observations,
  ) -> [Option<f64>; 4];
}

/// Replays an actual process/controller output trace. Missing output fails
/// closed; callers compare traces to detect a mismatched input/build timeline.
pub struct RecordedController {
  outputs: std::collections::BTreeMap<TimeNs, [Option<f64>; 4]>,
}
impl RecordedController {
  pub fn new(outputs: std::collections::BTreeMap<TimeNs, [Option<f64>; 4]>) -> Self {
    Self { outputs }
  }
}
impl RobotController for RecordedController {
  fn tick(&mut self, now: TimeNs, _: Option<&RobotCommand>, _: &Observations) -> [Option<f64>; 4] {
    self.outputs.remove(&now).unwrap_or([None; 4])
  }
}

/// Explicit surrogate until production onboard control has a stable interface.
/// Wheel feedback is derived from quantized encoders, not chassis ground truth.
pub struct EncoderVelocityController {
  robot: RobotConfig,
  motor: MotorConfig,
  observed_yaw: Option<(f64, TimeNs)>,
  team: crate::TeamColor,
  id: usize,
  target_wheel_speed: [f64; 4],
}
impl EncoderVelocityController {
  pub fn new(robot: RobotConfig, motor: MotorConfig, team: crate::TeamColor, id: usize) -> Self {
    Self {
      robot,
      motor,
      observed_yaw: None,
      team,
      id,
      target_wheel_speed: [0.0; 4],
    }
  }
}
impl RobotController for EncoderVelocityController {
  fn tick(
    &mut self,
    now: TimeNs,
    command: Option<&RobotCommand>,
    observations: &Observations,
  ) -> [Option<f64>; 4] {
    if let Some(detection) = observations.vision.as_ref().and_then(|v| {
      v.robots
        .iter()
        .find(|r| r.team == self.team && r.id == self.id)
    }) {
      self.observed_yaw = Some((
        detection.orientation_rad,
        observations.vision.as_ref().unwrap().captured_at,
      ));
    }
    if let Some(pose) = &observations.pose {
      if !pose.ambiguous
        && pose.covariance[2][2] < 0.5
        && let Some(yaw) = pose.yaw_rad
      {
        self.observed_yaw = Some((yaw, pose.estimated_at));
      }
    }
    let Some(command) = command else {
      return [None; 4];
    };
    let Some(encoder) = observations.encoder.as_ref() else {
      return [None; 4];
    };
    if now.saturating_sub(encoder.sampled_at) > 20_000_000 {
      return [None; 4];
    }
    let target = match command.move_command.as_ref() {
      None => self.target_wheel_speed,
      Some(MoveCommand::WheelVelocity(speed)) => *speed,
      Some(movement) => {
        let (mut vx, mut vy, angular) = match *movement {
          MoveCommand::LocalVelocity {
            forward,
            left,
            angular,
          } => (forward, left, angular),
          MoveCommand::GlobalVelocity { vx, vy, angular } => {
            // No heading observation means world-frame control is unavailable.
            let Some((yaw, _at)) = self
              .observed_yaw
              .filter(|(_, at)| now.saturating_sub(*at) <= 200_000_000)
            else {
              return [None; 4];
            };
            (
              vx * yaw.cos() + vy * yaw.sin(),
              -vx * yaw.sin() + vy * yaw.cos(),
              angular,
            )
          }
          MoveCommand::WheelVelocity(_) => unreachable!(),
        };
        let speed = vx.hypot(vy);
        if speed > self.robot.vel_absolute_max {
          vx *= self.robot.vel_absolute_max / speed;
          vy *= self.robot.vel_absolute_max / speed;
        }
        let angular = angular.clamp(-self.robot.vel_angular_max, self.robot.vel_angular_max);
        self.robot.wheel_angles.map(|degrees| {
          let angle = degrees.to_radians();
          (-vx * angle.sin() + vy * angle.cos() + self.robot.radius * angular)
            / self.robot.wheel_radius
        })
      }
    };
    self.target_wheel_speed = target;
    std::array::from_fn(|i| {
      Some(
        (self.motor.back_emf_v_per_rad_s * target[i]
          + self.motor.speed_gain_v_per_rad_s * (target[i] - encoder.speed_rad_s[i]))
          .clamp(-self.motor.supply_voltage_v, self.motor.supply_voltage_v),
      )
    })
  }
}
