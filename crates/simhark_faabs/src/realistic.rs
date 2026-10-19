//! Production Jetson logic runs on delivered host instructions and local sensor
//! samples. Its Teensy output stays onboard and never takes another radio trip.
use crate::Faabs;
use core_dump::types::Ai;
use prost::Message;
use simhark::realistic::{
  RealisticWorld,
  controller::{EncoderVelocityController, RobotController},
  sensors::Observations,
};
use simhark::{MoveCommand, RobotCommand, TeamColor, WorldCommand, WorldConfig};
use tf_jetsoncode::{Robot, TeensyRecMSG};

impl<A: Ai + Send> Faabs<A> {
  pub fn install_realistic(
    &mut self,
    sim: &mut RealisticWorld,
    cfg: &WorldConfig,
  ) -> Result<(), String> {
    for (id, robot) in std::mem::take(&mut self.robots).into_iter().enumerate() {
      let geometry = if self.team == TeamColor::Blue {
        &cfg.blue_robots
      } else {
        &cfg.yellow_robots
      };
      let calibration = &sim.config().known_calibration;
      let drive = EncoderVelocityController::new(
        calibration
          .robot
          .clone()
          .unwrap_or_else(|| geometry.clone()),
        calibration
          .motor
          .clone()
          .unwrap_or_else(|| sim.config().motor.clone()),
        self.team,
        id,
      );
      sim.set_controller(
        self.team,
        id,
        Box::new(Onboard {
          robot,
          drive,
          team: self.team,
          id,
        }),
      )?;
    }
    self.host_only = true;
    Ok(())
  }
}

struct Onboard {
  robot: Robot<()>,
  drive: EncoderVelocityController,
  team: TeamColor,
  id: usize,
}
impl RobotController for Onboard {
  fn onboard_command(
    &mut self,
    now: u64,
    payload: &[u8],
    observations: &Observations,
    kicker_ready: bool,
  ) -> Option<RobotCommand> {
    let cp = tf_jetsoncode::CrashpilotRobot::decode(payload).ok()?;
    if cp.robot_id as usize != self.id {
      return None;
    }
    let has_ball = observations
      .infrared
      .as_ref()
      .is_some_and(|ir| ir.detected && now.saturating_sub(ir.sampled_at) <= 100_000_000);
    let flags =
      (u32::from(has_ball) << 1) | (u32::from(kicker_ready) << 2) | (u32::from(kicker_ready) << 3);
    let (teensy, _) = self.robot.step_with_data(tf_jetsoncode::Events {
      cp: Some(cp),
      vis: None,
      teensy: Some(TeensyRecMSG {
        flags,
        batt_level: 0,
        current: 0,
      }),
    });
    let mut output = WorldCommand::default();
    crate::run_sim_action(self.id as u32, teensy, &mut output, self.team);
    if self.team == TeamColor::Blue {
      output.blue.pop()
    } else {
      output.yellow.pop()
    }
  }
  fn tick(
    &mut self,
    now: u64,
    command: Option<&RobotCommand>,
    observations: &Observations,
  ) -> [Option<f64>; 4] {
    // Unknown/malformed host instructions fail closed before reaching the drive.
    if command.is_some_and(|c| matches!(c.move_command, Some(MoveCommand::Onboard(_)))) {
      return [None; 4];
    }
    self.drive.tick(now, command, observations)
  }
}

#[cfg(test)]
mod tests {
  use super::*;
  #[test]
  fn production_jetson_kick_reads_local_ir_on_each_tick() {
    use core_dump::proto::{
      CrashpilotCommand, CrashpilotInfos, CrashpilotRobot, CrashpilotState, CrashpilotTask,
      CrashpilotTrackedRobot,
    };
    use simhark::realistic::{config::MotorConfig, sensors::InfraredSample};
    let mut config = tf_jetsoncode::Config::default();
    config.robot_id = 0;
    let mut onboard = Onboard {
      robot: Robot::new(config),
      drive: EncoderVelocityController::new(
        Default::default(),
        MotorConfig::default(),
        TeamColor::Blue,
        0,
      ),
      team: TeamColor::Blue,
      id: 0,
    };
    let mut cp = CrashpilotRobot {
      robot_id: 0,
      cmd: CrashpilotCommand {
        state: CrashpilotState::Free as i32,
        task: CrashpilotTask::Kick as i32,
        kick_orient: Some(0),
        kick_speed: Some(255),
        ..Default::default()
      },
      infos: CrashpilotInfos {
        team_color: true,
        width: 9000,
        height: 6000,
        ..Default::default()
      },
      robots_blue: vec![CrashpilotTrackedRobot {
        robot_id: 0,
        visibility: 100,
        ..Default::default()
      }],
      ..Default::default()
    };
    let payload = cp.encode_to_vec();
    let mut sensors = Observations::default();
    let initial = onboard
      .onboard_command(1_000_000, &payload, &sensors, true)
      .unwrap();
    assert_eq!(initial.kick_speed, 0.0);
    sensors.infrared = Some(InfraredSample {
      sampled_at: 2_000_000,
      detected: true,
    });
    let kick = onboard
      .onboard_command(2_000_000, &payload, &sensors, true)
      .unwrap();
    assert!(kick.kick_speed > 0.0);
    let stale = onboard
      .onboard_command(103_000_000, &payload, &sensors, true)
      .unwrap();
    assert_eq!(stale.kick_speed, 0.0);
    assert!(
      onboard
        .onboard_command(104_000_000, &[255], &sensors, true)
        .is_none()
    );
    cp.robot_id = 1;
    assert!(
      onboard
        .onboard_command(104_000_000, &cp.encode_to_vec(), &sensors, true)
        .is_none()
    );
  }
}
