use core_dump::proto::CrashpilotState;
use simhark::{DEFAULT_KICK_SPEED, MoveCommand, RobotCommand, TeamColor, WorldCommand};
use tf_jetsoncode::{TeensySendMsg, send_flags};

const HEADING_GAIN: f64 = 6.0;
const MAX_ANGULAR: f64 = 20.0;
const MAX_KICK_SPEED: f64 = DEFAULT_KICK_SPEED;
const CHIP_ANGLE_DEG: f64 = 45.0;

pub fn run_sim_action(
  robot_id: u32,
  teensy: TeensySendMsg,
  command: &mut WorldCommand,
  team: TeamColor,
) {
  let id = robot_id as usize;

  let team_cmds = match team {
    TeamColor::Yellow => &mut command.yellow,
    TeamColor::Blue => &mut command.blue,
  };

  if teensy.state == CrashpilotState::Halt as u8 {
    team_cmds.push(RobotCommand {
      id,
      move_command: Some(MoveCommand::GlobalVelocity {
        vx: 0.0,
        vy: 0.0,
        angular: 0.0,
      }),
      kick_speed: 0.0,
      kick_angle: 0.0,
      dribbler_on: false,
    });
    return;
  }

  let dir_rad = (teensy.dir as f64).to_radians();
  let speed_mps = teensy.speed as f64 / 1000.0;
  let vx = speed_mps * dir_rad.cos();
  let vy = speed_mps * dir_rad.sin();

  let heading_error =
    wrap_to_pi((teensy.orient as f64).to_radians() - (teensy.self_orient as f64).to_radians());
  let angular = (heading_error * HEADING_GAIN).clamp(-MAX_ANGULAR, MAX_ANGULAR);

  let kicking = teensy.flags & (send_flags::KICK | send_flags::CHIP) != 0;
  let kick_speed = if kicking {
    (teensy.kick_pwr as f64 / u8::MAX as f64) * MAX_KICK_SPEED
  } else {
    0.0
  };
  let kick_angle = if teensy.flags & send_flags::CHIP != 0 {
    CHIP_ANGLE_DEG
  } else {
    0.0
  };

  let dribbler_on = teensy.flags & send_flags::DRIBBLER != 0;
  if std::env::var_os("FAABS_DEBUG").is_some() && id == 5 {
    eprintln!(
      "[faabs] team={team:?} id={id} state={} dir={} speed={} orient={} self={} vx={vx:.2} vy={vy:.2} ang={angular:.2} drib={dribbler_on}",
      teensy.state, teensy.dir, teensy.speed, teensy.orient, teensy.self_orient,
    );
  }

  team_cmds.push(RobotCommand {
    id,
    move_command: Some(MoveCommand::GlobalVelocity { vx, vy, angular }),
    kick_speed,
    kick_angle,
    dribbler_on,
  });
}

fn wrap_to_pi(angle: f64) -> f64 {
  use std::f64::consts::PI;
  let wrapped = (angle + PI).rem_euclid(2.0 * PI) - PI;
  if wrapped <= -PI {
    wrapped + 2.0 * PI
  } else {
    wrapped
  }
}
