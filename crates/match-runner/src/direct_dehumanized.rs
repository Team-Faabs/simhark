//! Direct simhark binding for the 2027 Dehumanized AI.
//!
//! This intentionally bypasses CrashPilot, the robot protocol, tf_jetsoncode,
//! and ORCA. Position commands are converted to a velocity pointing straight
//! at the requested target.

use crate::controller::{Controller, GameCommand};
use core_dump::types::ai_types::{
  self as ai, Ai, Commands, GameStage, Kicker, RobotCommand as AiRobotCommand,
};
use core_dump::vec::types::Vec2;
use dehumanized::Dehumanized;
use simhark::{
  MoveCommand, RobotCommand as SimRobotCommand, RobotState as SimRobotState, TeamColor,
  WorldConfig, WorldState,
};

const MM_PER_M: f64 = 1_000.0;
const DEFAULT_SPEED_MM_S: f64 = 4_000.0;
const POSITION_GAIN_PER_S: f64 = 4.0;
const POSITION_TOLERANCE_M: f64 = 0.015;
const HEADING_GAIN_PER_S: f64 = 6.0;
const CHIP_ANGLE_DEG: f64 = 45.0;
const GRAVITY_M_S2: f64 = 9.81;
const ESTIMATED_ROLL_DECEL_M_S2: f64 = 0.7;

pub struct DirectDehumanizedController {
  ai: Dehumanized,
  num_robots: u8,
}

impl DirectDehumanizedController {
  pub fn new(num_robots: u8) -> Self {
    Self {
      ai: Dehumanized::with_robot_count(num_robots),
      num_robots,
    }
  }
}

impl Controller for DirectDehumanizedController {
  fn name(&self) -> &str {
    "dehumanized"
  }

  fn act(
    &mut self,
    state: &WorldState,
    cfg: &WorldConfig,
    color: TeamColor,
    gc: GameCommand,
  ) -> Vec<SimRobotCommand> {
    if matches!(gc, GameCommand::Halt) {
      return stopped_commands(self.num_robots);
    }

    let game_state = world_state_to_dehumanized(state, color, gc);
    let commands = self.ai.predict(game_state);
    commands_to_sim(
      commands,
      state,
      cfg,
      color,
      self.num_robots,
      matches!(gc, GameCommand::Stop),
    )
  }
}

fn world_state_to_dehumanized(
  state: &WorldState,
  color: TeamColor,
  gc: GameCommand,
) -> ai::GameState {
  let (own, opp) = match color {
    TeamColor::Blue => (&state.blue_robots, &state.yellow_robots),
    TeamColor::Yellow => (&state.yellow_robots, &state.blue_robots),
  };

  ai::GameState {
    world: ai::World {
      own_robots: robots_to_dehumanized(own, true),
      opp_robots: robots_to_dehumanized(opp, false),
      ball: ai::BallState {
        pos: meters_to_mm(state.ball.x, state.ball.y),
        vel: meters_to_mm(state.ball.vx, state.ball.vy),
        // simhark does not currently expose a rolling-ball stop estimate.
        stop_pos: meters_to_mm(state.ball.x, state.ball.y),
        stop_time: state.sim_time as f32,
      },
    },
    stage: game_stage(gc),
  }
}

fn robots_to_dehumanized(robots: &[SimRobotState], own_team: bool) -> ai::Robots {
  let mut converted = ai::Robots::default();

  for robot in robots.iter().filter(|robot| robot.is_on) {
    if robot.id >= converted.len() {
      continue;
    }

    converted[robot.id] = Some(ai::RobotState {
      id: robot.id as u8,
      pos: meters_to_mm(robot.x, robot.y),
      vel: meters_to_mm(robot.vx, robot.vy),
      heading: robot.orientation.to_degrees().rem_euclid(360.0) as f32,
      angular_vel: robot.v_angular.to_degrees() as f32,
      is_goalie: own_team && robot.id == 0,
    });
  }

  converted
}

fn meters_to_mm(x: f64, y: f64) -> Vec2<f32> {
  Vec2::new((x * MM_PER_M) as f32, (y * MM_PER_M) as f32)
}

fn game_stage(gc: GameCommand) -> GameStage {
  match gc {
    GameCommand::Halt => GameStage::Halt,
    GameCommand::Stop => GameStage::Stop,
    GameCommand::Running => GameStage::Running,
    GameCommand::FreeKickUs | GameCommand::FreeKickThem => GameStage::FreeKick,
    GameCommand::PrepareKickoffUs | GameCommand::PrepareKickoffThem => GameStage::PrepareKickoff,
  }
}

fn commands_to_sim(
  commands: Commands,
  state: &WorldState,
  cfg: &WorldConfig,
  color: TeamColor,
  num_robots: u8,
  stop: bool,
) -> Vec<SimRobotCommand> {
  let own_robots = match color {
    TeamColor::Blue => &state.blue_robots,
    TeamColor::Yellow => &state.yellow_robots,
  };
  let robot_cfg = match color {
    TeamColor::Blue => &cfg.blue_robots,
    TeamColor::Yellow => &cfg.yellow_robots,
  };

  (0..num_robots as usize)
    .map(|id| {
      let robot = own_robots
        .iter()
        .find(|robot| robot.id == id && robot.is_on);
      let command = commands.get(id).copied().flatten();

      match (robot, command) {
        (Some(robot), Some(command)) => command_to_sim(id, robot, command, robot_cfg, stop),
        _ => stopped_command(id),
      }
    })
    .collect()
}

fn command_to_sim(
  id: usize,
  robot: &SimRobotState,
  command: AiRobotCommand,
  cfg: &simhark::RobotConfig,
  stop: bool,
) -> SimRobotCommand {
  let (vx, vy, angular) = match command.pos {
    Some(pos) => {
      let dx = pos.pos.x as f64 / MM_PER_M - robot.x;
      let dy = pos.pos.y as f64 / MM_PER_M - robot.y;
      let distance = dx.hypot(dy);
      let configured_speed = pos.speed.map(f64::from).unwrap_or(DEFAULT_SPEED_MM_S);
      let stop_limit = if stop {
        dehumanized::MAX_STOP_VELOCITY as f64
      } else {
        f64::INFINITY
      };
      let max_speed_m_s = configured_speed
        .min(stop_limit)
        .min(cfg.vel_absolute_max * MM_PER_M)
        / MM_PER_M;
      let speed = if distance <= POSITION_TOLERANCE_M {
        0.0
      } else {
        (distance * POSITION_GAIN_PER_S).min(max_speed_m_s)
      };
      let (vx, vy) = if distance > 0.0 {
        (dx / distance * speed, dy / distance * speed)
      } else {
        (0.0, 0.0)
      };
      let angular = pos
        .face
        .map(|face| {
          let error = wrap_to_pi((face as f64).to_radians() - robot.orientation);
          (error * HEADING_GAIN_PER_S).clamp(-cfg.vel_angular_max, cfg.vel_angular_max)
        })
        .unwrap_or(0.0);
      (vx, vy, angular)
    }
    None => (0.0, 0.0, 0.0),
  };

  let (kick_speed, kick_angle) = match command.kicker {
    Kicker::None => (0.0, 0.0),
    Kicker::Kick(distance_mm) => (flat_kick_speed(distance_mm, cfg.max_linear_kick_speed), 0.0),
    Kicker::Chip(distance_mm) => (
      chip_kick_speed(distance_mm, cfg.max_chip_kick_speed),
      CHIP_ANGLE_DEG,
    ),
  };

  SimRobotCommand {
    id,
    move_command: Some(MoveCommand::GlobalVelocity { vx, vy, angular }),
    kick_speed,
    kick_angle,
    dribbler_on: command.dribbler,
  }
}

fn flat_kick_speed(distance_mm: f32, max_speed: f64) -> f64 {
  let distance_m = (distance_mm as f64 / MM_PER_M).max(0.0);
  (2.0 * ESTIMATED_ROLL_DECEL_M_S2 * distance_m)
    .sqrt()
    .min(max_speed)
}

fn chip_kick_speed(distance_mm: f32, max_speed: f64) -> f64 {
  let distance_m = (distance_mm as f64 / MM_PER_M).max(0.0);
  (distance_m * GRAVITY_M_S2).sqrt().min(max_speed)
}

fn wrap_to_pi(angle: f64) -> f64 {
  let wrapped =
    (angle + std::f64::consts::PI).rem_euclid(2.0 * std::f64::consts::PI) - std::f64::consts::PI;
  if wrapped <= -std::f64::consts::PI {
    wrapped + 2.0 * std::f64::consts::PI
  } else {
    wrapped
  }
}

fn stopped_commands(num_robots: u8) -> Vec<SimRobotCommand> {
  (0..num_robots as usize).map(stopped_command).collect()
}

fn stopped_command(id: usize) -> SimRobotCommand {
  SimRobotCommand {
    id,
    move_command: Some(MoveCommand::GlobalVelocity {
      vx: 0.0,
      vy: 0.0,
      angular: 0.0,
    }),
    kick_speed: 0.0,
    kick_angle: 0.0,
    dribbler_on: false,
  }
}

#[cfg(test)]
mod tests {
  use super::*;
  use core_dump::types::ai_types::{Pos, RobotCommand};
  use simhark::BallState;
  use simhark::state::KickStatus;

  #[test]
  fn world_conversion_uses_mm_degrees_and_team_order() {
    let state = test_world();
    let converted = world_state_to_dehumanized(&state, TeamColor::Yellow, GameCommand::Running);

    let own = converted.world.own_robots[1].unwrap();
    let opp = converted.world.opp_robots[0].unwrap();
    assert_eq!(own.pos, Vec2::new(-2_000.0, 500.0));
    assert_eq!(own.vel, Vec2::new(100.0, -200.0));
    assert!((own.heading - 90.0).abs() < 1.0e-4);
    assert!(!own.is_goalie);
    assert_eq!(opp.pos, Vec2::new(1_000.0, -250.0));
    assert_eq!(converted.world.ball.pos, Vec2::new(250.0, -100.0));
  }

  #[test]
  fn position_command_becomes_direct_global_velocity() {
    let state = test_world();
    let mut commands = Commands::default();
    commands[0] = Some(RobotCommand {
      dribbler: true,
      pos: Some(Pos {
        pos: Vec2::new(2_000.0, -250.0),
        face: Some(90.0),
        speed: Some(500),
      }),
      kicker: Kicker::Kick(2_000.0),
      ..RobotCommand::default()
    });

    let output = commands_to_sim(
      commands,
      &state,
      &WorldConfig::division_b(),
      TeamColor::Blue,
      1,
      false,
    );
    let command = &output[0];
    assert!(matches!(
      command.move_command,
      Some(MoveCommand::GlobalVelocity { vx, vy, .. })
        if (vx - 0.5).abs() < 1.0e-9 && vy.abs() < 1.0e-9
    ));
    assert!(command.kick_speed > 0.0);
    assert_eq!(command.kick_angle, 0.0);
    assert!(command.dribbler_on);
  }

  #[test]
  fn absent_ai_command_explicitly_stops_latched_motion() {
    let output = commands_to_sim(
      Commands::default(),
      &test_world(),
      &WorldConfig::division_b(),
      TeamColor::Blue,
      1,
      false,
    );

    assert!(matches!(
      output[0].move_command,
      Some(MoveCommand::GlobalVelocity {
        vx: 0.0,
        vy: 0.0,
        angular: 0.0
      })
    ));
  }

  #[test]
  fn controller_can_step_unfinished_dehumanized_without_panicking() {
    let mut controller = DirectDehumanizedController::new(1);
    let output = controller.act(
      &test_world(),
      &WorldConfig::division_b(),
      TeamColor::Blue,
      GameCommand::Running,
    );

    assert_eq!(output.len(), 1);
    assert!(matches!(
      output[0].move_command,
      Some(MoveCommand::GlobalVelocity {
        vx: 0.0,
        vy: 0.0,
        angular: 0.0
      })
    ));
  }

  fn test_world() -> WorldState {
    WorldState {
      world_id: 0,
      sim_time: 1.25,
      frame: 75,
      ball: BallState {
        x: 0.25,
        y: -0.1,
        z: 0.0,
        vx: 0.3,
        vy: 0.0,
        vz: 0.0,
      },
      blue_robots: vec![robot(0, TeamColor::Blue, 1.0, -0.25, 0.0)],
      yellow_robots: vec![robot(
        1,
        TeamColor::Yellow,
        -2.0,
        0.5,
        std::f64::consts::FRAC_PI_2,
      )],
      goal_blue: false,
      goal_yellow: false,
    }
  }

  fn robot(id: usize, team: TeamColor, x: f64, y: f64, orientation: f64) -> SimRobotState {
    SimRobotState {
      id,
      team,
      x,
      y,
      z: 0.1,
      orientation,
      vx: if team == TeamColor::Yellow { 0.1 } else { 0.0 },
      vy: if team == TeamColor::Yellow { -0.2 } else { 0.0 },
      vz: 0.0,
      v_angular: 0.0,
      infrared: false,
      dribbler_on: false,
      kick_status: KickStatus::NoKick,
      is_on: true,
      wheel_speeds: [0.0; 4],
    }
  }
}
