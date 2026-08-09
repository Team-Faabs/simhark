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
use dehumanized::mut_command::MutCommands;
use dehumanized::mut_state::MutGameState;
use dehumanized::skill::{Skill, SkillFactory};
use dehumanized::skills::registry::{PLAYS, SKILLS};
use serde_json::Value;
use simhark::{
  MoveCommand, RobotCommand as SimRobotCommand, RobotState as SimRobotState, TeamColor,
  WorldConfig, WorldState,
};
use std::panic::{AssertUnwindSafe, catch_unwind};
use webinterface_protocol::DeveloperRunState;

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
  /// The entry the operator selected. Loading never instantiates anything, so
  /// editing parameters in the AI Lab cannot disturb a run in progress.
  loaded: Option<LoadedEntry>,
  /// The one live instance, created by `start` and stepped until it finishes.
  run: Option<SkillRun>,
  state: DeveloperRunState,
  message: String,
}

#[derive(Clone)]
struct LoadedEntry {
  kind: String,
  name: String,
  factory: &'static dyn SkillFactory,
  config: Value,
  params: Value,
}

/// One live registry instance together with the buffers it borrows.
///
/// Registry entries are stateful — an async skill parks on a waiter and
/// resumes on the next step — so the instance has to outlive the tick that
/// created it. It holds references into `state` and `commands`, which are
/// therefore boxed (stable addresses) and updated in place each tick rather
/// than rebuilt.
///
/// Field order is drop order: `skill` is declared first so it is destroyed
/// before the buffers whose addresses it holds.
struct SkillRun {
  skill: Box<dyn Skill<'static> + 'static>,
  state: Box<MutGameState>,
  commands: Box<MutCommands>,
  finished: bool,
}

impl SkillRun {
  fn start(
    entry: &LoadedEntry,
    num_robots: u8,
    game_state: ai::GameState,
  ) -> Result<Self, String> {
    let state = Box::new(MutGameState::new(game_state));
    let commands = Box::new(MutCommands::new(initial_commands(num_robots)));

    // SAFETY: both buffers are boxed and owned by the returned `SkillRun`, so
    // their addresses stay valid and stable for the whole life of `skill`.
    // `skill` is dropped first, and the extended references never escape this
    // struct.
    let state_ref: &'static MutGameState = unsafe { &*(&raw const *state) };
    let commands_ref: &'static MutCommands = unsafe { &*(&raw const *commands) };

    // Entries are free to assume their configuration is sane and panic when it
    // is not (`PassTo` unwraps its passer), so instantiation is guarded too.
    let skill = catch_unwind(AssertUnwindSafe(|| {
      entry
        .factory
        .instantiate(
          entry.config.clone(),
          entry.params.clone(),
          state_ref,
          commands_ref,
        )
        .map_err(|error| error.to_string())
    }))
    .map_err(|_| "the entry panicked while being created".to_string())??;

    Ok(Self {
      skill,
      state,
      commands,
      finished: false,
    })
  }

  /// Steps the instance once against this tick's world.
  fn step(&mut self, game_state: ai::GameState, num_robots: u8) -> Result<Commands, String> {
    self.state.update(game_state);
    // Commands are an output buffer, not accumulated state: a robot the entry
    // stops writing to must fall still rather than latch its last target.
    self.commands.update(initial_commands(num_robots));

    let finished = catch_unwind(AssertUnwindSafe(|| self.skill.step()))
      .map_err(|_| "the entry panicked while stepping".to_string())?;
    self.finished = finished;

    let mut output = self.commands.commands();
    for command in output.iter_mut().flatten() {
      // Direct registry execution intentionally bypasses the normal AI and
      // collision planner. The simhark binding below turns these targets
      // straight into simulator drive velocities.
      command.raw_movement = true;
    }
    Ok(output)
  }
}

fn initial_commands(num_robots: u8) -> Commands {
  let mut commands = Commands::default();
  for command in commands.iter_mut().take(num_robots as usize) {
    *command = Some(AiRobotCommand::default());
  }
  commands
}

impl DirectDehumanizedController {
  pub fn new(num_robots: u8) -> Self {
    Self {
      ai: Dehumanized::with_robot_count(num_robots),
      num_robots,
      loaded: None,
      run: None,
      state: DeveloperRunState::Idle,
      message: "No entry loaded".to_string(),
    }
  }

  /// Selects and validates an entry without running it.
  fn load(
    &mut self,
    kind: &str,
    entry: &str,
    config: &Value,
    params: &Value,
  ) -> Result<String, String> {
    let registry = match kind {
      "skill" | "skills" => SKILLS.0,
      "play" | "plays" => PLAYS.0,
      other => return Err(format!("unknown registry kind: {other}")),
    };
    let Some((_, factory)) = registry.iter().find(|(name, _)| *name == entry) else {
      return Err(format!("{entry:?} is not registered in {kind}"));
    };
    factory
      .validate(config, params)
      .map_err(|error| format!("invalid {entry} values: {error}"))?;

    // Loading replaces whatever was running: the operator asked for a
    // different configuration, and silently keeping the old instance alive
    // under a new label would be worse than ending it.
    self.run = None;
    self.loaded = Some(LoadedEntry {
      kind: kind.to_string(),
      name: entry.to_string(),
      factory: *factory,
      config: config.clone(),
      params: params.clone(),
    });
    Ok(self.set_state(DeveloperRunState::Loaded, format!("{entry} is ready to start")))
  }

  fn start(&mut self, state: &WorldState, color: TeamColor, gc: GameCommand) -> Result<String, String> {
    let Some(entry) = self.loaded.clone() else {
      return Err("load an entry before starting it".to_string());
    };
    if self
      .run
      .as_ref()
      .is_some_and(|run| !run.finished)
    {
      return Err(format!("{} is already running", entry.name));
    }

    let game_state = world_state_to_dehumanized(state, color, gc);
    match SkillRun::start(&entry, self.num_robots, game_state) {
      Ok(run) => {
        self.run = Some(run);
        Ok(self.set_state(
          DeveloperRunState::Running,
          format!("{} is driving directly", entry.name),
        ))
      }
      Err(error) => {
        self.run = None;
        let message = format!("{}: {error}", entry.name);
        self.set_state(DeveloperRunState::Failed, message.clone());
        Err(message)
      }
    }
  }

  /// Ends the run but keeps the selection, so it can be started again.
  fn stop(&mut self) -> Result<String, String> {
    let Some(entry) = self.loaded.as_ref().map(|entry| entry.name.clone()) else {
      return Err("nothing is loaded".to_string());
    };
    self.run = None;
    Ok(self.set_state(
      DeveloperRunState::Stopped,
      format!("{entry} stopped; start it again to re-run"),
    ))
  }

  fn disable(&mut self) -> String {
    self.run = None;
    self.loaded = None;
    self.set_state(DeveloperRunState::Idle, "Match AI restored".to_string())
  }

  fn set_state(&mut self, state: DeveloperRunState, message: String) -> String {
    self.state = state;
    self.message = message.clone();
    message
  }

  fn fail_run(&mut self, error: String) {
    let entry = self
      .loaded
      .as_ref()
      .map(|entry| entry.name.as_str())
      .unwrap_or("entry");
    let message = format!("{entry}: {error}");
    eprintln!("[dehumanized-dev] {message}");
    self.run = None;
    self.set_state(DeveloperRunState::Failed, message);
  }

  fn finish_run(&mut self) {
    let entry = self
      .loaded
      .as_ref()
      .map(|entry| entry.name.as_str())
      .unwrap_or("entry");
    let message = format!("{entry} finished");
    self.set_state(DeveloperRunState::Finished, message);
  }

  /// Steps the live instance, or falls back to the match AI when no run owns
  /// this side. A finished run is never restarted on its own.
  fn drive(&mut self, game_state: ai::GameState) -> Commands {
    if !self.run.as_ref().is_some_and(|run| !run.finished) {
      return self.ai.predict(game_state);
    }

    let num_robots = self.num_robots;
    let stepped = self
      .run
      .as_mut()
      .expect("a live run was just observed")
      .step(game_state, num_robots);

    match stepped {
      Ok(commands) => {
        if self.run.as_ref().is_some_and(|run| run.finished) {
          self.finish_run();
        }
        commands
      }
      Err(error) => {
        self.fail_run(error);
        self.ai.predict(game_state)
      }
    }
  }
}

impl Controller for DirectDehumanizedController {
  fn name(&self) -> &str {
    "dehumanized"
  }

  #[cfg(feature = "viewer")]
  fn developer_schema(&self) -> Option<Value> {
    Some(dehumanized::skills::registry::renderer_schema())
  }

  #[cfg(feature = "viewer")]
  fn developer_run(&self) -> Option<simhark::viewer::DeveloperRun> {
    Some(simhark::viewer::DeveloperRun {
      target: String::new(),
      kind: self.loaded.as_ref().map(|entry| entry.kind.clone()),
      entry: self.loaded.as_ref().map(|entry| entry.name.clone()),
      state: self.state,
      message: self.message.clone(),
      // The match runner owns the frame numbers; the controller only knows
      // which entry it is holding and what happened to it.
      started_frame: None,
      finished_frame: None,
    })
  }

  #[cfg(feature = "viewer")]
  fn apply_developer_request(
    &mut self,
    request: &simhark::viewer::DeveloperRequest,
    world: &WorldState,
    color: TeamColor,
    gc: GameCommand,
  ) -> Result<String, String> {
    match request {
      simhark::viewer::DeveloperRequest::Load {
        kind,
        entry,
        config,
        params,
        ..
      } => self.load(kind, entry, config, params),
      simhark::viewer::DeveloperRequest::Start { .. } => self.start(world, color, gc),
      simhark::viewer::DeveloperRequest::Stop { .. } => self.stop(),
      simhark::viewer::DeveloperRequest::Disable { .. } => Ok(self.disable()),
      _ => Err("this request is handled by the match runner".to_string()),
    }
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
    let commands = self.drive(game_state);
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
      // A target position is optional: an entry may ask only for a heading,
      // which turns the robot on the spot instead of driving it anywhere.
      let (vx, vy) = match pos.pos {
        Some(target) => {
          let dx = target.x as f64 / MM_PER_M - robot.x;
          let dy = target.y as f64 / MM_PER_M - robot.y;
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
          if distance > 0.0 {
            (dx / distance * speed, dy / distance * speed)
          } else {
            (0.0, 0.0)
          }
        }
        None => (0.0, 0.0),
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
        pos: Some(Vec2::new(2_000.0, -250.0)),
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

  #[cfg(feature = "viewer")]
  fn load_pass_to(controller: &mut DirectDehumanizedController) -> Result<String, String> {
    controller.apply_developer_request(
      &simhark::viewer::DeveloperRequest::Load {
        target: "blue".to_string(),
        kind: "skill".to_string(),
        entry: "Pass To".to_string(),
        config: serde_json::json!({}),
        params: serde_json::json!({
          "passer": "R0",
          "receiver": "R0",
        }),
      },
      &test_world(),
      TeamColor::Blue,
      GameCommand::Running,
    )
  }

  #[cfg(feature = "viewer")]
  fn start(controller: &mut DirectDehumanizedController) -> Result<String, String> {
    controller.apply_developer_request(
      &simhark::viewer::DeveloperRequest::Start {
        target: "blue".to_string(),
      },
      &test_world(),
      TeamColor::Blue,
      GameCommand::Running,
    )
  }

  #[cfg(feature = "viewer")]
  #[test]
  fn loading_an_entry_does_not_run_it() {
    let mut controller = DirectDehumanizedController::new(1);
    load_pass_to(&mut controller).unwrap();

    assert_eq!(controller.state, DeveloperRunState::Loaded);
    assert!(controller.run.is_none());

    controller.act(
      &test_world(),
      &WorldConfig::division_b(),
      TeamColor::Blue,
      GameCommand::Running,
    );

    // Stepping the match must not instantiate anything on its own.
    assert!(controller.run.is_none());
    assert_eq!(controller.state, DeveloperRunState::Loaded);
  }

  #[cfg(feature = "viewer")]
  #[test]
  fn starting_requires_a_loaded_entry() {
    let mut controller = DirectDehumanizedController::new(1);
    assert!(start(&mut controller).is_err());
    assert_eq!(controller.state, DeveloperRunState::Idle);
  }

  #[cfg(feature = "viewer")]
  #[test]
  fn started_entry_drives_directly_and_keeps_one_instance() {
    let mut controller = DirectDehumanizedController::new(1);
    load_pass_to(&mut controller).unwrap();
    start(&mut controller).unwrap();
    assert_eq!(controller.state, DeveloperRunState::Running);

    let instance = controller
      .run
      .as_ref()
      .map(|run| std::ptr::from_ref(&*run.state))
      .unwrap();

    let output = controller.act(
      &test_world(),
      &WorldConfig::division_b(),
      TeamColor::Blue,
      GameCommand::Running,
    );

    assert_eq!(output.len(), 1);
    assert!(matches!(
      output[0].move_command,
      Some(MoveCommand::GlobalVelocity { .. })
    ));
    // The same instance survives the tick; only its world was updated.
    assert_eq!(
      controller
        .run
        .as_ref()
        .map(|run| std::ptr::from_ref(&*run.state)),
      Some(instance)
    );
  }

  #[cfg(feature = "viewer")]
  #[test]
  fn a_second_start_is_refused_while_the_entry_runs() {
    let mut controller = DirectDehumanizedController::new(1);
    load_pass_to(&mut controller).unwrap();
    start(&mut controller).unwrap();

    assert!(start(&mut controller).is_err());
    assert_eq!(controller.state, DeveloperRunState::Running);
  }

  #[cfg(feature = "viewer")]
  #[test]
  fn stopping_keeps_the_selection_and_allows_a_restart() {
    let mut controller = DirectDehumanizedController::new(1);
    load_pass_to(&mut controller).unwrap();
    start(&mut controller).unwrap();

    controller
      .apply_developer_request(
        &simhark::viewer::DeveloperRequest::Stop {
          target: "blue".to_string(),
        },
        &test_world(),
        TeamColor::Blue,
        GameCommand::Running,
      )
      .unwrap();

    assert_eq!(controller.state, DeveloperRunState::Stopped);
    assert!(controller.run.is_none());
    assert!(controller.loaded.is_some());
    assert!(start(&mut controller).is_ok());
  }

  #[cfg(feature = "viewer")]
  #[test]
  fn disabling_hands_the_side_back_to_the_match_ai() {
    let mut controller = DirectDehumanizedController::new(1);
    load_pass_to(&mut controller).unwrap();
    start(&mut controller).unwrap();

    controller
      .apply_developer_request(
        &simhark::viewer::DeveloperRequest::Disable {
          target: "blue".to_string(),
        },
        &test_world(),
        TeamColor::Blue,
        GameCommand::Running,
      )
      .unwrap();

    assert_eq!(controller.state, DeveloperRunState::Idle);
    assert!(controller.run.is_none());
    assert!(controller.loaded.is_none());
  }

  #[cfg(feature = "viewer")]
  #[test]
  fn an_entry_that_cannot_be_built_fails_the_run_instead_of_panicking() {
    let mut controller = DirectDehumanizedController::new(1);
    // Robot 3 does not exist in the test world, and `PassTo` unwraps it.
    controller
      .apply_developer_request(
        &simhark::viewer::DeveloperRequest::Load {
          target: "blue".to_string(),
          kind: "skill".to_string(),
          entry: "Pass To".to_string(),
          config: serde_json::json!({}),
          params: serde_json::json!({ "passer": "R3", "receiver": "R0" }),
        },
        &test_world(),
        TeamColor::Blue,
        GameCommand::Running,
      )
      .unwrap();

    assert!(start(&mut controller).is_err());
    assert_eq!(controller.state, DeveloperRunState::Failed);
    assert!(controller.run.is_none());
  }

  /// A registry entry whose whole point is to remember how often it was
  /// stepped. Re-instantiating it per tick would keep `steps` at one forever.
  #[derive(Debug)]
  struct CountingFactory;

  struct CountingSkill<'a> {
    steps: usize,
    commands: &'a MutCommands,
  }

  impl<'a> Skill<'a> for CountingSkill<'a> {
    fn step(&mut self) -> bool {
      self.steps += 1;
      self
        .commands
        .i(core_dump::types::ai_types::Robot::R0)
        .set_speed(self.steps as u32);
      self.steps >= 3
    }
  }

  impl dehumanized::skill::SkillFactory for CountingFactory {
    fn name(&self) -> &'static str {
      "Counting"
    }

    fn def(&self) -> dehumanized::skill::SkillDefinition {
      dehumanized::skill::SkillDefinition {
        name: "Counting",
        config: dehumanized::skill::schema::ObjectSchema {
          name: "Counting",
          fields: &[],
        },
        params: dehumanized::skill::schema::ObjectSchema {
          name: "Counting",
          fields: &[],
        },
      }
    }

    fn default_config(&self) -> Value {
      serde_json::json!({})
    }

    fn default_params(&self) -> Value {
      serde_json::json!({})
    }

    fn validate(
      &self,
      _config: &Value,
      _params: &Value,
    ) -> Result<(), Box<dyn std::error::Error>> {
      Ok(())
    }

    fn instantiate<'a>(
      &'_ self,
      _config: Value,
      _params: Value,
      _state: &'a MutGameState,
      cmds: &'a MutCommands,
    ) -> Result<Box<dyn Skill<'a> + 'a>, Box<dyn std::error::Error>> {
      Ok(Box::new(CountingSkill {
        steps: 0,
        commands: cmds,
      }))
    }
  }

  fn load_counting(controller: &mut DirectDehumanizedController) {
    controller.run = None;
    controller.loaded = Some(LoadedEntry {
      kind: "skill".to_string(),
      name: "Counting".to_string(),
      factory: &CountingFactory,
      config: serde_json::json!({}),
      params: serde_json::json!({}),
    });
    controller.set_state(DeveloperRunState::Loaded, "loaded".to_string());
  }

  #[cfg(feature = "viewer")]
  #[test]
  fn a_run_keeps_its_state_across_ticks_and_finishes_once() {
    let mut controller = DirectDehumanizedController::new(1);
    load_counting(&mut controller);
    start(&mut controller).unwrap();

    for _ in 0..2 {
      controller.act(
        &test_world(),
        &WorldConfig::division_b(),
        TeamColor::Blue,
        GameCommand::Running,
      );
      assert_eq!(controller.state, DeveloperRunState::Running);
    }

    // Third step returns "finished"; the run is not started again afterwards.
    controller.act(
      &test_world(),
      &WorldConfig::division_b(),
      TeamColor::Blue,
      GameCommand::Running,
    );
    assert_eq!(controller.state, DeveloperRunState::Finished);

    let steps_at_finish = controller
      .run
      .as_ref()
      .map(|run| run.commands.commands()[0].and_then(|cmd| cmd.pos).and_then(|pos| pos.speed));
    assert_eq!(steps_at_finish, Some(Some(3)));

    controller.act(
      &test_world(),
      &WorldConfig::division_b(),
      TeamColor::Blue,
      GameCommand::Running,
    );
    assert_eq!(controller.state, DeveloperRunState::Finished);
    assert_eq!(
      controller
        .run
        .as_ref()
        .map(|run| run.commands.commands()[0].and_then(|cmd| cmd.pos).and_then(|pos| pos.speed)),
      Some(Some(3)),
      "a finished run must not be stepped again"
    );
  }

  #[cfg(feature = "viewer")]
  #[test]
  fn unknown_entries_are_rejected_at_load_time() {
    let mut controller = DirectDehumanizedController::new(1);
    let result = controller.apply_developer_request(
      &simhark::viewer::DeveloperRequest::Load {
        target: "blue".to_string(),
        kind: "skill".to_string(),
        entry: "Not A Skill".to_string(),
        config: serde_json::json!({}),
        params: serde_json::json!({}),
      },
      &test_world(),
      TeamColor::Blue,
      GameCommand::Running,
    );

    assert!(result.is_err());
    assert_eq!(controller.state, DeveloperRunState::Idle);
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
