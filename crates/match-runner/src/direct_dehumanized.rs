//! Direct simhark binding for the 2027 Dehumanized AI.
//!
//! This intentionally bypasses CrashPilot, the robot protocol, tf_jetsoncode,
//! and ORCA. Motion requests are converted directly to simulator velocities,
//! with measured request progress fed back through the next `RobotState`.

use crate::controller::{Controller, GameCommand};
use core_dump::types::ai_types::{
  self as ai, Ai, Commands, DriveStatus, GameStage, HeadingMode, HeadingStatus, Id, Kicker,
  MotionCommand, MotionStatus, RobotCommand as AiRobotCommand, Target,
};
use core_dump::vec::types::Vec2;
use dehumanized::Dehumanized;
use dehumanized::mut_command::MutCommands;
use dehumanized::mut_state::MutGameState;
use dehumanized::play::{Play, PlayFactory};
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
const HEADING_GAIN_PER_S: f64 = 6.0;
const DEFAULT_ANGULAR_RAD_S: f64 = 20.0;
const CHIP_ANGLE_DEG: f64 = 45.0;
const GRAVITY_M_S2: f64 = 9.81;
const ESTIMATED_ROLL_DECEL_M_S2: f64 = 0.7;

pub struct DirectDehumanizedController {
  ai: Dehumanized,
  num_robots: u8,
  motion_feedback: [MotionFeedback; 16],
  /// The entry the operator selected. Loading never instantiates anything, so
  /// editing parameters in the AI Lab cannot disturb a run in progress.
  loaded: Option<LoadedEntry>,
  /// The one live instance, created by `start` and stepped until it finishes.
  run: Option<EntryRun>,
  state: DeveloperRunState,
  message: String,
}

#[derive(Debug, Clone, Copy, Default)]
struct MotionFeedback {
  request: Option<MotionCommand>,
  initial_drive_dist_mm: f32,
  initial_heading_diff_deg: f32,
}

#[derive(Clone)]
struct LoadedEntry {
  kind: String,
  name: String,
  factory: EntryFactory,
  config: Value,
  params: Value,
}

#[derive(Clone, Copy)]
enum EntryFactory {
  Skill(&'static dyn SkillFactory),
  Play(&'static dyn PlayFactory),
}

impl EntryFactory {
  fn validate(&self, config: &Value, params: &Value) -> Result<(), Box<dyn std::error::Error>> {
    match self {
      Self::Skill(factory) => factory.validate(config, params),
      Self::Play(factory) => factory.validate(config, params),
    }
  }

  fn instantiate<'a>(
    &self,
    config: Value,
    params: Value,
    state: &'a MutGameState,
    commands: &'a MutCommands,
  ) -> Result<EntryInstance<'a>, Box<dyn std::error::Error>> {
    match self {
      Self::Skill(factory) => factory
        .instantiate(config, params, state, commands)
        .map(EntryInstance::Skill),
      Self::Play(factory) => factory
        .instantiate(config, params, state, commands)
        .map(EntryInstance::Play),
    }
  }
}

enum EntryInstance<'a> {
  Skill(Box<dyn Skill<'a> + 'a>),
  Play(Box<dyn Play<'a> + 'a>),
}

impl EntryInstance<'_> {
  fn step(&mut self) -> bool {
    match self {
      Self::Skill(skill) => skill.step(),
      Self::Play(play) => play.step(),
    }
  }
}

/// One live registry instance together with the buffers it borrows.
///
/// Registry entries are stateful — an async skill parks on a waiter and
/// resumes on the next step — so the instance has to outlive the tick that
/// created it. It holds references into `state` and `commands`, which are
/// therefore boxed (stable addresses) and updated in place each tick rather
/// than rebuilt.
///
/// Field order is drop order: `entry` is declared first so it is destroyed
/// before the buffers whose addresses it holds.
struct EntryRun {
  entry: EntryInstance<'static>,
  state: Box<MutGameState>,
  commands: Box<MutCommands>,
  finished: bool,
}

impl EntryRun {
  fn start(entry: &LoadedEntry, num_robots: u8, game_state: ai::GameState) -> Result<Self, String> {
    let state = Box::new(MutGameState::new(game_state));
    let commands = Box::new(MutCommands::new(initial_commands(num_robots)));

    // SAFETY: both buffers are boxed and owned by the returned `EntryRun`, so
    // their addresses stay valid and stable for the whole life of `entry`.
    // `entry` is dropped first, and the extended references never escape this
    // struct.
    let state_ref: &'static MutGameState = unsafe { &*(&raw const *state) };
    let commands_ref: &'static MutCommands = unsafe { &*(&raw const *commands) };

    // Entries are free to assume their configuration is sane and panic when it
    // is not (`PassTo` unwraps its passer), so instantiation is guarded too.
    let instance = catch_unwind(AssertUnwindSafe(|| {
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
      entry: instance,
      state,
      commands,
      finished: false,
    })
  }

  /// Steps the instance once against this tick's world.
  fn step(&mut self, game_state: ai::GameState) -> Result<Commands, String> {
    self.state.update(game_state);

    // Commands are persistent intent for the lifetime of a skill. In
    // particular, movement helpers set their target/face once and then park
    // on a waiter across subsequent ticks. Clearing the buffer here made a
    // one-shot `set_face` disappear on the very next tick.
    let finished = catch_unwind(AssertUnwindSafe(|| self.entry.step()))
      .map_err(|_| "the entry panicked while stepping".to_string())?;
    self.finished = finished;

    let mut output = self.commands.commands();
    mark_motion_requests_raw(&mut output);
    Ok(output)
  }
}

fn mark_motion_requests_raw(commands: &mut Commands) {
  for motion in commands
    .iter_mut()
    .flatten()
    .filter_map(|command| command.motion.as_mut())
  {
    // Direct registry execution intentionally bypasses the normal AI and
    // collision planner. The simhark binding below turns these targets
    // straight into simulator drive velocities.
    motion.obstacles.raw_movement = true;
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
      motion_feedback: [MotionFeedback::default(); 16],
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
    let factory = match kind {
      "skill" | "skills" => SKILLS
        .iter()
        .find(|(name, _)| *name == entry)
        .map(|(_, factory)| EntryFactory::Skill(*factory)),
      "play" | "plays" => PLAYS
        .iter()
        .find(|(name, _)| *name == entry)
        .map(|(_, factory)| EntryFactory::Play(*factory)),
      other => return Err(format!("unknown registry kind: {other}")),
    };
    let Some(factory) = factory else {
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
      factory,
      config: config.clone(),
      params: params.clone(),
    });
    Ok(self.set_state(
      DeveloperRunState::Loaded,
      format!("{entry} is ready to start"),
    ))
  }

  fn start(
    &mut self,
    state: &WorldState,
    color: TeamColor,
    gc: GameCommand,
  ) -> Result<String, String> {
    let Some(entry) = self.loaded.clone() else {
      return Err("load an entry before starting it".to_string());
    };
    if self.run.as_ref().is_some_and(|run| !run.finished) {
      return Err(format!("{} is already running", entry.name));
    }

    let game_state = world_state_to_dehumanized(state, color, gc);
    match EntryRun::start(&entry, self.num_robots, game_state) {
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

    let stepped = self
      .run
      .as_mut()
      .expect("a live run was just observed")
      .step(game_state);

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
      self.motion_feedback = [MotionFeedback::default(); 16];
      return stopped_commands(self.num_robots);
    }

    let game_state =
      world_state_to_dehumanized_with_feedback(state, color, gc, &self.motion_feedback);
    let commands = self.drive(game_state);
    let sim_commands = commands_to_sim(
      commands,
      state,
      cfg,
      color,
      self.num_robots,
      matches!(gc, GameCommand::Stop),
    );
    update_motion_feedback(&mut self.motion_feedback, &commands, state, color);
    sim_commands
  }
}

fn world_state_to_dehumanized(
  state: &WorldState,
  color: TeamColor,
  gc: GameCommand,
) -> ai::GameState {
  world_state_to_dehumanized_with_feedback(state, color, gc, &[MotionFeedback::default(); 16])
}

fn world_state_to_dehumanized_with_feedback(
  state: &WorldState,
  color: TeamColor,
  gc: GameCommand,
  motion_feedback: &[MotionFeedback; 16],
) -> ai::GameState {
  let (own, opp) = match color {
    TeamColor::Blue => (&state.blue_robots, &state.yellow_robots),
    TeamColor::Yellow => (&state.yellow_robots, &state.blue_robots),
  };

  ai::GameState {
    world: ai::World {
      own_robots: robots_to_dehumanized(own, true, Some((state, color, motion_feedback))),
      opp_robots: robots_to_dehumanized(opp, false, None),
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

fn robots_to_dehumanized(
  robots: &[SimRobotState],
  own_team: bool,
  feedback: Option<(&WorldState, TeamColor, &[MotionFeedback; 16])>,
) -> ai::Robots {
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
      has_ball: robot.infrared,
      motion_status: feedback
        .and_then(|(state, color, feedback)| {
          feedback.get(robot.id).map(|item| (state, color, item))
        })
        .map(|(state, color, feedback)| motion_status(robot, state, color, feedback))
        .unwrap_or_else(no_motion_status),
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

fn update_motion_feedback(
  feedback: &mut [MotionFeedback; 16],
  commands: &Commands,
  state: &WorldState,
  color: TeamColor,
) {
  let own_robots = team_robots(state, color);

  for (id, item) in feedback.iter_mut().enumerate() {
    let request = commands
      .get(id)
      .copied()
      .flatten()
      .and_then(|command| command.motion);
    let robot = own_robots
      .iter()
      .find(|robot| robot.id == id && robot.is_on);

    if !same_motion_goal(item.request, request) {
      item.initial_drive_dist_mm = robot
        .zip(request)
        .map(|(robot, request)| drive_distance_mm(robot, request.target))
        .unwrap_or(0.0);
      item.initial_heading_diff_deg = robot
        .zip(request)
        .and_then(|(robot, request)| heading_error_deg(robot, state, color, request.heading))
        .map(f32::abs)
        .unwrap_or(0.0);
    }

    item.request = request;
  }
}

fn motion_status(
  robot: &SimRobotState,
  state: &WorldState,
  color: TeamColor,
  feedback: &MotionFeedback,
) -> MotionStatus {
  let Some(request) = feedback.request else {
    return no_motion_status();
  };

  let drive = match request.target {
    Target::Hold => DriveStatus::Reached,
    Target::Pos(_) => {
      let dist = drive_distance_mm(robot, request.target);
      let speed_mm_s = (robot.vx.hypot(robot.vy) * MM_PER_M) as f32;
      let velocity_reached = request.tolerance.vel <= 0.0 || speed_mm_s <= request.tolerance.vel;
      if dist <= request.tolerance.pos_mm.max(0.0) && velocity_reached {
        DriveStatus::Reached
      } else {
        DriveStatus::Running {
          eta: dist / request_max_speed_mm_s(request).max(f32::EPSILON),
          progress: normalized_progress(
            feedback.initial_drive_dist_mm,
            dist,
            request.tolerance.pos_mm.max(0.0),
          ),
          dist,
        }
      }
    }
    // Directional and velocity requests have no finite destination. They stay
    // running until the caller replaces the request.
    Target::Heading { .. } | Target::Velocity { .. } => DriveStatus::Running {
      eta: f32::INFINITY,
      progress: 0.0,
      dist: f32::INFINITY,
    },
  };

  let heading = match request.heading {
    HeadingMode::Free => HeadingStatus::Reached,
    HeadingMode::Fixed(_) => {
      let diff = heading_error_deg(robot, state, color, request.heading).unwrap_or(0.0);
      if diff.abs() <= request.tolerance.heading_deg.max(0.0) {
        HeadingStatus::Reached
      } else {
        HeadingStatus::Running {
          eta: diff.abs() / request_max_angular_deg_s(request).max(f32::EPSILON),
          progress: normalized_progress(
            feedback.initial_heading_diff_deg,
            diff.abs(),
            request.tolerance.heading_deg.max(0.0),
          ),
          diff,
        }
      }
    }
    HeadingMode::FaceTarget(_) | HeadingMode::FaceBall | HeadingMode::FaceRobot(_, _) => {
      let Some(diff) = heading_error_deg(robot, state, color, request.heading) else {
        return MotionStatus {
          drive,
          // The requested dynamic target is currently unavailable (for
          // example, a robot disappeared). Keep the request incomplete.
          heading: HeadingStatus::TrackingBehind(180.0),
          id: request.id,
        };
      };
      if diff.abs() <= request.tolerance.heading_deg.max(0.0) {
        HeadingStatus::Tracking
      } else {
        HeadingStatus::TrackingBehind(diff)
      }
    }
  };

  MotionStatus {
    drive,
    heading,
    id: request.id,
  }
}

fn no_motion_status() -> MotionStatus {
  MotionStatus {
    drive: DriveStatus::Reached,
    heading: HeadingStatus::Reached,
    id: Id::ZERO,
  }
}

fn normalized_progress(initial: f32, remaining: f32, tolerance: f32) -> f32 {
  let range = initial - tolerance;
  if range <= f32::EPSILON {
    return if remaining <= tolerance { 1.0 } else { 0.0 };
  }
  ((initial - remaining) / range).clamp(0.0, 1.0)
}

fn drive_distance_mm(robot: &SimRobotState, target: Target) -> f32 {
  match target {
    Target::Pos(target) => {
      let current = meters_to_mm(robot.x, robot.y);
      (target.x - current.x).hypot(target.y - current.y)
    }
    Target::Hold | Target::Heading { .. } | Target::Velocity { .. } => 0.0,
  }
}

fn heading_error_deg(
  robot: &SimRobotState,
  state: &WorldState,
  color: TeamColor,
  mode: HeadingMode,
) -> Option<f32> {
  let target = match mode {
    HeadingMode::Fixed(heading) => heading,
    HeadingMode::FaceTarget(target) => angle_to_target_deg(robot, target)?,
    HeadingMode::FaceBall => angle_to_target_deg(robot, meters_to_mm(state.ball.x, state.ball.y))?,
    HeadingMode::FaceRobot(id, team) => {
      let target_color = match team {
        ai::Team::Own => color,
        ai::Team::Opp => opposite_team(color),
      };
      let target = team_robots(state, target_color)
        .iter()
        .find(|candidate| candidate.id == id as usize && candidate.is_on)?;
      angle_to_target_deg(robot, meters_to_mm(target.x, target.y))?
    }
    HeadingMode::Free => return None,
  };
  let current = robot.orientation.to_degrees().rem_euclid(360.0) as f32;
  Some(wrap_degrees(target - current))
}

fn angle_to_target_deg(robot: &SimRobotState, target: Vec2<f32>) -> Option<f32> {
  let current = meters_to_mm(robot.x, robot.y);
  let dx = target.x - current.x;
  let dy = target.y - current.y;
  if dx.abs() <= f32::EPSILON && dy.abs() <= f32::EPSILON {
    None
  } else {
    Some(dy.atan2(dx).to_degrees().rem_euclid(360.0))
  }
}

fn request_max_speed_mm_s(request: MotionCommand) -> f32 {
  request
    .limits
    .map(|limits| limits.v_max.max(0.0))
    .unwrap_or(DEFAULT_SPEED_MM_S as f32)
}

fn request_max_angular_deg_s(request: MotionCommand) -> f32 {
  request
    .limits
    .map(|limits| limits.omega_max.max(0.0))
    .unwrap_or(DEFAULT_ANGULAR_RAD_S.to_degrees() as f32)
}

fn same_motion_goal(a: Option<MotionCommand>, b: Option<MotionCommand>) -> bool {
  match (a, b) {
    (None, None) => true,
    (Some(a), Some(b)) => {
      a.id == b.id && same_target(a.target, b.target) && same_heading(a.heading, b.heading)
    }
    _ => false,
  }
}

fn same_target(a: Target, b: Target) -> bool {
  match (a, b) {
    (Target::Hold, Target::Hold) => true,
    (Target::Pos(a), Target::Pos(b)) => same_f32(a.x, b.x) && same_f32(a.y, b.y),
    (Target::Heading { heading: a }, Target::Heading { heading: b }) => same_f32(a, b),
    (Target::Velocity { vx: ax, vy: ay }, Target::Velocity { vx: bx, vy: by }) => {
      same_f32(ax, bx) && same_f32(ay, by)
    }
    _ => false,
  }
}

fn same_heading(a: HeadingMode, b: HeadingMode) -> bool {
  match (a, b) {
    (HeadingMode::Free, HeadingMode::Free) | (HeadingMode::FaceBall, HeadingMode::FaceBall) => true,
    (HeadingMode::Fixed(a), HeadingMode::Fixed(b)) => same_f32(a, b),
    (HeadingMode::FaceTarget(a), HeadingMode::FaceTarget(b)) => {
      same_f32(a.x, b.x) && same_f32(a.y, b.y)
    }
    (HeadingMode::FaceRobot(ar, at), HeadingMode::FaceRobot(br, bt)) => {
      ar as u8 == br as u8 && at == bt
    }
    _ => false,
  }
}

fn same_f32(a: f32, b: f32) -> bool {
  a.to_bits() == b.to_bits()
}

fn team_robots(state: &WorldState, color: TeamColor) -> &[SimRobotState] {
  match color {
    TeamColor::Blue => &state.blue_robots,
    TeamColor::Yellow => &state.yellow_robots,
  }
}

fn opposite_team(color: TeamColor) -> TeamColor {
  match color {
    TeamColor::Blue => TeamColor::Yellow,
    TeamColor::Yellow => TeamColor::Blue,
  }
}

fn wrap_degrees(angle: f32) -> f32 {
  let wrapped = (angle + 180.0).rem_euclid(360.0) - 180.0;
  if wrapped <= -180.0 {
    wrapped + 360.0
  } else {
    wrapped
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
        (Some(robot), Some(command)) => {
          command_to_sim(id, robot, command, state, color, robot_cfg, stop)
        }
        _ => stopped_command(id),
      }
    })
    .collect()
}

fn command_to_sim(
  id: usize,
  robot: &SimRobotState,
  command: AiRobotCommand,
  state: &WorldState,
  color: TeamColor,
  cfg: &simhark::RobotConfig,
  stop: bool,
) -> SimRobotCommand {
  let motion = command.motion.unwrap_or_default();
  let stop_limit = if stop {
    dehumanized::MAX_STOP_VELOCITY as f64
  } else {
    f64::INFINITY
  };
  let requested_speed_limit_mm_s = motion.limits.map(|limits| limits.v_max.max(0.0) as f64);
  let default_speed_limit_mm_s = match motion.target {
    // An explicit velocity is already its own speed request; without Limits,
    // only the physical robot and referee-state caps should constrain it.
    Target::Velocity { .. } => f64::INFINITY,
    Target::Pos(_) | Target::Heading { .. } | Target::Hold => DEFAULT_SPEED_MM_S,
  };
  let max_speed_m_s = requested_speed_limit_mm_s
    .unwrap_or(default_speed_limit_mm_s)
    .min(stop_limit)
    .min(cfg.vel_absolute_max * MM_PER_M)
    / MM_PER_M;

  let (mut vx, mut vy) = match motion.target {
    Target::Pos(target) => {
      let dx = target.x as f64 / MM_PER_M - robot.x;
      let dy = target.y as f64 / MM_PER_M - robot.y;
      let distance = dx.hypot(dy);
      let tolerance_m = motion.tolerance.pos_mm.max(0.0) as f64 / MM_PER_M;
      let speed = if distance <= tolerance_m {
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
    Target::Heading { heading } => {
      let heading = (heading as f64).to_radians();
      (heading.cos() * max_speed_m_s, heading.sin() * max_speed_m_s)
    }
    Target::Velocity { vx, vy } => {
      let vx = vx as f64 / MM_PER_M;
      let vy = vy as f64 / MM_PER_M;
      let speed = vx.hypot(vy);
      if speed > max_speed_m_s && speed > 0.0 {
        (vx / speed * max_speed_m_s, vy / speed * max_speed_m_s)
      } else {
        (vx, vy)
      }
    }
    Target::Hold => (0.0, 0.0),
  };

  if !vx.is_finite() || !vy.is_finite() {
    (vx, vy) = (0.0, 0.0);
  }

  let angular = heading_error_deg(robot, state, color, motion.heading)
    .map(|error_deg| {
      let max_angular = motion
        .limits
        .map(|limits| (limits.omega_max.max(0.0) as f64).to_radians())
        .unwrap_or(cfg.vel_angular_max)
        .min(cfg.vel_angular_max);
      let error = (error_deg as f64).to_radians();
      if error_deg.abs() <= motion.tolerance.heading_deg.max(0.0) {
        0.0
      } else {
        (error * HEADING_GAIN_PER_S).clamp(-max_angular, max_angular)
      }
    })
    .unwrap_or(0.0);

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
  use core_dump::types::ai_types::{Limits, RobotCommand, Tolerance};
  use simhark::BallState;
  use simhark::state::KickStatus;

  #[test]
  fn world_conversion_uses_mm_degrees_and_team_order() {
    let mut state = test_world();
    state.yellow_robots[0].infrared = true;
    let converted = world_state_to_dehumanized(&state, TeamColor::Yellow, GameCommand::Running);

    let own = converted.world.own_robots[1].unwrap();
    let opp = converted.world.opp_robots[0].unwrap();
    assert_eq!(own.pos, Vec2::new(-2_000.0, 500.0));
    assert_eq!(own.vel, Vec2::new(100.0, -200.0));
    assert!((own.heading - 90.0).abs() < 1.0e-4);
    assert!(!own.is_goalie);
    assert!(own.has_ball);
    assert!(!opp.has_ball);
    assert!(matches!(own.motion_status.drive, DriveStatus::Reached));
    assert!(matches!(own.motion_status.heading, HeadingStatus::Reached));
    assert_eq!(own.motion_status.id, Id::ZERO);
    assert_eq!(opp.pos, Vec2::new(1_000.0, -250.0));
    assert_eq!(converted.world.ball.pos, Vec2::new(250.0, -100.0));
  }

  #[test]
  fn position_command_becomes_direct_global_velocity() {
    let state = test_world();
    let mut commands = Commands::default();
    commands[0] = Some(RobotCommand {
      dribbler: true,
      motion: Some(MotionCommand {
        target: Target::Pos(Vec2::new(2_000.0, -250.0)),
        heading: HeadingMode::Fixed(90.0),
        limits: Some(Limits {
          v_max: 500.0,
          a_max: 0.0,
          omega_max: 360.0,
          alpha_max: 0.0,
          jerk: 0.0,
        }),
        ..MotionCommand::default()
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
  fn direct_execution_marks_motion_requests_raw_without_creating_one() {
    let mut commands = Commands::default();
    commands[0] = Some(RobotCommand {
      motion: Some(MotionCommand::default()),
      ..RobotCommand::default()
    });
    commands[1] = Some(RobotCommand {
      dribbler: true,
      ..RobotCommand::default()
    });

    mark_motion_requests_raw(&mut commands);

    assert!(
      commands[0]
        .and_then(|command| command.motion)
        .is_some_and(|motion| motion.obstacles.raw_movement)
    );
    assert!(commands[1].is_some_and(|command| command.motion.is_none()));
  }

  #[test]
  fn motion_status_tracks_the_previous_position_and_heading_request() {
    let mut state = test_world();
    let request = MotionCommand {
      target: Target::Pos(Vec2::new(2_000.0, -250.0)),
      heading: HeadingMode::Fixed(90.0),
      tolerance: Tolerance {
        pos_mm: 10.0,
        heading_deg: 2.0,
        vel: 0.0,
      },
      ..MotionCommand::default()
    };
    let mut commands = Commands::default();
    commands[0] = Some(RobotCommand {
      motion: Some(request),
      ..RobotCommand::default()
    });
    let mut feedback = [MotionFeedback::default(); 16];
    update_motion_feedback(&mut feedback, &commands, &state, TeamColor::Blue);

    let initial = world_state_to_dehumanized_with_feedback(
      &state,
      TeamColor::Blue,
      GameCommand::Running,
      &feedback,
    )
    .world
    .own_robots[0]
      .unwrap()
      .motion_status;
    assert!(matches!(
      initial.drive,
      DriveStatus::Running { progress, dist, .. }
        if progress.abs() < f32::EPSILON && (dist - 1_000.0).abs() < 1.0e-3
    ));
    assert!(matches!(
      initial.heading,
      HeadingStatus::Running { progress, diff, .. }
        if progress.abs() < f32::EPSILON && (diff - 90.0).abs() < 1.0e-3
    ));
    assert_eq!(initial.id, request.id);

    state.blue_robots[0].x = 1.5;
    state.blue_robots[0].orientation = 45.0_f64.to_radians();
    let halfway = world_state_to_dehumanized_with_feedback(
      &state,
      TeamColor::Blue,
      GameCommand::Running,
      &feedback,
    )
    .world
    .own_robots[0]
      .unwrap()
      .motion_status;
    assert!(matches!(
      halfway.drive,
      DriveStatus::Running { progress, dist, .. }
        if (progress - 500.0 / 990.0).abs() < 1.0e-3 && (dist - 500.0).abs() < 1.0e-3
    ));
    assert!(matches!(
      halfway.heading,
      HeadingStatus::Running { progress, diff, .. }
        if (progress - 45.0 / 88.0).abs() < 1.0e-3 && (diff - 45.0).abs() < 1.0e-3
    ));
    assert_eq!(halfway.id, request.id);

    state.blue_robots[0].x = 2.0;
    state.blue_robots[0].orientation = 90.0_f64.to_radians();
    let reached = world_state_to_dehumanized_with_feedback(
      &state,
      TeamColor::Blue,
      GameCommand::Running,
      &feedback,
    )
    .world
    .own_robots[0]
      .unwrap()
      .motion_status;
    assert!(matches!(reached.drive, DriveStatus::Reached));
    assert!(matches!(reached.heading, HeadingStatus::Reached));
    assert_eq!(reached.id, request.id);
  }

  #[test]
  fn a_new_request_id_resets_progress_for_an_identical_goal() {
    let mut state = test_world();
    let first = MotionCommand {
      target: Target::Pos(Vec2::new(2_000.0, -250.0)),
      ..MotionCommand::default()
    };
    let mut commands = Commands::default();
    commands[0] = Some(RobotCommand {
      motion: Some(first),
      ..RobotCommand::default()
    });
    let mut feedback = [MotionFeedback::default(); 16];
    update_motion_feedback(&mut feedback, &commands, &state, TeamColor::Blue);

    state.blue_robots[0].x = 1.5;
    let second = MotionCommand {
      target: first.target,
      ..MotionCommand::default()
    };
    assert_ne!(first.id, second.id);
    commands[0].as_mut().unwrap().motion = Some(second);
    update_motion_feedback(&mut feedback, &commands, &state, TeamColor::Blue);

    let status = motion_status(&state.blue_robots[0], &state, TeamColor::Blue, &feedback[0]);
    assert!(matches!(
      status.drive,
      DriveStatus::Running { progress, dist, .. }
        if progress.abs() < f32::EPSILON && (dist - 500.0).abs() < 1.0e-3
    ));
    assert_eq!(status.id, second.id);
  }

  #[test]
  fn face_ball_reports_tracking_error_and_drives_rotation() {
    let state = test_world();
    let mut commands = Commands::default();
    commands[0] = Some(RobotCommand {
      motion: Some(MotionCommand {
        heading: HeadingMode::FaceBall,
        ..MotionCommand::default()
      }),
      ..RobotCommand::default()
    });
    let mut feedback = [MotionFeedback::default(); 16];
    update_motion_feedback(&mut feedback, &commands, &state, TeamColor::Blue);
    let status = motion_status(&state.blue_robots[0], &state, TeamColor::Blue, &feedback[0]);
    assert!(matches!(status.heading, HeadingStatus::TrackingBehind(diff) if diff.abs() > 2.0));
    assert_eq!(status.id, commands[0].unwrap().motion.unwrap().id);

    let output = commands_to_sim(
      commands,
      &state,
      &WorldConfig::division_b(),
      TeamColor::Blue,
      1,
      false,
    );
    assert!(matches!(
      output[0].move_command,
      Some(MoveCommand::GlobalVelocity { vx: 0.0, vy: 0.0, angular }) if angular.abs() > 0.0
    ));
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
  fn explicit_velocity_is_not_capped_by_the_position_default() {
    let mut commands = Commands::default();
    commands[0] = Some(RobotCommand {
      motion: Some(MotionCommand {
        target: Target::Velocity {
          vx: 4_500.0,
          vy: 0.0,
        },
        ..MotionCommand::default()
      }),
      ..RobotCommand::default()
    });

    let state = test_world();
    let output = commands_to_sim(
      commands,
      &state,
      &WorldConfig::division_b(),
      TeamColor::Blue,
      1,
      false,
    );
    assert!(matches!(
      output[0].move_command,
      Some(MoveCommand::GlobalVelocity { vx, vy: 0.0, .. }) if (vx - 4.5).abs() < 1.0e-9
    ));

    let stopped = commands_to_sim(
      commands,
      &state,
      &WorldConfig::division_b(),
      TeamColor::Blue,
      1,
      true,
    );
    assert!(matches!(
      stopped[0].move_command,
      Some(MoveCommand::GlobalVelocity { vx, vy: 0.0, .. }) if (vx - 1.5).abs() < 1.0e-9
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
  fn a_registered_play_can_be_loaded_started_and_stepped() {
    let (name, factory) = PLAYS.first().expect("the play registry is not empty");
    let mut controller = DirectDehumanizedController::new(1);

    controller
      .load(
        "play",
        name,
        &factory.default_config(),
        &factory.default_params(),
      )
      .unwrap();
    start(&mut controller).unwrap();
    assert!(matches!(
      controller.run.as_ref().map(|run| &run.entry),
      Some(EntryInstance::Play(_))
    ));

    let output = controller.act(
      &test_world(),
      &WorldConfig::division_b(),
      TeamColor::Blue,
      GameCommand::Running,
    );
    assert_eq!(output.len(), 1);
    assert_ne!(controller.state, DeveloperRunState::Failed);
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

  struct PersistentFaceSkill<'a> {
    steps: usize,
    commands: &'a MutCommands,
  }

  impl<'a> Skill<'a> for PersistentFaceSkill<'a> {
    fn step(&mut self) -> bool {
      self.steps += 1;
      if self.steps == 1 {
        self
          .commands
          .i(core_dump::types::ai_types::Robot::R0)
          .replace(RobotCommand {
            motion: Some(MotionCommand {
              heading: HeadingMode::Fixed(180.0),
              ..MotionCommand::default()
            }),
            ..RobotCommand::default()
          });
      }
      false
    }
  }

  #[test]
  fn face_command_persists_while_skill_waits() {
    let state = Box::new(MutGameState::new(world_state_to_dehumanized(
      &test_world(),
      TeamColor::Blue,
      GameCommand::Running,
    )));
    let commands = Box::new(MutCommands::new(initial_commands(1)));
    let mut run = EntryRun {
      entry: EntryInstance::Skill(Box::new(PersistentFaceSkill {
        steps: 0,
        commands: unsafe { &*(&raw const *commands) },
      })),
      state,
      commands,
      finished: false,
    };

    let game_state =
      world_state_to_dehumanized(&test_world(), TeamColor::Blue, GameCommand::Running);
    assert!(matches!(
      run.step(game_state).unwrap()[0]
        .unwrap()
        .motion
        .unwrap()
        .heading,
      HeadingMode::Fixed(180.0)
    ));
    assert!(matches!(
      run.step(game_state).unwrap()[0]
        .unwrap()
        .motion
        .unwrap()
        .heading,
      HeadingMode::Fixed(180.0)
    ));
  }

  impl<'a> Skill<'a> for CountingSkill<'a> {
    fn step(&mut self) -> bool {
      self.steps += 1;
      self
        .commands
        .i(core_dump::types::ai_types::Robot::R0)
        .replace(RobotCommand {
          motion: Some(MotionCommand {
            priority: self.steps as u8,
            ..MotionCommand::default()
          }),
          ..RobotCommand::default()
        });
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

    fn validate(&self, _config: &Value, _params: &Value) -> Result<(), Box<dyn std::error::Error>> {
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
      factory: EntryFactory::Skill(&CountingFactory),
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

    let steps_at_finish = controller.run.as_ref().map(|run| {
      run.commands.commands()[0]
        .and_then(|cmd| cmd.motion)
        .map(|motion| motion.priority)
    });
    assert_eq!(steps_at_finish, Some(Some(3)));

    controller.act(
      &test_world(),
      &WorldConfig::division_b(),
      TeamColor::Blue,
      GameCommand::Running,
    );
    assert_eq!(controller.state, DeveloperRunState::Finished);
    assert_eq!(
      controller.run.as_ref().map(|run| {
        run.commands.commands()[0]
          .and_then(|cmd| cmd.motion)
          .map(|motion| motion.priority)
      }),
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
