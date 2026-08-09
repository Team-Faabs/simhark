//! Optional browser-based debug viewer.

use std::collections::HashMap;
use std::io::{Error, Result};
use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicIsize, AtomicUsize, Ordering};
use std::thread;
use std::time::Duration;

use parking_lot::Mutex;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use webinterface_assets::embedded_assets;
use webinterface_core::{InterfaceConfig, InterfaceHost, InterfaceHostGuard, SystemPublisher};
use webinterface_protocol as interface_protocol;
use webinterface_protocol::{
  Capability as InterfaceCapability, CommandStatus as InterfaceCommandStatus,
  SessionId as InterfaceSessionId, SessionKind as InterfaceSessionKind,
  SessionLifecycle as InterfaceSessionLifecycle, SimharkCommand, SystemCommand,
  SystemDescriptor as InterfaceSystemDescriptor, SystemKind as InterfaceSystemKind,
};

use crate::command::{TeleportBall, TeleportRobot};
use crate::config::{FieldConfig, WorldConfig};
use crate::engine::SimulationEngine;
#[cfg(feature = "viewer-debug")]
use crate::replay::{ReplayDebugOverlay, ReplayDebugSnapshot};
use crate::replay::{ReplayEvent, ReplayFrame, RobotInputInfo, robot_inputs_for_frame};
use crate::state::{TeamColor, WorldState};

#[derive(Debug, Clone, Copy)]
pub struct ViewerConfig {
  pub host: IpAddr,
  pub http_port: u16,
}

impl Default for ViewerConfig {
  fn default() -> Self {
    Self {
      host: IpAddr::V4(Ipv4Addr::UNSPECIFIED),
      http_port: std::env::var("SIMHARK_VIEWER_PORT")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(8315),
    }
  }
}

impl ViewerConfig {
  pub fn websocket_port(self) -> u16 {
    self.http_port.saturating_add(1)
  }

  pub fn http_url(self) -> String {
    match self.host {
      IpAddr::V4(ip) if ip.is_unspecified() => format!("http://127.0.0.1:{}", self.http_port),
      IpAddr::V6(ip) if ip.is_unspecified() => format!("http://[::1]:{}", self.http_port),
      host => format!("http://{}:{}", host, self.http_port),
    }
  }
}

/// Game-state info pushed to the viewer alongside world state.
///
/// Mirrors the shape of `referris::RefereeSnapshot` without taking on the
/// dependency: callers may translate from any source (referris, SSL
/// game-controller, sumatra default referee, etc.) into this struct.
#[derive(Debug, Clone, Default, Serialize)]
pub struct GameStateInfo {
  /// Current command, normalised to UPPER_SNAKE_CASE (e.g. `FORCE_START`).
  pub command: String,
  /// Monotonic command counter from the game controller.
  pub command_counter: u32,
  /// Optional stage label, e.g. `NORMAL_FIRST_HALF`.
  pub stage: Option<String>,
  pub blue_name: Option<String>,
  pub yellow_name: Option<String>,
}

/// Schema-driven developer console published alongside the live viewer frame.
#[derive(Debug, Clone, Serialize)]
pub struct DeveloperSnapshot {
  pub schema: Value,
  pub results: HashMap<String, DeveloperResult>,
  /// Lifecycle of every target that has one, keyed by target id. This is what
  /// lets the AI Lab show whether an entry is merely loaded or actually
  /// running, instead of inferring it from the last acknowledgement.
  pub runs: HashMap<String, DeveloperRun>,
}

/// Latest direct-invocation result for one developer target.
#[derive(Debug, Clone, Serialize)]
pub struct DeveloperResult {
  pub target: String,
  pub entry: Option<String>,
  pub ok: bool,
  pub message: String,
}

/// Lifecycle of one AI Lab target.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct DeveloperRun {
  pub target: String,
  pub kind: Option<String>,
  pub entry: Option<String>,
  pub state: interface_protocol::DeveloperRunState,
  pub message: String,
  /// Simulation frame the run was started on, for correlating with the
  /// timeline. `None` until something is started.
  pub started_frame: Option<u64>,
  pub finished_frame: Option<u64>,
}

/// A schema-renderer action sent from the frontend to the simulation binding.
///
/// Loading and starting are separate actions on purpose: registry entries keep
/// state once instantiated, so editing a parameter must not restart a run.
#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "action", rename_all = "snake_case")]
pub enum DeveloperRequest {
  Load {
    target: String,
    kind: String,
    entry: String,
    config: Value,
    params: Value,
  },
  Start {
    target: String,
  },
  Stop {
    target: String,
  },
  Disable {
    target: String,
  },
  SwitchAi {
    target: String,
    ai: String,
  },
  SetBallRecovery {
    target: String,
    enabled: bool,
  },
}

impl DeveloperRequest {
  pub fn target(&self) -> &str {
    match self {
      Self::Load { target, .. }
      | Self::Start { target }
      | Self::Stop { target }
      | Self::Disable { target }
      | Self::SwitchAi { target, .. }
      | Self::SetBallRecovery { target, .. } => target,
    }
  }
}

#[derive(Debug, Clone, Serialize)]
pub struct BallTrajectory {
  pub world_id: usize,
  pub points: Vec<BallTrajectoryPoint>,
  pub stop_time: f64,
}

#[derive(Debug, Clone, Copy, Serialize)]
pub struct BallTrajectoryPoint {
  pub x: f64,
  pub y: f64,
}

#[cfg(feature = "viewer-debug")]
#[derive(Debug, Clone, Default, Serialize)]
pub struct ViewerDebugSnapshot {
  pub world_id: usize,
  pub strategy: Option<String>,
  pub robots: Vec<RobotDebugInfo>,
  pub overlays: Vec<DebugOverlay>,
}

#[cfg(feature = "viewer-debug")]
#[derive(Debug, Clone, Serialize)]
pub struct RobotDebugInfo {
  pub team: TeamColor,
  pub id: usize,
  /// Short task label, e.g. `Attacker`, `Receiver`, `Marking`.
  pub task: String,
  /// CSS color string used by the browser viewer, preferably `#RRGGBB`.
  pub color: String,
  pub message: Option<String>,
}

#[cfg(feature = "viewer-debug")]
#[derive(Debug, Clone, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum DebugOverlay {
  HoloRobot(DebugHoloRobot),
  KickLine(DebugKickLine),
}

#[cfg(feature = "viewer-debug")]
#[derive(Debug, Clone, Serialize)]
pub struct DebugHoloRobot {
  pub team: TeamColor,
  pub id: usize,
  pub x: f64,
  pub y: f64,
  pub orientation: Option<f64>,
  pub color: String,
  pub label: Option<String>,
}

#[cfg(feature = "viewer-debug")]
#[derive(Debug, Clone, Serialize)]
pub struct DebugKickLine {
  pub team: TeamColor,
  pub id: usize,
  pub from_x: f64,
  pub from_y: f64,
  pub angle: f64,
  pub color: String,
  pub label: Option<String>,
}

#[derive(Default)]
struct GoalTracker {
  blue: u32,
  yellow: u32,
  last_blue: bool,
  last_yellow: bool,
}

impl GoalTracker {
  fn observe(&mut self, state: &WorldState) {
    if state.goal_blue && !self.last_blue {
      self.blue += 1;
    }
    if state.goal_yellow && !self.last_yellow {
      self.yellow += 1;
    }
    self.last_blue = state.goal_blue;
    self.last_yellow = state.goal_yellow;
  }
}

#[derive(Serialize)]
struct GoalSummary {
  blue: u32,
  yellow: u32,
  blue_active: bool,
  yellow_active: bool,
}

/// Shared run-state handle used when an application opts in to web-driven
/// start/stop/restart via [`ViewerServer::enable_web_control`].
#[derive(Default)]
struct WebControlState {
  enabled: AtomicBool,
  running: AtomicBool,
  restart_requested: AtomicBool,
  stop_requested: AtomicBool,
  speed_percent: AtomicUsize,
  frame_step_requested: AtomicIsize,
  frame_skip_requested: AtomicIsize,
  frame_seek_requested: AtomicIsize,
}

/// A robot reposition requested by dragging it in the browser viewer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RobotMoveRequest {
  pub world_id: usize,
  pub team: TeamColor,
  pub id: usize,
  pub x: f64,
  pub y: f64,
}

type RobotMoveKey = (usize, TeamColor, usize);

/// A robot orientation requested by dragging its heading handle in the browser viewer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RobotRotateRequest {
  pub world_id: usize,
  pub team: TeamColor,
  pub id: usize,
  pub orientation: f64,
}

/// A robot activation change requested from the browser viewer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RobotPresenceRequest {
  pub world_id: usize,
  pub team: TeamColor,
  pub id: usize,
  pub present: bool,
}

/// A ball reposition requested by dragging it in the browser viewer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BallMoveRequest {
  pub world_id: usize,
  pub x: f64,
  pub y: f64,
}

#[derive(Serialize)]
struct ControlSnapshot {
  web_enabled: bool,
  running: bool,
  speed: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct ReplayStatus {
  pub enabled: bool,
  pub frame_index: usize,
  pub frame_count: usize,
  pub base_speed: f64,
}

#[derive(Default)]
struct GameStateTracker {
  info: Option<GameStateInfo>,
  counts: HashMap<String, u32>,
  last_command: Option<String>,
  last_counter: Option<u32>,
}

impl GameStateTracker {
  fn update(&mut self, info: GameStateInfo) {
    let command_changed = self.last_command.as_deref() != Some(info.command.as_str());
    let counter_advanced = self
      .last_counter
      .is_none_or(|previous| info.command_counter != previous);
    if command_changed || counter_advanced {
      *self.counts.entry(info.command.clone()).or_insert(0) += 1;
    }
    self.last_command = Some(info.command.clone());
    self.last_counter = Some(info.command_counter);
    self.info = Some(info);
  }

  fn snapshot(&self) -> Option<PublishedGameState<'_>> {
    self.info.as_ref().map(|info| PublishedGameState {
      command: &info.command,
      command_counter: info.command_counter,
      stage: info.stage.as_deref(),
      blue_name: info.blue_name.as_deref(),
      yellow_name: info.yellow_name.as_deref(),
      state_counts: &self.counts,
    })
  }
}

#[derive(Serialize)]
struct PublishedGameState<'a> {
  command: &'a str,
  command_counter: u32,
  stage: Option<&'a str>,
  blue_name: Option<&'a str>,
  yellow_name: Option<&'a str>,
  state_counts: &'a HashMap<String, u32>,
}

pub struct ViewerServer {
  world_count: usize,
  field: FieldConfig,
  robot_radius: f64,
  ball_radius: f64,
  ball_friction: f64,
  gravity: f64,
  selected_world: Arc<AtomicUsize>,
  selected_worlds: Arc<Mutex<Vec<usize>>>,
  latest_frame: Arc<Mutex<Option<String>>>,
  game_state: Arc<Mutex<GameStateTracker>>,
  test_suite: Arc<Mutex<Option<Value>>>,
  developer: Arc<Mutex<Option<DeveloperSnapshot>>>,
  developer_requests: Arc<Mutex<Vec<DeveloperRequest>>>,
  goal_tracker: Arc<Mutex<GoalTracker>>,
  #[cfg(feature = "viewer-debug")]
  debug: Arc<Mutex<HashMap<usize, ViewerDebugSnapshot>>>,
  control: Arc<WebControlState>,
  robot_move_requests: Arc<Mutex<HashMap<RobotMoveKey, RobotMoveRequest>>>,
  robot_rotate_requests: Arc<Mutex<HashMap<RobotMoveKey, RobotRotateRequest>>>,
  robot_presence_requests: Arc<Mutex<HashMap<RobotMoveKey, RobotPresenceRequest>>>,
  ball_move_requests: Arc<Mutex<HashMap<usize, BallMoveRequest>>>,
  interface_publisher: SystemPublisher,
  interface_session: InterfaceSessionId,
  interface_handle: webinterface_core::InterfaceHandle,
  interface_session_terminal: AtomicBool,
  command_thread: Option<thread::JoinHandle<()>>,
  _interface_host: InterfaceHostGuard,
}

#[derive(Serialize)]
struct ViewerFrame<'a> {
  world_count: usize,
  selected_world: usize,
  selected_worlds: Vec<usize>,
  field: &'a FieldConfig,
  robot_radius: f64,
  ball_radius: f64,
  ball_trajectory: Option<BallTrajectory>,
  state: &'a WorldState,
  states: Vec<&'a WorldState>,
  game_state: Option<PublishedGameState<'a>>,
  test_suite: Option<Value>,
  developer: Option<DeveloperSnapshot>,
  goals: GoalSummary,
  control: ControlSnapshot,
  replay: ReplayStatus,
  events: Vec<ReplayEvent>,
  robot_inputs: Vec<RobotInputInfo>,
  #[cfg(feature = "viewer-debug")]
  debug: Option<ViewerDebugSnapshot>,
}

fn simhark_capabilities() -> Vec<InterfaceCapability> {
  [
    ("simhark.lifecycle", true),
    ("simhark.speed", true),
    ("simhark.replay", true),
    ("simhark.world_selection", true),
    ("simhark.move_robot", true),
    ("simhark.rotate_robot", true),
    ("simhark.robot_presence", true),
    ("simhark.move_ball", true),
    ("simhark.developer", true),
    ("simhark.world_state", false),
  ]
  .into_iter()
  .map(|(id, mutable)| InterfaceCapability {
    id: id.into(),
    mutable,
    description: id.replace('.', " "),
  })
  .collect()
}

#[allow(clippy::too_many_arguments)]
fn run_interface_commands(
  commands: &mut tokio::sync::mpsc::UnboundedReceiver<webinterface_core::QueuedSystemCommand>,
  selected_world: Arc<AtomicUsize>,
  selected_worlds: Arc<Mutex<Vec<usize>>>,
  control: Arc<WebControlState>,
  robot_move_requests: Arc<Mutex<HashMap<RobotMoveKey, RobotMoveRequest>>>,
  robot_rotate_requests: Arc<Mutex<HashMap<RobotMoveKey, RobotRotateRequest>>>,
  robot_presence_requests: Arc<Mutex<HashMap<RobotMoveKey, RobotPresenceRequest>>>,
  ball_move_requests: Arc<Mutex<HashMap<usize, BallMoveRequest>>>,
  developer_requests: Arc<Mutex<Vec<DeveloperRequest>>>,
  publisher: SystemPublisher,
) {
  while let Some(queued) = commands.blocking_recv() {
    let result = match queued.command {
      SystemCommand::Simhark(command) => apply_simhark_command(
        command,
        &selected_world,
        &selected_worlds,
        &control,
        &robot_move_requests,
        &robot_rotate_requests,
        &robot_presence_requests,
        &ball_move_requests,
      ),
      SystemCommand::Developer(command) => {
        let request = match command {
          interface_protocol::DeveloperCommand::Load {
            target,
            kind,
            entry,
            config,
            params,
          } => DeveloperRequest::Load {
            target,
            kind,
            entry,
            config,
            params,
          },
          interface_protocol::DeveloperCommand::Start { target } => {
            DeveloperRequest::Start { target }
          }
          interface_protocol::DeveloperCommand::Stop { target } => {
            DeveloperRequest::Stop { target }
          }
          interface_protocol::DeveloperCommand::Disable { target } => {
            DeveloperRequest::Disable { target }
          }
          interface_protocol::DeveloperCommand::SwitchAi { target, ai } => {
            DeveloperRequest::SwitchAi { target, ai }
          }
          interface_protocol::DeveloperCommand::SetBallRecovery { target, enabled } => {
            DeveloperRequest::SetBallRecovery { target, enabled }
          }
        };
        queue_developer_request(&developer_requests, request)
      }
      _ => Err("unsupported command for simhark".to_string()),
    };
    match result {
      Ok(()) => publisher.acknowledge(
        queued.browser_command_id,
        InterfaceCommandStatus::Applied,
        "applied by simhark",
      ),
      Err(error) => publisher.acknowledge(
        queued.browser_command_id,
        InterfaceCommandStatus::Rejected,
        error,
      ),
    }
  }
}

/// Number of developer requests kept before new ones are refused.
///
/// The simulation loop drains this queue every iteration, including while
/// paused, so it only fills up if the loop is wedged. Refusing loudly beats
/// growing without bound or dropping the operator's `start`.
const DEVELOPER_REQUEST_QUEUE_LIMIT: usize = 64;

fn queue_developer_request(
  queue: &Mutex<Vec<DeveloperRequest>>,
  request: DeveloperRequest,
) -> std::result::Result<(), String> {
  let mut queue = queue.lock();
  if queue.len() >= DEVELOPER_REQUEST_QUEUE_LIMIT {
    return Err("the simulation is not draining developer requests".into());
  }
  queue.push(request);
  Ok(())
}

#[allow(clippy::too_many_arguments)]
fn apply_simhark_command(
  command: SimharkCommand,
  selected_world: &AtomicUsize,
  selected_worlds: &Mutex<Vec<usize>>,
  control: &WebControlState,
  robot_move_requests: &Mutex<HashMap<RobotMoveKey, RobotMoveRequest>>,
  robot_rotate_requests: &Mutex<HashMap<RobotMoveKey, RobotRotateRequest>>,
  robot_presence_requests: &Mutex<HashMap<RobotMoveKey, RobotPresenceRequest>>,
  ball_move_requests: &Mutex<HashMap<usize, BallMoveRequest>>,
) -> std::result::Result<(), String> {
  match command {
    SimharkCommand::Start => control.running.store(true, Ordering::Relaxed),
    SimharkCommand::Pause => control.running.store(false, Ordering::Relaxed),
    SimharkCommand::Stop | SimharkCommand::CancelSession => {
      control.stop_requested.store(true, Ordering::Relaxed);
      control.running.store(false, Ordering::Relaxed);
    }
    SimharkCommand::Restart => {
      control.restart_requested.store(true, Ordering::Relaxed);
      control.running.store(true, Ordering::Relaxed);
    }
    SimharkCommand::Step { frames } => {
      control
        .frame_step_requested
        .fetch_add(frames.clamp(-10_000, 10_000) as isize, Ordering::Relaxed);
      control.running.store(false, Ordering::Relaxed);
    }
    SimharkCommand::Skip { frames } => {
      control
        .frame_skip_requested
        .fetch_add(frames.clamp(-100_000, 100_000) as isize, Ordering::Relaxed);
    }
    SimharkCommand::Seek { frame } => {
      control.frame_seek_requested.store(
        isize::try_from(frame).unwrap_or(isize::MAX),
        Ordering::Relaxed,
      );
    }
    SimharkCommand::SetSpeed { multiplier } => {
      if !multiplier.is_finite() {
        return Err("speed must be finite".into());
      }
      control.speed_percent.store(
        (multiplier.clamp(0.05, 4.0) * 100.0).round() as usize,
        Ordering::Relaxed,
      );
    }
    SimharkCommand::SelectWorlds { world_ids } => {
      let worlds = world_ids
        .into_iter()
        .map(|world| world as usize)
        .collect::<Vec<_>>();
      if let Some(first) = worlds.first() {
        selected_world.store(*first, Ordering::Relaxed);
      }
      *selected_worlds.lock() = worlds;
    }
    SimharkCommand::MoveRobot {
      world_id,
      team,
      id,
      position,
    } => {
      let team = protocol_team_to_simhark(team);
      let request = RobotMoveRequest {
        world_id: world_id as usize,
        team,
        id: id as usize,
        x: position.x_mm.0 / 1000.0,
        y: position.y_mm.0 / 1000.0,
      };
      robot_move_requests
        .lock()
        .insert((request.world_id, team, request.id), request);
    }
    SimharkCommand::RotateRobot {
      world_id,
      team,
      id,
      orientation_rad,
    } => {
      let team = protocol_team_to_simhark(team);
      let request = RobotRotateRequest {
        world_id: world_id as usize,
        team,
        id: id as usize,
        orientation: orientation_rad.0,
      };
      robot_rotate_requests
        .lock()
        .insert((request.world_id, team, request.id), request);
    }
    SimharkCommand::SetRobotPresent {
      world_id,
      team,
      id,
      present,
    } => {
      let team = protocol_team_to_simhark(team);
      let request = RobotPresenceRequest {
        world_id: world_id as usize,
        team,
        id: id as usize,
        present,
      };
      robot_presence_requests
        .lock()
        .insert((request.world_id, team, request.id), request);
    }
    SimharkCommand::MoveBall { world_id, position } => {
      ball_move_requests.lock().insert(
        world_id as usize,
        BallMoveRequest {
          world_id: world_id as usize,
          x: position.x_mm.0 / 1000.0,
          y: position.y_mm.0 / 1000.0,
        },
      );
    }
    SimharkCommand::LaunchMatch(_) => {
      return Err("match launch is handled by match-runner".into());
    }
  }
  Ok(())
}

fn protocol_team_to_simhark(team: interface_protocol::TeamColor) -> TeamColor {
  match team {
    interface_protocol::TeamColor::Blue => TeamColor::Blue,
    interface_protocol::TeamColor::Yellow => TeamColor::Yellow,
  }
}

/// Referee state as the canonical protocol wants it.
///
/// simhark has no game controller of its own — whatever drives the match
/// (referris, the SSL GC, the sumatra default referee) pushes a
/// [`GameStateInfo`] in, and this is the one place that becomes protocol.
fn canonical_referee(
  game_state: Option<&PublishedGameState<'_>>,
  score: interface_protocol::Score,
) -> Option<interface_protocol::RefereeState> {
  let game_state = game_state?;
  Some(interface_protocol::RefereeState {
    stage: game_state.stage.map(str::to_string),
    command: game_state.command.to_string(),
    next_command: None,
    command_counter: game_state.command_counter,
    stage_time_left_ns: None,
    action_time_remaining_ns: None,
    designated_position: None,
    blue_team_on_positive_half: None,
    score,
  })
}

/// Builds the canonical snapshot for one publish.
///
/// `score` is the accumulated match score: `WorldState::goal_blue` is a
/// one-frame "a goal is happening right now" flag, not a running score, so it
/// cannot stand in for one.
fn canonical_simhark_snapshot(
  states: &[WorldState],
  field: &FieldConfig,
  properties: std::collections::BTreeMap<String, Value>,
  score: interface_protocol::Score,
  referee: Option<interface_protocol::RefereeState>,
  #[cfg(feature = "viewer-debug")] debug: &HashMap<usize, ViewerDebugSnapshot>,
) -> interface_protocol::SystemSnapshot {
  #[cfg(feature = "viewer-debug")]
  let debug_items = canonical_debug_items(debug, states, field);
  #[cfg(feature = "viewer-debug")]
  let debug_layers = simhark_debug_layers();
  #[cfg(not(feature = "viewer-debug"))]
  let (debug_items, debug_layers) = (Vec::new(), Vec::new());

  let field = interface_protocol::FieldGeometry {
    field_length_mm: interface_protocol::Millimetres(field.field_length * 1000.0),
    field_width_mm: interface_protocol::Millimetres(field.field_width * 1000.0),
    goal_width_mm: interface_protocol::Millimetres(field.goal_width * 1000.0),
    goal_depth_mm: interface_protocol::Millimetres(field.goal_depth * 1000.0),
    boundary_width_mm: interface_protocol::Millimetres(
      field.margin_touch_line.max(field.margin_goal_line) * 1000.0,
    ),
    penalty_area_depth_mm: interface_protocol::Millimetres(field.penalty_depth * 1000.0),
    penalty_area_width_mm: interface_protocol::Millimetres(field.penalty_width * 1000.0),
    center_circle_radius_mm: interface_protocol::Millimetres(field.field_center_radius * 1000.0),
    line_thickness_mm: interface_protocol::Millimetres(field.field_line_width * 1000.0),
    max_robot_radius_mm: interface_protocol::Millimetres(90.0),
    ball_radius_mm: interface_protocol::Millimetres(21.5),
  };
  let worlds = states
    .iter()
    .map(|state| {
      let mut robots = state
        .blue_robots
        .iter()
        .chain(state.yellow_robots.iter())
        .map(|robot| interface_protocol::RobotState {
          id: robot.id as u32,
          team: match robot.team {
            TeamColor::Blue => interface_protocol::TeamColor::Blue,
            TeamColor::Yellow => interface_protocol::TeamColor::Yellow,
          },
          position: interface_protocol::PointMm {
            x_mm: interface_protocol::Millimetres(robot.x * 1000.0),
            y_mm: interface_protocol::Millimetres(robot.y * 1000.0),
          },
          orientation_rad: interface_protocol::Radians(robot.orientation),
          velocity: interface_protocol::VelocityMmPerS {
            x_mm_per_s: interface_protocol::MillimetresPerSecond(robot.vx * 1000.0),
            y_mm_per_s: interface_protocol::MillimetresPerSecond(robot.vy * 1000.0),
            z_mm_per_s: interface_protocol::MillimetresPerSecond(robot.vz * 1000.0),
          },
          angular_velocity_rad_per_s: interface_protocol::RadiansPerSecond(robot.v_angular),
          visible: robot.is_on,
          visibility: Some(if robot.is_on { 1.0 } else { 0.0 }),
          infrared: Some(robot.infrared),
          dribbler_enabled: Some(robot.dribbler_on),
          // The controller's own label for what this robot is doing, so the
          // task table reads the same as the field overlay.
          #[cfg(feature = "viewer-debug")]
          task: debug
            .get(&state.world_id)
            .and_then(|snapshot| {
              snapshot
                .robots
                .iter()
                .find(|info| info.team == robot.team && info.id == robot.id)
            })
            .map(|info| info.task.clone()),
          #[cfg(not(feature = "viewer-debug"))]
          task: None,
        })
        .collect::<Vec<_>>();
      robots.sort_by_key(|robot| {
        (
          matches!(robot.team, interface_protocol::TeamColor::Yellow),
          robot.id,
        )
      });
      interface_protocol::WorldState {
        world_id: state.world_id as u32,
        frame: state.frame,
        simulation_time_ns: interface_protocol::TimestampNs(
          (state.sim_time.max(0.0) * 1_000_000_000.0) as u64,
        ),
        field: field.clone(),
        ball: Some(interface_protocol::BallState {
          position: interface_protocol::Point3Mm {
            x_mm: interface_protocol::Millimetres(state.ball.x * 1000.0),
            y_mm: interface_protocol::Millimetres(state.ball.y * 1000.0),
            z_mm: interface_protocol::Millimetres(state.ball.z * 1000.0),
          },
          velocity: interface_protocol::VelocityMmPerS {
            x_mm_per_s: interface_protocol::MillimetresPerSecond(state.ball.vx * 1000.0),
            y_mm_per_s: interface_protocol::MillimetresPerSecond(state.ball.vy * 1000.0),
            z_mm_per_s: interface_protocol::MillimetresPerSecond(state.ball.vz * 1000.0),
          },
          visibility: Some(1.0),
          source: Some("simhark".into()),
        }),
        robots,
        referee: referee.clone(),
        score: score.clone(),
        events: Vec::new(),
      }
    })
    .collect();
  interface_protocol::SystemSnapshot {
    worlds,
    debug_layers,
    debug_items,
    properties,
  }
}

/// The stable layer tree the strategy overlays hang off.
///
/// Layer ids never change with content, which is what lets the Layers panel
/// keep a visibility or solo choice across frames and across restarts. Team
/// leaves exist so a whole side can be hidden — the plan's team filter.
#[cfg(feature = "viewer-debug")]
fn simhark_debug_layers() -> Vec<interface_protocol::DebugLayer> {
  fn layer(id: &str, parent: Option<&str>, label: &str) -> interface_protocol::DebugLayer {
    interface_protocol::DebugLayer {
      id: id.to_string(),
      parent_id: parent.map(str::to_string),
      label: label.to_string(),
      default_visible: true,
    }
  }

  let mut layers = vec![
    layer("simhark.debug", None, "Strategy debug"),
    layer("simhark.debug.strategy", Some("simhark.debug"), "Strategy"),
    layer("simhark.debug.tasks", Some("simhark.debug"), "Robot tasks"),
    layer("simhark.debug.holograms", Some("simhark.debug"), "Holograms"),
    layer("simhark.debug.kicks", Some("simhark.debug"), "Kick lines"),
  ];
  for group in ["tasks", "holograms", "kicks"] {
    let parent = format!("simhark.debug.{group}");
    for (team, label) in [("blue", "Blue"), ("yellow", "Yellow")] {
      layers.push(layer(
        &format!("{parent}.{team}"),
        Some(&parent),
        label,
      ));
    }
  }
  layers
}

#[cfg(feature = "viewer-debug")]
fn team_layer(group: &str, team: TeamColor) -> String {
  match team {
    TeamColor::Blue => format!("simhark.debug.{group}.blue"),
    TeamColor::Yellow => format!("simhark.debug.{group}.yellow"),
  }
}

#[cfg(feature = "viewer-debug")]
fn debug_style(color: &str, label: Option<String>) -> interface_protocol::DebugStyle {
  interface_protocol::DebugStyle {
    stroke: Some(color.to_string()),
    fill: None,
    stroke_width_mm: Some(interface_protocol::Millimetres(20.0)),
    opacity: 1.0,
    label,
    tooltip: None,
  }
}

#[cfg(feature = "viewer-debug")]
fn point_mm(x: f64, y: f64) -> interface_protocol::PointMm {
  interface_protocol::PointMm {
    x_mm: interface_protocol::Millimetres(x * 1000.0),
    y_mm: interface_protocol::Millimetres(y * 1000.0),
  }
}

/// Converts the controllers' debug snapshots into canonical primitives.
///
/// The renderer only knows about protocol primitives, so anything a controller
/// publishes has to become one here rather than growing a simhark-shaped
/// special case in the canvas.
#[cfg(feature = "viewer-debug")]
fn canonical_debug_items(
  debug: &HashMap<usize, ViewerDebugSnapshot>,
  states: &[WorldState],
  field: &FieldConfig,
) -> Vec<interface_protocol::DebugItem> {
  let half_x = field.field_length * 0.5;
  let half_y = field.field_width * 0.5;
  let mut items = Vec::new();

  for state in states {
    let Some(snapshot) = debug.get(&state.world_id) else {
      continue;
    };
    let world_id = Some(state.world_id as u32);

    if let Some(strategy) = &snapshot.strategy {
      items.push(interface_protocol::DebugItem {
        id: format!("simhark.debug.strategy.{}", state.world_id),
        layer_id: "simhark.debug.strategy".into(),
        world_id,
        robot_id: None,
        primitive: interface_protocol::DebugPrimitive::Text {
          at: point_mm(0.0, half_y - 0.2),
          text: strategy.clone(),
          style: debug_style("#e6edf3", None),
        },
        scalar: None,
        unit: None,
        range: None,
      });
    }

    for robot in &snapshot.robots {
      // A task label is only meaningful where its robot is, so a robot that
      // has left the world drops its label rather than drawing it at the
      // origin.
      let Some(position) = robot_position(state, robot.team, robot.id) else {
        continue;
      };
      let label = match &robot.message {
        Some(message) => format!("{} · {message}", robot.task),
        None => robot.task.clone(),
      };
      items.push(interface_protocol::DebugItem {
        id: format!(
          "simhark.debug.task.{}.{:?}.{}",
          state.world_id, robot.team, robot.id
        ),
        layer_id: team_layer("tasks", robot.team),
        world_id,
        robot_id: Some(robot.id as u32),
        primitive: interface_protocol::DebugPrimitive::Text {
          at: point_mm(position.0, position.1 + 0.16),
          text: label,
          style: debug_style(&robot.color, None),
        },
        scalar: None,
        unit: None,
        range: None,
      });
    }

    for (index, overlay) in snapshot.overlays.iter().enumerate() {
      match overlay {
        DebugOverlay::HoloRobot(holo) => items.push(interface_protocol::DebugItem {
          id: format!("simhark.debug.holo.{}.{index}", state.world_id),
          layer_id: team_layer("holograms", holo.team),
          world_id,
          robot_id: Some(holo.id as u32),
          primitive: interface_protocol::DebugPrimitive::RobotPose {
            at: point_mm(holo.x, holo.y),
            orientation_rad: interface_protocol::Radians(holo.orientation.unwrap_or(0.0)),
            team: match holo.team {
              TeamColor::Blue => interface_protocol::TeamColor::Blue,
              TeamColor::Yellow => interface_protocol::TeamColor::Yellow,
            },
            robot_id: Some(holo.id as u32),
            style: debug_style(&holo.color, holo.label.clone()),
          },
          scalar: None,
          unit: None,
          range: None,
        }),
        DebugOverlay::KickLine(kick) => {
          // Draw the shot to where it leaves the field, which is what makes a
          // kick line readable; a fixed-length stub says nothing about aim.
          let (dir_x, dir_y) = (kick.angle.cos(), kick.angle.sin());
          let end = field_boundary_intersection(kick.from_x, kick.from_y, dir_x, dir_y, half_x, half_y)
            .unwrap_or(BallTrajectoryPoint {
              x: kick.from_x + dir_x,
              y: kick.from_y + dir_y,
            });
          items.push(interface_protocol::DebugItem {
            id: format!("simhark.debug.kick.{}.{index}", state.world_id),
            layer_id: team_layer("kicks", kick.team),
            world_id,
            robot_id: Some(kick.id as u32),
            primitive: interface_protocol::DebugPrimitive::Arrow {
              from: point_mm(kick.from_x, kick.from_y),
              to: point_mm(end.x, end.y),
              style: debug_style(&kick.color, kick.label.clone()),
            },
            scalar: None,
            unit: None,
            range: None,
          });
        }
      }
    }
  }

  items
}

#[cfg(feature = "viewer-debug")]
fn robot_position(state: &WorldState, team: TeamColor, id: usize) -> Option<(f64, f64)> {
  let robots = match team {
    TeamColor::Blue => &state.blue_robots,
    TeamColor::Yellow => &state.yellow_robots,
  };
  robots
    .iter()
    .find(|robot| robot.id == id && robot.is_on)
    .map(|robot| (robot.x, robot.y))
}

impl ViewerServer {
  pub fn bind(
    config: ViewerConfig,
    world_count: usize,
    world_config: &WorldConfig,
  ) -> Result<Self> {
    let selected_world = Arc::new(AtomicUsize::new(0));
    let selected_worlds = Arc::new(Mutex::new(vec![0]));
    let latest_frame = Arc::new(Mutex::new(None));
    let game_state = Arc::new(Mutex::new(GameStateTracker::default()));
    let test_suite = Arc::new(Mutex::new(None));
    let developer = Arc::new(Mutex::new(None));
    let developer_requests = Arc::new(Mutex::new(Vec::new()));
    let goal_tracker = Arc::new(Mutex::new(GoalTracker::default()));
    #[cfg(feature = "viewer-debug")]
    let debug = Arc::new(Mutex::new(HashMap::new()));
    let control = Arc::new(WebControlState::default());
    let robot_move_requests = Arc::new(Mutex::new(HashMap::new()));
    let robot_rotate_requests = Arc::new(Mutex::new(HashMap::new()));
    let robot_presence_requests = Arc::new(Mutex::new(HashMap::new()));
    let ball_move_requests = Arc::new(Mutex::new(HashMap::new()));
    control.frame_seek_requested.store(-1, Ordering::Relaxed);
    // When web control is disabled the simulator is considered always
    // running, so callers that don't opt in see the legacy behaviour.
    control.running.store(true, Ordering::Relaxed);
    control.speed_percent.store(100, Ordering::Relaxed);

    let (interface_host, interface_handle) = InterfaceHost::start(InterfaceConfig {
      bind_address: SocketAddr::new(config.host, config.http_port),
      assets: embedded_assets(),
      ..InterfaceConfig::default()
    })
    .map_err(|error| Error::other(error.to_string()))?;
    let session = interface_handle.create_session(
      "simhark live",
      InterfaceSessionKind::Simulation,
      true,
      vec!["simhark".into()],
      world_count as u32,
    );
    interface_handle
      .update_session(session.id, InterfaceSessionLifecycle::Running, None)
      .map_err(|error| Error::other(error.to_string()))?;
    let registered = interface_handle
      .register_system(InterfaceSystemDescriptor {
        id: "simhark".into(),
        label: "simhark".into(),
        kind: InterfaceSystemKind::Simhark,
        generation: 1,
        capabilities: simhark_capabilities(),
      })
      .map_err(|error| Error::other(error.to_string()))?;
    let interface_publisher = registered.publisher;
    let command_thread = {
      let selected_world = Arc::clone(&selected_world);
      let selected_worlds = Arc::clone(&selected_worlds);
      let control_for_ws = Arc::clone(&control);
      let robot_move_requests = Arc::clone(&robot_move_requests);
      let robot_rotate_requests = Arc::clone(&robot_rotate_requests);
      let robot_presence_requests = Arc::clone(&robot_presence_requests);
      let ball_move_requests = Arc::clone(&ball_move_requests);
      let developer_requests = Arc::clone(&developer_requests);
      let publisher = interface_publisher.clone();
      let mut commands = registered.commands;
      thread::spawn(move || {
        run_interface_commands(
          &mut commands,
          selected_world,
          selected_worlds,
          control_for_ws,
          robot_move_requests,
          robot_rotate_requests,
          robot_presence_requests,
          ball_move_requests,
          developer_requests,
          publisher,
        )
      })
    };

    Ok(Self {
      world_count,
      field: world_config.field.clone(),
      robot_radius: world_config.blue_robots.radius,
      ball_radius: world_config.ball.radius,
      ball_friction: world_config.ball.friction,
      gravity: world_config.physics.gravity,
      selected_world,
      selected_worlds,
      latest_frame,
      game_state,
      test_suite,
      developer,
      developer_requests,
      goal_tracker,
      #[cfg(feature = "viewer-debug")]
      debug,
      control,
      robot_move_requests,
      robot_rotate_requests,
      robot_presence_requests,
      ball_move_requests,
      interface_publisher,
      interface_session: session.id,
      interface_handle,
      interface_session_terminal: AtomicBool::new(false),
      command_thread: Some(command_thread),
      _interface_host: interface_host,
    })
  }

  /// Opt in to web-driven start/stop/restart. The simulator starts in the
  /// stopped state; the application is expected to gate stepping on
  /// [`Self::is_running`] and react to [`Self::take_restart_request`].
  pub fn enable_web_control(&self) {
    self.enable_web_control_with_running(false);
  }

  pub fn interface_handle(&self) -> webinterface_core::InterfaceHandle {
    self.interface_handle.clone()
  }

  pub fn interface_session_id(&self) -> InterfaceSessionId {
    self.interface_session
  }

  pub fn start_recording(&self) -> Result<()> {
    self
      .interface_handle
      .start_recording(self.interface_session)
      .map_err(|error| Error::other(error.to_string()))
  }

  pub fn finish_session(
    &self,
    lifecycle: InterfaceSessionLifecycle,
    error: Option<String>,
  ) -> Result<()> {
    self
      .interface_handle
      .update_session(self.interface_session, lifecycle, error)
      .map_err(|error| Error::other(error.to_string()))?;
    self
      .interface_session_terminal
      .store(true, Ordering::Relaxed);
    Ok(())
  }

  /// Opt in to web-driven playback control without stopping an already
  /// running simulation.
  pub fn enable_web_control_running(&self) {
    self.enable_web_control_with_running(true);
  }

  fn enable_web_control_with_running(&self, running: bool) {
    self.control.enabled.store(true, Ordering::Relaxed);
    self.control.running.store(running, Ordering::Relaxed);
    self
      .control
      .restart_requested
      .store(false, Ordering::Relaxed);
    self.control.stop_requested.store(false, Ordering::Relaxed);
    self.control.speed_percent.store(100, Ordering::Relaxed);
  }

  /// True when the simulator should keep stepping. Always true when web
  /// control is disabled.
  pub fn is_running(&self) -> bool {
    self.control.running.load(Ordering::Relaxed)
  }

  /// Returns true once when the web UI has asked for a restart, then resets
  /// the flag. Always false when web control is disabled.
  pub fn take_restart_request(&self) -> bool {
    self
      .control
      .restart_requested
      .swap(false, Ordering::Relaxed)
  }

  pub fn take_stop_request(&self) -> bool {
    self.control.stop_requested.swap(false, Ordering::Relaxed)
  }

  pub fn take_frame_step_request(&self) -> isize {
    self.control.frame_step_requested.swap(0, Ordering::Relaxed)
  }

  pub fn take_frame_skip_request(&self) -> isize {
    self.control.frame_skip_requested.swap(0, Ordering::Relaxed)
  }

  pub fn take_frame_seek_request(&self) -> Option<usize> {
    let value = self
      .control
      .frame_seek_requested
      .swap(-1, Ordering::Relaxed);
    usize::try_from(value).ok()
  }

  pub fn speed(&self) -> f64 {
    self.control.speed_percent.load(Ordering::Relaxed) as f64 / 100.0
  }

  pub fn scaled_sleep(&self, base: Duration) -> Duration {
    let speed = self.speed();
    if speed <= 0.0 {
      base
    } else {
      Duration::from_secs_f64(base.as_secs_f64() / speed)
    }
  }

  /// Apply the latest browser edits for robots and balls to their simulation
  /// worlds. Returns the number of entities that were changed.
  ///
  /// Applications with a viewer should call this once per loop, including
  /// while paused, so dragging remains responsive without advancing physics.
  pub fn apply_robot_move_requests(&self, engine: &mut SimulationEngine) -> usize {
    let requests = std::mem::take(&mut *self.robot_move_requests.lock());
    let rotate_requests = std::mem::take(&mut *self.robot_rotate_requests.lock());
    let presence_requests = std::mem::take(&mut *self.robot_presence_requests.lock());
    let ball_requests = std::mem::take(&mut *self.ball_move_requests.lock());
    let mut applied = 0;
    for request in requests.into_values() {
      let Some(world) = engine.worlds.get_mut(request.world_id) else {
        continue;
      };
      let robot_count = match request.team {
        TeamColor::Blue => world.blue_sims.len(),
        TeamColor::Yellow => world.yellow_sims.len(),
      };
      if request.id >= robot_count {
        continue;
      }
      let robot_radius = match request.team {
        TeamColor::Blue => world.config.blue_robots.radius,
        TeamColor::Yellow => world.config.yellow_robots.radius,
      };
      let x_limit = (world.config.field.field_length * 0.5 - robot_radius).max(0.0);
      let y_limit = (world.config.field.field_width * 0.5 - robot_radius).max(0.0);
      world.teleport_robot(&TeleportRobot {
        id: request.id,
        team: request.team,
        x: Some(request.x.clamp(-x_limit, x_limit)),
        y: Some(request.y.clamp(-y_limit, y_limit)),
        orientation: None,
        vx: Some(0.0),
        vy: Some(0.0),
        v_angular: Some(0.0),
        present: None,
      });
      applied += 1;
    }
    for request in rotate_requests.into_values() {
      let Some(world) = engine.worlds.get_mut(request.world_id) else {
        continue;
      };
      let robot_count = match request.team {
        TeamColor::Blue => world.blue_sims.len(),
        TeamColor::Yellow => world.yellow_sims.len(),
      };
      if request.id >= robot_count {
        continue;
      }
      let orientation = request.orientation.sin().atan2(request.orientation.cos());
      world.teleport_robot(&TeleportRobot {
        id: request.id,
        team: request.team,
        x: None,
        y: None,
        orientation: Some(orientation),
        vx: Some(0.0),
        vy: Some(0.0),
        v_angular: Some(0.0),
        present: None,
      });
      applied += 1;
    }
    for request in presence_requests.into_values() {
      let Some(world) = engine.worlds.get_mut(request.world_id) else {
        continue;
      };
      let sims = match request.team {
        TeamColor::Blue => &world.blue_sims,
        TeamColor::Yellow => &world.yellow_sims,
      };
      let Some(robot) = sims.get(request.id) else {
        continue;
      };
      if robot.is_on == request.present {
        continue;
      }

      let (x, y, orientation) = if request.present {
        let robot_count = sims.len();
        let spacing = world.config.field.field_width / (robot_count as f64 + 1.0);
        let x = match request.team {
          TeamColor::Blue => -world.config.field.field_length / 4.0,
          TeamColor::Yellow => world.config.field.field_length / 4.0,
        };
        let y = -world.config.field.field_width / 2.0 + spacing * (request.id as f64 + 1.0);
        let orientation = match request.team {
          TeamColor::Blue => 0.0,
          TeamColor::Yellow => std::f64::consts::PI,
        };
        (Some(x), Some(y), Some(orientation))
      } else {
        (None, None, None)
      };
      world.teleport_robot(&TeleportRobot {
        id: request.id,
        team: request.team,
        x,
        y,
        orientation,
        vx: Some(0.0),
        vy: Some(0.0),
        v_angular: Some(0.0),
        present: Some(request.present),
      });
      applied += 1;
    }
    for request in ball_requests.into_values() {
      let Some(world) = engine.worlds.get_mut(request.world_id) else {
        continue;
      };
      let ball_radius = world.config.ball.radius;
      let x_limit = (world.config.field.field_length * 0.5 - ball_radius).max(0.0);
      let y_limit = (world.config.field.field_width * 0.5 - ball_radius).max(0.0);
      world.teleport_ball(&TeleportBall {
        x: Some(request.x.clamp(-x_limit, x_limit)),
        y: Some(request.y.clamp(-y_limit, y_limit)),
        z: Some(0.0),
        vx: Some(0.0),
        vy: Some(0.0),
        vz: Some(0.0),
      });
      applied += 1;
    }
    applied
  }

  pub fn selected_world(&self) -> usize {
    self
      .selected_world
      .load(Ordering::Relaxed)
      .min(self.world_count.saturating_sub(1))
  }

  pub fn selected_worlds(&self) -> Vec<usize> {
    selected_worlds_snapshot(&self.selected_worlds, self.world_count)
  }

  pub fn select_world(&self, index: usize) {
    let index = index.min(self.world_count.saturating_sub(1));
    self.selected_world.store(index, Ordering::Relaxed);
    *self.selected_worlds.lock() = vec![index];
  }

  /// Push a new referee snapshot. The viewer accumulates per-command counts
  /// so the UI can show "how many times have we entered each game state".
  pub fn set_game_state(&self, info: GameStateInfo) {
    self.game_state.lock().update(info);
  }

  pub fn set_test_suite<T: Serialize>(&self, suite: T) {
    *self.test_suite.lock() = serde_json::to_value(suite).ok();
  }

  /// Enable the schema-driven developer console for this viewer.
  pub fn set_developer_schema<T: Serialize>(&self, schema: T) {
    let Ok(schema) = serde_json::to_value(schema) else {
      return;
    };
    let mut developer = self.developer.lock();
    let (results, runs) = developer
      .take()
      .map(|snapshot| (snapshot.results, snapshot.runs))
      .unwrap_or_default();
    *developer = Some(DeveloperSnapshot {
      schema,
      results,
      runs,
    });
  }

  /// Drain the queued requests in the order the operator issued them.
  ///
  /// Order matters here: `load` followed by `start` in the same tick has to
  /// stay in that order, and neither may swallow the other.
  pub fn take_developer_requests(&self) -> Vec<DeveloperRequest> {
    std::mem::take(&mut *self.developer_requests.lock())
  }

  pub fn set_developer_result(&self, result: DeveloperResult) {
    if let Some(developer) = self.developer.lock().as_mut() {
      developer.results.insert(result.target.clone(), result);
    }
  }

  /// Publishes the lifecycle of one AI Lab target.
  pub fn set_developer_run(&self, run: DeveloperRun) {
    if let Some(developer) = self.developer.lock().as_mut() {
      developer.runs.insert(run.target.clone(), run);
    }
  }

  /// The lifecycle currently published for `target`, if any.
  pub fn developer_run(&self, target: &str) -> Option<DeveloperRun> {
    self
      .developer
      .lock()
      .as_ref()
      .and_then(|developer| developer.runs.get(target).cloned())
  }

  #[cfg(feature = "viewer-debug")]
  pub fn set_debug_snapshot(&self, snapshot: ViewerDebugSnapshot) {
    self.debug.lock().insert(snapshot.world_id, snapshot);
  }

  #[cfg(feature = "viewer-debug")]
  pub fn clear_debug_snapshot(&self, world_id: usize) {
    self.debug.lock().remove(&world_id);
  }

  #[cfg(feature = "viewer-debug")]
  pub fn set_strategy_debug_message(&self, world_id: usize, message: impl Into<String>) {
    let mut debug = self.debug.lock();
    let snapshot = debug
      .entry(world_id)
      .or_insert_with(|| ViewerDebugSnapshot {
        world_id,
        ..ViewerDebugSnapshot::default()
      });
    snapshot.strategy = Some(message.into());
  }

  #[cfg(feature = "viewer-debug")]
  pub fn clear_strategy_debug_message(&self, world_id: usize) {
    if let Some(snapshot) = self.debug.lock().get_mut(&world_id) {
      snapshot.strategy = None;
    }
  }

  #[cfg(feature = "viewer-debug")]
  pub fn set_robot_debug(&self, world_id: usize, info: RobotDebugInfo) {
    let mut debug = self.debug.lock();
    let snapshot = debug
      .entry(world_id)
      .or_insert_with(|| ViewerDebugSnapshot {
        world_id,
        ..ViewerDebugSnapshot::default()
      });
    if let Some(existing) = snapshot
      .robots
      .iter_mut()
      .find(|robot| robot.team == info.team && robot.id == info.id)
    {
      *existing = info;
    } else {
      snapshot.robots.push(info);
    }
  }

  #[cfg(feature = "viewer-debug")]
  pub fn clear_robot_debug(&self, world_id: usize, team: TeamColor, id: usize) {
    if let Some(snapshot) = self.debug.lock().get_mut(&world_id) {
      snapshot
        .robots
        .retain(|robot| robot.team != team || robot.id != id);
    }
  }

  pub fn publish(&self, state: &WorldState) {
    self.publish_states(std::slice::from_ref(state));
  }

  pub fn publish_states(&self, states: &[WorldState]) {
    let Some(state) = selected_state(states, self.selected_world()) else {
      return;
    };
    let game_state_guard = self.game_state.lock();
    let test_suite = self.test_suite.lock().clone();
    let developer = self.developer.lock().clone();
    let mut goal_guard = self.goal_tracker.lock();
    goal_guard.observe(state);
    let selected_worlds = selected_worlds_snapshot(&self.selected_worlds, self.world_count);
    let selected_states = selected_worlds
      .iter()
      .filter_map(|world| state_by_world_id(states, *world))
      .collect::<Vec<_>>();
    #[cfg(feature = "viewer-debug")]
    let debug = self.debug.lock().get(&state.world_id).cloned();
    let frame = ViewerFrame {
      world_count: self.world_count,
      selected_world: self.selected_world(),
      selected_worlds,
      field: &self.field,
      robot_radius: self.robot_radius,
      ball_radius: self.ball_radius,
      ball_trajectory: predicted_ball_trajectory(
        state,
        &self.field,
        self.ball_radius,
        self.ball_friction,
        self.gravity,
      ),
      state,
      states: if selected_states.is_empty() {
        vec![state]
      } else {
        selected_states
      },
      game_state: game_state_guard.snapshot(),
      test_suite,
      developer,
      goals: GoalSummary {
        blue: goal_guard.blue,
        yellow: goal_guard.yellow,
        blue_active: state.goal_blue,
        yellow_active: state.goal_yellow,
      },
      control: ControlSnapshot {
        web_enabled: self.control.enabled.load(Ordering::Relaxed),
        running: self.control.running.load(Ordering::Relaxed),
        speed: self.speed(),
      },
      replay: ReplayStatus {
        enabled: false,
        frame_index: 0,
        frame_count: 0,
        base_speed: 1.0,
      },
      events: Vec::new(),
      robot_inputs: Vec::new(),
      #[cfg(feature = "viewer-debug")]
      debug,
    };

    if let Ok(json) = serde_json::to_string(&frame) {
      *self.latest_frame.lock() = Some(json);
    }
    let mut properties = std::collections::BTreeMap::new();
    properties.insert(
      "selected_world".into(),
      serde_json::json!(self.selected_world()),
    );
    properties.insert(
      "selected_worlds".into(),
      serde_json::json!(self.selected_worlds()),
    );
    properties.insert(
      "control.running".into(),
      serde_json::json!(self.is_running()),
    );
    properties.insert("control.speed".into(), serde_json::json!(self.speed()));
    properties.insert(
      "test_suite".into(),
      frame.test_suite.clone().unwrap_or(Value::Null),
    );
    properties.insert(
      "developer".into(),
      serde_json::to_value(&frame.developer).unwrap_or(Value::Null),
    );
    #[cfg(feature = "viewer-debug")]
    properties.insert(
      "debug".into(),
      serde_json::to_value(&frame.debug).unwrap_or(Value::Null),
    );
    let score = interface_protocol::Score {
      blue: goal_guard.blue,
      yellow: goal_guard.yellow,
    };
    let referee = canonical_referee(frame.game_state.as_ref(), score.clone());
    let _ = self.interface_publisher.publish(
      self.interface_session,
      canonical_simhark_snapshot(
        states,
        &self.field,
        properties,
        score,
        referee,
        #[cfg(feature = "viewer-debug")]
        &self.debug.lock(),
      ),
    );
  }

  pub fn publish_replay_frame(
    &self,
    replay_frame: &ReplayFrame,
    frame_index: usize,
    frame_count: usize,
    timeline: &[ReplayEvent],
    base_speed: f64,
  ) {
    let Some(state) = selected_state(&replay_frame.states, self.selected_world()) else {
      return;
    };
    let selected_worlds = selected_worlds_snapshot(&self.selected_worlds, self.world_count);
    let selected_states = selected_worlds
      .iter()
      .filter_map(|world| state_by_world_id(&replay_frame.states, *world))
      .collect::<Vec<_>>();
    let mut goal_guard = self.goal_tracker.lock();
    goal_guard.observe(state);
    let game_state_guard = self.game_state.lock();
    let test_suite = self.test_suite.lock().clone();
    let developer = self.developer.lock().clone();
    // A replayed frame carries its own recorded debug data. A live snapshot
    // for the same world still wins, so a controller that is being stepped
    // alongside the replay is not overwritten by history.
    #[cfg(feature = "viewer-debug")]
    let debug_by_world: HashMap<usize, ViewerDebugSnapshot> = {
      let live = self.debug.lock();
      replay_frame
        .states
        .iter()
        .filter_map(|state| {
          live
            .get(&state.world_id)
            .cloned()
            .or_else(|| replay_frame_debug_snapshot(state.world_id, replay_frame))
            .map(|snapshot| (state.world_id, snapshot))
        })
        .collect()
    };
    #[cfg(feature = "viewer-debug")]
    let debug = debug_by_world.get(&state.world_id).cloned();
    let robot_inputs = replay_robot_inputs(replay_frame);
    let frame = ViewerFrame {
      world_count: self.world_count,
      selected_world: self.selected_world(),
      selected_worlds,
      field: &self.field,
      robot_radius: self.robot_radius,
      ball_radius: self.ball_radius,
      ball_trajectory: predicted_ball_trajectory(
        state,
        &self.field,
        self.ball_radius,
        self.ball_friction,
        self.gravity,
      ),
      state,
      states: if selected_states.is_empty() {
        vec![state]
      } else {
        selected_states
      },
      game_state: game_state_guard.snapshot(),
      test_suite,
      developer,
      goals: GoalSummary {
        blue: goal_guard.blue,
        yellow: goal_guard.yellow,
        blue_active: state.goal_blue,
        yellow_active: state.goal_yellow,
      },
      control: ControlSnapshot {
        web_enabled: self.control.enabled.load(Ordering::Relaxed),
        running: self.control.running.load(Ordering::Relaxed),
        speed: self.speed(),
      },
      replay: ReplayStatus {
        enabled: true,
        frame_index,
        frame_count,
        base_speed,
      },
      events: timeline.to_vec(),
      robot_inputs,
      #[cfg(feature = "viewer-debug")]
      debug,
    };

    if let Ok(json) = serde_json::to_string(&frame) {
      *self.latest_frame.lock() = Some(json);
    }
    let score = interface_protocol::Score {
      blue: goal_guard.blue,
      yellow: goal_guard.yellow,
    };
    let referee = canonical_referee(frame.game_state.as_ref(), score.clone());
    let mut properties = std::collections::BTreeMap::new();
    properties.insert("replay.enabled".into(), Value::Bool(true));
    properties.insert("replay.frame_index".into(), serde_json::json!(frame_index));
    properties.insert("replay.frame_count".into(), serde_json::json!(frame_count));
    properties.insert("replay.base_speed".into(), serde_json::json!(base_speed));
    properties.insert(
      "replay.events".into(),
      serde_json::to_value(timeline).unwrap_or(Value::Null),
    );
    properties.insert(
      "replay.robot_inputs".into(),
      serde_json::to_value(&frame.robot_inputs).unwrap_or(Value::Null),
    );
    #[cfg(feature = "viewer-debug")]
    properties.insert(
      "debug".into(),
      serde_json::to_value(&frame.debug).unwrap_or(Value::Null),
    );
    let _ = self.interface_publisher.publish(
      self.interface_session,
      canonical_simhark_snapshot(
        &replay_frame.states,
        &self.field,
        properties,
        score,
        referee,
        #[cfg(feature = "viewer-debug")]
        &debug_by_world,
      ),
    );
  }

  /// Reset the accumulated goal counters (useful when restarting a match).
  pub fn reset_goals(&self) {
    *self.goal_tracker.lock() = GoalTracker::default();
  }
}

impl Drop for ViewerServer {
  fn drop(&mut self) {
    if !self
      .interface_session_terminal
      .swap(true, Ordering::Relaxed)
    {
      let _ = self.interface_handle.update_session(
        self.interface_session,
        InterfaceSessionLifecycle::Cancelled,
        Some("viewer host stopped before the session was finalized".into()),
      );
    }
    self.interface_handle.unregister_system("simhark");
    if let Some(thread) = self.command_thread.take() {
      let _ = thread.join();
    }
  }
}

fn replay_robot_inputs(replay_frame: &ReplayFrame) -> Vec<RobotInputInfo> {
  let inputs = replay_frame
    .debug
    .iter()
    .flat_map(|snapshot| {
      snapshot.robots.iter().map(|robot| RobotInputInfo {
        world_id: snapshot.world_id,
        team: robot.team,
        id: robot.id,
        input: robot
          .message
          .as_ref()
          .filter(|message| !message.is_empty())
          .cloned()
          .unwrap_or_else(|| robot.task.clone()),
      })
    })
    .collect::<Vec<_>>();
  if inputs.is_empty() {
    robot_inputs_for_frame(replay_frame)
  } else {
    inputs
  }
}

#[cfg(feature = "viewer-debug")]
fn replay_frame_debug_snapshot(
  world_id: usize,
  replay_frame: &ReplayFrame,
) -> Option<ViewerDebugSnapshot> {
  replay_frame
    .debug
    .iter()
    .find(|snapshot| snapshot.world_id == world_id)
    .map(ViewerDebugSnapshot::from)
}

#[cfg(feature = "viewer-debug")]
impl From<&ReplayDebugSnapshot> for ViewerDebugSnapshot {
  fn from(snapshot: &ReplayDebugSnapshot) -> Self {
    Self {
      world_id: snapshot.world_id,
      strategy: snapshot.strategy.clone(),
      robots: snapshot
        .robots
        .iter()
        .map(|robot| RobotDebugInfo {
          team: robot.team,
          id: robot.id,
          task: robot.task.clone(),
          color: robot.color.clone(),
          message: robot.message.clone(),
        })
        .collect(),
      overlays: snapshot.overlays.iter().map(DebugOverlay::from).collect(),
    }
  }
}

#[cfg(feature = "viewer-debug")]
impl From<&ReplayDebugOverlay> for DebugOverlay {
  fn from(overlay: &ReplayDebugOverlay) -> Self {
    match overlay {
      ReplayDebugOverlay::HoloRobot(overlay) => DebugOverlay::HoloRobot(DebugHoloRobot {
        team: overlay.team,
        id: overlay.id,
        x: overlay.x,
        y: overlay.y,
        orientation: overlay.orientation,
        color: overlay.color.clone(),
        label: overlay.label.clone(),
      }),
      ReplayDebugOverlay::KickLine(overlay) => DebugOverlay::KickLine(DebugKickLine {
        team: overlay.team,
        id: overlay.id,
        from_x: overlay.from_x,
        from_y: overlay.from_y,
        angle: overlay.angle,
        color: overlay.color.clone(),
        label: overlay.label.clone(),
      }),
    }
  }
}

#[cfg(any())]
fn run_http_server(server: Server, ws_port: u16) {
  let html_type = Header::from_bytes(&b"Content-Type"[..], &b"text/html; charset=utf-8"[..]).ok();

  for request in server.incoming_requests() {
    let path = request
      .url()
      .split_once('?')
      .map_or(request.url(), |(path, _)| path);
    let response = match (request.method(), path) {
      (&Method::Get, "/")
      | (&Method::Get, "/index.html")
      | (&Method::Get, "/debug")
      | (&Method::Get, "/debug-big")
      | (&Method::Get, "/dev") => {
        let body = render_index(ws_port);
        let mut response = Response::from_string(body).with_status_code(StatusCode(200));
        if let Some(header) = html_type.clone() {
          response = response.with_header(header);
        }
        response
      }
      _ => Response::from_string("not found").with_status_code(StatusCode(404)),
    };

    let _ = request.respond(response);
  }
}

const BALL_TRAJECTORY_MIN_SPEED: f64 = 0.05;
const BALL_TRAJECTORY_MAX_VERTICAL_SPEED: f64 = 0.2;
const BALL_TRAJECTORY_MAX_SECONDS: f64 = 8.0;
const BALL_TRAJECTORY_STEP_SECONDS: f64 = 0.12;
const BALL_TRAJECTORY_MIN_POINT_SPACING: f64 = 0.03;

fn predicted_ball_trajectory(
  state: &WorldState,
  field: &FieldConfig,
  ball_radius: f64,
  ball_friction: f64,
  gravity: f64,
) -> Option<BallTrajectory> {
  let planar_speed = state.ball.vx.hypot(state.ball.vy);
  if planar_speed < BALL_TRAJECTORY_MIN_SPEED {
    return None;
  }
  if state.ball.z > ball_radius * 2.5 || state.ball.vz.abs() > BALL_TRAJECTORY_MAX_VERTICAL_SPEED {
    return None;
  }

  let deceleration = ball_friction * gravity;
  if deceleration <= f64::EPSILON {
    return None;
  }

  let dir_x = state.ball.vx / planar_speed;
  let dir_y = state.ball.vy / planar_speed;
  let stop_time = (planar_speed / deceleration).min(BALL_TRAJECTORY_MAX_SECONDS);
  let half_x = field.field_length / 2.0 + field.goal_depth;
  let half_y = field.field_width / 2.0;
  let mut points = vec![BallTrajectoryPoint {
    x: state.ball.x,
    y: state.ball.y,
  }];

  let mut reached_boundary = false;
  let mut t = BALL_TRAJECTORY_STEP_SECONDS;
  while t < stop_time {
    let distance = planar_speed * t - 0.5 * deceleration * t * t;
    let point = BallTrajectoryPoint {
      x: state.ball.x + dir_x * distance,
      y: state.ball.y + dir_y * distance,
    };
    if point.x.abs() > half_x || point.y.abs() > half_y {
      if let Some(boundary) =
        field_boundary_intersection(state.ball.x, state.ball.y, dir_x, dir_y, half_x, half_y)
      {
        push_spaced_trajectory_point(&mut points, boundary);
      }
      reached_boundary = true;
      break;
    }
    push_spaced_trajectory_point(&mut points, point);
    t += BALL_TRAJECTORY_STEP_SECONDS;
  }

  if !reached_boundary {
    let stop_distance = planar_speed * stop_time - 0.5 * deceleration * stop_time * stop_time;
    let stop = BallTrajectoryPoint {
      x: state.ball.x + dir_x * stop_distance,
      y: state.ball.y + dir_y * stop_distance,
    };
    if stop.x.abs() <= half_x && stop.y.abs() <= half_y {
      push_spaced_trajectory_point(&mut points, stop);
    } else if let Some(boundary) =
      field_boundary_intersection(state.ball.x, state.ball.y, dir_x, dir_y, half_x, half_y)
    {
      push_spaced_trajectory_point(&mut points, boundary);
    }
  }

  (points.len() >= 2).then_some(BallTrajectory {
    world_id: state.world_id,
    points,
    stop_time,
  })
}

fn push_spaced_trajectory_point(points: &mut Vec<BallTrajectoryPoint>, point: BallTrajectoryPoint) {
  let Some(previous) = points.last() else {
    points.push(point);
    return;
  };
  if (point.x - previous.x).hypot(point.y - previous.y) >= BALL_TRAJECTORY_MIN_POINT_SPACING {
    points.push(point);
  }
}

fn field_boundary_intersection(
  x: f64,
  y: f64,
  dir_x: f64,
  dir_y: f64,
  half_x: f64,
  half_y: f64,
) -> Option<BallTrajectoryPoint> {
  let mut candidates = Vec::new();
  if dir_x.abs() > f64::EPSILON {
    candidates.push((half_x - x) / dir_x);
    candidates.push((-half_x - x) / dir_x);
  }
  if dir_y.abs() > f64::EPSILON {
    candidates.push((half_y - y) / dir_y);
    candidates.push((-half_y - y) / dir_y);
  }

  candidates
    .into_iter()
    .filter(|t| *t > 0.0)
    .map(|t| BallTrajectoryPoint {
      x: x + dir_x * t,
      y: y + dir_y * t,
    })
    .filter(|point| {
      point.x >= -half_x - 1e-6
        && point.x <= half_x + 1e-6
        && point.y >= -half_y - 1e-6
        && point.y <= half_y + 1e-6
    })
    .min_by(|left, right| {
      let left_dist = (left.x - x).hypot(left.y - y);
      let right_dist = (right.x - x).hypot(right.y - y);
      left_dist.total_cmp(&right_dist)
    })
}

#[cfg(any())]
fn run_websocket_server(
  listener: TcpListener,
  latest_frame: Arc<Mutex<Option<String>>>,
  selected_world: Arc<AtomicUsize>,
  selected_worlds: Arc<Mutex<Vec<usize>>>,
  control: Arc<WebControlState>,
  robot_move_requests: Arc<Mutex<HashMap<RobotMoveKey, RobotMoveRequest>>>,
  robot_rotate_requests: Arc<Mutex<HashMap<RobotMoveKey, RobotRotateRequest>>>,
  robot_presence_requests: Arc<Mutex<HashMap<RobotMoveKey, RobotPresenceRequest>>>,
  ball_move_requests: Arc<Mutex<HashMap<usize, BallMoveRequest>>>,
  developer_requests: Arc<Mutex<Vec<DeveloperRequest>>>,
) {
  for stream in listener.incoming() {
    let Ok(stream) = stream else {
      continue;
    };

    let latest_frame = Arc::clone(&latest_frame);
    let selected_world = Arc::clone(&selected_world);
    let selected_worlds = Arc::clone(&selected_worlds);
    let control = Arc::clone(&control);
    let robot_move_requests = Arc::clone(&robot_move_requests);
    let robot_rotate_requests = Arc::clone(&robot_rotate_requests);
    let robot_presence_requests = Arc::clone(&robot_presence_requests);
    let ball_move_requests = Arc::clone(&ball_move_requests);
    let developer_requests = Arc::clone(&developer_requests);
    thread::spawn(move || {
      let Ok(mut websocket) = accept(stream) else {
        return;
      };
      let _ = websocket
        .get_mut()
        .set_read_timeout(Some(Duration::from_millis(1)));

      let mut last_sent = String::new();

      loop {
        let mut close_requested = false;
        loop {
          match websocket.read() {
            Ok(Message::Text(text)) => handle_client_message(
              text.as_str(),
              &selected_world,
              &selected_worlds,
              &control,
              &robot_move_requests,
              &robot_rotate_requests,
              &robot_presence_requests,
              &ball_move_requests,
              &developer_requests,
            ),
            Ok(Message::Close(_)) => {
              close_requested = true;
              break;
            }
            Ok(_) => {}
            Err(tungstenite::Error::Io(err))
              if matches!(
                err.kind(),
                std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
              ) =>
            {
              break;
            }
            Err(tungstenite::Error::ConnectionClosed | tungstenite::Error::AlreadyClosed) => {
              close_requested = true;
              break;
            }
            Err(_) => {
              close_requested = true;
              break;
            }
          }
        }
        if close_requested {
          break;
        }

        if let Some(frame) = latest_frame.lock().clone() {
          if frame != last_sent {
            if websocket.send(Message::Text(frame.clone().into())).is_err() {
              break;
            }
            last_sent = frame;
          }
        }

        thread::sleep(Duration::from_millis(16));
      }
    });
  }
}

#[cfg(any())]
fn handle_client_message(
  message: &str,
  selected_world: &AtomicUsize,
  selected_worlds: &Mutex<Vec<usize>>,
  control: &WebControlState,
  robot_move_requests: &Mutex<HashMap<RobotMoveKey, RobotMoveRequest>>,
  robot_rotate_requests: &Mutex<HashMap<RobotMoveKey, RobotRotateRequest>>,
  robot_presence_requests: &Mutex<HashMap<RobotMoveKey, RobotPresenceRequest>>,
  ball_move_requests: &Mutex<HashMap<usize, BallMoveRequest>>,
  developer_requests: &Mutex<Vec<DeveloperRequest>>,
) {
  if let Some(value) = message.strip_prefix("developer:") {
    if let Ok(request) = serde_json::from_str::<DeveloperRequest>(value) {
      let _ = queue_developer_request(developer_requests, request);
    }
    return;
  }

  if let Some(value) = message.strip_prefix("world:") {
    if let Ok(index) = value.trim().parse::<usize>() {
      selected_world.store(index, Ordering::Relaxed);
      *selected_worlds.lock() = vec![index];
    }
    return;
  }

  if let Some(value) = message.strip_prefix("worlds:") {
    let worlds = parse_world_selection(value);
    if value.trim().eq_ignore_ascii_case("all") {
      selected_world.store(0, Ordering::Relaxed);
      *selected_worlds.lock() = Vec::new();
    } else if let Some(first) = worlds.first() {
      selected_world.store(*first, Ordering::Relaxed);
      *selected_worlds.lock() = worlds;
    }
    return;
  }

  if let Some(action) = message.strip_prefix("control:") {
    // Control commands are silently ignored when web control wasn't
    // opted in, so a buggy/old UI can't restart a headless training job.
    if !control.enabled.load(Ordering::Relaxed) {
      return;
    }
    match action.trim() {
      "start" => control.running.store(true, Ordering::Relaxed),
      "pause" => control.running.store(false, Ordering::Relaxed),
      "stop" => {
        control.stop_requested.store(true, Ordering::Relaxed);
        control.running.store(false, Ordering::Relaxed);
      }
      "restart" => {
        control.restart_requested.store(true, Ordering::Relaxed);
        control.running.store(true, Ordering::Relaxed);
      }
      _ => {}
    }
    return;
  }

  if let Some(value) = message.strip_prefix("speed:") {
    if !control.enabled.load(Ordering::Relaxed) {
      return;
    }
    if let Ok(speed) = value.trim().parse::<f64>() {
      let speed_percent = (speed.clamp(0.05, 4.0) * 100.0).round() as usize;
      control
        .speed_percent
        .store(speed_percent.max(1), Ordering::Relaxed);
    }
    return;
  }

  if let Some(value) = message.strip_prefix("replay:step:") {
    if !control.enabled.load(Ordering::Relaxed) {
      return;
    }
    if let Ok(delta) = value.trim().parse::<isize>() {
      control
        .frame_step_requested
        .fetch_add(delta.clamp(-10_000, 10_000), Ordering::Relaxed);
      control.running.store(false, Ordering::Relaxed);
    }
    return;
  }

  if let Some(value) = message.strip_prefix("replay:skip:") {
    if !control.enabled.load(Ordering::Relaxed) {
      return;
    }
    if let Ok(delta) = value.trim().parse::<isize>() {
      control
        .frame_skip_requested
        .fetch_add(delta.clamp(-100_000, 100_000), Ordering::Relaxed);
    }
    return;
  }

  if let Some(value) = message.strip_prefix("replay:seek:") {
    if !control.enabled.load(Ordering::Relaxed) {
      return;
    }
    if let Ok(frame) = value.trim().parse::<isize>() {
      control
        .frame_seek_requested
        .store(frame.max(0), Ordering::Relaxed);
    }
    return;
  }

  if let Some(request) = parse_robot_move_request(message) {
    robot_move_requests
      .lock()
      .insert((request.world_id, request.team, request.id), request);
    return;
  }

  if let Some(request) = parse_robot_rotate_request(message) {
    robot_rotate_requests
      .lock()
      .insert((request.world_id, request.team, request.id), request);
    return;
  }

  if let Some(request) = parse_robot_presence_request(message) {
    robot_presence_requests
      .lock()
      .insert((request.world_id, request.team, request.id), request);
    return;
  }

  if let Some(request) = parse_ball_move_request(message) {
    ball_move_requests.lock().insert(request.world_id, request);
  }
}

#[cfg(any())]
fn parse_robot_move_request(message: &str) -> Option<RobotMoveRequest> {
  let value = message.strip_prefix("robot:move:")?;
  let mut parts = value.split(':');
  let world_id = parts.next()?.parse().ok()?;
  let team = match parts.next()? {
    "Blue" => TeamColor::Blue,
    "Yellow" => TeamColor::Yellow,
    _ => return None,
  };
  let id = parts.next()?.parse().ok()?;
  let x = parts.next()?.parse::<f64>().ok()?;
  let y = parts.next()?.parse::<f64>().ok()?;
  if parts.next().is_some() || !x.is_finite() || !y.is_finite() {
    return None;
  }
  Some(RobotMoveRequest {
    world_id,
    team,
    id,
    x,
    y,
  })
}

#[cfg(any())]
fn parse_robot_rotate_request(message: &str) -> Option<RobotRotateRequest> {
  let value = message.strip_prefix("robot:rotate:")?;
  let mut parts = value.split(':');
  let world_id = parts.next()?.parse().ok()?;
  let team = match parts.next()? {
    "Blue" => TeamColor::Blue,
    "Yellow" => TeamColor::Yellow,
    _ => return None,
  };
  let id = parts.next()?.parse().ok()?;
  let orientation = parts.next()?.parse::<f64>().ok()?;
  if parts.next().is_some() || !orientation.is_finite() {
    return None;
  }
  Some(RobotRotateRequest {
    world_id,
    team,
    id,
    orientation,
  })
}

#[cfg(any())]
fn parse_robot_presence_request(message: &str) -> Option<RobotPresenceRequest> {
  let value = message.strip_prefix("robot:presence:")?;
  let mut parts = value.split(':');
  let world_id = parts.next()?.parse().ok()?;
  let team = match parts.next()? {
    "Blue" => TeamColor::Blue,
    "Yellow" => TeamColor::Yellow,
    _ => return None,
  };
  let id = parts.next()?.parse().ok()?;
  let present = match parts.next()? {
    "true" => true,
    "false" => false,
    _ => return None,
  };
  if parts.next().is_some() {
    return None;
  }
  Some(RobotPresenceRequest {
    world_id,
    team,
    id,
    present,
  })
}

#[cfg(any())]
fn parse_ball_move_request(message: &str) -> Option<BallMoveRequest> {
  let value = message.strip_prefix("ball:move:")?;
  let mut parts = value.split(':');
  let world_id = parts.next()?.parse().ok()?;
  let x = parts.next()?.parse::<f64>().ok()?;
  let y = parts.next()?.parse::<f64>().ok()?;
  if parts.next().is_some() || !x.is_finite() || !y.is_finite() {
    return None;
  }
  Some(BallMoveRequest { world_id, x, y })
}

#[cfg(any())]
fn parse_world_selection(value: &str) -> Vec<usize> {
  let value = value.trim();
  if value.eq_ignore_ascii_case("all") {
    return Vec::new();
  }

  let mut worlds = value
    .split(',')
    .filter_map(|part| part.trim().parse::<usize>().ok())
    .collect::<Vec<_>>();
  worlds.sort_unstable();
  worlds.dedup();
  worlds
}

fn selected_worlds_snapshot(selected_worlds: &Mutex<Vec<usize>>, world_count: usize) -> Vec<usize> {
  let selected = selected_worlds.lock().clone();
  let mut worlds = if selected.is_empty() {
    (0..world_count).collect::<Vec<_>>()
  } else {
    selected
      .into_iter()
      .filter(|world| *world < world_count)
      .collect::<Vec<_>>()
  };
  if worlds.is_empty() {
    worlds.push(0);
  }
  worlds
}

fn selected_state(states: &[WorldState], selected_world: usize) -> Option<&WorldState> {
  if states.is_empty() {
    return None;
  }
  state_by_world_id(states, selected_world)
    .or_else(|| states.get(selected_world))
    .or_else(|| states.first())
}

fn state_by_world_id(states: &[WorldState], selected_world: usize) -> Option<&WorldState> {
  states.iter().find(|state| state.world_id == selected_world)
}

// The v4 server is hosted by `webinterface-core`. This fallback only keeps
// legacy helper code source-compatible while downstream callers migrate.
#[cfg(any())]
const FRONTEND_HTML: &str = "<!doctype html><html><body><div id=\"root\"></div></body></html>";

#[cfg(any())]
fn render_index(ws_port: u16) -> String {
  let injected = format!("<script>window.__SIMHARK_WS_PORT__={ws_port};</script>");
  if let Some((head, tail)) = FRONTEND_HTML.split_once("</head>") {
    format!("{head}{injected}</head>{tail}")
  } else {
    // No </head> tag — fall back to prepending the script to the body.
    format!("{injected}{FRONTEND_HTML}")
  }
}

#[cfg(any())]
mod tests {
  use super::{
    BallMoveRequest, DeveloperRequest, RobotMoveRequest, RobotPresenceRequest, RobotRotateRequest,
    parse_ball_move_request, parse_robot_move_request, parse_robot_presence_request,
    parse_robot_rotate_request,
  };
  use crate::state::TeamColor;

  #[test]
  fn parses_robot_move_request() {
    assert_eq!(
      parse_robot_move_request("robot:move:3:Yellow:7:-1.25:2.5"),
      Some(RobotMoveRequest {
        world_id: 3,
        team: TeamColor::Yellow,
        id: 7,
        x: -1.25,
        y: 2.5,
      })
    );
  }

  #[test]
  fn rejects_invalid_robot_move_request() {
    assert_eq!(parse_robot_move_request("robot:move:0:Green:1:0:0"), None);
    assert_eq!(parse_robot_move_request("robot:move:0:Blue:1:NaN:0"), None);
    assert_eq!(
      parse_robot_move_request("robot:move:0:Blue:1:0:0:extra"),
      None
    );
  }

  #[test]
  fn parses_robot_rotate_request() {
    assert_eq!(
      parse_robot_rotate_request("robot:rotate:3:Yellow:7:-1.5708"),
      Some(RobotRotateRequest {
        world_id: 3,
        team: TeamColor::Yellow,
        id: 7,
        orientation: -1.5708,
      })
    );
  }

  #[test]
  fn rejects_invalid_robot_rotate_request() {
    assert_eq!(parse_robot_rotate_request("robot:rotate:0:Green:1:0"), None);
    assert_eq!(
      parse_robot_rotate_request("robot:rotate:0:Blue:1:NaN"),
      None
    );
    assert_eq!(
      parse_robot_rotate_request("robot:rotate:0:Blue:1:0:extra"),
      None
    );
  }

  #[test]
  fn parses_robot_presence_request() {
    assert_eq!(
      parse_robot_presence_request("robot:presence:2:Blue:4:false"),
      Some(RobotPresenceRequest {
        world_id: 2,
        team: TeamColor::Blue,
        id: 4,
        present: false,
      })
    );
    assert_eq!(
      parse_robot_presence_request("robot:presence:0:Yellow:1:true"),
      Some(RobotPresenceRequest {
        world_id: 0,
        team: TeamColor::Yellow,
        id: 1,
        present: true,
      })
    );
  }

  #[test]
  fn rejects_invalid_robot_presence_request() {
    assert_eq!(
      parse_robot_presence_request("robot:presence:0:Green:1:true"),
      None
    );
    assert_eq!(
      parse_robot_presence_request("robot:presence:0:Blue:1:yes"),
      None
    );
    assert_eq!(
      parse_robot_presence_request("robot:presence:0:Blue:1:false:extra"),
      None
    );
  }

  #[test]
  fn parses_ball_move_request() {
    assert_eq!(
      parse_ball_move_request("ball:move:2:-3.25:1.5"),
      Some(BallMoveRequest {
        world_id: 2,
        x: -3.25,
        y: 1.5,
      })
    );
  }

  #[test]
  fn rejects_invalid_ball_move_request() {
    assert_eq!(parse_ball_move_request("ball:move:0:NaN:0"), None);
    assert_eq!(parse_ball_move_request("ball:move:0:0:0:extra"), None);
  }

  #[test]
  fn parses_match_developer_requests() {
    let switch = serde_json::from_str::<DeveloperRequest>(
      r#"{"action":"switch_ai","target":"yellow","ai":"bongka"}"#,
    )
    .unwrap();
    assert!(matches!(
      switch,
      DeveloperRequest::SwitchAi { target, ai }
        if target == "yellow" && ai == "bongka"
    ));

    let recovery = serde_json::from_str::<DeveloperRequest>(
      r#"{"action":"set_ball_recovery","target":"ball-recovery","enabled":false}"#,
    )
    .unwrap();
    assert!(matches!(
      recovery,
      DeveloperRequest::SetBallRecovery {
        target,
        enabled: false,
      } if target == "ball-recovery"
    ));
  }
}

#[cfg(test)]
mod interface_tests {
  use super::*;

  #[test]
  fn parses_developer_load_request() {
    let request = serde_json::from_str::<DeveloperRequest>(
      r#"{
        "action": "load",
        "target": "blue",
        "kind": "skill",
        "entry": "Pass To",
        "config": {},
        "params": {"passer": "R0", "receiver": "R1"}
      }"#,
    )
    .unwrap();

    assert!(matches!(
      request,
      DeveloperRequest::Load { target, kind, entry, .. }
        if target == "blue" && kind == "skill" && entry == "Pass To"
    ));
  }

  #[test]
  fn parses_developer_run_lifecycle_requests() {
    let start =
      serde_json::from_str::<DeveloperRequest>(r#"{"action":"start","target":"blue"}"#).unwrap();
    assert!(matches!(start, DeveloperRequest::Start { target } if target == "blue"));

    let stop =
      serde_json::from_str::<DeveloperRequest>(r#"{"action":"stop","target":"blue"}"#).unwrap();
    assert!(matches!(stop, DeveloperRequest::Stop { target } if target == "blue"));
  }

  #[test]
  fn queued_requests_keep_their_order() {
    let queue = Mutex::new(Vec::new());
    queue_developer_request(
      &queue,
      DeveloperRequest::Load {
        target: "blue".into(),
        kind: "skill".into(),
        entry: "Pass To".into(),
        config: Value::Null,
        params: Value::Null,
      },
    )
    .unwrap();
    queue_developer_request(
      &queue,
      DeveloperRequest::Start {
        target: "blue".into(),
      },
    )
    .unwrap();

    let queued = queue.lock();
    // `load` then `start` in one tick must arrive in that order; coalescing
    // them per target used to drop the load and fail the start.
    assert!(matches!(queued[0], DeveloperRequest::Load { .. }));
    assert!(matches!(queued[1], DeveloperRequest::Start { .. }));
  }

  #[test]
  fn a_wedged_queue_refuses_new_requests_instead_of_growing() {
    let queue = Mutex::new(Vec::new());
    for _ in 0..DEVELOPER_REQUEST_QUEUE_LIMIT {
      queue_developer_request(
        &queue,
        DeveloperRequest::Start {
          target: "blue".into(),
        },
      )
      .unwrap();
    }

    assert!(
      queue_developer_request(
        &queue,
        DeveloperRequest::Start {
          target: "blue".into(),
        },
      )
      .is_err()
    );
    assert_eq!(queue.lock().len(), DEVELOPER_REQUEST_QUEUE_LIMIT);
  }

  #[cfg(feature = "viewer-debug")]
  mod debug_conversion {
    use super::*;
    use crate::state::{BallState, RobotState};
    use crate::state::KickStatus;

    fn world() -> WorldState {
      WorldState {
        world_id: 0,
        sim_time: 1.0,
        frame: 60,
        ball: BallState {
          x: 0.0,
          y: 0.0,
          z: 0.0,
          vx: 0.0,
          vy: 0.0,
          vz: 0.0,
        },
        blue_robots: vec![RobotState {
          id: 2,
          team: TeamColor::Blue,
          x: 1.0,
          y: -0.5,
          z: 0.1,
          orientation: 0.0,
          vx: 0.0,
          vy: 0.0,
          vz: 0.0,
          v_angular: 0.0,
          infrared: false,
          dribbler_on: false,
          kick_status: KickStatus::NoKick,
          is_on: true,
          wheel_speeds: [0.0; 4],
        }],
        yellow_robots: Vec::new(),
        goal_blue: false,
        goal_yellow: false,
      }
    }

    fn snapshot() -> ViewerDebugSnapshot {
      ViewerDebugSnapshot {
        world_id: 0,
        strategy: Some("attacking".to_string()),
        robots: vec![RobotDebugInfo {
          team: TeamColor::Blue,
          id: 2,
          task: "Attacker".to_string(),
          color: "#4488ff".to_string(),
          message: Some("chasing".to_string()),
        }],
        overlays: vec![
          DebugOverlay::HoloRobot(DebugHoloRobot {
            team: TeamColor::Blue,
            id: 2,
            x: 2.0,
            y: 0.25,
            orientation: Some(1.0),
            color: "#4488ff".to_string(),
            label: Some("target".to_string()),
          }),
          DebugOverlay::KickLine(DebugKickLine {
            team: TeamColor::Blue,
            id: 2,
            from_x: 0.0,
            from_y: 0.0,
            angle: 0.0,
            color: "#ffffff".to_string(),
            label: None,
          }),
        ],
      }
    }

    #[test]
    fn every_layer_an_item_uses_is_declared() {
      let field = crate::WorldConfig::division_b().field;
      let debug = HashMap::from([(0, snapshot())]);
      let items = canonical_debug_items(&debug, &[world()], &field);
      let layers = simhark_debug_layers()
        .into_iter()
        .map(|layer| layer.id)
        .collect::<Vec<_>>();

      assert!(!items.is_empty());
      for item in &items {
        assert!(
          layers.contains(&item.layer_id),
          "undeclared layer {}",
          item.layer_id
        );
      }
    }

    #[test]
    fn a_task_label_follows_its_robot_in_millimetres() {
      let field = crate::WorldConfig::division_b().field;
      let debug = HashMap::from([(0, snapshot())]);
      let items = canonical_debug_items(&debug, &[world()], &field);

      let task = items
        .iter()
        .find(|item| item.layer_id == "simhark.debug.tasks.blue")
        .expect("a task label");
      assert_eq!(task.robot_id, Some(2));
      match &task.primitive {
        interface_protocol::DebugPrimitive::Text { at, text, .. } => {
          assert_eq!(at.x_mm.0, 1000.0);
          assert!((at.y_mm.0 - -340.0).abs() < 1.0e-6);
          assert!(text.contains("Attacker") && text.contains("chasing"));
        }
        other => panic!("expected text, got {other:?}"),
      }
    }

    #[test]
    fn a_task_label_for_an_absent_robot_is_dropped() {
      let field = crate::WorldConfig::division_b().field;
      let mut state = world();
      state.blue_robots.clear();
      let debug = HashMap::from([(0, snapshot())]);
      let items = canonical_debug_items(&debug, &[state], &field);

      assert!(
        !items
          .iter()
          .any(|item| item.layer_id == "simhark.debug.tasks.blue")
      );
    }

    #[test]
    fn a_kick_line_ends_on_the_field_boundary() {
      let config = crate::WorldConfig::division_b();
      let debug = HashMap::from([(0, snapshot())]);
      let items = canonical_debug_items(&debug, &[world()], &config.field);

      let kick = items
        .iter()
        .find(|item| item.layer_id == "simhark.debug.kicks.blue")
        .expect("a kick line");
      match &kick.primitive {
        interface_protocol::DebugPrimitive::Arrow { from, to, .. } => {
          assert_eq!(from.x_mm.0, 0.0);
          assert!(
            (to.x_mm.0 - config.field.field_length * 0.5 * 1000.0).abs() < 1.0,
            "expected the goal line, got {}",
            to.x_mm.0
          );
        }
        other => panic!("expected an arrow, got {other:?}"),
      }
    }

    #[test]
    fn worlds_without_debug_data_contribute_nothing() {
      let field = crate::WorldConfig::division_b().field;
      let items = canonical_debug_items(&HashMap::new(), &[world()], &field);
      assert!(items.is_empty());
    }
  }
}
