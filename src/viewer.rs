//! Optional browser-based debug viewer.

use std::collections::HashMap;
use std::io::{Error, Result};
use std::net::{IpAddr, Ipv4Addr, SocketAddr, TcpListener};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicIsize, AtomicUsize, Ordering};
use std::thread;
use std::time::Duration;

use parking_lot::Mutex;
use serde::Serialize;
use serde_json::Value;
use tiny_http::{Header, Method, Response, Server, StatusCode};
use tungstenite::{Message, accept};

use crate::command::TeleportRobot;
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
      http_port: 8315,
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
  goal_tracker: Arc<Mutex<GoalTracker>>,
  #[cfg(feature = "viewer-debug")]
  debug: Arc<Mutex<HashMap<usize, ViewerDebugSnapshot>>>,
  control: Arc<WebControlState>,
  robot_move_requests: Arc<Mutex<HashMap<RobotMoveKey, RobotMoveRequest>>>,
  _http_thread: thread::JoinHandle<()>,
  _ws_thread: thread::JoinHandle<()>,
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
  goals: GoalSummary,
  control: ControlSnapshot,
  replay: ReplayStatus,
  events: Vec<ReplayEvent>,
  robot_inputs: Vec<RobotInputInfo>,
  #[cfg(feature = "viewer-debug")]
  debug: Option<ViewerDebugSnapshot>,
}

impl ViewerServer {
  pub fn bind(
    config: ViewerConfig,
    world_count: usize,
    world_config: &WorldConfig,
  ) -> Result<Self> {
    let http_addr = SocketAddr::new(config.host, config.http_port);
    let ws_addr = SocketAddr::new(config.host, config.websocket_port());

    let http_server = Server::http(http_addr).map_err(|err| Error::other(err.to_string()))?;
    let ws_listener = TcpListener::bind(ws_addr)?;

    let selected_world = Arc::new(AtomicUsize::new(0));
    let selected_worlds = Arc::new(Mutex::new(vec![0]));
    let latest_frame = Arc::new(Mutex::new(None));
    let game_state = Arc::new(Mutex::new(GameStateTracker::default()));
    let test_suite = Arc::new(Mutex::new(None));
    let goal_tracker = Arc::new(Mutex::new(GoalTracker::default()));
    #[cfg(feature = "viewer-debug")]
    let debug = Arc::new(Mutex::new(HashMap::new()));
    let control = Arc::new(WebControlState::default());
    let robot_move_requests = Arc::new(Mutex::new(HashMap::new()));
    control.frame_seek_requested.store(-1, Ordering::Relaxed);
    // When web control is disabled the simulator is considered always
    // running, so callers that don't opt in see the legacy behaviour.
    control.running.store(true, Ordering::Relaxed);
    control.speed_percent.store(100, Ordering::Relaxed);

    let http_thread = {
      let ws_port = config.websocket_port();
      thread::spawn(move || run_http_server(http_server, ws_port))
    };

    let ws_thread = {
      let latest_frame = Arc::clone(&latest_frame);
      let selected_world = Arc::clone(&selected_world);
      let selected_worlds = Arc::clone(&selected_worlds);
      let control_for_ws = Arc::clone(&control);
      let robot_move_requests = Arc::clone(&robot_move_requests);
      thread::spawn(move || {
        run_websocket_server(
          ws_listener,
          latest_frame,
          selected_world,
          selected_worlds,
          control_for_ws,
          robot_move_requests,
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
      goal_tracker,
      #[cfg(feature = "viewer-debug")]
      debug,
      control,
      robot_move_requests,
      _http_thread: http_thread,
      _ws_thread: ws_thread,
    })
  }

  /// Opt in to web-driven start/stop/restart. The simulator starts in the
  /// stopped state; the application is expected to gate stepping on
  /// [`Self::is_running`] and react to [`Self::take_restart_request`].
  pub fn enable_web_control(&self) {
    self.control.enabled.store(true, Ordering::Relaxed);
    self.control.running.store(false, Ordering::Relaxed);
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

  /// Apply the latest browser drag request for each robot to its simulation
  /// world. Returns the number of robots that were repositioned.
  ///
  /// Applications with a viewer should call this once per loop, including
  /// while paused, so dragging remains responsive without advancing physics.
  pub fn apply_robot_move_requests(&self, engine: &mut SimulationEngine) -> usize {
    let requests = std::mem::take(&mut *self.robot_move_requests.lock());
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
    #[cfg(feature = "viewer-debug")]
    let debug = self
      .debug
      .lock()
      .get(&state.world_id)
      .cloned()
      .or_else(|| replay_frame_debug_snapshot(state.world_id, replay_frame));
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
  }

  /// Reset the accumulated goal counters (useful when restarting a match).
  pub fn reset_goals(&self) {
    *self.goal_tracker.lock() = GoalTracker::default();
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
      | (&Method::Get, "/debug-big") => {
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

fn run_websocket_server(
  listener: TcpListener,
  latest_frame: Arc<Mutex<Option<String>>>,
  selected_world: Arc<AtomicUsize>,
  selected_worlds: Arc<Mutex<Vec<usize>>>,
  control: Arc<WebControlState>,
  robot_move_requests: Arc<Mutex<HashMap<RobotMoveKey, RobotMoveRequest>>>,
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

fn handle_client_message(
  message: &str,
  selected_world: &AtomicUsize,
  selected_worlds: &Mutex<Vec<usize>>,
  control: &WebControlState,
  robot_move_requests: &Mutex<HashMap<RobotMoveKey, RobotMoveRequest>>,
) {
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
  }
}

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

const FRONTEND_HTML: &str = include_str!("../frontend/dist/index.html");

fn render_index(ws_port: u16) -> String {
  let injected = format!("<script>window.__SIMHARK_WS_PORT__={ws_port};</script>");
  if let Some((head, tail)) = FRONTEND_HTML.split_once("</head>") {
    format!("{head}{injected}</head>{tail}")
  } else {
    // No </head> tag — fall back to prepending the script to the body.
    format!("{injected}{FRONTEND_HTML}")
  }
}

#[cfg(test)]
mod tests {
  use super::{RobotMoveRequest, parse_robot_move_request};
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
}
