//! Library core for the SSL match runner: a reusable `run_match` plus the
//! controller / director / evaluator / logging building blocks.

pub mod controller;
#[cfg(feature = "dehumanized")]
mod direct_dehumanized;
pub mod director;
pub mod evaluator;
pub mod logio;
#[cfg(feature = "referris")]
pub mod referris_autoref;
#[cfg(feature = "sumatra")]
pub mod sumatra_match;

#[cfg(feature = "viewer")]
use controller::{Controller, hot_swappable_team_kinds};
use controller::{TeamKind, build_controller};
use director::MatchDirector;
use evaluator::{Evaluator, MatchReport};
use logio::GameLog;
#[cfg(feature = "viewer-debug")]
use simhark::ReplayDebugSnapshot;
use simhark::{
  MoveCommand, ReplayLog, ReplayRecorder, RobotCommand, RobotState, SimulationEngine, TeamColor,
  WorldConfig, WorldState,
};

#[cfg(feature = "sim-time")]
const SIM_TIME_STEP_MS: &str = "16.666666";

/// Everything needed to play one match.
#[derive(Clone)]
pub struct MatchConfig {
  pub blue: TeamKind,
  pub yellow: TeamKind,
  /// Available blue robots. Defaults to the division robot count.
  pub blue_bots: Option<usize>,
  /// Available yellow robots. Defaults to the division robot count.
  pub yellow_bots: Option<usize>,
  pub seconds: f64,
  pub div: char,
  pub seed: u64,
  pub log: Option<String>,
  pub replay: Option<String>,
  /// Record canonical interface state/events to `.faabsrec`.
  pub interface_recording: bool,
  pub log_every: u64,
  pub quiet: bool,
  /// Open the live web viewer (requires the `viewer` build feature).
  pub viewer: bool,
  /// Pace the simulation to ~60 Hz wall-clock (implied by `viewer`).
  pub realtime: bool,
  /// Run an unlimited, viewer-backed development match with live controls.
  pub dev: bool,
  /// Recover a ball that has made no progress by teleporting it to the centre.
  pub teleport_ball_on_no_progress: bool,
  /// Print simulator-level robot commands at a throttled interval.
  pub print_commands: bool,
  /// Frame interval for command printing.
  pub print_commands_every: u64,
  /// Warn when a close slow ball is not acquired or a fast reachable pickup point is idle.
  pub validate_pickup: bool,
}

impl Default for MatchConfig {
  fn default() -> Self {
    Self {
      blue: TeamKind::Bangka,
      yellow: TeamKind::Bangka,
      blue_bots: None,
      yellow_bots: None,
      seconds: 60.0,
      div: 'b',
      seed: 1,
      log: None,
      replay: None,
      interface_recording: false,
      log_every: 2,
      quiet: false,
      viewer: false,
      realtime: false,
      dev: false,
      teleport_ball_on_no_progress: true,
      print_commands: false,
      print_commands_every: 60,
      validate_pickup: false,
    }
  }
}

pub fn world_config(div: char, seed: u64) -> WorldConfig {
  let mut cfg = match div {
    'a' | 'A' => WorldConfig::division_a(),
    _ => WorldConfig::division_b(),
  };
  cfg.seed = seed;
  cfg
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct TeamBotCounts {
  pub blue: usize,
  pub yellow: usize,
}

impl MatchConfig {
  pub(crate) fn bot_counts(&self, default: usize) -> TeamBotCounts {
    TeamBotCounts {
      blue: self.blue_bots.unwrap_or(default),
      yellow: self.yellow_bots.unwrap_or(default),
    }
  }

  pub(crate) fn physical_bot_counts(&self, default: usize) -> TeamBotCounts {
    TeamBotCounts {
      blue: physical_bot_count(self.blue_bots, default),
      yellow: physical_bot_count(self.yellow_bots, default),
    }
  }
}

fn physical_bot_count(configured: Option<usize>, default: usize) -> usize {
  match configured {
    Some(0) | None => default,
    Some(count) => count,
  }
}

/// Play one match start-to-finish and return its evaluation.
///
/// If either side is an external AI (e.g. the real Sumatra), the match is run
/// through the SimNet hybrid path; otherwise both sides are driven in-process.
pub fn run_match(mc: &MatchConfig) -> MatchReport {
  #[cfg(feature = "sim-time")]
  let _fixed_firmware_dt = FixedFirmwareDt::enable();

  #[cfg(not(feature = "viewer"))]
  if mc.dev {
    eprintln!("development matches require building match-runner with `--features viewer`");
    let cfg = world_config(mc.div, mc.seed);
    return Evaluator::new(
      cfg,
      format!("blue:{}", mc.blue.label()),
      format!("yellow:{}", mc.yellow.label()),
    )
    .finish(0.0);
  }

  let default_bots = world_config(mc.div, mc.seed).robots_per_team;
  let route_bots = mc.bot_counts(default_bots);
  let has_active_external = (route_bots.blue > 0 && mc.blue.is_external())
    || (route_bots.yellow > 0 && mc.yellow.is_external());
  if mc.dev && has_active_external {
    eprintln!("--dev does not support matches containing Sumatra; use in-process AIs only");
    let cfg = world_config(mc.div, mc.seed);
    return Evaluator::new(
      cfg,
      format!("blue:{}", mc.blue.label()),
      format!("yellow:{}", mc.yellow.label()),
    )
    .finish(0.0);
  }
  if has_active_external {
    #[cfg(feature = "sumatra")]
    return sumatra_match::run(mc);
    #[cfg(not(feature = "sumatra"))]
    {
      eprintln!(
        "sumatra matches require building match-runner with `--features sumatra` and setting SIMHARK_SUMATRA_REPO_ROOT"
      );
      let cfg = world_config(mc.div, mc.seed);
      return Evaluator::new(
        cfg,
        format!("blue:{}", mc.blue.label()),
        format!("yellow:{}", mc.yellow.label()),
      )
      .finish(0.0);
    }
  }
  let mut cfg = world_config(mc.div, mc.seed);
  let default_bots = cfg.robots_per_team;
  let bots = mc.bot_counts(default_bots);
  let physical_bots = mc.physical_bot_counts(default_bots);
  cfg.robots_per_team = physical_bots.blue.max(physical_bots.yellow);
  let mut engine = SimulationEngine::new(1, cfg.clone());

  let mut blue_ctrl =
    (bots.blue > 0).then(|| build_controller(&mc.blue, TeamColor::Blue, bots.blue as u8));
  let mut yellow_ctrl =
    (bots.yellow > 0).then(|| build_controller(&mc.yellow, TeamColor::Yellow, bots.yellow as u8));
  let blue_name = format!(
    "blue:{}",
    side_name(&mc.blue, blue_ctrl.as_deref(), bots.blue)
  );
  let yellow_name = format!(
    "yellow:{}",
    side_name(&mc.yellow, yellow_ctrl.as_deref(), bots.yellow)
  );

  let duration = if mc.dev { f64::INFINITY } else { mc.seconds };
  let mut director = MatchDirector::new(cfg.clone(), duration)
    .with_bot_counts(physical_bots.blue, physical_bots.yellow);
  director.set_teleport_ball_on_no_progress(mc.teleport_ball_on_no_progress);
  let mut evaluator = Evaluator::new(cfg.clone(), blue_name.clone(), yellow_name.clone());
  #[cfg(feature = "referris")]
  let mut referris = referris_autoref::ReferrisAutoref::new();
  let mut pickup_validator = PickupValidator::default();

  let mut log = match &mc.log {
    Some(path) => GameLog::create(path, &cfg, &blue_name, &yellow_name).ok(),
    None => None,
  };

  #[cfg(feature = "viewer")]
  let viewer = if (mc.viewer || mc.dev) && (mc.replay.is_none() || mc.interface_recording) {
    let vc = simhark::viewer::ViewerConfig::default();
    match simhark::viewer::ViewerServer::bind(vc, 1, &cfg) {
      Ok(v) => {
        v.enable_web_control_running();
        if mc.interface_recording
          && let Err(error) = v.start_recording()
        {
          eprintln!("failed to start interface recording: {error}");
        }
        configure_developer_console(
          &v,
          mc.dev,
          blue_ctrl.as_deref(),
          yellow_ctrl.as_deref(),
          director.teleport_ball_on_no_progress(),
        );
        if mc.dev {
          println!("dev viewer: {}/dev", vc.http_url());
        } else {
          println!("viewer: {}", vc.http_url());
        }
        Some(v)
      }
      Err(e) => {
        eprintln!("viewer bind failed: {e}");
        None
      }
    }
  } else {
    None
  };
  let pace = mc.replay.is_none() && (mc.realtime || mc.viewer || mc.dev);
  #[cfg(feature = "viewer")]
  if let Some(viewer) = &viewer {
    attach_controller_interfaces(viewer, &mut blue_ctrl, &mut yellow_ctrl);
  }

  let kickoff = director.kickoff_reset();
  let mut state = engine
    .step_with_commands(std::slice::from_ref(&kickoff))
    .remove(0);
  let mut replay = mc
    .replay
    .as_ref()
    .map(|_| ReplayRecorder::new(1, cfg.clone(), 60.0, "match-sim".to_string()));
  if let Some(replay) = replay.as_mut() {
    replay.push_frame_with_debug(vec![state.clone()], vec![kickoff], Vec::new());
  }

  let mut command_counter: u32 = 1;
  #[cfg(feature = "viewer")]
  let mut cancelled_by_operator = false;
  while !director.is_over(&state) {
    #[cfg(feature = "viewer")]
    if let Some(v) = &viewer {
      if v.take_stop_request() {
        cancelled_by_operator = true;
        break;
      }
      if v.take_restart_request() {
        engine = SimulationEngine::new(1, cfg.clone());
        blue_ctrl =
          (bots.blue > 0).then(|| build_controller(&mc.blue, TeamColor::Blue, bots.blue as u8));
        yellow_ctrl = (bots.yellow > 0)
          .then(|| build_controller(&mc.yellow, TeamColor::Yellow, bots.yellow as u8));
        director = MatchDirector::new(cfg.clone(), duration)
          .with_bot_counts(physical_bots.blue, physical_bots.yellow);
        director.set_teleport_ball_on_no_progress(mc.teleport_ball_on_no_progress);
        evaluator = Evaluator::new(cfg.clone(), blue_name.clone(), yellow_name.clone());
        #[cfg(feature = "referris")]
        {
          referris = referris_autoref::ReferrisAutoref::new();
        }
        pickup_validator = PickupValidator::default();
        command_counter = 1;
        let kickoff = director.kickoff_reset();
        state = engine
          .step_with_commands(std::slice::from_ref(&kickoff))
          .remove(0);
        v.reset_goals();
        configure_developer_console(
          v,
          mc.dev,
          blue_ctrl.as_deref(),
          yellow_ctrl.as_deref(),
          director.teleport_ball_on_no_progress(),
        );
        attach_controller_interfaces(v, &mut blue_ctrl, &mut yellow_ctrl);
      }
      apply_developer_requests(
        v,
        mc.dev,
        &mut blue_ctrl,
        &mut yellow_ctrl,
        bots,
        &mut director,
      );
      if mc.dev {
        let current_blue_name = format!(
          "blue:{}",
          side_name(&mc.blue, blue_ctrl.as_deref(), bots.blue)
        );
        let current_yellow_name = format!(
          "yellow:{}",
          side_name(&mc.yellow, yellow_ctrl.as_deref(), bots.yellow)
        );
        evaluator.set_team_name(TeamColor::Blue, current_blue_name);
        evaluator.set_team_name(TeamColor::Yellow, current_yellow_name);
      }
      if v.apply_robot_move_requests(&mut engine) > 0 {
        state = engine.world(0).get_state();
      }
      if !v.is_running() {
        v.publish(&state);
        std::thread::sleep(std::time::Duration::from_millis(16));
        continue;
      }
    }

    let gc_blue = director.command_for(TeamColor::Blue);
    let gc_yellow = director.command_for(TeamColor::Yellow);
    let blue_cmds = blue_ctrl
      .as_mut()
      .map(|ctrl| ctrl.act(&state, &cfg, TeamColor::Blue, gc_blue))
      .unwrap_or_default();
    let yellow_cmds = yellow_ctrl
      .as_mut()
      .map(|ctrl| ctrl.act(&state, &cfg, TeamColor::Yellow, gc_yellow))
      .unwrap_or_default();
    maybe_print_commands(mc, state.sim_time, state.frame, &blue_cmds, &yellow_cmds);
    pickup_validator.maybe_validate(mc, &state, &blue_cmds, &yellow_cmds);

    let mut wc = director.update(&state);
    if let Some(scorer) = director.take_goal() {
      evaluator.record_goal(scorer);
      command_counter = command_counter.wrapping_add(1);
      if !mc.quiet {
        if let Some(ev) = &director.last_event {
          println!("  [{:6.1}s] {}", state.sim_time, ev);
        }
      }
    }
    wc.blue = blue_cmds;
    wc.yellow = yellow_cmds;

    let replay_wc = wc.clone();
    let new_state = engine.step_with_commands(&[wc]).remove(0);
    if let Some(replay) = replay.as_mut() {
      #[cfg(feature = "viewer-debug")]
      let debug = build_controller_debug_snapshot(
        new_state.world_id,
        blue_ctrl.as_deref(),
        yellow_ctrl.as_deref(),
      )
      .map(|snapshot| ReplayDebugSnapshot::from(&snapshot))
      .into_iter()
      .collect();
      #[cfg(not(feature = "viewer-debug"))]
      let debug = Vec::new();
      replay.push_frame_with_debug(vec![new_state.clone()], vec![replay_wc], debug);
    }
    evaluator.tick(&new_state, Some(&state));

    #[cfg(feature = "referris")]
    let referris_tick = referris.step(
      &new_state,
      &cfg,
      director.score,
      director.referee_command_code(),
      mc.quiet,
    );

    maybe_debug_match_state(&new_state);

    if let Some(log) = log.as_mut() {
      if new_state.frame % mc.log_every == 0 {
        #[cfg(feature = "referris")]
        let (referee_command_code, command_counter) =
          (referris_tick.command_code, referris_tick.command_counter);
        #[cfg(not(feature = "referris"))]
        let (referee_command_code, command_counter) =
          (director.referee_command_code(), command_counter);
        let _ = log.write_frame(
          &new_state,
          director.score,
          referee_command_code,
          command_counter,
        );
      }
    }
    #[cfg(feature = "viewer")]
    if let Some(v) = &viewer {
      #[cfg(feature = "referris")]
      let published_blue_name = mc
        .dev
        .then(|| {
          format!(
            "blue:{}",
            side_name(&mc.blue, blue_ctrl.as_deref(), bots.blue)
          )
        })
        .unwrap_or_else(|| blue_name.clone());
      #[cfg(feature = "referris")]
      let published_yellow_name = mc
        .dev
        .then(|| {
          format!(
            "yellow:{}",
            side_name(&mc.yellow, yellow_ctrl.as_deref(), bots.yellow)
          )
        })
        .unwrap_or_else(|| yellow_name.clone());
      #[cfg(feature = "referris")]
      v.set_game_state(simhark::viewer::GameStateInfo {
        command: referris_tick.command_label.to_string(),
        command_counter: referris_tick.command_counter,
        stage: None,
        blue_name: Some(published_blue_name),
        yellow_name: Some(published_yellow_name),
      });
      #[cfg(feature = "viewer-debug")]
      publish_controller_debug(
        v,
        new_state.world_id,
        blue_ctrl.as_deref(),
        yellow_ctrl.as_deref(),
      );
      v.publish(&new_state);
    }
    if pace {
      let delay = std::time::Duration::from_millis(16);
      #[cfg(feature = "viewer")]
      let delay = viewer
        .as_ref()
        .map_or(delay, |viewer| viewer.scaled_sleep(delay));
      std::thread::sleep(delay);
    }

    state = new_state;
  }

  if let Some(log) = log {
    let _ = log.close();
  }
  if let (Some(path), Some(replay)) = (&mc.replay, replay) {
    write_replay(path, replay.finish());
  }

  #[cfg(feature = "viewer")]
  if let Some(viewer) = &viewer {
    let lifecycle = if cancelled_by_operator {
      webinterface_protocol::SessionLifecycle::Cancelled
    } else {
      webinterface_protocol::SessionLifecycle::Completed
    };
    if let Err(error) = viewer.finish_session(lifecycle, None) {
      eprintln!("failed to finalize interface session: {error}");
    }
  }
  evaluator.finish(state.sim_time)
}

#[cfg(feature = "viewer")]
fn attach_controller_interfaces(
  viewer: &simhark::viewer::ViewerServer,
  blue: &mut Option<Box<dyn Controller>>,
  yellow: &mut Option<Box<dyn Controller>>,
) {
  let handle = viewer.interface_handle();
  let session_id = viewer.interface_session_id();
  if let Some(controller) = blue.as_deref_mut()
    && let Err(error) = controller.attach_interface(&handle, session_id)
  {
    eprintln!("failed to attach blue controller to shared interface: {error}");
  }
  if let Some(controller) = yellow.as_deref_mut()
    && let Err(error) = controller.attach_interface(&handle, session_id)
  {
    eprintln!("failed to attach yellow controller to shared interface: {error}");
  }
}

#[cfg(feature = "viewer")]
fn configure_developer_console(
  viewer: &simhark::viewer::ViewerServer,
  dev: bool,
  blue: Option<&dyn controller::Controller>,
  yellow: Option<&dyn controller::Controller>,
  teleport_ball_on_no_progress: bool,
) {
  if dev {
    let ais = hot_swappable_team_kinds()
      .into_iter()
      .map(|id| {
        serde_json::json!({
          "id": id,
          "label": id,
        })
      })
      .collect::<Vec<_>>();
    viewer.set_developer_schema(serde_json::json!({
      "id": "match-runner-dev",
      "title": "Match development",
      "description": "Replace either in-process AI while preserving the live world.",
      "matchControls": {
        "availableAis": ais,
        "blueAi": blue.map(controller::Controller::name),
        "yellowAi": yellow.map(controller::Controller::name),
        "blueDeveloperSchema": blue.and_then(controller::Controller::developer_schema),
        "yellowDeveloperSchema": yellow.and_then(controller::Controller::developer_schema),
        "teleportBallOnNoProgress": teleport_ball_on_no_progress,
      },
      "modes": [
        { "id": "blue", "label": "Blue team" },
        { "id": "yellow", "label": "Yellow team" },
      ],
      "tabs": [{
        "id": "match",
        "label": "Match",
        "icon": "schema",
        "source": {
          "kind": "inline",
          "part": {
            "kind": "empty",
            "id": "match-controls",
            "title": "Match controls",
          },
        },
      }],
      "initialTabId": "match",
    }));
    return;
  }

  let targets = [("blue", "Blue", blue), ("yellow", "Yellow", yellow)]
    .into_iter()
    .filter_map(|(id, label, controller)| {
      controller
        .and_then(controller::Controller::developer_schema)
        .map(|schema| (id, label, schema))
    })
    .collect::<Vec<_>>();
  let Some((initial_id, _, mut schema)) = targets.first().cloned() else {
    return;
  };

  schema["modes"] = serde_json::Value::Array(
    targets
      .iter()
      .map(|(id, label, _)| {
        serde_json::json!({
          "id": id,
          "label": format!("{label} team"),
          "description": format!("Invoke against the {label} Dehumanized controller"),
          "icon": "pulse",
        })
      })
      .collect(),
  );
  schema["initialModeId"] = serde_json::Value::String(initial_id.to_string());
  viewer.set_developer_schema(schema);
}

#[cfg(feature = "viewer")]
fn apply_developer_requests(
  viewer: &simhark::viewer::ViewerServer,
  dev: bool,
  blue: &mut Option<Box<dyn controller::Controller>>,
  yellow: &mut Option<Box<dyn controller::Controller>>,
  bots: TeamBotCounts,
  director: &mut MatchDirector,
) {
  let requests = viewer.take_developer_requests();
  let controls_changed = !requests.is_empty();
  for request in requests {
    let target = request.target().to_string();
    let result = match &request {
      simhark::viewer::DeveloperRequest::SwitchAi { ai, .. } if dev => {
        swap_controller(ai, &target, blue, yellow, bots)
      }
      simhark::viewer::DeveloperRequest::SetBallRecovery { enabled, .. } if dev => {
        director.set_teleport_ball_on_no_progress(*enabled);
        Ok(format!(
          "Ball teleport on no progress {}",
          if *enabled { "enabled" } else { "disabled" }
        ))
      }
      simhark::viewer::DeveloperRequest::Activate { .. }
      | simhark::viewer::DeveloperRequest::Disable { .. } => {
        let controller = match target.as_str() {
          "blue" => blue.as_deref_mut(),
          "yellow" => yellow.as_deref_mut(),
          _ => None,
        };
        controller
          .ok_or_else(|| format!("no controller is available for target {target}"))
          .and_then(|controller| controller.apply_developer_request(&request))
      }
      _ => Err("request is not available in this match mode".to_string()),
    };
    let entry = match &request {
      simhark::viewer::DeveloperRequest::Activate { entry, .. } => Some(entry.clone()),
      simhark::viewer::DeveloperRequest::SwitchAi { ai, .. } => Some(ai.clone()),
      simhark::viewer::DeveloperRequest::Disable { .. } => None,
      simhark::viewer::DeveloperRequest::SetBallRecovery { .. } => None,
    };
    viewer.set_developer_result(simhark::viewer::DeveloperResult {
      target,
      entry,
      ok: result.is_ok(),
      message: result.unwrap_or_else(|error| error),
    });
  }
  if dev && controls_changed {
    configure_developer_console(
      viewer,
      true,
      blue.as_deref(),
      yellow.as_deref(),
      director.teleport_ball_on_no_progress(),
    );
  }
}

#[cfg(feature = "viewer")]
fn swap_controller(
  ai: &str,
  target: &str,
  blue: &mut Option<Box<dyn controller::Controller>>,
  yellow: &mut Option<Box<dyn controller::Controller>>,
  bots: TeamBotCounts,
) -> Result<String, String> {
  if !hot_swappable_team_kinds().contains(&ai) {
    return Err(format!("{ai:?} is not an available in-process AI"));
  }
  let kind = TeamKind::parse(ai)?;
  let (slot, color, count) = match target {
    "blue" => (blue, TeamColor::Blue, bots.blue),
    "yellow" => (yellow, TeamColor::Yellow, bots.yellow),
    _ => return Err(format!("unknown team target: {target}")),
  };
  if count == 0 {
    return Err(format!("{target} has no active AI robots"));
  }
  *slot = Some(build_controller(&kind, color, count as u8));
  Ok(format!("{target} is now controlled by {ai}"))
}

pub(crate) fn write_replay(path: &str, replay: ReplayLog) {
  if let Some(parent) = std::path::Path::new(path)
    .parent()
    .filter(|parent| !parent.as_os_str().is_empty())
  {
    if let Err(err) = std::fs::create_dir_all(parent) {
      eprintln!(
        "failed to create replay directory {}: {err}",
        parent.display()
      );
      return;
    }
  }
  if let Err(err) = replay.write_zstd(path) {
    eprintln!("failed to write replay {path}: {err}");
  }
}

#[cfg(feature = "sim-time")]
struct FixedFirmwareDt {
  previous: Option<std::ffi::OsString>,
}

#[cfg(feature = "sim-time")]
impl FixedFirmwareDt {
  fn enable() -> Self {
    let previous = std::env::var_os("SIMHARK_FIXED_DT_MS");
    if previous.is_none() {
      unsafe {
        std::env::set_var("SIMHARK_FIXED_DT_MS", SIM_TIME_STEP_MS);
      }
    }
    Self { previous }
  }
}

#[cfg(feature = "sim-time")]
impl Drop for FixedFirmwareDt {
  fn drop(&mut self) {
    match self.previous.take() {
      Some(value) => unsafe {
        std::env::set_var("SIMHARK_FIXED_DT_MS", value);
      },
      None => unsafe {
        std::env::remove_var("SIMHARK_FIXED_DT_MS");
      },
    }
  }
}

fn maybe_debug_match_state(state: &WorldState) {
  let frame = state.frame;
  let in_requested_window = std::env::var("MATCH_DEBUG_FRAMES")
    .ok()
    .and_then(|value| parse_frame_range(&value))
    .is_some_and(|(start, end)| frame >= start && frame <= end);
  if !in_requested_window && !(std::env::var("MATCH_DEBUG").is_ok() && frame % 120 == 0) {
    return;
  }

  let bh = state.blue_robots.iter().filter(|r| r.infrared).count();
  let yh = state.yellow_robots.iter().filter(|r| r.infrared).count();
  let bd = state.blue_robots.iter().filter(|r| r.dribbler_on).count();
  let yd = state.yellow_robots.iter().filter(|r| r.dribbler_on).count();
  let (bx, by) = (state.ball.x, state.ball.y);
  let nearest = |rs: &[RobotState]| {
    rs.iter()
      .map(|r| (r.id, ((r.x - bx).powi(2) + (r.y - by).powi(2)).sqrt()))
      .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
      .unwrap_or((usize::MAX, f64::INFINITY))
  };
  let (near_blue_id, near_blue_dist) = nearest(&state.blue_robots);
  let (near_yellow_id, near_yellow_dist) = nearest(&state.yellow_robots);
  eprintln!(
    "[match] frame={} t={:.2} ball=({:.3},{:.3}) v=({:.3},{:.3}) blue_ir={bh} blue_drib={bd} yel_ir={yh} yel_drib={yd} near_b={}:{} near_y={}:{}",
    frame,
    state.sim_time,
    bx,
    by,
    state.ball.vx,
    state.ball.vy,
    near_blue_id,
    near_blue_dist,
    near_yellow_id,
    near_yellow_dist,
  );

  if in_requested_window {
    for robot in &state.yellow_robots {
      eprintln!(
        "  y#{:<2} pos=({:+.3},{:+.3}) v=({:+.3},{:+.3}) heading={:+.3} ir={} drib={}",
        robot.id,
        robot.x,
        robot.y,
        robot.vx,
        robot.vy,
        robot.orientation,
        robot.infrared,
        robot.dribbler_on,
      );
    }
  }
}

fn parse_frame_range(value: &str) -> Option<(u64, u64)> {
  let (start, end) = value.split_once(':')?;
  Some((start.parse().ok()?, end.parse().ok()?))
}

pub(crate) fn maybe_print_commands(
  mc: &MatchConfig,
  sim_time: f64,
  frame: u64,
  blue: &[RobotCommand],
  yellow: &[RobotCommand],
) {
  if !mc.print_commands && std::env::var_os("MATCH_PRINT_COMMANDS").is_none() {
    return;
  }
  let every = mc.print_commands_every.max(1);
  if frame % every != 0 {
    return;
  }

  eprintln!("[commands] t={sim_time:.2} frame={frame}");
  print_team_commands("blue", blue);
  print_team_commands("yellow", yellow);
}

#[derive(Default)]
pub(crate) struct PickupValidator {
  blue_slow_active: bool,
  yellow_slow_active: bool,
  blue_fast_active: bool,
  yellow_fast_active: bool,
  blue_warnings: u32,
  yellow_warnings: u32,
}

impl PickupValidator {
  pub(crate) fn maybe_validate(
    &mut self,
    mc: &MatchConfig,
    state: &WorldState,
    blue: &[RobotCommand],
    yellow: &[RobotCommand],
  ) {
    if !mc.validate_pickup && std::env::var_os("MATCH_VALIDATE_PICKUP").is_none() {
      return;
    }

    validate_pickup_for_team(
      state,
      TeamColor::Blue,
      blue,
      &mut self.blue_slow_active,
      &mut self.blue_fast_active,
      &mut self.blue_warnings,
    );
    validate_pickup_for_team(
      state,
      TeamColor::Yellow,
      yellow,
      &mut self.yellow_slow_active,
      &mut self.yellow_fast_active,
      &mut self.yellow_warnings,
    );
  }
}

fn validate_pickup_for_team(
  state: &WorldState,
  team: TeamColor,
  commands: &[RobotCommand],
  slow_active: &mut bool,
  fast_active: &mut bool,
  warnings: &mut u32,
) {
  let ball_speed = state.ball.vx.hypot(state.ball.vy);
  if ball_speed > 1.0 {
    *slow_active = false;
    validate_fast_pickup_for_team(state, team, commands, fast_active, warnings, ball_speed);
    return;
  }
  *fast_active = false;

  let (own, opp) = match team {
    TeamColor::Blue => (&state.blue_robots, &state.yellow_robots),
    TeamColor::Yellow => (&state.yellow_robots, &state.blue_robots),
  };
  let Some(closest) = closest_robot(own, state.ball.x, state.ball.y) else {
    *slow_active = false;
    return;
  };
  let Some(opp_closest) = closest_robot(opp, state.ball.x, state.ball.y) else {
    *slow_active = false;
    return;
  };

  let close_and_first = closest.1 <= 0.18 && opp_closest.1 >= closest.1 + 0.12;
  if !close_and_first {
    *slow_active = false;
    return;
  }

  if closest.0.infrared || command_tries_to_acquire(commands, closest.0, state.ball.x, state.ball.y)
  {
    *slow_active = false;
    return;
  }

  if !*slow_active {
    *warnings += 1;
    let command = format_optional_robot_command(commands, closest.0.id);
    eprintln!(
      "[pickup-validator] t={:.2} frame={} team={:?} robot={} dist={:.3}m ball_speed={:.2}m/s opp_dist={:.3}m command={}: close slow ball but command is not acquiring",
      state.sim_time,
      state.frame,
      team,
      closest.0.id,
      closest.1,
      ball_speed,
      opp_closest.1,
      command,
    );
  }
  *slow_active = true;
}

fn validate_fast_pickup_for_team(
  state: &WorldState,
  team: TeamColor,
  commands: &[RobotCommand],
  active: &mut bool,
  warnings: &mut u32,
  ball_speed: f64,
) {
  let (own, opp) = match team {
    TeamColor::Blue => (&state.blue_robots, &state.yellow_robots),
    TeamColor::Yellow => (&state.yellow_robots, &state.blue_robots),
  };
  let Some(candidate) = predicted_fast_pickup_candidate(state, own, opp, ball_speed) else {
    *active = false;
    return;
  };

  if command_tries_to_reach_point(
    commands,
    candidate.robot,
    candidate.target_x,
    candidate.target_y,
  ) {
    *active = false;
    return;
  }

  if !*active {
    *warnings += 1;
    let command = format_optional_robot_command(commands, candidate.robot.id);
    eprintln!(
      "[pickup-validator] t={:.2} frame={} team={:?} robot={} target=({:.3},{:.3}) lead={:.2}s dist={:.3}m ball_speed={:.2}m/s opp_dist={:.3}m command={}: fast ball predicted pickup point is reachable but command is idle",
      state.sim_time,
      state.frame,
      team,
      candidate.robot.id,
      candidate.target_x,
      candidate.target_y,
      candidate.lead_s,
      candidate.dist,
      ball_speed,
      candidate.opp_dist,
      command,
    );
  }
  *active = true;
}

struct FastPickupCandidate<'a> {
  robot: &'a RobotState,
  target_x: f64,
  target_y: f64,
  lead_s: f64,
  dist: f64,
  opp_dist: f64,
}

fn predicted_fast_pickup_candidate<'a>(
  state: &WorldState,
  own: &'a [RobotState],
  opp: &[RobotState],
  ball_speed: f64,
) -> Option<FastPickupCandidate<'a>> {
  let max_lead = (0.90 / ball_speed).clamp(0.30, 0.85);
  let mut lead = 0.15;

  while lead <= max_lead {
    let target_x = (state.ball.x + state.ball.vx * lead).clamp(-0.5, 0.5);
    let target_y = (state.ball.y + state.ball.vy * lead).clamp(-0.5, 0.5);
    let Some(closest) = closest_robot(own, target_x, target_y) else {
      return None;
    };
    let Some(opp_closest) = closest_robot(opp, target_x, target_y) else {
      return None;
    };

    let reachable_dist = (0.16 + 0.55 * lead).min(0.42);
    if closest.1 <= reachable_dist && opp_closest.1 >= closest.1 + 0.15 {
      return Some(FastPickupCandidate {
        robot: closest.0,
        target_x,
        target_y,
        lead_s: lead,
        dist: closest.1,
        opp_dist: opp_closest.1,
      });
    }

    lead += 1.0 / 30.0;
  }

  None
}

fn closest_robot(robots: &[RobotState], ball_x: f64, ball_y: f64) -> Option<(&RobotState, f64)> {
  robots
    .iter()
    .filter(|robot| robot.is_on)
    .map(|robot| (robot, (robot.x - ball_x).hypot(robot.y - ball_y)))
    .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
}

fn command_tries_to_acquire(
  commands: &[RobotCommand],
  robot: &RobotState,
  ball_x: f64,
  ball_y: f64,
) -> bool {
  commands
    .iter()
    .find(|command| command.id == robot.id)
    .is_some_and(|command| {
      command.dribbler_on && command_moves_toward_point(command, robot, ball_x, ball_y, 0.05)
    })
}

fn command_tries_to_reach_point(
  commands: &[RobotCommand],
  robot: &RobotState,
  target_x: f64,
  target_y: f64,
) -> bool {
  commands
    .iter()
    .find(|command| command.id == robot.id)
    .is_some_and(|command| command_moves_toward_point(command, robot, target_x, target_y, 0.10))
}

fn command_moves_toward_point(
  command: &RobotCommand,
  robot: &RobotState,
  target_x: f64,
  target_y: f64,
  close_enough: f64,
) -> bool {
  let dx = target_x - robot.x;
  let dy = target_y - robot.y;
  let dist = dx.hypot(dy);
  if dist <= close_enough {
    return command_is_active(command);
  }

  let Some((vx, vy)) = command_world_velocity(command, robot) else {
    return false;
  };
  let speed = vx.hypot(vy);
  if speed < 0.03 {
    return false;
  }

  let closing_speed = (vx * dx + vy * dy) / dist;
  closing_speed >= 0.03 || closing_speed >= speed * 0.35
}

fn command_world_velocity(command: &RobotCommand, robot: &RobotState) -> Option<(f64, f64)> {
  match command.move_command.as_ref()? {
    MoveCommand::GlobalVelocity { vx, vy, .. } => Some((*vx, *vy)),
    MoveCommand::LocalVelocity { forward, left, .. } => {
      let (sin, cos) = robot.orientation.sin_cos();
      Some((forward * cos - left * sin, forward * sin + left * cos))
    }
    MoveCommand::WheelVelocity(_) => None,
  }
}

fn side_name(kind: &TeamKind, ctrl: Option<&dyn controller::Controller>, bots: usize) -> String {
  if bots == 0 {
    return format!("{}:0bots", kind.label());
  }
  match ctrl {
    Some(c) => c.name().to_string(),
    None => kind.label().to_string(),
  }
}

#[cfg(feature = "viewer-debug")]
pub(crate) fn publish_controller_debug(
  viewer: &simhark::viewer::ViewerServer,
  world_id: usize,
  blue: Option<&dyn controller::Controller>,
  yellow: Option<&dyn controller::Controller>,
) {
  let Some(snapshot) = build_controller_debug_snapshot(world_id, blue, yellow) else {
    viewer.clear_debug_snapshot(world_id);
    return;
  };
  viewer.set_debug_snapshot(snapshot);
}

#[cfg(feature = "viewer-debug")]
pub(crate) fn build_controller_debug_snapshot(
  world_id: usize,
  blue: Option<&dyn controller::Controller>,
  yellow: Option<&dyn controller::Controller>,
) -> Option<simhark::viewer::ViewerDebugSnapshot> {
  let snapshots = [blue, yellow]
    .into_iter()
    .flatten()
    .filter_map(controller::Controller::debug_snapshot)
    .collect::<Vec<_>>();

  if snapshots.is_empty() {
    return None;
  }

  let strategy = snapshots
    .iter()
    .filter_map(|snapshot| snapshot.strategy.as_deref())
    .filter(|message| !message.is_empty())
    .collect::<Vec<_>>()
    .join(" | ");
  let robots = snapshots
    .into_iter()
    .flat_map(|snapshot| snapshot.robots)
    .collect();
  let overlays = [blue, yellow]
    .into_iter()
    .flatten()
    .filter_map(controller::Controller::debug_snapshot)
    .flat_map(|snapshot| snapshot.overlays)
    .collect();

  Some(simhark::viewer::ViewerDebugSnapshot {
    world_id,
    strategy: (!strategy.is_empty()).then_some(strategy),
    robots,
    overlays,
  })
}

fn format_optional_robot_command(commands: &[RobotCommand], robot_id: usize) -> String {
  commands
    .iter()
    .find(|command| command.id == robot_id)
    .map(format_robot_command)
    .unwrap_or_else(|| "<missing>".to_string())
}

fn print_team_commands(team: &str, commands: &[RobotCommand]) {
  let active: Vec<String> = commands
    .iter()
    .filter(|command| command_is_active(command))
    .map(format_robot_command)
    .collect();

  if active.is_empty() {
    eprintln!("  {team}: <none>");
  } else {
    eprintln!("  {team}: {}", active.join(" | "));
  }
}

fn command_is_active(command: &RobotCommand) -> bool {
  command.move_command.is_some() || command.kick_speed.abs() > 1e-6 || command.dribbler_on
}

fn format_robot_command(command: &RobotCommand) -> String {
  let motion = command
    .move_command
    .as_ref()
    .map(format_move_command)
    .unwrap_or_else(|| "hold".to_string());
  let mut parts = vec![format!("#{} {motion}", command.id)];
  if command.dribbler_on {
    parts.push("drib".to_string());
  }
  if command.kick_speed.abs() > 1e-6 {
    parts.push(format!(
      "kick={:.2}m/s@{:.0}deg",
      command.kick_speed, command.kick_angle
    ));
  }
  parts.join(" ")
}

fn format_move_command(command: &MoveCommand) -> String {
  match command {
    MoveCommand::LocalVelocity {
      forward,
      left,
      angular,
    } => format!("local f={forward:.2} l={left:.2} w={angular:.2}"),
    MoveCommand::GlobalVelocity { vx, vy, angular } => {
      format!("global vx={vx:.2} vy={vy:.2} w={angular:.2}")
    }
    MoveCommand::WheelVelocity(wheels) => format!(
      "wheels [{:.1},{:.1},{:.1},{:.1}]",
      wheels[0], wheels[1], wheels[2], wheels[3]
    ),
  }
}
