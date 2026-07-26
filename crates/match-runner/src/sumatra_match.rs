//! Hybrid match path: one (or both) sides are the **real Sumatra**, the
//! external Java AI, driven over the SimNet protocol; any non-external side is a
//! `Faabs` controller running in-process (e.g. our `bangka`).
//!
//! simhark stays the single source of truth for the world. Each tick we:
//!   1. build the in-process side's commands (+ the director's kickoff teleports),
//!   2. hand them to `SumatraSimNetServer::step_with_local_commands`, which
//!      publishes the world to Sumatra, merges Sumatra's commands for the
//!      team(s) it registered, and steps the engine once.
//!
//! Scoring/goal-detection/kickoffs reuse the same `MatchDirector` + `Evaluator`
//! as the pure in-process path, so reports are directly comparable.

use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use simhark::{
  ReplayDebugSnapshot, ReplayRecorder, SimulationEngine, TeamColor, WorldCommand, WorldState,
};
use simhark_sumatra::{
  SumatraInstance, SumatraLaunchConfig, SumatraSimNetConfig, SumatraSimNetServer,
};

use crate::controller::{Controller, TeamKind, build_controller};
use crate::director::MatchDirector;
use crate::evaluator::{Evaluator, MatchReport};
use crate::logio::GameLog;
#[cfg(feature = "viewer-debug")]
use crate::publish_controller_debug;
use crate::{MatchConfig, PickupValidator, maybe_print_commands, world_config};

/// How long to wait for the Sumatra JVM(s) to connect before giving up.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(40);
/// Wall-clock pacing per tick. Sumatra runs in real time, so we feed it states
/// at ~60 Hz rather than as fast as the simulator can go.
const TICK: Duration = Duration::from_millis(16);

pub fn run(mc: &MatchConfig) -> MatchReport {
  match try_run(mc) {
    Ok(report) => report,
    Err(e) => {
      eprintln!("sumatra match failed: {e:#}");
      // Produce an empty 0-0 report so callers still get a structured result.
      let cfg = world_config(mc.div, mc.seed);
      Evaluator::new(
        cfg,
        format!("blue:{}", mc.blue.label()),
        format!("yellow:{}", mc.yellow.label()),
      )
      .finish(0.0)
    }
  }
}

fn try_run(mc: &MatchConfig) -> Result<MatchReport> {
  let mut cfg = world_config(mc.div, mc.seed);
  let default_bots = cfg.robots_per_team;
  let bots = mc.bot_counts(default_bots);
  let physical_bots = mc.physical_bot_counts(default_bots);
  cfg.robots_per_team = physical_bots.blue.max(physical_bots.yellow);

  // Any in-process side runs through CrashPilot, whose GameState holds at most
  // 8 robots per team. Division A has 11, so reject it with a clear message
  // rather than letting CrashPilot panic on an out-of-bounds robot id.
  let has_crashpilot_bound_side = (bots.blue > 0 && mc.blue.uses_crashpilot_binding())
    || (bots.yellow > 0 && mc.yellow.uses_crashpilot_binding());
  if has_crashpilot_bound_side && bots.blue.max(bots.yellow) > 8 {
    anyhow::bail!(
      "division {} has {} robots/team, but the in-process AI (CrashPilot) supports at most 8. Use --div b or --blue-bots/--yellow-bots <= 8 for matches against Sumatra.",
      mc.div,
      bots.blue.max(bots.yellow),
    );
  }

  let mut engine = SimulationEngine::new(1, cfg.clone());

  // In-process controllers for any non-external side.
  let mut blue_ctrl: Option<Box<dyn Controller>> = (bots.blue > 0 && !mc.blue.is_external())
    .then(|| build_controller(&mc.blue, TeamColor::Blue, bots.blue as u8));
  let mut yellow_ctrl: Option<Box<dyn Controller>> = (bots.yellow > 0 && !mc.yellow.is_external())
    .then(|| build_controller(&mc.yellow, TeamColor::Yellow, bots.yellow as u8));

  let blue_name = format!(
    "blue:{}",
    side_name(&mc.blue, blue_ctrl.as_deref(), bots.blue)
  );
  let yellow_name = format!(
    "yellow:{}",
    side_name(&mc.yellow, yellow_ctrl.as_deref(), bots.yellow)
  );

  // Bind the SimNet server, then launch one Sumatra JVM per external side.
  let mut server = SumatraSimNetServer::bind(SumatraSimNetConfig::default())
    .context("bind Sumatra SimNet server")?;
  let mut instances = Vec::new();
  for (kind, color, count) in [
    (&mc.blue, TeamColor::Blue, bots.blue),
    (&mc.yellow, TeamColor::Yellow, bots.yellow),
  ] {
    if count > 0 && kind.is_external() {
      let inst = SumatraInstance::spawn(&SumatraLaunchConfig {
        remote_client: true,
        ai_blue: color == TeamColor::Blue,
        ai_yellow: color == TeamColor::Yellow,
        host: Some("127.0.0.1".to_string()),
        ..SumatraLaunchConfig::default()
      })
      .with_context(|| format!("spawn Sumatra for {color:?}"))?;
      instances.push(inst);
    }
  }

  if !mc.quiet {
    println!("waiting for Sumatra to connect ({blue_name} vs {yellow_name})...");
  }
  // Warm up: pump the server (publishing world snapshots) until Sumatra
  // connects, then reset the world so the match starts cleanly at t=0.
  let warm = Instant::now();
  loop {
    let _ = server.step_with_local_commands(&mut engine, &[WorldCommand::default()]);
    if server.has_clients().unwrap_or(false) {
      break;
    }
    if warm.elapsed() > CONNECT_TIMEOUT {
      anyhow::bail!("timed out waiting for Sumatra to connect on SimNet");
    }
    if instances
      .iter_mut()
      .any(|i| matches!(i.try_wait(), Ok(Some(_))))
    {
      anyhow::bail!("a Sumatra process exited before connecting");
    }
    std::thread::sleep(Duration::from_millis(50));
  }
  // Fresh world for the actual match (clients stay connected to the server).
  engine = SimulationEngine::new(1, cfg.clone());
  server.reset_tracking();
  if !mc.quiet {
    println!("Sumatra connected; kickoff.");
  }

  let mut director = MatchDirector::new(cfg.clone(), mc.seconds)
    .with_bot_counts(physical_bots.blue, physical_bots.yellow);
  let mut evaluator = Evaluator::new(cfg.clone(), blue_name.clone(), yellow_name.clone());
  #[cfg(feature = "referris")]
  let mut referris = crate::referris_autoref::ReferrisAutoref::new();
  let mut pickup_validator = PickupValidator::default();

  let mut log = match &mc.log {
    Some(path) => GameLog::create(path, &cfg, &blue_name, &yellow_name).ok(),
    None => None,
  };

  #[cfg(feature = "viewer")]
  let viewer = if mc.viewer {
    let vc = simhark::viewer::ViewerConfig::default();
    match simhark::viewer::ViewerServer::bind(vc, 1, &cfg) {
      Ok(v) => {
        println!("viewer: {}", vc.http_url());
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

  let kickoff = director.kickoff_reset();
  let mut state =
    pop_state(server.step_with_local_commands(&mut engine, std::slice::from_ref(&kickoff))?)
      .context("first step produced no state")?;
  let mut replay = mc
    .replay
    .as_ref()
    .map(|_| ReplayRecorder::new(1, cfg.clone(), 60.0, "match-sim sumatra".to_string()));
  if let Some(replay) = replay.as_mut() {
    replay.push_frame_with_debug(vec![state.clone()], vec![kickoff], Vec::new());
  }

  let mut command_counter: u32 = 1;
  while !director.is_over(&state) {
    #[cfg(feature = "viewer")]
    if let Some(v) = &viewer
      && v.apply_robot_move_requests(&mut engine) > 0
    {
      state = engine.world(0).get_state();
    }

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

    // In-process sides fill their command vec; external sides are left empty
    // and filled by Sumatra's merged actions inside step_with_local_commands.
    if let Some(ctrl) = blue_ctrl.as_mut() {
      let gc = director.command_for(TeamColor::Blue);
      wc.blue = ctrl.act(&state, &cfg, TeamColor::Blue, gc);
    }
    if let Some(ctrl) = yellow_ctrl.as_mut() {
      let gc = director.command_for(TeamColor::Yellow);
      wc.yellow = ctrl.act(&state, &cfg, TeamColor::Yellow, gc);
    }
    maybe_print_commands(mc, state.sim_time, state.frame, &wc.blue, &wc.yellow);
    pickup_validator.maybe_validate(mc, &state, &wc.blue, &wc.yellow);

    let replay_wc = wc.clone();
    let new_state = match pop_state(server.step_with_local_commands(&mut engine, &[wc])?) {
      Some(s) => s,
      None => break,
    };
    if let Some(replay) = replay.as_mut() {
      #[cfg(feature = "viewer-debug")]
      let debug = crate::build_controller_debug_snapshot(
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
      v.set_game_state(simhark::viewer::GameStateInfo {
        command: referris_tick.command_label.to_string(),
        command_counter: referris_tick.command_counter,
        stage: None,
        blue_name: Some(blue_name.clone()),
        yellow_name: Some(yellow_name.clone()),
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

    // Detect a Sumatra crash so we do not silently play against a dead AI.
    if instances
      .iter_mut()
      .any(|i| matches!(i.try_wait(), Ok(Some(_))))
    {
      eprintln!(
        "warning: a Sumatra process exited mid-match at {:.1}s",
        new_state.sim_time
      );
      break;
    }

    std::thread::sleep(TICK);
    state = new_state;
  }

  if let Some(log) = log {
    let _ = log.close();
  }
  if let (Some(path), Some(replay)) = (&mc.replay, replay) {
    crate::write_replay(path, replay.finish());
  }
  // Instances are killed on drop.
  Ok(evaluator.finish(state.sim_time))
}

fn pop_state(mut states: Vec<WorldState>) -> Option<WorldState> {
  if states.is_empty() {
    None
  } else {
    Some(states.remove(0))
  }
}

fn side_name(kind: &TeamKind, ctrl: Option<&dyn Controller>, bots: usize) -> String {
  if bots == 0 {
    return format!("{}:0bots", kind.label());
  }
  match ctrl {
    Some(c) => c.name().to_string(),
    None => kind.label().to_string(),
  }
}
