//! Optional match path. Team AIs receive their delivered tracker/telemetry;
//! officials receive an independent tracker. Truth is used only by evaluation
//! and the debug viewer. Normal run_match is compiled without this module.
use crate::{
  MatchConfig,
  controller::{GameCommand, build_controller},
  director::MatchDirector,
  evaluator::{Evaluator, MatchReport},
  logio::GameLog,
};
#[cfg(feature = "sumatra")]
use anyhow::Context;
use anyhow::Result;
use simhark::{
  TeamColor, WorldCommand, WorldState,
  realistic::{RealisticConfig, RealisticWorld, live::LivePacer, transport::Safety},
};

pub fn run(mc: &MatchConfig, profile: RealisticConfig) -> Result<MatchReport> {
  #[cfg(feature = "sim-time")]
  let _fixed = super::FixedFirmwareDt::enable();
  let ai_rate = profile.ai_rate_hz;
  anyhow::ensure!(
    !mc.dev,
    "realistic matches use fixed profile ownership; live profile changes need a new recorded run"
  );
  anyhow::ensure!(
    mc.seconds.is_finite() && mc.seconds > 0.0,
    "invalid match duration"
  );
  let mut cfg = mc.world_config();
  let bots = mc.bot_counts(cfg.robots_per_team);
  let physical = mc.physical_bot_counts(cfg.robots_per_team);
  cfg.robots_per_team = physical.blue.max(physical.yellow);
  let counts = [bots.blue, bots.yellow];
  let kinds = [&mc.blue, &mc.yellow];
  let colors = [TeamColor::Blue, TeamColor::Yellow];
  let mut controllers = std::array::from_fn::<_, 2, _>(|i| {
    (counts[i] > 0 && !kinds[i].is_external())
      .then(|| build_controller(kinds[i], colors[i], counts[i] as u8))
  });
  let names = std::array::from_fn::<_, 2, _>(|i| format!("{:?}:{}", colors[i], kinds[i].label()));
  let mut evaluator = Evaluator::new(cfg.clone(), names[0].clone(), names[1].clone());
  let mut sim = RealisticWorld::new(0, cfg.clone(), profile).map_err(anyhow::Error::msg)?;
  if mc.replay.is_some() {
    sim.enable_recording();
  }
  let mut director =
    MatchDirector::new(cfg.clone(), mc.seconds).with_bot_counts(physical.blue, physical.yellow);
  director.set_teleport_ball_on_no_progress(mc.teleport_ball_on_no_progress);
  // Formation is declared match setup, independently of observations.
  sim.teleport(&director.kickoff_reset());
  let mut log = mc
    .log
    .as_ref()
    .map(|p| GameLog::create(p, &cfg, &names[0], &names[1]))
    .transpose()?;
  #[cfg(feature = "referris")]
  let mut autoref = crate::referris_autoref::ReferrisAutoref::new();
  #[cfg(feature = "viewer")]
  let viewer = if mc.viewer {
    Some(simhark::viewer::ViewerServer::bind(
      Default::default(),
      1,
      &cfg,
    )?)
  } else {
    None
  };
  let external = kinds
    .iter()
    .zip(counts)
    .any(|(k, n)| n > 0 && k.is_external());
  #[cfg(not(feature = "sumatra"))]
  anyhow::ensure!(
    !external,
    "external Sumatra requires the sumatra build feature"
  );
  #[cfg(feature = "sumatra")]
  let mut peers = Vec::new();
  #[cfg(feature = "sumatra")]
  for i in 0..2 {
    if counts[i] > 0 && kinds[i].is_external() {
      let port = 14242 + i as u16;
      let server =
        simhark_sumatra::SumatraSimNetServer::bind(simhark_sumatra::SumatraSimNetConfig {
          bind_addr: format!("127.0.0.1:{port}").parse()?,
        })?;
      let child = simhark_sumatra::SumatraInstance::spawn(&simhark_sumatra::SumatraLaunchConfig {
        remote_client: true,
        ai_blue: i == 0,
        ai_yellow: i == 1,
        host: Some("127.0.0.1".into()),
        sim_net_port: Some(port),
        ..Default::default()
      })
      .context("start actual Sumatra client")?;
      peers.push((i, server, child));
    }
  }
  #[cfg(feature = "sumatra")]
  if external {
    let start = std::time::Instant::now();
    loop {
      let ready = peers
        .iter_mut()
        .map(|(_, s, _)| s.has_clients())
        .collect::<std::io::Result<Vec<_>>>()?;
      if ready.iter().all(|v| *v) {
        break;
      }
      anyhow::ensure!(
        start.elapsed().as_secs() < 40,
        "Sumatra connection timed out"
      );
      for (_, _, child) in &mut peers {
        anyhow::ensure!(
          child.try_wait()?.is_none(),
          "Sumatra exited before connecting"
        );
      }
      std::thread::sleep(std::time::Duration::from_millis(10));
    }
  }
  let paced = mc.realtime || mc.viewer || external;
  let pacer = LivePacer::new(0);
  let end = (mc.seconds * 1e9).round() as u64;
  let mut frame = 0;
  let mut previous = sim.truth();
  let mut epoch = 1;
  let mut last_safety = Safety::Run;
  while sim.now() < end {
    let states = colors.map(|c| team_state(&sim, c, frame, &cfg));
    let officials = sim.official_observations().legacy_state(0, frame, &cfg);
    let mut commands = WorldCommand::default();
    for i in 0..2 {
      if let (Some(controller), Some(observed)) = (&mut controllers[i], &states[i]) {
        let result = controller.act(observed, &cfg, colors[i], director.command_for(colors[i]));
        if i == 0 {
          commands.blue = result;
        } else {
          commands.yellow = result;
        }
      }
    }
    #[cfg(feature = "sumatra")]
    for (i, server, _) in &mut peers {
      if let Some(observed) = &states[*i] {
        let remote = server.exchange_observed(observed, &cfg)?;
        if *i == 0 {
          commands.blue = remote.blue;
        } else {
          commands.yellow = remote.yellow;
        }
      }
    }
    if let Some(observed) = &officials {
      let setup = director.update(observed);
      if setup.teleport_ball.is_some() || !setup.teleport_robots.is_empty() {
        sim.teleport(&setup);
      }
      if let Some(team) = director.take_goal() {
        evaluator.record_goal(team);
      }
      #[cfg(feature = "referris")]
      let _ = autoref.step(
        observed,
        &cfg,
        director.score,
        director.referee_command_code(),
        mc.quiet,
      );
      if let Some(log) = &mut log {
        if frame % mc.log_every.max(1) == 0 {
          log.write_frame(
            observed,
            director.score,
            director.referee_command_code(),
            frame as u32,
          )?;
        }
      }
    }
    let safety = if colors
      .iter()
      .all(|c| director.command_for(*c) == GameCommand::Halt)
    {
      Safety::Halt
    } else {
      Safety::Run
    };
    if safety != last_safety {
      epoch += 1;
      last_safety = safety;
    }
    sim
      .submit_decision(commands, sim.now(), epoch, safety)
      .map_err(anyhow::Error::msg)?;
    frame += 1;
    let next = ((frame as u128 * 1_000_000_000 / u128::from(ai_rate)) as u64).min(end);
    sim.advance_to(next).map_err(anyhow::Error::msg)?;
    let truth = sim.truth();
    evaluator.tick(&truth, Some(&previous));
    #[cfg(feature = "viewer")]
    if let Some(v) = &viewer {
      v.publish(&truth);
    }
    previous = truth;
    if paced {
      pacer.pace(sim.now());
    }
  }
  if let Some(path) = &mc.replay {
    let binary = std::fs::read(std::env::current_exe()?)?;
    let identity = format!(
      "match-runner executable {}",
      simhark::realistic::calibration::digest(&binary).map_err(anyhow::Error::msg)?
    );
    serde_json::to_writer(
      std::io::BufWriter::new(std::fs::File::create(path)?),
      &sim.recording(identity),
    )?;
  }
  if !mc.quiet {
    println!(
      "realistic diagnostics: {}",
      serde_json::to_string(sim.diagnostics())?
    );
  }
  Ok(evaluator.finish(sim.now() as f64 * 1e-9))
}
fn team_state(
  sim: &RealisticWorld,
  color: TeamColor,
  frame: u64,
  cfg: &simhark::WorldConfig,
) -> Option<WorldState> {
  let mut state = sim.team_observations(color).legacy_state(0, frame, cfg)?;
  let robots = if color == TeamColor::Blue {
    &mut state.blue_robots
  } else {
    &mut state.yellow_robots
  };
  for robot in robots {
    if let Some((_, Some(ir), _)) = sim.host_feedback(color, robot.id) {
      robot.infrared = ir.detected && sim.now().saturating_sub(ir.sampled_at) <= 100_000_000;
    }
  }
  Some(state)
}
