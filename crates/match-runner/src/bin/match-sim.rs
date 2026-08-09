//! `match-sim` — run an AI-vs-AI RoboCup SSL match inside simhark, score it
//! RL-style, and optionally write an SSL log for Loguna.
//!
//! Examples:
//! ```text
//! match-sim --blue bangka --yellow sumatra --div b --seconds 60 --log game.log
//! match-sim --blue bangka --yellow bangka --div b --matches 3
//! ```

use match_runner::controller::TeamKind;
use match_runner::evaluator::MatchReport;
use match_runner::{MatchConfig, run_match};
use simhark::config::MAX_ROBOTS_PER_TEAM;

const DEFAULT_REPLAY_PATH: &str = "__simhark_default_replay_path__";

struct Args {
  mc: MatchConfig,
  matches: usize,
  summary: Option<String>,
}

fn parse() -> Result<Args, String> {
  let mut mc = MatchConfig::default();
  let mut matches = 1;
  let mut summary = None;
  let log_base: Option<String>;
  let mut log = None;

  let mut it = std::env::args().skip(1).peekable();
  while let Some(a) = it.next() {
    match a.as_str() {
      "--blue" => mc.blue = TeamKind::parse(&next_arg(&mut it, &a)?)?,
      "--yellow" => mc.yellow = TeamKind::parse(&next_arg(&mut it, &a)?)?,
      "--blue-bots" => mc.blue_bots = Some(parse_bots(&next_arg(&mut it, &a)?, "--blue-bots")?),
      "--yellow-bots" => {
        mc.yellow_bots = Some(parse_bots(&next_arg(&mut it, &a)?, "--yellow-bots")?)
      }
      "--seconds" => {
        mc.seconds = next_arg(&mut it, &a)?
          .parse()
          .map_err(|_| "bad --seconds")?
      }
      "--div" => mc.div = next_arg(&mut it, &a)?.chars().next().unwrap_or('b'),
      "--seed" => mc.seed = next_arg(&mut it, &a)?.parse().map_err(|_| "bad --seed")?,
      "--matches" => {
        matches = next_arg(&mut it, &a)?
          .parse()
          .map_err(|_| "bad --matches")?
      }
      "--log" => log = Some(next_arg(&mut it, &a)?),
      "--replay" => {
        mc.replay =
          Some(optional_path_arg(&mut it).unwrap_or_else(|| DEFAULT_REPLAY_PATH.to_string()))
      }
      "--record-interface" => {
        mc.interface_recording = true;
        mc.viewer = true;
      }
      "--summary" => summary = Some(next_arg(&mut it, &a)?),
      "--log-every" => {
        mc.log_every = next_arg(&mut it, &a)?
          .parse()
          .map_err(|_| "bad --log-every")?
      }
      "--print-commands" => mc.print_commands = true,
      "--print-commands-every" => {
        mc.print_commands_every = next_arg(&mut it, &a)?
          .parse()
          .map_err(|_| "bad --print-commands-every")?
      }
      "--validate-pickup" => mc.validate_pickup = true,
      "--viewer" => mc.viewer = true,
      "--realtime" => mc.realtime = true,
      "--dev" => {
        #[cfg(not(feature = "viewer"))]
        return Err("--dev requires building match-runner with `--features viewer`".to_string());
        #[cfg(feature = "viewer")]
        {
          mc.dev = true;
          mc.viewer = true;
          mc.realtime = true;
          // Dev matches start without ball recovery; toggle it in the dev console.
          mc.teleport_ball_on_no_progress = false;
        }
      }
      "--quiet" => mc.quiet = true,
      "-h" | "--help" => {
        print_help();
        std::process::exit(0);
      }
      other => return Err(format!("unknown argument: {other}")),
    }
  }
  log_base = log;
  mc.log = log_base;
  if mc.dev && mc.replay.is_some() {
    return Err("--dev cannot be combined with --replay".to_string());
  }
  if mc.dev && matches != 1 {
    return Err("--dev cannot be combined with --matches".to_string());
  }
  if mc.dev && (mc.blue.is_external() || mc.yellow.is_external()) {
    return Err("--dev supports in-process AIs only; Sumatra cannot be hot-swapped".to_string());
  }
  if mc.replay.as_deref() == Some(DEFAULT_REPLAY_PATH) {
    mc.replay = Some(default_replay_path(&mc));
  }
  Ok(Args {
    mc,
    matches,
    summary,
  })
}

fn next_arg<I>(it: &mut std::iter::Peekable<I>, flag: &str) -> Result<String, String>
where
  I: Iterator<Item = String>,
{
  it.next().ok_or_else(|| format!("missing value for {flag}"))
}

fn optional_path_arg<I>(it: &mut std::iter::Peekable<I>) -> Option<String>
where
  I: Iterator<Item = String>,
{
  it.next_if(|value| !value.starts_with('-'))
}

fn parse_bots(value: &str, flag: &str) -> Result<usize, String> {
  let bots = value.parse().map_err(|_| format!("bad {flag}"))?;
  if bots > MAX_ROBOTS_PER_TEAM {
    return Err(format!(
      "{flag} must be <= {MAX_ROBOTS_PER_TEAM}, got {bots}"
    ));
  }
  Ok(bots)
}

fn print_help() {
  println!(
    "match-sim — AI vs AI RoboCup SSL match in simhark\n\
\n\
Options:\n\
  --blue   <kind>   team controlling blue   (default bangka)\n\
  --yellow <kind>   team controlling yellow (default bangka)\n\
  --blue-bots <n>   blue AI robot count; 0 disables blue AI (default from --div)\n\
  --yellow-bots <n> yellow AI robot count; 0 disables yellow AI (default from --div)\n\
  --seconds <f>     match length in sim seconds (default 60)\n\
  --div <a|b>       division / field+robot count (default b)\n\
  --seed <u>        RNG seed (default 1)\n\
  --matches <n>     play n matches (seeds seed..seed+n) and aggregate\n\
  --log <path>      write SSL log file (Loguna-compatible)\n\
  --replay [path]   write native simhark replay file (.shreplay); with --viewer, serve it afterward\n\
  --record-interface record canonical state/events to .faabsrec (implies --viewer)\n\
  --summary <path>  append one JSON summary line per run\n\
  --log-every <n>   log every n-th frame (default 2)\n\
  --print-commands  print simulator robot commands to stderr\n\
  --print-commands-every <n> frame interval for command printing (default 60)\n\
  --validate-pickup warn if close slow pickup or fast predicted pickup is neglected\n\
  --viewer          open the live web viewer (build with --features viewer)\n\
  --realtime        pace the sim to ~60Hz wall-clock\n\
  --dev             unlimited live match with AI hot-swap and ball-recovery controls (recovery off by default)\n\
  --quiet           less stdout\n\
\n\
Team kinds: bangka | bongka[:params.json] | ungabunga[:params.json] | crashpilot[:model.safetensors] | dummy | sumatra (real, external JVM)\n\
\n\
'crashpilot' defaults to /run/media/shark/data/dev/robocup/ai/crashpilot.safetensors.\n\
'dummy' keeps that team's robots idle with zero wheel velocity.\n\
'sumatra' launches the real Sumatra over SimNet and runs in real time. Use\n\
--dev supports in-process AIs only; Sumatra is not a hot-swap target.\n\
--div b: the in-process AI (CrashPilot) supports at most 8 robots/team."
  );
}

fn print_report(report: &MatchReport, quiet: bool) {
  println!(
    "\n=== Final: {} {} - {} {}  | winner: {} ===",
    report.blue.name,
    report.blue.metrics.goals_for,
    report.yellow.metrics.goals_for,
    report.yellow.name,
    report.winner,
  );
  if quiet {
    return;
  }
  for t in [&report.blue, &report.yellow] {
    println!(
      "  {:<22} score={:+7.2}  poss={:>4.0}%  shots={}({} on target)  progress={:.1}m",
      t.name,
      t.score,
      t.possession_pct,
      t.metrics.shots,
      t.metrics.shots_on_target,
      t.metrics.ball_progress,
    );
    for n in &t.notes {
      println!("      - {n}");
    }
  }
}

fn append_summary(path: &str, report: &MatchReport) {
  use std::io::Write;
  if let Ok(mut f) = std::fs::OpenOptions::new()
    .create(true)
    .append(true)
    .open(path)
  {
    if let Ok(line) = serde_json::to_string(report) {
      let _ = writeln!(f, "{line}");
    }
  }
}

#[tokio::main]
async fn main() {
  if std::env::args_os().len() == 1 {
    if let Err(error) = serve_empty_start_center().await {
      eprintln!("failed to start interface: {error:#}");
      std::process::exit(1);
    }
    return;
  }

  let args = match parse() {
    Ok(a) => a,
    Err(e) => {
      eprintln!("error: {e}\n");
      print_help();
      std::process::exit(2);
    }
  };

  let (mut blue_total, mut yellow_total) = (0.0, 0.0);
  let (mut blue_goals, mut yellow_goals) = (0u32, 0u32);
  let mut replay_to_serve = None::<String>;

  for i in 0..args.matches {
    if !args.mc.quiet && args.matches > 1 {
      println!("--- match {}/{} ---", i + 1, args.matches);
    }
    let mut mc = args.mc.clone();
    mc.seed = args.mc.seed.wrapping_add(i as u64);
    if let (Some(base), true) = (&args.mc.log, args.matches > 1) {
      mc.log = Some(base.replace(".log", &format!("_{i}.log")));
    }
    if let (Some(base), true) = (&args.mc.replay, args.matches > 1) {
      mc.replay = Some(suffix_path(base, i));
    }
    if let Some(path) = &mc.replay {
      replay_to_serve = Some(path.clone());
    }
    let report = run_match(&mc);
    blue_total += report.blue.score;
    yellow_total += report.yellow.score;
    blue_goals += report.blue.metrics.goals_for;
    yellow_goals += report.yellow.metrics.goals_for;
    print_report(&report, args.mc.quiet);
    if let Some(path) = &args.summary {
      append_summary(path, &report);
    }
  }

  if args.matches > 1 {
    let n = args.matches as f64;
    println!(
      "\n=== Aggregate over {} matches ===\n  blue   avg {:+.2} (goals {})\n  yellow avg {:+.2} (goals {})",
      args.matches,
      blue_total / n,
      blue_goals,
      yellow_total / n,
      yellow_goals,
    );
  }

  if args.mc.viewer
    && let Some(path) = replay_to_serve
  {
    serve_replay(&path);
  }
}

async fn serve_empty_start_center() -> anyhow::Result<()> {
  let port = std::env::var("SIMHARK_VIEWER_PORT")
    .ok()
    .and_then(|value| value.parse().ok())
    .unwrap_or(8315);
  loop {
    let (guard, handle) =
      webinterface_core::InterfaceHost::start(webinterface_core::InterfaceConfig {
        bind_address: ([0, 0, 0, 0], port).into(),
        assets: webinterface_assets::embedded_assets(),
        ..webinterface_core::InterfaceConfig::default()
      })?;
    let mut launcher = handle.register_system(webinterface_protocol::SystemDescriptor {
      id: "match-runner".into(),
      label: "Match Runner".into(),
      kind: webinterface_protocol::SystemKind::Simhark,
      generation: 1,
      capabilities: vec![webinterface_protocol::Capability {
        id: "simhark.launch_match".into(),
        mutable: true,
        description: "Validate and launch a match configuration".into(),
      }],
    })?;
    println!("Start Center: {}", handle.http_url());
    println!("No simulation, world, AI, or match has been created.");

    let configuration = tokio::select! {
      _ = tokio::signal::ctrl_c() => return Ok(()),
      command = launcher.commands.recv() => {
        match command {
          Some(command) => match command.command {
            webinterface_protocol::SystemCommand::Simhark(
              webinterface_protocol::SimharkCommand::LaunchMatch(configuration)
            ) => {
              launcher.publisher.acknowledge(
                command.browser_command_id,
                webinterface_protocol::CommandStatus::Applied,
                "validated; rebuilding host for match session",
              );
              configuration
            }
            _ => {
              launcher.publisher.acknowledge(
                command.browser_command_id,
                webinterface_protocol::CommandStatus::Rejected,
                "match-runner only accepts launch_match",
              );
              continue;
            }
          },
          None => continue,
        }
      }
    };
    drop(handle);
    drop(guard);
    run_configured_matches(configuration)?;
  }
}

fn run_configured_matches(
  configuration: webinterface_protocol::MatchConfiguration,
) -> anyhow::Result<()> {
  let blue = TeamKind::parse(&configuration.blue_controller).map_err(anyhow::Error::msg)?;
  let yellow = TeamKind::parse(&configuration.yellow_controller).map_err(anyhow::Error::msg)?;
  if configuration.blue_robots as usize > MAX_ROBOTS_PER_TEAM
    || configuration.yellow_robots as usize > MAX_ROBOTS_PER_TEAM
  {
    anyhow::bail!("robot count exceeds {MAX_ROBOTS_PER_TEAM}");
  }
  let seconds = configuration
    .duration_ns
    .map(|duration| duration as f64 / 1_000_000_000.0)
    .unwrap_or(60.0);
  if !seconds.is_finite() || seconds <= 0.0 {
    anyhow::bail!("duration must be positive and finite");
  }
  let division = configuration
    .division
    .chars()
    .next()
    .unwrap_or('b')
    .to_ascii_lowercase();
  if !matches!(division, 'a' | 'b') {
    anyhow::bail!("division must be A or B");
  }
  let count = configuration.batch_count.max(1);
  for index in 0..count {
    let config = MatchConfig {
      blue: blue.clone(),
      yellow: yellow.clone(),
      blue_bots: Some(configuration.blue_robots as usize),
      yellow_bots: Some(configuration.yellow_robots as usize),
      seconds,
      div: division,
      seed: configuration.seed.wrapping_add(index as u64),
      viewer: true,
      realtime: !configuration.precompute,
      dev: configuration.development,
      interface_recording: configuration.record,
      ..MatchConfig::default()
    };
    let report = run_match(&config);
    print_report(&report, false);
  }
  Ok(())
}

fn default_replay_path(mc: &MatchConfig) -> String {
  let timestamp = std::time::SystemTime::now()
    .duration_since(std::time::UNIX_EPOCH)
    .map(|duration| duration.as_secs())
    .unwrap_or(0);
  format!(
    "recordings/{}_vs_{}_div{}_seed{}_{}.shreplay",
    safe_name(mc.blue.label()),
    safe_name(mc.yellow.label()),
    mc.div.to_ascii_lowercase(),
    mc.seed,
    timestamp,
  )
}

fn safe_name(value: &str) -> String {
  let mut out = value
    .chars()
    .map(|ch| {
      if ch.is_ascii_alphanumeric() {
        ch.to_ascii_lowercase()
      } else {
        '-'
      }
    })
    .collect::<String>();
  while out.contains("--") {
    out = out.replace("--", "-");
  }
  out.trim_matches('-').to_string()
}

fn suffix_path(path: &str, index: usize) -> String {
  let path = std::path::Path::new(path);
  let stem = path
    .file_stem()
    .and_then(|stem| stem.to_str())
    .unwrap_or("replay");
  let ext = path.extension().and_then(|ext| ext.to_str());
  let name = match ext {
    Some(ext) => format!("{stem}_{index}.{ext}"),
    None => format!("{stem}_{index}"),
  };
  path.with_file_name(name).display().to_string()
}

#[cfg(feature = "viewer")]
fn serve_replay(path: &str) {
  use std::thread;
  use std::time::Duration;

  let replay = match simhark::ReplayLog::read_zstd(path) {
    Ok(replay) => replay,
    Err(err) => {
      eprintln!("failed to load replay {path}: {err}");
      return;
    }
  };
  if replay.frames.is_empty() {
    eprintln!("replay {path} has no frames");
    return;
  }

  let vc = simhark::viewer::ViewerConfig::default();
  let viewer = match simhark::viewer::ViewerServer::bind(
    vc,
    replay.metadata.world_count,
    &replay.metadata.world_config,
  ) {
    Ok(viewer) => viewer,
    Err(err) => {
      eprintln!("replay viewer bind failed: {err}");
      return;
    }
  };
  viewer.enable_web_control();
  println!("replay viewer: {}", vc.http_url());
  println!(
    "loaded replay: {} frames, {} events from {}",
    replay.frames.len(),
    replay.events.len(),
    path
  );

  let mut index = 0usize;
  viewer.publish_replay_frame(
    &replay.frames[index],
    index,
    replay.frames.len(),
    &replay.events,
    replay.metadata.tick_hz,
  );
  loop {
    let step = viewer.take_frame_step_request();
    if step != 0 {
      index = apply_frame_step(index, step, replay.frames.len());
    }
    if viewer.is_running() {
      viewer.publish_replay_frame(
        &replay.frames[index],
        index,
        replay.frames.len(),
        &replay.events,
        replay.metadata.tick_hz,
      );
      index = (index + 1).min(replay.frames.len().saturating_sub(1));
      let delay = Duration::from_secs_f64(1.0 / replay.metadata.tick_hz.max(1.0));
      thread::sleep(viewer.scaled_sleep(delay));
    } else {
      viewer.publish_replay_frame(
        &replay.frames[index],
        index,
        replay.frames.len(),
        &replay.events,
        replay.metadata.tick_hz,
      );
      thread::sleep(Duration::from_millis(33));
    }
    if viewer.take_restart_request() {
      viewer.reset_goals();
      index = 0;
    }
  }
}

#[cfg(feature = "viewer")]
fn apply_frame_step(index: usize, step: isize, len: usize) -> usize {
  if len == 0 {
    return 0;
  }
  let max = len.saturating_sub(1) as isize;
  (index as isize + step).clamp(0, max) as usize
}

#[cfg(not(feature = "viewer"))]
fn serve_replay(path: &str) {
  println!(
    "wrote replay to {path}; rebuild match-runner with `--features viewer` to auto-open the replay viewer"
  );
}
