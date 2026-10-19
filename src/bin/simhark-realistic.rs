//! Explicit opt-in hardware/timing experiment; ordinary simhark is unaffected.
use simhark::{
  MoveCommand, RobotCommand, WorldCommand, WorldConfig,
  realistic::{RealisticConfig, RealisticWorld, Recording, time::SECOND, transport::Safety},
};
use std::{error::Error, path::PathBuf, time::Instant};

fn main() -> Result<(), Box<dyn Error>> {
  let mut profile = Vec::new();
  let mut physics_config: Option<PathBuf> = None;
  let mut planar = false;
  let mut controller_program: Option<PathBuf> = None;
  let mut replay_controls = false;
  let mut serve = false;
  let mut render_dir: Option<PathBuf> = None;
  let mut external_vision: Option<std::net::SocketAddr> = None;
  let mut seconds = 3.0_f64;
  let mut robots = 1;
  let mut seed = 42;
  let mut record: Option<PathBuf> = None;
  let mut replay: Option<PathBuf> = None;
  let mut write_default: Option<PathBuf> = None;
  let mut budget_only = false;
  let mut args = std::env::args().skip(1);
  while let Some(flag) = args.next() {
    match flag.as_str() {
      "--physics-config" => {
        physics_config = Some(args.next().ok_or("missing --physics-config path")?.into())
      }
      "--profile" => profile.push(PathBuf::from(args.next().ok_or("missing --profile path")?)),
      "--planar" => planar = true,
      "--controller-program" => {
        controller_program = Some(args.next().ok_or("missing controller program")?.into())
      }
      "--replay-controller-output" => replay_controls = true,
      "--serve" => serve = true,
      "--render-dir" => render_dir = Some(args.next().ok_or("missing render directory")?.into()),
      "--external-vision" => {
        external_vision = Some(
          args
            .next()
            .ok_or("missing external vision bind address")?
            .parse()?,
        )
      }
      "--seconds" => seconds = args.next().ok_or("missing --seconds")?.parse()?,
      "--robots" => robots = args.next().ok_or("missing --robots")?.parse()?,
      "--seed" => seed = args.next().ok_or("missing --seed")?.parse()?,
      "--record" => record = Some(args.next().ok_or("missing --record path")?.into()),
      "--replay" => replay = Some(args.next().ok_or("missing --replay path")?.into()),
      "--write-default-profile" => {
        write_default = Some(args.next().ok_or("missing output path")?.into())
      }
      "--budget-only" => budget_only = true,
      "--help" | "-h" => {
        println!(
          "simhark-realistic [--profile profile.toml] [--seconds 3] [--robots 1] [--seed 42]\n  [--record run.json] [--replay run.json] [--write-default-profile profile.toml] [--budget-only]\n\nFull optional hardware/timing experiment: forward, reversal, coast.\n  --controller-program PATH: paced JSONL controller per robot\n  --replay-controller-output: replay recorded control voltages\n  --planar: minimal compatibility profile\n  --serve: paced standard SSL/grSim command and vision UDP server\n  --render-dir DIR: export each camera as BMP at 10 Hz\n  --external-vision IP:PORT: actual SSL-Vision UDP instead of synthetic detections\n--physics-config PATH: load shared [simulation] settings (default config.toml)\nRepeat --profile to overlay hardware/venue/scenario profiles.\nController and hardware defaults are surrogates requiring calibration.\nEnable with cargo run -r --features realistic --bin simhark-realistic -- ..."
        );
        return Ok(());
      }
      _ => return Err(format!("unknown argument: {flag}").into()),
    }
  }
  if let Some(path) = replay {
    if !profile.is_empty()
      || physics_config.is_some()
      || record.is_some()
      || write_default.is_some()
      || budget_only
    {
      return Err("--replay cannot be combined with profile, recording, or budget options".into());
    }
    let recording: Recording =
      serde_json::from_reader(std::io::BufReader::new(std::fs::File::open(path)?))?;
    let sim = if replay_controls {
      recording.replay_recorded_controls(&build_identity()?)?
    } else {
      recording.replay(&build_identity()?)?
    };
    let actual = sim.recording(recording.build_identity.clone());
    if serde_json::to_vec(&recording.events)? != serde_json::to_vec(&actual.events)?
      || serde_json::to_vec(&recording.diagnostics)? != serde_json::to_vec(&actual.diagnostics)?
      || serde_json::to_vec(&recording.final_truth)? != serde_json::to_vec(&actual.final_truth)?
      || serde_json::to_vec(&recording.final_hardware)?
        != serde_json::to_vec(&actual.final_hardware)?
    {
      return Err("replay diverged from recorded events or diagnostics".into());
    }
    println!(
      "{}",
      serde_json::to_string_pretty(&serde_json::json!({ "replay_verified": true,
      "diagnostics": sim.diagnostics(), "truth_for_evaluation": sim.truth() }))?
    );
    return Ok(());
  }
  let mut config = if !profile.is_empty() {
    simhark::realistic::profiles::resolve(&profile)?.config
  } else if planar {
    RealisticConfig::default()
  } else {
    RealisticConfig::full()
  };
  config.external_vision = external_vision.is_some();
  config.validate()?;
  if let Some(path) = write_default {
    std::fs::write(path, config.to_toml()?)?;
    return Ok(());
  }
  println!(
    "{}",
    serde_json::to_string_pretty(&config.transport.airtime_budget())?
  );
  let required = config.transport.command_format.airtime_ns(robots)?;
  if config.transport.slots.iter().any(|s| {
    matches!(
      s.purpose,
      simhark::realistic::config::SlotPurpose::BlueCommand
        | simhark::realistic::config::SlotPurpose::YellowCommand
    ) && s.occupied_ns < required
  }) {
    return Err(
      format!(
        "{robots} robots need {required} ns of command airtime; configured slots are too short"
      )
      .into(),
    );
  }
  if budget_only {
    return Ok(());
  }
  if !seconds.is_finite() || seconds <= 0.0 || seconds > 3600.0 {
    return Err("--seconds must be finite and in (0, 3600]".into());
  }
  let mut world_config = WorldConfig::division_b();
  if let Some(path) = physics_config {
    world_config.load_simulation_config(path)?;
  } else {
    world_config.load_default_simulation_config()?;
  }
  world_config.robots_per_team = robots;
  world_config.seed = seed;
  let mut server = if serve {
    Some(simhark::realistic::protocol::StandardServer::bind(
      "0.0.0.0".parse()?,
      20011,
      10301,
      10302,
      "224.5.23.2:10020".parse()?,
      world_config.clone(),
    )?)
  } else {
    None
  };
  if let Some(server) = &mut server {
    server.bind_setup("0.0.0.0:10300".parse()?)?;
  }
  let mut world = RealisticWorld::new(0, world_config, config)?;
  if let Some(program) = &controller_program {
    for team in [simhark::TeamColor::Blue, simhark::TeamColor::Yellow] {
      for id in 0..robots {
        let mut command = std::process::Command::new(program);
        command
          .env(
            "SIMHARK_TEAM",
            if team == simhark::TeamColor::Blue {
              "blue"
            } else {
              "yellow"
            },
          )
          .env("SIMHARK_ROBOT_ID", id.to_string());
        world.set_controller(
          team,
          id,
          Box::new(simhark::realistic::live::ProcessController::spawn(
            &mut command,
            20_000_000,
          )?),
        )?;
      }
    }
  }
  let host_epoch = std::time::SystemTime::now()
    .duration_since(std::time::UNIX_EPOCH)?
    .as_secs_f64();
  let mut vision_ingress = external_vision
    .map(|addr| simhark::realistic::protocol::ExternalVisionReceiver::bind(addr, host_epoch, 0))
    .transpose()?;
  if let Some(ingress) = &vision_ingress {
    ingress.join_multicast("224.5.23.2".parse()?, std::net::Ipv4Addr::UNSPECIFIED)?;
  }
  let pacer = simhark::realistic::live::LivePacer::new(0);
  if let Some(dir) = &render_dir {
    std::fs::create_dir_all(dir)?;
    for camera in &world.config().cameras {
      std::fs::create_dir_all(dir.join(format!("camera-{}", camera.id)))?;
    }
    std::fs::write(
      dir.join("manifest.json"),
      serde_json::to_vec_pretty(
        &serde_json::json!({"schema_version":1,"profile_sha256":simhark::realistic::calibration::digest(world.config())?,"cameras":world.config().cameras,"format":"24-bit BMP for SSL-Vision CaptureFromFile","motion_model":"constant twist across exposure/readout","live_nondeterministic":serve||vision_ingress.is_some()}),
      )?,
    )?;
  }
  if record.is_some() {
    world.enable_recording();
  }
  let end = (seconds * SECOND as f64).round() as u64;
  let start = Instant::now();
  // A 1 kHz publisher can repeat decisions from a slower planner. Keep the
  // original decision time to expose that age independently of publication age.
  let mut tick = 0_u64;
  while world.now() < end {
    let now = world.now();
    let phase = if now < end / 3 {
      0
    } else if now < 2 * end / 3 {
      1
    } else {
      2
    };
    let decision_at = match phase {
      0 => 0,
      1 => end / 3,
      _ => 2 * end / 3,
    };
    let forward = match phase {
      0 => 1.0,
      1 => -1.0,
      _ => 0.0,
    };
    let commands = WorldCommand {
      blue: (0..robots)
        .map(|id| RobotCommand {
          move_command: Some(MoveCommand::LocalVelocity {
            forward,
            left: 0.0,
            angular: 0.0,
          }),
          ..RobotCommand::noop(id)
        })
        .collect(),
      ..WorldCommand::default()
    };
    let safety = if phase == 2 {
      Safety::Halt
    } else {
      Safety::Run
    };
    let epoch = if phase == 2 { 2 } else { 1 };
    if let Some(server) = &mut server {
      server.poll(&mut world)?;
    } else {
      world.publish(commands, decision_at, epoch, safety)?;
    }
    if let Some(ingress) = &mut vision_ingress {
      ingress.poll(&mut world)?;
    }
    if let Some(dir) = &render_dir {
      if tick % 100 == 0 {
        for camera in world
          .config()
          .cameras
          .iter()
          .map(|c| c.id)
          .collect::<Vec<_>>()
        {
          world
            .render_camera(camera, &Default::default())?
            .write_bmp(&dir.join(format!("camera-{camera}/{tick:09}.bmp")))?;
        }
      }
    }
    tick += 1;
    world.advance_to((tick * 1_000_000).min(end))?;
    if let Some(server) = &mut server {
      server.vision(&world.delivered_camera_frames())?;
    }
    if serve || vision_ingress.is_some() || controller_program.is_some() {
      pacer.pace(world.now());
    }
  }
  let elapsed = start.elapsed().as_secs_f64();
  println!(
    "{}",
    serde_json::to_string_pretty(&serde_json::json!({
      "simulated_seconds": seconds, "wall_seconds": elapsed, "sim_seconds_per_wall_second": seconds / elapsed,
      "diagnostics": world.diagnostics(), "robots": world.robot_diagnostics(), "truth_for_evaluation": world.truth()
    }))?
  );
  if let Some(path) = record {
    let file = std::fs::File::create(path)?;
    serde_json::to_writer(
      std::io::BufWriter::new(file),
      &world.recording(build_identity()?),
    )?;
  }
  Ok(())
}

fn build_identity() -> Result<String, Box<dyn Error>> {
  let executable = std::fs::read(std::env::current_exe()?)?;
  let digest = executable
    .iter()
    .fold(0xcbf29ce484222325_u64, |hash, byte| {
      (hash ^ u64::from(*byte)).wrapping_mul(0x100000001b3)
    });
  Ok(format!(
    "simhark {} executable-fnv1a64:{digest:016x}; {}-{}; {}",
    env!("CARGO_PKG_VERSION"),
    std::env::consts::OS,
    std::env::consts::ARCH,
    option_env!("SIMHARK_BUILD_ID").unwrap_or("revision-unspecified")
  ))
}
