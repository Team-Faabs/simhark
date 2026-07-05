use std::path::{Path, PathBuf};
use std::thread;
use std::time::Duration;

use loguna::{LogReader, MessageId};
use prost::Message;
use simhark::replay::{ReplayEventKind, ReplayFrame};
use simhark::viewer::{ViewerConfig, ViewerServer};
use simhark::{
  BallState, MoveCommand, ReplayEvent, ReplayLog, ReplayMetadata, ReplayRecorder, RobotCommand,
  RobotState, SimulationEngine, TeamColor, WorldCommand, WorldConfig,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
  env_logger::init();
  let args = std::env::args().collect::<Vec<_>>();
  match args.get(1).map(String::as_str) {
    Some("record-demo") => record_demo(&args[2..])?,
    Some("serve") => serve(&args[2..])?,
    _ => {
      eprintln!("usage:");
      eprintln!("  simhark-replay record-demo <out.shreplay> [worlds] [steps]");
      eprintln!("  simhark-replay serve <file.shreplay|ssl.log|ssl.log.gz> [--viewer-port PORT]");
      std::process::exit(2);
    }
  }
  Ok(())
}

fn record_demo(args: &[String]) -> Result<(), Box<dyn std::error::Error>> {
  let out = args
    .first()
    .map(PathBuf::from)
    .unwrap_or_else(|| PathBuf::from("recordings/demo.shreplay"));
  let worlds = args
    .get(1)
    .and_then(|value| value.parse().ok())
    .unwrap_or(1);
  let steps = args
    .get(2)
    .and_then(|value| value.parse().ok())
    .unwrap_or(1200);
  let config = WorldConfig::division_a();
  let mut engine = SimulationEngine::new(worlds, config.clone());
  let mut recorder = ReplayRecorder::new(worlds, config, 60.0, "simhark demo".to_string());

  for step in 0..steps {
    let commands = demo_command(step, worlds);
    let states = engine.step_with_commands(&commands);
    recorder.push_frame(states, commands);
  }

  if let Some(parent) = out.parent().filter(|parent| !parent.as_os_str().is_empty()) {
    std::fs::create_dir_all(parent)?;
  }
  let log = recorder.finish();
  log.write_zstd(&out)?;
  println!(
    "wrote {} frames, {} events to {}",
    log.frames.len(),
    log.events.len(),
    out.display()
  );
  Ok(())
}

fn serve(args: &[String]) -> Result<(), Box<dyn std::error::Error>> {
  let Some(path) = args.first() else {
    return Err("missing replay/log path".into());
  };
  let port = args
    .windows(2)
    .find(|window| window[0] == "--viewer-port")
    .and_then(|window| window[1].parse::<u16>().ok())
    .unwrap_or(8315);
  let path = Path::new(path);
  let replay = if path.extension().is_some_and(|ext| ext == "shreplay") {
    ReplayLog::read_zstd(path)?
  } else {
    ssl_log_to_replay(path)?
  };
  if replay.frames.is_empty() {
    return Err("replay contains no frames".into());
  }

  let config = ViewerConfig {
    http_port: port,
    ..ViewerConfig::default()
  };
  let viewer = ViewerServer::bind(
    config,
    replay.metadata.world_count,
    &replay.metadata.world_config,
  )?;
  viewer.enable_web_control();
  println!("Viewer: {}", config.http_url());
  println!(
    "Loaded {} frames, {} events from {}",
    replay.frames.len(),
    replay.events.len(),
    path.display()
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
      let frame_delay = Duration::from_secs_f64(1.0 / replay.metadata.tick_hz.max(1.0));
      thread::sleep(viewer.scaled_sleep(frame_delay));
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

fn apply_frame_step(index: usize, step: isize, len: usize) -> usize {
  if len == 0 {
    return 0;
  }
  let max = len.saturating_sub(1) as isize;
  (index as isize + step).clamp(0, max) as usize
}

fn ssl_log_to_replay(path: &Path) -> Result<ReplayLog, Box<dyn std::error::Error>> {
  let mut reader = LogReader::open(path)?;
  let mut config = WorldConfig::division_a();
  let mut frames = Vec::new();
  let mut events = Vec::new();
  let mut first_capture = None::<f64>;
  let mut last_command_counter = None::<u32>;

  while let Some(message) = reader.next_message()? {
    match message.message_id {
      MessageId::Vision2014 | MessageId::Vision2010 => {
        let Ok(wrapper) = loguna::proto::SslWrapperPacket::decode(message.payload.as_slice())
        else {
          continue;
        };
        if let Some(geometry) = wrapper.geometry {
          apply_geometry(&mut config, &geometry.field);
        }
        let Some(detection) = wrapper.detection else {
          continue;
        };
        let base = *first_capture.get_or_insert(detection.t_capture);
        let sim_time = (detection.t_capture - base).max(0.0);
        let frame_number = detection.frame_number as u64;
        let state = detection_to_state(&detection, frame_number, sim_time);
        frames.push(ReplayFrame {
          frame: frame_number,
          sim_time,
          states: vec![state],
          commands: vec![WorldCommand::default()],
          debug: Vec::new(),
          events: Vec::new(),
        });
      }
      MessageId::Referee2013 => {
        let Ok(referee) = loguna::proto::Referee::decode(message.payload.as_slice()) else {
          continue;
        };
        let command = loguna::proto::referee::Command::try_from(referee.command)
          .map(|command| command.as_str_name().to_string())
          .unwrap_or_else(|_| format!("COMMAND_{}", referee.command));
        if last_command_counter != Some(referee.command_counter) {
          let sim_time = first_capture.map_or(0.0, |_| message.timestamp_secs());
          events.push(ReplayEvent {
            frame: frames.len() as u64,
            sim_time,
            world_id: None,
            kind: ReplayEventKind::Referee,
            label: command.clone(),
            details: Some(format!("counter {}", referee.command_counter)),
          });
          last_command_counter = Some(referee.command_counter);
        }
      }
      _ => {}
    }
  }

  let tick_hz = estimate_tick_hz(&frames).unwrap_or(60.0);
  Ok(ReplayLog {
    metadata: ReplayMetadata {
      world_count: 1,
      world_config: config,
      tick_hz,
      source: path.display().to_string(),
    },
    frames,
    events,
  })
}

fn apply_geometry(config: &mut WorldConfig, field: &loguna::proto::SslGeometryFieldSize) {
  config.field.field_length = field.field_length as f64 / 1000.0;
  config.field.field_width = field.field_width as f64 / 1000.0;
  config.field.goal_width = field.goal_width as f64 / 1000.0;
  config.field.goal_depth = field.goal_depth as f64 / 1000.0;
  config.field.margin_touch_line = field.boundary_width as f64 / 1000.0;
  config.field.margin_goal_line = field
    .boundary_width_goal_line
    .unwrap_or(field.boundary_width) as f64
    / 1000.0;
  if let Some(value) = field.penalty_area_width {
    config.field.penalty_width = value as f64 / 1000.0;
  }
  if let Some(value) = field.penalty_area_depth {
    config.field.penalty_depth = value as f64 / 1000.0;
  }
  if let Some(value) = field.center_circle_radius {
    config.field.field_center_radius = value as f64 / 1000.0;
  }
  if let Some(value) = field.line_thickness {
    config.field.field_line_width = value as f64 / 1000.0;
  }
  if let Some(value) = field.goal_height {
    config.field.goal_height = value as f64 / 1000.0;
  }
  if let Some(value) = field.ball_radius {
    config.ball.radius = value as f64 / 1000.0;
  }
  if let Some(value) = field.max_robot_radius {
    config.blue_robots.radius = value as f64 / 1000.0;
    config.yellow_robots.radius = value as f64 / 1000.0;
  }
}

fn detection_to_state(
  detection: &loguna::proto::SslDetectionFrame,
  frame: u64,
  sim_time: f64,
) -> simhark::WorldState {
  let ball = detection.balls.first().map_or(
    BallState {
      x: 0.0,
      y: 0.0,
      z: 0.0,
      vx: 0.0,
      vy: 0.0,
      vz: 0.0,
    },
    |ball| BallState {
      x: ball.x as f64 / 1000.0,
      y: ball.y as f64 / 1000.0,
      z: ball.z.unwrap_or(0.0) as f64 / 1000.0,
      vx: 0.0,
      vy: 0.0,
      vz: 0.0,
    },
  );

  simhark::WorldState {
    world_id: 0,
    sim_time,
    frame,
    ball,
    blue_robots: detection
      .robots_blue
      .iter()
      .map(|robot| detection_robot(robot, TeamColor::Blue))
      .collect(),
    yellow_robots: detection
      .robots_yellow
      .iter()
      .map(|robot| detection_robot(robot, TeamColor::Yellow))
      .collect(),
    goal_blue: false,
    goal_yellow: false,
  }
}

fn detection_robot(robot: &loguna::proto::SslDetectionRobot, team: TeamColor) -> RobotState {
  RobotState {
    id: robot.robot_id.unwrap_or(0) as usize,
    team,
    x: robot.x as f64 / 1000.0,
    y: robot.y as f64 / 1000.0,
    z: 0.0,
    orientation: robot.orientation.unwrap_or(0.0) as f64,
    vx: 0.0,
    vy: 0.0,
    vz: 0.0,
    v_angular: 0.0,
    infrared: false,
    dribbler_on: false,
    kick_status: Default::default(),
    is_on: true,
    wheel_speeds: [0.0; 4],
  }
}

fn estimate_tick_hz(frames: &[ReplayFrame]) -> Option<f64> {
  let first = frames.first()?.sim_time;
  let last = frames.last()?.sim_time;
  if frames.len() < 2 || last <= first {
    return None;
  }
  Some((frames.len() - 1) as f64 / (last - first))
}

fn demo_command(step: usize, num_worlds: usize) -> Vec<WorldCommand> {
  (0..num_worlds)
    .map(|world| demo_world_command(step, world))
    .collect()
}

fn demo_world_command(step: usize, world_index: usize) -> WorldCommand {
  let offset = world_index as f64 * 0.12;
  let wave = ((step as f64) * 0.05 + offset).sin();
  let sweep = ((step as f64) * 0.03 + offset * 0.5).cos();
  WorldCommand {
    blue: vec![
      RobotCommand {
        id: 4,
        move_command: Some(MoveCommand::LocalVelocity {
          forward: 1.0,
          left: 0.35 * wave,
          angular: 0.4,
        }),
        kick_speed: 0.0,
        kick_angle: 0.0,
        dribbler_on: false,
      },
      RobotCommand {
        id: 5,
        move_command: Some(MoveCommand::LocalVelocity {
          forward: 0.75,
          left: -0.6 * sweep,
          angular: 0.8,
        }),
        kick_speed: 0.0,
        kick_angle: 0.0,
        dribbler_on: false,
      },
    ],
    yellow: vec![RobotCommand {
      id: 4,
      move_command: Some(MoveCommand::LocalVelocity {
        forward: 0.9,
        left: -0.3 * sweep,
        angular: -0.35,
      }),
      kick_speed: 0.0,
      kick_angle: 0.0,
      dribbler_on: false,
    }],
    ..WorldCommand::default()
  }
}
