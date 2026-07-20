use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::thread;
use std::time::Duration;

use loguna::{LogReader, MessageId};
use prost::Message;
use simhark::replay::{ReplayEventKind, ReplayFrame};
use simhark::viewer::{GameStateInfo, ViewerConfig, ViewerServer};
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
      eprintln!(
        "  simhark-replay serve <file.shreplay|ssl.log|ssl.log.gz> [--viewer-port PORT] [--vision2014]"
      );
      eprintln!(
        "    note: SSL log replay uses only VisionTracker2020 by default; pass --vision2014 for raw SSL-Vision packets."
      );
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
  let options = ServeOptions::parse(args)?;
  let path = options.path.as_path();
  let replay = if path.extension().is_some_and(|ext| ext == "shreplay") {
    ReplayLog::read_zstd(path)?
  } else {
    ssl_log_to_replay(path, options.vision_source)?
  };
  if replay.frames.is_empty() {
    return Err("replay contains no frames".into());
  }

  let config = ViewerConfig {
    http_port: options.viewer_port,
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
    if let Some(frame) = viewer.take_frame_seek_request() {
      index = frame.min(replay.frames.len().saturating_sub(1));
    }
    let skip = viewer.take_frame_skip_request();
    if skip != 0 {
      index = apply_frame_step(index, skip, replay.frames.len());
    }
    let step = viewer.take_frame_step_request();
    if step != 0 {
      index = apply_frame_step(index, step, replay.frames.len());
    }
    if let Some(game_state) = replay_game_state_for_frame(&replay.events, index) {
      viewer.set_game_state(game_state);
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

#[derive(Clone, Copy)]
enum VisionLogSource {
  Tracker2020,
  Vision2014,
}

struct ServeOptions {
  path: PathBuf,
  viewer_port: u16,
  vision_source: VisionLogSource,
}

impl ServeOptions {
  fn parse(args: &[String]) -> Result<Self, Box<dyn std::error::Error>> {
    let mut path = None;
    let mut viewer_port = 8315;
    let mut vision_source = VisionLogSource::Tracker2020;
    let mut i = 0;
    while i < args.len() {
      match args[i].as_str() {
        "--viewer-port" => {
          let Some(value) = args.get(i + 1) else {
            return Err("missing value for --viewer-port".into());
          };
          viewer_port = value.parse::<u16>()?;
          i += 2;
        }
        "--vision2014" | "--raw-vision" => {
          vision_source = VisionLogSource::Vision2014;
          i += 1;
        }
        value if value.starts_with("--") => {
          return Err(format!("unknown serve option {value}").into());
        }
        value => {
          if path.is_some() {
            return Err(format!("unexpected extra replay/log path {value}").into());
          }
          path = Some(PathBuf::from(value));
          i += 1;
        }
      }
    }

    Ok(Self {
      path: path.ok_or("missing replay/log path")?,
      viewer_port,
      vision_source,
    })
  }
}

fn apply_frame_step(index: usize, step: isize, len: usize) -> usize {
  if len == 0 {
    return 0;
  }
  let max = len.saturating_sub(1) as isize;
  (index as isize + step).clamp(0, max) as usize
}

fn ssl_log_to_replay(
  path: &Path,
  vision_source: VisionLogSource,
) -> Result<ReplayLog, Box<dyn std::error::Error>> {
  let mut reader = LogReader::open(path)?;
  let mut config = WorldConfig::division_a();
  let mut frames = Vec::new();
  let mut events = Vec::new();
  let mut first_capture = None::<f64>;
  let mut last_command_counter = None::<u32>;
  let mut tracker = VisionTracker::default();

  while let Some(message) = reader.next_message()? {
    match message.message_id {
      MessageId::VisionTracker2020 if matches!(vision_source, VisionLogSource::Tracker2020) => {
        let Ok(wrapper) = loguna::proto::TrackerWrapperPacket::decode(message.payload.as_slice())
        else {
          continue;
        };
        let Some(tracked_frame) = wrapper.tracked_frame else {
          continue;
        };
        let base = *first_capture.get_or_insert(tracked_frame.timestamp);
        let sim_time = (tracked_frame.timestamp - base).max(0.0);
        let frame_number = tracked_frame.frame_number as u64;
        frames.push(ReplayFrame {
          frame: frame_number,
          sim_time,
          states: vec![tracked_frame_to_state(
            &tracked_frame,
            frame_number,
            sim_time,
          )],
          commands: vec![WorldCommand::default()],
          debug: Vec::new(),
          events: Vec::new(),
        });
      }
      MessageId::Vision2014 | MessageId::Vision2010 => {
        if !matches!(vision_source, VisionLogSource::Vision2014) {
          continue;
        }
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
        let state = tracker.detection_to_state(&detection, frame_number, sim_time);
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
          let sim_time = frames.last().map_or(0.0, |frame| frame.sim_time);
          let stage = loguna::proto::referee::Stage::try_from(referee.stage)
            .map(|stage| stage.as_str_name().to_string())
            .unwrap_or_else(|_| format!("STAGE_{}", referee.stage));
          events.push(ReplayEvent {
            frame: frames.len() as u64,
            sim_time,
            world_id: None,
            kind: ReplayEventKind::Referee,
            label: command.clone(),
            details: Some(format!(
              "counter {}\nstage {}\nblue {}\nyellow {}",
              referee.command_counter, stage, referee.blue.name, referee.yellow.name
            )),
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

fn tracked_frame_to_state(
  frame: &loguna::proto::TrackedFrame,
  frame_number: u64,
  sim_time: f64,
) -> simhark::WorldState {
  let ball = frame.balls.first().map_or(
    BallState {
      x: 0.0,
      y: 0.0,
      z: 0.0,
      vx: 0.0,
      vy: 0.0,
      vz: 0.0,
    },
    tracked_ball,
  );
  let mut blue_robots = Vec::new();
  let mut yellow_robots = Vec::new();
  for robot in &frame.robots {
    match robot
      .robot_id
      .team
      .and_then(|team| loguna::proto::Team::try_from(team).ok())
    {
      Some(loguna::proto::Team::Blue) => blue_robots.push(tracked_robot(robot, TeamColor::Blue)),
      Some(loguna::proto::Team::Yellow) => {
        yellow_robots.push(tracked_robot(robot, TeamColor::Yellow));
      }
      _ => {}
    }
  }
  blue_robots.sort_by_key(|robot| robot.id);
  yellow_robots.sort_by_key(|robot| robot.id);

  simhark::WorldState {
    world_id: 0,
    sim_time,
    frame: frame_number,
    ball,
    blue_robots,
    yellow_robots,
    goal_blue: false,
    goal_yellow: false,
  }
}

fn tracked_ball(ball: &loguna::proto::TrackedBall) -> BallState {
  BallState {
    x: ball.pos.x as f64,
    y: ball.pos.y as f64,
    z: ball.pos.z as f64,
    vx: ball.vel.as_ref().map_or(0.0, |vel| vel.x as f64),
    vy: ball.vel.as_ref().map_or(0.0, |vel| vel.y as f64),
    vz: ball.vel.as_ref().map_or(0.0, |vel| vel.z as f64),
  }
}

fn tracked_robot(robot: &loguna::proto::TrackedRobot, team: TeamColor) -> RobotState {
  RobotState {
    id: robot.robot_id.id.unwrap_or(0) as usize,
    team,
    x: robot.pos.x as f64,
    y: robot.pos.y as f64,
    z: 0.0,
    orientation: robot.orientation as f64,
    vx: robot.vel.as_ref().map_or(0.0, |vel| vel.x as f64),
    vy: robot.vel.as_ref().map_or(0.0, |vel| vel.y as f64),
    vz: 0.0,
    v_angular: robot.vel_angular.unwrap_or(0.0) as f64,
    infrared: false,
    dribbler_on: false,
    kick_status: Default::default(),
    is_on: robot.visibility.is_none_or(|visibility| visibility > 0.0),
    wheel_speeds: [0.0; 4],
  }
}

fn replay_game_state_for_frame(
  events: &[ReplayEvent],
  frame_index: usize,
) -> Option<GameStateInfo> {
  events
    .iter()
    .filter(|event| event.kind == ReplayEventKind::Referee && event.frame as usize <= frame_index)
    .next_back()
    .map(|event| {
      let details = event.details.as_deref().unwrap_or_default();
      GameStateInfo {
        command: event.label.clone(),
        command_counter: parse_detail_value(details, "counter")
          .and_then(|value| value.parse::<u32>().ok())
          .unwrap_or(event.frame as u32),
        stage: parse_detail_value(details, "stage").map(str::to_string),
        blue_name: parse_detail_value(details, "blue")
          .filter(|name| !name.is_empty())
          .map(str::to_string),
        yellow_name: parse_detail_value(details, "yellow")
          .filter(|name| !name.is_empty())
          .map(str::to_string),
      }
    })
}

fn parse_detail_value<'a>(details: &'a str, key: &str) -> Option<&'a str> {
  details.lines().find_map(|line| {
    let (line_key, value) = line.split_once(' ')?;
    (line_key == key).then_some(value.trim())
  })
}

const VISION_TRACK_ROBOT_SECONDS: f64 = 0.45;

#[derive(Default)]
struct VisionTracker {
  ball: Option<TrackedBall>,
  robots: HashMap<(TeamColor, usize), TrackedRobot>,
}

#[derive(Clone)]
struct TrackedBall {
  state: BallState,
  sim_time: f64,
}

#[derive(Clone)]
struct TrackedRobot {
  state: RobotState,
  sim_time: f64,
}

impl VisionTracker {
  fn detection_to_state(
    &mut self,
    detection: &loguna::proto::SslDetectionFrame,
    frame: u64,
    sim_time: f64,
  ) -> simhark::WorldState {
    let ball = self.update_ball(detection, sim_time);
    let blue_robots = self.update_robots(&detection.robots_blue, TeamColor::Blue, sim_time);
    let yellow_robots = self.update_robots(&detection.robots_yellow, TeamColor::Yellow, sim_time);

    simhark::WorldState {
      world_id: 0,
      sim_time,
      frame,
      ball,
      blue_robots,
      yellow_robots,
      goal_blue: false,
      goal_yellow: false,
    }
  }

  fn update_ball(
    &mut self,
    detection: &loguna::proto::SslDetectionFrame,
    sim_time: f64,
  ) -> BallState {
    let Some(raw_ball) = detection.balls.first() else {
      return self.ball.as_ref().map_or(
        BallState {
          x: 0.0,
          y: 0.0,
          z: 0.0,
          vx: 0.0,
          vy: 0.0,
          vz: 0.0,
        },
        |tracked| tracked.state.clone(),
      );
    };

    let mut ball = BallState {
      x: raw_ball.x as f64 / 1000.0,
      y: raw_ball.y as f64 / 1000.0,
      z: raw_ball.z.unwrap_or(0.0) as f64 / 1000.0,
      vx: 0.0,
      vy: 0.0,
      vz: 0.0,
    };
    if let Some(previous) = self.ball.as_ref() {
      let dt = (sim_time - previous.sim_time).max(1e-6);
      ball.vx = (ball.x - previous.state.x) / dt;
      ball.vy = (ball.y - previous.state.y) / dt;
      ball.vz = (ball.z - previous.state.z) / dt;
    }
    self.ball = Some(TrackedBall {
      state: ball.clone(),
      sim_time,
    });
    ball
  }

  fn update_robots(
    &mut self,
    detections: &[loguna::proto::SslDetectionRobot],
    team: TeamColor,
    sim_time: f64,
  ) -> Vec<RobotState> {
    for robot in detections {
      let id = robot.robot_id.unwrap_or(0) as usize;
      let key = (team, id);
      let mut state = detection_robot(robot, team);
      if let Some(previous) = self.robots.get(&key) {
        let dt = (sim_time - previous.sim_time).max(1e-6);
        state.vx = (state.x - previous.state.x) / dt;
        state.vy = (state.y - previous.state.y) / dt;
        state.v_angular = angle_delta(state.orientation, previous.state.orientation) / dt;
      }
      self.robots.insert(key, TrackedRobot { state, sim_time });
    }

    let mut robots = self
      .robots
      .iter()
      .filter_map(|((tracked_team, _), tracked)| {
        if *tracked_team != team {
          return None;
        }
        let age = sim_time - tracked.sim_time;
        if age > VISION_TRACK_ROBOT_SECONDS {
          return None;
        }
        let mut state = tracked.state.clone();
        state.is_on = true;
        if age > 0.0 {
          state.x += state.vx * age;
          state.y += state.vy * age;
          state.orientation = normalize_angle(state.orientation + state.v_angular * age);
        }
        Some(state)
      })
      .collect::<Vec<_>>();
    robots.sort_by_key(|robot| robot.id);
    robots
  }
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

fn angle_delta(current: f64, previous: f64) -> f64 {
  normalize_angle(current - previous)
}

fn normalize_angle(angle: f64) -> f64 {
  let two_pi = std::f64::consts::PI * 2.0;
  (angle + std::f64::consts::PI).rem_euclid(two_pi) - std::f64::consts::PI
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
