//! Compressed binary replay logs for simhark debugger sessions.

use std::fs::File;
use std::io::{BufReader, BufWriter, Read, Write};
use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::command::WorldCommand;
use crate::config::WorldConfig;
use crate::state::{TeamColor, WorldState};

const MAGIC: &[u8; 8] = b"SHREPL01";
const ZSTD_LEVEL: i32 = 3;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReplayLog {
  pub metadata: ReplayMetadata,
  pub frames: Vec<ReplayFrame>,
  pub events: Vec<ReplayEvent>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReplayMetadata {
  pub world_count: usize,
  pub world_config: WorldConfig,
  pub tick_hz: f64,
  pub source: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReplayFrame {
  pub frame: u64,
  pub sim_time: f64,
  pub states: Vec<WorldState>,
  pub commands: Vec<WorldCommand>,
  pub debug: Vec<ReplayDebugSnapshot>,
  pub events: Vec<ReplayEvent>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReplayDebugSnapshot {
  pub world_id: usize,
  pub strategy: Option<String>,
  pub robots: Vec<ReplayRobotDebugInfo>,
  pub overlays: Vec<ReplayDebugOverlay>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReplayRobotDebugInfo {
  pub team: TeamColor,
  pub id: usize,
  pub task: String,
  pub color: String,
  pub message: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReplayDebugOverlay {
  HoloRobot(ReplayDebugHoloRobot),
  KickLine(ReplayDebugKickLine),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReplayDebugHoloRobot {
  pub team: TeamColor,
  pub id: usize,
  pub x: f64,
  pub y: f64,
  pub orientation: Option<f64>,
  pub color: String,
  pub label: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReplayDebugKickLine {
  pub team: TeamColor,
  pub id: usize,
  pub from_x: f64,
  pub from_y: f64,
  pub angle: f64,
  pub color: String,
  pub label: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReplayEvent {
  pub frame: u64,
  pub sim_time: f64,
  pub world_id: Option<usize>,
  pub kind: ReplayEventKind,
  pub label: String,
  pub details: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReplayEventKind {
  GoalBlue,
  GoalYellow,
  Foul,
  Referee,
  Custom,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotInputInfo {
  pub world_id: usize,
  pub team: TeamColor,
  pub id: usize,
  pub input: String,
}

#[derive(Debug)]
pub enum ReplayError {
  Io(std::io::Error),
  Encode(Box<bincode::ErrorKind>),
  BadMagic([u8; 8]),
}

impl std::fmt::Display for ReplayError {
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    match self {
      Self::Io(err) => write!(f, "I/O error: {err}"),
      Self::Encode(err) => write!(f, "replay codec error: {err}"),
      Self::BadMagic(found) => write!(f, "not a simhark replay log: {found:?}"),
    }
  }
}

impl std::error::Error for ReplayError {}

impl From<std::io::Error> for ReplayError {
  fn from(value: std::io::Error) -> Self {
    Self::Io(value)
  }
}

impl From<Box<bincode::ErrorKind>> for ReplayError {
  fn from(value: Box<bincode::ErrorKind>) -> Self {
    Self::Encode(value)
  }
}

impl ReplayLog {
  pub fn write_zstd<P: AsRef<Path>>(&self, path: P) -> Result<(), ReplayError> {
    let file = File::create(path)?;
    let mut writer = BufWriter::new(file);
    writer.write_all(MAGIC)?;
    let mut encoder = zstd::Encoder::new(writer, ZSTD_LEVEL)?;
    bincode::serialize_into(&mut encoder, self)?;
    encoder.finish()?;
    Ok(())
  }

  pub fn read_zstd<P: AsRef<Path>>(path: P) -> Result<Self, ReplayError> {
    let file = File::open(path)?;
    let mut reader = BufReader::new(file);
    let mut magic = [0u8; 8];
    reader.read_exact(&mut magic)?;
    if &magic != MAGIC {
      return Err(ReplayError::BadMagic(magic));
    }
    let decoder = zstd::Decoder::new(reader)?;
    Ok(bincode::deserialize_from(decoder)?)
  }
}

#[derive(Debug, Clone)]
pub struct ReplayRecorder {
  metadata: ReplayMetadata,
  frames: Vec<ReplayFrame>,
  events: Vec<ReplayEvent>,
  last_goal_blue: Vec<bool>,
  last_goal_yellow: Vec<bool>,
}

impl ReplayRecorder {
  pub fn new(world_count: usize, world_config: WorldConfig, tick_hz: f64, source: String) -> Self {
    Self {
      metadata: ReplayMetadata {
        world_count,
        world_config,
        tick_hz,
        source,
      },
      frames: Vec::new(),
      events: Vec::new(),
      last_goal_blue: vec![false; world_count],
      last_goal_yellow: vec![false; world_count],
    }
  }

  pub fn push_custom_event(
    &mut self,
    frame: u64,
    sim_time: f64,
    world_id: Option<usize>,
    label: impl Into<String>,
    details: impl Into<Option<String>>,
  ) {
    self.events.push(ReplayEvent {
      frame,
      sim_time,
      world_id,
      kind: ReplayEventKind::Custom,
      label: label.into(),
      details: details.into(),
    });
  }

  pub fn push_referee_event(
    &mut self,
    frame: u64,
    sim_time: f64,
    label: impl Into<String>,
    details: impl Into<Option<String>>,
  ) {
    self.events.push(ReplayEvent {
      frame,
      sim_time,
      world_id: None,
      kind: ReplayEventKind::Referee,
      label: label.into(),
      details: details.into(),
    });
  }

  pub fn push_foul_event(
    &mut self,
    frame: u64,
    sim_time: f64,
    world_id: Option<usize>,
    label: impl Into<String>,
    details: impl Into<Option<String>>,
  ) {
    self.events.push(ReplayEvent {
      frame,
      sim_time,
      world_id,
      kind: ReplayEventKind::Foul,
      label: label.into(),
      details: details.into(),
    });
  }

  pub fn push_frame(&mut self, states: Vec<WorldState>, commands: Vec<WorldCommand>) {
    self.push_frame_with_debug(states, commands, Vec::new());
  }

  pub fn push_frame_with_debug(
    &mut self,
    states: Vec<WorldState>,
    commands: Vec<WorldCommand>,
    debug: Vec<ReplayDebugSnapshot>,
  ) {
    let frame = states.first().map_or(0, |state| state.frame);
    let sim_time = states.first().map_or(0.0, |state| state.sim_time);
    let mut frame_events = Vec::new();
    for state in &states {
      let world_id = state.world_id;
      if world_id < self.last_goal_blue.len() && state.goal_blue && !self.last_goal_blue[world_id] {
        frame_events.push(ReplayEvent {
          frame: state.frame,
          sim_time: state.sim_time,
          world_id: Some(world_id),
          kind: ReplayEventKind::GoalBlue,
          label: "Blue goal".to_string(),
          details: None,
        });
      }
      if world_id < self.last_goal_yellow.len()
        && state.goal_yellow
        && !self.last_goal_yellow[world_id]
      {
        frame_events.push(ReplayEvent {
          frame: state.frame,
          sim_time: state.sim_time,
          world_id: Some(world_id),
          kind: ReplayEventKind::GoalYellow,
          label: "Yellow goal".to_string(),
          details: None,
        });
      }
      if world_id < self.last_goal_blue.len() {
        self.last_goal_blue[world_id] = state.goal_blue;
      }
      if world_id < self.last_goal_yellow.len() {
        self.last_goal_yellow[world_id] = state.goal_yellow;
      }
    }

    self.events.extend(frame_events.iter().cloned());
    self.frames.push(ReplayFrame {
      frame,
      sim_time,
      states,
      commands,
      debug,
      events: frame_events,
    });
  }

  pub fn finish(self) -> ReplayLog {
    ReplayLog {
      metadata: self.metadata,
      frames: self.frames,
      events: self.events,
    }
  }
}

#[cfg(feature = "viewer-debug")]
impl From<&crate::viewer::ViewerDebugSnapshot> for ReplayDebugSnapshot {
  fn from(snapshot: &crate::viewer::ViewerDebugSnapshot) -> Self {
    Self {
      world_id: snapshot.world_id,
      strategy: snapshot.strategy.clone(),
      robots: snapshot
        .robots
        .iter()
        .map(|robot| ReplayRobotDebugInfo {
          team: robot.team,
          id: robot.id,
          task: robot.task.clone(),
          color: robot.color.clone(),
          message: robot.message.clone(),
        })
        .collect(),
      overlays: snapshot
        .overlays
        .iter()
        .map(ReplayDebugOverlay::from)
        .collect(),
    }
  }
}

#[cfg(feature = "viewer-debug")]
impl From<&crate::viewer::DebugOverlay> for ReplayDebugOverlay {
  fn from(overlay: &crate::viewer::DebugOverlay) -> Self {
    match overlay {
      crate::viewer::DebugOverlay::HoloRobot(overlay) => Self::HoloRobot(ReplayDebugHoloRobot {
        team: overlay.team,
        id: overlay.id,
        x: overlay.x,
        y: overlay.y,
        orientation: overlay.orientation,
        color: overlay.color.clone(),
        label: overlay.label.clone(),
      }),
      crate::viewer::DebugOverlay::KickLine(overlay) => Self::KickLine(ReplayDebugKickLine {
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

pub fn robot_inputs_for_frame(frame: &ReplayFrame) -> Vec<RobotInputInfo> {
  frame
    .commands
    .iter()
    .enumerate()
    .flat_map(|(world_id, command)| {
      command
        .blue
        .iter()
        .map(move |robot| RobotInputInfo {
          world_id,
          team: TeamColor::Blue,
          id: robot.id,
          input: format!("{robot:?}"),
        })
        .chain(command.yellow.iter().map(move |robot| RobotInputInfo {
          world_id,
          team: TeamColor::Yellow,
          id: robot.id,
          input: format!("{robot:?}"),
        }))
    })
    .collect()
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::config::WorldConfig;
  use crate::state::TeamColor;

  #[test]
  fn zstd_bincode_round_trip_supports_debug_overlays() {
    let path = std::env::temp_dir().join(format!(
      "simhark-replay-round-trip-{}.shreplay",
      std::process::id()
    ));

    let replay = ReplayLog {
      metadata: ReplayMetadata {
        world_count: 1,
        world_config: WorldConfig::division_b(),
        tick_hz: 60.0,
        source: "test".to_string(),
      },
      frames: vec![ReplayFrame {
        frame: 7,
        sim_time: 7.0 / 60.0,
        states: Vec::new(),
        commands: Vec::new(),
        debug: vec![ReplayDebugSnapshot {
          world_id: 0,
          strategy: Some("test strategy".to_string()),
          robots: Vec::new(),
          overlays: vec![ReplayDebugOverlay::KickLine(ReplayDebugKickLine {
            team: TeamColor::Blue,
            id: 2,
            from_x: 1.0,
            from_y: -0.5,
            angle: 0.25,
            color: "#00aaff".to_string(),
            label: Some("shot".to_string()),
          })],
        }],
        events: Vec::new(),
      }],
      events: Vec::new(),
    };

    replay.write_zstd(&path).unwrap();
    let decoded = ReplayLog::read_zstd(&path).unwrap();
    let _ = std::fs::remove_file(path);

    assert_eq!(decoded.frames.len(), 1);
    assert_eq!(decoded.frames[0].debug.len(), 1);
    match &decoded.frames[0].debug[0].overlays[0] {
      ReplayDebugOverlay::KickLine(line) => {
        assert_eq!(line.team, TeamColor::Blue);
        assert_eq!(line.id, 2);
        assert_eq!(line.label.as_deref(), Some("shot"));
      }
      ReplayDebugOverlay::HoloRobot(_) => panic!("expected kick line overlay"),
    }
  }
}
