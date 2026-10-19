use std::collections::{HashMap, HashSet};
use std::env;
use std::io::{ErrorKind, Read, Result, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};

use prost::Message;

use simhark::command::{MoveCommand, RobotCommand, WorldCommand};
use simhark::config::RobotConfig;
use simhark::engine::SimulationEngine;
use simhark::state::{TeamColor, WorldState};

use crate::proto::DriveMode;
use crate::proto::{
  BotId, SimBallState, SimBotAction, SimBotState, SimReferee, SimRegister, SimRequest, SimResponse,
  Vector2, Vector3, sim_referee,
};

#[derive(Debug, Clone)]
pub struct SumatraSimNetConfig {
  pub bind_addr: SocketAddr,
}

impl Default for SumatraSimNetConfig {
  fn default() -> Self {
    Self {
      bind_addr: "127.0.0.1:14242".parse().expect("valid default bind addr"),
    }
  }
}

pub struct SumatraSimNetServer {
  listener: TcpListener,
  clients: Vec<SumatraClient>,
  previous_ball_sample: Option<PreviousBallSample>,
  world_index: usize,
}

struct SumatraClient {
  stream: TcpStream,
  teams: HashSet<TeamColor>,
  read_buf: Vec<u8>,
  registered: bool,
}

impl SumatraSimNetServer {
  /// The optional backend supplies a tracker snapshot, never a physics World.
  #[cfg(feature = "realistic")]
  pub fn exchange_observed(
    &mut self,
    observed: &WorldState,
    config: &simhark::WorldConfig,
  ) -> Result<WorldCommand> {
    self.accept_new_clients()?;
    self.publish_state(observed)?;
    let actions = self.receive_actions(
      observed,
      TeamConfigs {
        blue: &config.blue_robots,
        yellow: &config.yellow_robots,
      },
    )?;
    Ok(merge_actions_into_world_command(actions))
  }
  pub fn bind(config: SumatraSimNetConfig) -> Result<Self> {
    let listener = TcpListener::bind(config.bind_addr)?;
    listener.set_nonblocking(true)?;
    Ok(Self {
      listener,
      clients: Vec::new(),
      previous_ball_sample: None,
      world_index: 0,
    })
  }

  pub fn bind_for_world(config: SumatraSimNetConfig, world_index: usize) -> Result<Self> {
    let mut server = Self::bind(config)?;
    server.world_index = world_index;
    Ok(server)
  }

  pub fn step(&mut self, engine: &mut SimulationEngine) -> Result<Vec<WorldState>> {
    self.accept_new_clients()?;
    let states = engine.get_all_states();
    if let Some(state) = states.get(self.world_index) {
      self.publish_state(state)?;
      let (blue, yellow) = engine.world(self.world_index).team_configs();
      let actions = self.receive_actions(state, TeamConfigs { blue, yellow })?;
      let command = merge_actions_into_world_command(actions);
      return Ok(engine.step_subset(&[self.world_index], &[command]));
    }
    Ok(Vec::new())
  }

  pub fn step_with_local_commands(
    &mut self,
    engine: &mut SimulationEngine,
    local_commands: &[WorldCommand],
  ) -> Result<Vec<WorldState>> {
    self.accept_new_clients()?;
    let states = engine.get_all_states();
    if let Some(state) = states.get(self.world_index) {
      self.publish_state(state)?;
      let (blue, yellow) = engine.world(self.world_index).team_configs();
      let actions = self.receive_actions(state, TeamConfigs { blue, yellow })?;
      let mut command = local_commands
        .get(self.world_index)
        .cloned()
        .unwrap_or_default();
      merge_actions_into_existing_world_command(&mut command, actions);
      return Ok(engine.step_subset(&[self.world_index], &[command]));
    }
    Ok(Vec::new())
  }

  pub fn has_clients(&mut self) -> Result<bool> {
    self.accept_new_clients()?;
    Ok(!self.clients.is_empty())
  }

  pub fn reset_tracking(&mut self) {
    self.previous_ball_sample = None;
  }

  fn accept_new_clients(&mut self) -> Result<()> {
    loop {
      match self.listener.accept() {
        Ok((stream, _)) => {
          stream.set_nodelay(true)?;
          stream.set_nonblocking(true)?;
          self.clients.push(SumatraClient {
            stream,
            teams: HashSet::new(),
            read_buf: Vec::new(),
            registered: false,
          });
        }
        Err(err) if err.kind() == ErrorKind::WouldBlock => return Ok(()),
        Err(err) => return Err(err),
      }
    }
  }

  fn publish_state(&mut self, state: &WorldState) -> Result<()> {
    let request = build_request(state, &mut self.previous_ball_sample);
    self
      .clients
      .retain_mut(|client| match ensure_client_registered(client) {
        Ok(()) => write_length_delimited(&mut client.stream, &request).is_ok(),
        Err(err) if err.kind() == ErrorKind::WouldBlock => true,
        Err(_) => false,
      });
    Ok(())
  }

  fn receive_actions(
    &mut self,
    state: &WorldState,
    team_config: TeamConfigs<'_>,
  ) -> Result<HashMap<(TeamColor, usize), RobotCommand>> {
    let mut actions = HashMap::new();
    self.clients.retain_mut(|client| {
      match ensure_client_registered(client) {
        Ok(()) => {}
        Err(err) if err.kind() == ErrorKind::WouldBlock => return true,
        Err(_) => return false,
      }
      match read_available(&mut client.stream, &mut client.read_buf) {
        Ok(false) => false,
        Ok(true) => loop {
          match try_take_length_delimited::<SimResponse>(&mut client.read_buf) {
            Ok(Some(response)) => {
              for action in response.action {
                if let Some((key, command)) =
                  decode_action(action, &client.teams, state, team_config)
                {
                  actions.insert(key, command);
                }
              }
            }
            Ok(None) => return true,
            Err(_) => return false,
          }
        },
        Err(err) if err.kind() == ErrorKind::WouldBlock => true,
        Err(_) => false,
      }
    });
    Ok(actions)
  }
}

#[derive(Clone, Copy)]
struct TeamConfigs<'a> {
  blue: &'a RobotConfig,
  yellow: &'a RobotConfig,
}

fn build_request(
  state: &WorldState,
  previous_ball_sample: &mut Option<PreviousBallSample>,
) -> SimRequest {
  let ball_motion = estimate_ball_motion(state, previous_ball_sample);
  SimRequest {
    timestamp: (state.sim_time * 1_000_000_000.0) as i64,
    frame_id: state.frame as i64,
    bot_state: state
      .blue_robots
      .iter()
      .map(|robot| bot_state(robot, TeamColor::Blue))
      .chain(
        state
          .yellow_robots
          .iter()
          .map(|robot| bot_state(robot, TeamColor::Yellow)),
      )
      .collect(),
    ball_state: Some(SimBallState {
      pose: Some(Vector3 {
        x: state.ball.x * 1000.0,
        y: state.ball.y * 1000.0,
        z: state.ball.z * 1000.0,
      }),
      vel: Some(Vector3 {
        x: state.ball.vx * 1000.0,
        y: state.ball.vy * 1000.0,
        z: state.ball.vz * 1000.0,
      }),
      acc: Some(Vector3 {
        x: ball_motion.acc.0 * 1000.0,
        y: ball_motion.acc.1 * 1000.0,
        z: ball_motion.acc.2 * 1000.0,
      }),
      spin: Some(Vector2 {
        x: ball_motion.spin.0,
        y: ball_motion.spin.1,
      }),
    }),
    last_kick_event: None,
    referee_message: Some(default_referee(state)),
  }
}

fn bot_state(robot: &simhark::state::RobotState, team: TeamColor) -> SimBotState {
  SimBotState {
    bot_id: Some(BotId {
      id: robot.id as i32,
      color: local_team_to_proto(team),
    }),
    pose: Some(Vector3 {
      x: robot.x * 1000.0,
      y: robot.y * 1000.0,
      z: robot.orientation,
    }),
    vel: Some(Vector3 {
      x: robot.vx,
      y: robot.vy,
      z: robot.v_angular,
    }),
    barrier_interrupted: robot.infrared,
  }
}

fn default_referee(state: &WorldState) -> SimReferee {
  SimReferee {
    packet_timestamp: (state.sim_time * 1_000_000_000.0) as u64,
    stage: sim_referee::Stage::NormalFirstHalf as i32,
    stage_time_left: Some(0),
    command: sim_referee::Command::ForceStart as i32,
    command_counter: state.frame as u32,
    command_timestamp: (state.sim_time * 1_000_000_000.0) as u64,
    yellow: sim_referee::TeamInfo {
      name: "Yellow".into(),
      score: 0,
      red_cards: 0,
      yellow_card_times: Vec::new(),
      yellow_cards: 0,
      timeouts: 0,
      timeout_time: 0,
      goalie: 0,
    },
    blue: sim_referee::TeamInfo {
      name: "Blue".into(),
      score: 0,
      red_cards: 0,
      yellow_card_times: Vec::new(),
      yellow_cards: 0,
      timeouts: 0,
      timeout_time: 0,
      goalie: 0,
    },
    designated_position: None,
    blue_team_on_positive_half: Some(false),
  }
}

fn decode_action(
  action: SimBotAction,
  registered_teams: &HashSet<TeamColor>,
  state: &WorldState,
  team_config: TeamConfigs<'_>,
) -> Option<((TeamColor, usize), RobotCommand)> {
  let bot_id = action.bot_id?;
  let team = proto_team_to_local(bot_id.color)?;
  if !registered_teams.contains(&team) {
    return None;
  }
  let robot = robot_state(state, team, bot_id.id as usize)?;
  let mode_xy = DriveMode::try_from(action.mode_xy).ok()?;
  let mode_w = DriveMode::try_from(action.mode_w).ok()?;
  if env::var_os("SIMHARK_SUMATRA_TRACE_ACTIONS").is_some() {
    eprintln!(
      "sumatra-action team={team:?} id={} mode_xy={mode_xy:?} mode_w={mode_w:?} target_pos={:?} target_vel_local={:?} kick_speed={:.2} dribble_rpm={:.1} disarm={}",
      bot_id.id,
      action.target_pos,
      action.target_vel_local,
      action.kick_speed,
      action.dribble_rpm,
      action.disarm,
    );
  }
  let move_command = match mode_xy {
    DriveMode::WheelVel => action.target_wheel_vel.as_ref().map(|wheel| {
      let mut values = [0.0; 4];
      for (index, value) in wheel.x.iter().copied().take(4).enumerate() {
        values[index] = value;
      }
      let robot_cfg = match team {
        TeamColor::Blue => team_config.blue,
        TeamColor::Yellow => team_config.yellow,
      };
      let (forward, left, angular) = decode_sumatra_wheel_velocity(values, robot_cfg);
      MoveCommand::LocalVelocity {
        forward,
        left,
        angular,
      }
    }),
    _ => decode_velocity_action(&action, robot, mode_xy, mode_w),
  };
  Some((
    (team, bot_id.id as usize),
    RobotCommand {
      id: bot_id.id as usize,
      move_command,
      kick_speed: if action.disarm {
        0.0
      } else {
        action.kick_speed * 0.001
      },
      kick_angle: if action.chip { 45.0 } else { 0.0 },
      dribbler_on: action.dribble_rpm > 0.0,
    },
  ))
}

fn decode_velocity_action(
  action: &SimBotAction,
  robot: &simhark::state::RobotState,
  mode_xy: DriveMode,
  mode_w: DriveMode,
) -> Option<MoveCommand> {
  let angular = decode_angular_velocity(action, robot, mode_w)?;
  match mode_xy {
    DriveMode::Off => {
      if angular.abs() > 1e-6 {
        Some(MoveCommand::LocalVelocity {
          forward: 0.0,
          left: 0.0,
          angular,
        })
      } else {
        None
      }
    }
    DriveMode::LocalVel => action
      .target_vel_local
      .as_ref()
      .map(|vel| MoveCommand::LocalVelocity {
        forward: vel.x * 0.001,
        left: vel.y * 0.001,
        angular,
      }),
    DriveMode::GlobalPos => {
      let target = action.target_pos.as_ref()?;
      let limits = action.drive_limits.as_ref();
      let target_x = target.x * 0.001;
      let target_y = target.y * 0.001;
      let (vx, vy) = damped_global_velocity(
        (target_x - robot.x, target_y - robot.y),
        (robot.vx, robot.vy),
        action.primary_direction.as_ref().map(|dir| (dir.x, dir.y)),
        limits.map_or(3.0, |limits| limits.vel_max),
        limits.map_or(4.0, |limits| limits.acc_max),
      );
      Some(MoveCommand::GlobalVelocity { vx, vy, angular })
    }
    DriveMode::WheelVel => None,
  }
}

fn decode_angular_velocity(
  action: &SimBotAction,
  robot: &simhark::state::RobotState,
  mode_w: DriveMode,
) -> Option<f64> {
  match mode_w {
    DriveMode::Off | DriveMode::WheelVel => Some(0.0),
    DriveMode::LocalVel => Some(action.target_vel_local.as_ref()?.z),
    DriveMode::GlobalPos => {
      let target = action.target_pos.as_ref()?;
      Some(damped_angular_velocity(
        normalize_angle(target.z - robot.orientation),
        robot.v_angular,
        action
          .drive_limits
          .as_ref()
          .map_or(10.0, |limits| limits.vel_max_w),
        action
          .drive_limits
          .as_ref()
          .map_or(50.0, |limits| limits.acc_max_w),
      ))
    }
  }
}

fn damped_global_velocity(
  position_error: (f64, f64),
  current_velocity: (f64, f64),
  primary_direction: Option<(f64, f64)>,
  vel_limit: f64,
  acc_limit: f64,
) -> (f64, f64) {
  let vel_limit = vel_limit.max(0.1);
  let acc_limit = acc_limit.max(0.1);
  let distance = (position_error.0 * position_error.0 + position_error.1 * position_error.1).sqrt();
  if distance <= 1e-5 {
    return (0.0, 0.0);
  }

  let mut dir_x = position_error.0 / distance;
  let mut dir_y = position_error.1 / distance;
  if let Some((px, py)) = primary_direction {
    let mag = (px * px + py * py).sqrt();
    if mag > 1e-6 {
      let primary_x = px / mag;
      let primary_y = py / mag;
      let alignment = dir_x * primary_x + dir_y * primary_y;
      if alignment > -0.25 {
        dir_x = (dir_x + primary_x * 0.35).clamp(-1.0, 1.0);
        dir_y = (dir_y + primary_y * 0.35).clamp(-1.0, 1.0);
        let norm = (dir_x * dir_x + dir_y * dir_y).sqrt();
        if norm > 1e-6 {
          dir_x /= norm;
          dir_y /= norm;
        }
      }
    }
  }
  let braking_speed = (2.0 * acc_limit * distance).sqrt().min(vel_limit);
  let current_speed_along = current_velocity.0 * dir_x + current_velocity.1 * dir_y;
  let requested_speed =
    (distance * 4.0 - current_speed_along * 0.75).clamp(-braking_speed, braking_speed);

  (dir_x * requested_speed, dir_y * requested_speed)
}

fn damped_angular_velocity(
  angle_error: f64,
  current_angular_velocity: f64,
  vel_limit: f64,
  acc_limit: f64,
) -> f64 {
  let vel_limit = vel_limit.max(0.1);
  let acc_limit = acc_limit.max(0.1);
  if angle_error.abs() <= 1e-5 {
    return 0.0;
  }

  let braking_speed = (2.0 * acc_limit * angle_error.abs()).sqrt().min(vel_limit);
  (angle_error * 6.0 - current_angular_velocity * 0.35).clamp(-braking_speed, braking_speed)
}

fn robot_state(
  state: &WorldState,
  team: TeamColor,
  id: usize,
) -> Option<&simhark::state::RobotState> {
  let robots = match team {
    TeamColor::Blue => &state.blue_robots,
    TeamColor::Yellow => &state.yellow_robots,
  };
  robots.iter().find(|robot| robot.id == id)
}

fn normalize_angle(angle: f64) -> f64 {
  angle.sin().atan2(angle.cos())
}

fn decode_sumatra_wheel_velocity(wheels: [f64; 4], robot_cfg: &RobotConfig) -> (f64, f64, f64) {
  let angles = robot_cfg.wheel_angles.map(f64::to_radians);

  // Matches Sumatra's MatrixMotorModel pseudoinverse mapping.
  let mut d = [[0.0; 3]; 4];
  for (row, angle) in d.iter_mut().zip(angles) {
    row[0] = -angle.sin();
    row[1] = angle.cos();
    row[2] = robot_cfg.radius;
  }

  let mut gram = [[0.0; 3]; 3];
  for row in d {
    for i in 0..3 {
      for j in 0..3 {
        gram[i][j] += row[i] * row[j];
      }
    }
  }

  let inv = invert_3x3(gram).unwrap_or([[0.0; 3]; 3]);

  let mut dt_wheels = [0.0; 3];
  for (row, wheel) in d.into_iter().zip(wheels) {
    for i in 0..3 {
      dt_wheels[i] += row[i] * wheel;
    }
  }

  let mut body = [0.0; 3];
  for i in 0..3 {
    for (j, value) in dt_wheels.iter().enumerate() {
      body[i] += inv[i][j] * value;
    }
    body[i] *= robot_cfg.wheel_radius;
  }

  (body[0], body[1], body[2])
}

fn invert_3x3(matrix: [[f64; 3]; 3]) -> Option<[[f64; 3]; 3]> {
  let det = matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
    - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
    + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
  if det.abs() <= f64::EPSILON {
    return None;
  }
  let inv_det = 1.0 / det;
  Some([
    [
      (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) * inv_det,
      (matrix[0][2] * matrix[2][1] - matrix[0][1] * matrix[2][2]) * inv_det,
      (matrix[0][1] * matrix[1][2] - matrix[0][2] * matrix[1][1]) * inv_det,
    ],
    [
      (matrix[1][2] * matrix[2][0] - matrix[1][0] * matrix[2][2]) * inv_det,
      (matrix[0][0] * matrix[2][2] - matrix[0][2] * matrix[2][0]) * inv_det,
      (matrix[0][2] * matrix[1][0] - matrix[0][0] * matrix[1][2]) * inv_det,
    ],
    [
      (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]) * inv_det,
      (matrix[0][1] * matrix[2][0] - matrix[0][0] * matrix[2][1]) * inv_det,
      (matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]) * inv_det,
    ],
  ])
}

#[derive(Clone, Copy)]
struct BallMotionEstimate {
  acc: (f64, f64, f64),
  spin: (f64, f64),
}

#[derive(Clone, Copy)]
struct PreviousBallSample {
  sim_time: f64,
  vx: f64,
  vy: f64,
  vz: f64,
}

fn estimate_ball_motion(
  state: &WorldState,
  previous_ball_sample: &mut Option<PreviousBallSample>,
) -> BallMotionEstimate {
  let dt = previous_ball_sample
    .as_ref()
    .map(|sample| state.sim_time - sample.sim_time)
    .filter(|dt| *dt > f64::EPSILON)
    .unwrap_or(0.0);

  let acc = previous_ball_sample
    .as_ref()
    .filter(|_| dt > 0.0)
    .map(|sample| {
      (
        (state.ball.vx - sample.vx) / dt,
        (state.ball.vy - sample.vy) / dt,
        (state.ball.vz - sample.vz) / dt,
      )
    })
    .unwrap_or((0.0, 0.0, 0.0));

  *previous_ball_sample = Some(PreviousBallSample {
    sim_time: state.sim_time,
    vx: state.ball.vx,
    vy: state.ball.vy,
    vz: state.ball.vz,
  });

  let planar_speed = (state.ball.vx * state.ball.vx + state.ball.vy * state.ball.vy).sqrt();
  let spin = if planar_speed > 1e-5 {
    let inv_radius = 1.0 / 0.0215;
    (-state.ball.vy * inv_radius, state.ball.vx * inv_radius)
  } else {
    (0.0, 0.0)
  };

  BallMotionEstimate { acc, spin }
}

fn ensure_client_registered(client: &mut SumatraClient) -> Result<()> {
  if client.registered {
    return Ok(());
  }

  read_available(&mut client.stream, &mut client.read_buf)?;
  let Some(register) = try_take_length_delimited::<SimRegister>(&mut client.read_buf)? else {
    return Err(std::io::Error::new(
      ErrorKind::WouldBlock,
      "waiting for Sumatra registration",
    ));
  };
  client.teams = register
    .team_color
    .into_iter()
    .filter_map(proto_team_to_local)
    .collect::<HashSet<_>>();
  client.registered = true;
  Ok(())
}

fn merge_actions_into_world_command(
  actions: HashMap<(TeamColor, usize), RobotCommand>,
) -> WorldCommand {
  let mut command = WorldCommand::default();
  merge_actions_into_existing_world_command(&mut command, actions);
  command
}

fn merge_actions_into_existing_world_command(
  command: &mut WorldCommand,
  actions: HashMap<(TeamColor, usize), RobotCommand>,
) {
  for ((team, _), robot_command) in actions {
    match team {
      TeamColor::Blue => command.blue.push(robot_command),
      TeamColor::Yellow => command.yellow.push(robot_command),
    }
  }
}

fn proto_team_to_local(value: i32) -> Option<TeamColor> {
  match crate::proto::TeamColor::try_from(value).ok()? {
    crate::proto::TeamColor::Yellow => Some(TeamColor::Yellow),
    crate::proto::TeamColor::Blue => Some(TeamColor::Blue),
  }
}

fn local_team_to_proto(value: TeamColor) -> i32 {
  match value {
    TeamColor::Yellow => 0,
    TeamColor::Blue => 1,
  }
}

fn decode_error(err: impl std::error::Error) -> std::io::Error {
  std::io::Error::new(ErrorKind::InvalidData, err.to_string())
}

fn read_available(stream: &mut TcpStream, buffer: &mut Vec<u8>) -> Result<bool> {
  let mut temp = [0u8; 4096];
  loop {
    match stream.read(&mut temp) {
      Ok(0) => return Ok(false),
      Ok(read) => buffer.extend_from_slice(&temp[..read]),
      Err(err) if err.kind() == ErrorKind::WouldBlock => return Ok(true),
      Err(err) if err.kind() == ErrorKind::Interrupted => continue,
      Err(err) => return Err(err),
    }
  }
}

fn try_take_length_delimited<M: Message + Default>(buffer: &mut Vec<u8>) -> Result<Option<M>> {
  let Some((len, header_len)) = try_read_varint_from_slice(buffer.as_slice())? else {
    return Ok(None);
  };
  let total_len = header_len + len as usize;
  if buffer.len() < total_len {
    return Ok(None);
  }

  let message = M::decode(&buffer[header_len..total_len]).map_err(decode_error)?;
  buffer.drain(..total_len);
  Ok(Some(message))
}

fn write_length_delimited<M: Message>(stream: &mut TcpStream, message: &M) -> Result<()> {
  let mut payload = Vec::new();
  message.encode(&mut payload).map_err(decode_error)?;
  write_varint(stream, payload.len() as u64)?;
  stream.write_all(&payload)?;
  Ok(())
}

fn try_read_varint_from_slice(buf: &[u8]) -> Result<Option<(u64, usize)>> {
  let mut value = 0u64;
  let mut shift = 0u32;
  for (index, byte) in buf.iter().copied().enumerate() {
    value |= ((byte & 0x7f) as u64) << shift;
    if byte & 0x80 == 0 {
      return Ok(Some((value, index + 1)));
    }
    shift += 7;
    if shift >= 64 {
      return Err(std::io::Error::new(
        ErrorKind::InvalidData,
        "varint too long",
      ));
    }
  }
  Ok(None)
}

fn write_varint(stream: &mut TcpStream, mut value: u64) -> Result<()> {
  let mut buf = [0u8; 10];
  let mut index = 0usize;
  loop {
    let mut byte = (value & 0x7f) as u8;
    value >>= 7;
    if value != 0 {
      byte |= 0x80;
    }
    buf[index] = byte;
    index += 1;
    if value == 0 {
      break;
    }
  }
  stream.write_all(&buf[..index])
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn decodes_wheel_velocity_with_robot_geometry() {
    let mut cfg = RobotConfig::default();
    cfg.radius = 0.09;
    cfg.wheel_radius = 0.027;
    cfg.wheel_angles = [30.0, 150.0, 225.0, 315.0];

    let (vx, vy, vw) = decode_sumatra_wheel_velocity([10.0, 10.0, 10.0, 10.0], &cfg);
    assert!(vx.abs() < 1e-9);
    assert!(vy.abs() < 1e-9);
    assert!(vw > 0.0);
  }

  #[test]
  fn estimate_ball_motion_is_local_to_server_state() {
    let state = WorldState {
      world_id: 0,
      sim_time: 1.0,
      frame: 1,
      ball: simhark::state::BallState {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        vx: 1.0,
        vy: 0.0,
        vz: 0.0,
      },
      blue_robots: Vec::new(),
      yellow_robots: Vec::new(),
      goal_blue: false,
      goal_yellow: false,
    };

    let mut first_cache = None;
    let mut second_cache = None;
    let _ = estimate_ball_motion(&state, &mut first_cache);
    let estimate = estimate_ball_motion(&state, &mut second_cache);
    assert_eq!(estimate.acc, (0.0, 0.0, 0.0));
    assert!(second_cache.is_some());
  }
}
