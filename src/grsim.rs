//! grSim-compatible UDP/protobuf API surface.

use std::io::{ErrorKind, Result};
use std::net::{IpAddr, Ipv4Addr, SocketAddr, UdpSocket};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use prost::Message;

use crate::command::{
  MoveCommand, RobotCommand, TeleportBall as WorldTeleportBall,
  TeleportRobot as WorldTeleportRobot, WorldCommand,
};
use crate::config::{FieldConfig, RobotConfig, WorldConfig};
use crate::engine::SimulationEngine;
use crate::proto::{
  GrSimCommands, GrSimPacket, GrSimReplacement, GrSimRobotCommand, RobotControl,
  RobotControlResponse, RobotFeedback, RobotLimits, RobotSpecs, RobotStatus, RobotsStatus,
  SimulatorCommand, SimulatorError, SimulatorResponse, SslDetectionBall, SslDetectionFrame,
  SslDetectionRobot, SslFieldCircularArc, SslFieldLineSegment, SslGeometryData,
  SslGeometryFieldSize, SslWrapperPacket, Team, robot_move_command,
};
use crate::state::{KickStatus, TeamColor, WorldState};

const DEFAULT_VISION_ADDR: Ipv4Addr = Ipv4Addr::new(224, 5, 23, 2);
const DEFAULT_VISION_PORT: u16 = 10020;
const DEFAULT_COMMAND_PORT: u16 = 20011;
const DEFAULT_BLUE_STATUS_PORT: u16 = 30011;
const DEFAULT_YELLOW_STATUS_PORT: u16 = 30012;
const DEFAULT_SIM_CONTROL_PORT: u16 = 10300;
const DEFAULT_BLUE_CONTROL_PORT: u16 = 10301;
const DEFAULT_YELLOW_CONTROL_PORT: u16 = 10302;

#[derive(Debug, Clone)]
pub struct GrSimCompatConfig {
  pub bind_ip: IpAddr,
  pub vision_addr: Ipv4Addr,
  pub vision_port: u16,
  pub command_port: u16,
  pub blue_status_port: u16,
  pub yellow_status_port: u16,
  pub sim_control_port: u16,
  pub blue_control_port: u16,
  pub yellow_control_port: u16,
  pub read_timeout: Duration,
}

impl Default for GrSimCompatConfig {
  fn default() -> Self {
    Self {
      bind_ip: IpAddr::V4(Ipv4Addr::UNSPECIFIED),
      vision_addr: DEFAULT_VISION_ADDR,
      vision_port: DEFAULT_VISION_PORT,
      command_port: DEFAULT_COMMAND_PORT,
      blue_status_port: DEFAULT_BLUE_STATUS_PORT,
      yellow_status_port: DEFAULT_YELLOW_STATUS_PORT,
      sim_control_port: DEFAULT_SIM_CONTROL_PORT,
      blue_control_port: DEFAULT_BLUE_CONTROL_PORT,
      yellow_control_port: DEFAULT_YELLOW_CONTROL_PORT,
      read_timeout: Duration::from_millis(2),
    }
  }
}

pub struct GrSimCompatServer {
  config: GrSimCompatConfig,
  command_socket: UdpSocket,
  sim_control_socket: UdpSocket,
  blue_control_socket: UdpSocket,
  yellow_control_socket: UdpSocket,
  blue_status_socket: UdpSocket,
  yellow_status_socket: UdpSocket,
  vision_socket: UdpSocket,
  last_blue_command: Vec<RobotCommand>,
  last_yellow_command: Vec<RobotCommand>,
  last_blue_status: Vec<RobotStatusCache>,
  last_yellow_status: Vec<RobotStatusCache>,
  blue_status_target: Option<SocketAddr>,
  yellow_status_target: Option<SocketAddr>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct RobotStatusCache {
  infrared: bool,
  kick_status: KickStatus,
}

impl RobotStatusCache {
  fn from_state(state: &crate::state::RobotState) -> Self {
    Self {
      infrared: state.infrared,
      kick_status: state.kick_status,
    }
  }
}

impl GrSimCompatServer {
  pub fn bind(config: GrSimCompatConfig) -> Result<Self> {
    let command_socket = bind_socket(config.bind_ip, config.command_port, config.read_timeout)?;
    let sim_control_socket =
      bind_socket(config.bind_ip, config.sim_control_port, config.read_timeout)?;
    let blue_control_socket = bind_socket(
      config.bind_ip,
      config.blue_control_port,
      config.read_timeout,
    )?;
    let yellow_control_socket = bind_socket(
      config.bind_ip,
      config.yellow_control_port,
      config.read_timeout,
    )?;
    let blue_status_socket = bind_socket(config.bind_ip, 0, config.read_timeout)?;
    let yellow_status_socket = bind_socket(config.bind_ip, 0, config.read_timeout)?;
    let vision_socket = bind_socket(config.bind_ip, 0, config.read_timeout)?;
    vision_socket.set_multicast_loop_v4(true)?;

    Ok(Self {
      config,
      command_socket,
      sim_control_socket,
      blue_control_socket,
      yellow_control_socket,
      blue_status_socket,
      yellow_status_socket,
      vision_socket,
      last_blue_command: Vec::new(),
      last_yellow_command: Vec::new(),
      last_blue_status: Vec::new(),
      last_yellow_status: Vec::new(),
      blue_status_target: None,
      yellow_status_target: None,
    })
  }

  pub fn step(&mut self, engine: &mut SimulationEngine) -> Result<Vec<WorldState>> {
    if engine.count() == 0 {
      return Ok(Vec::new());
    }

    let mut frame_command = WorldCommand {
      blue: self.last_blue_command.clone(),
      yellow: self.last_yellow_command.clone(),
      ..Default::default()
    };

    self.handle_legacy_packets(&mut frame_command)?;
    self.handle_sim_control_packets(engine, &mut frame_command)?;

    let world_config = engine.world(0).config.clone();
    let current_state = engine.world(0).get_state();
    self.handle_robot_control_packets(
      TeamColor::Blue,
      &world_config.blue_robots,
      &current_state,
      &mut frame_command,
    )?;
    self.handle_robot_control_packets(
      TeamColor::Yellow,
      &world_config.yellow_robots,
      &current_state,
      &mut frame_command,
    )?;

    let states = engine.step_all_same(&frame_command);
    if let Some(state) = states.first() {
      self.publish_status_and_vision(state, &world_config)?;
    }

    Ok(states)
  }

  pub fn run_steps(
    &mut self,
    engine: &mut SimulationEngine,
    steps: usize,
  ) -> Result<Vec<WorldState>> {
    let mut last_states = Vec::new();
    for _ in 0..steps {
      last_states = self.step(engine)?;
    }
    Ok(last_states)
  }

  fn handle_legacy_packets(&mut self, frame_command: &mut WorldCommand) -> Result<()> {
    let mut buf = [0_u8; 65535];
    loop {
      match self.command_socket.recv_from(&mut buf) {
        Ok((size, peer)) => {
          let packet = GrSimPacket::decode(&buf[..size]).map_err(decode_error)?;
          if let Some(commands) = packet.commands {
            let (team, cmds) = world_commands_from_grsim(commands);
            match team {
              TeamColor::Blue => {
                self.last_blue_command = cmds.clone();
                frame_command.blue = cmds;
                self.blue_status_target = Some(peer);
              }
              TeamColor::Yellow => {
                self.last_yellow_command = cmds.clone();
                frame_command.yellow = cmds;
                self.yellow_status_target = Some(peer);
              }
            }
          }
          if let Some(replacement) = packet.replacement {
            apply_grsim_replacement(replacement, frame_command);
          }
        }
        Err(err) if would_block(&err) => return Ok(()),
        Err(err) => return Err(err),
      }
    }
  }

  fn handle_sim_control_packets(
    &mut self,
    engine: &mut SimulationEngine,
    frame_command: &mut WorldCommand,
  ) -> Result<()> {
    let mut buf = [0_u8; 65535];
    loop {
      match self.sim_control_socket.recv_from(&mut buf) {
        Ok((size, peer)) => {
          let command = SimulatorCommand::decode(&buf[..size]).map_err(decode_error)?;
          let response =
            process_simulator_command(engine, command, frame_command, &mut self.config);
          let mut out = Vec::new();
          response.encode(&mut out).map_err(decode_error)?;
          self.sim_control_socket.send_to(&out, peer)?;
        }
        Err(err) if would_block(&err) => return Ok(()),
        Err(err) => return Err(err),
      }
    }
  }

  fn handle_robot_control_packets(
    &mut self,
    team: TeamColor,
    robot_cfg: &RobotConfig,
    state: &WorldState,
    frame_command: &mut WorldCommand,
  ) -> Result<()> {
    let socket = match team {
      TeamColor::Blue => &self.blue_control_socket,
      TeamColor::Yellow => &self.yellow_control_socket,
    };

    let mut buf = [0_u8; 65535];
    loop {
      match socket.recv_from(&mut buf) {
        Ok((size, peer)) => {
          let command = RobotControl::decode(&buf[..size]).map_err(decode_error)?;
          let commands = world_commands_from_robot_control(command, robot_cfg);
          match team {
            TeamColor::Blue => {
              self.last_blue_command = commands.clone();
              frame_command.blue = commands.clone();
              self.blue_status_target = Some(peer);
            }
            TeamColor::Yellow => {
              self.last_yellow_command = commands.clone();
              frame_command.yellow = commands.clone();
              self.yellow_status_target = Some(peer);
            }
          }

          let response = RobotControlResponse {
            errors: Vec::new(),
            feedback: commands
              .into_iter()
              .map(|cmd| RobotFeedback {
                id: cmd.id as u32,
                dribbler_ball_contact: Some(robot_has_ball_contact(state, team, cmd.id)),
                custom: None,
              })
              .collect(),
          };
          let mut out = Vec::new();
          response.encode(&mut out).map_err(decode_error)?;
          socket.send_to(&out, peer)?;
        }
        Err(err) if would_block(&err) => return Ok(()),
        Err(err) => return Err(err),
      }
    }
  }

  fn publish_status_and_vision(&mut self, state: &WorldState, config: &WorldConfig) -> Result<()> {
    self.send_status_if_changed(TeamColor::Blue, &state.blue_robots)?;
    self.send_status_if_changed(TeamColor::Yellow, &state.yellow_robots)?;
    self.send_vision_packet(state, config)?;
    Ok(())
  }

  fn send_status_if_changed(
    &mut self,
    team: TeamColor,
    robots: &[crate::state::RobotState],
  ) -> Result<()> {
    let new_cache: Vec<_> = robots.iter().map(RobotStatusCache::from_state).collect();
    let last_cache = match team {
      TeamColor::Blue => &mut self.last_blue_status,
      TeamColor::Yellow => &mut self.last_yellow_status,
    };

    if *last_cache == new_cache {
      return Ok(());
    }

    let packet = RobotsStatus {
      robots_status: robots
        .iter()
        .map(|robot| RobotStatus {
          robot_id: robot.id as i32,
          infrared: robot.infrared,
          flat_kick: robot.kick_status == KickStatus::FlatKick,
          chip_kick: robot.kick_status == KickStatus::ChipKick,
        })
        .collect(),
    };

    let mut out = Vec::new();
    packet.encode(&mut out).map_err(decode_error)?;
    let port = match team {
      TeamColor::Blue => self.config.blue_status_port,
      TeamColor::Yellow => self.config.yellow_status_port,
    };
    let socket = match team {
      TeamColor::Blue => &self.blue_status_socket,
      TeamColor::Yellow => &self.yellow_status_socket,
    };
    let target = match team {
      TeamColor::Blue => self.blue_status_target,
      TeamColor::Yellow => self.yellow_status_target,
    };
    let Some(target) = target else {
      return Ok(());
    };
    socket.send_to(&out, SocketAddr::new(target.ip(), port))?;
    *last_cache = new_cache;
    Ok(())
  }

  fn send_vision_packet(&self, state: &WorldState, config: &WorldConfig) -> Result<()> {
    let now = unix_time_seconds();
    let detection = SslDetectionFrame {
      frame_number: state.frame as u32,
      t_capture: now,
      t_sent: now,
      camera_id: 0,
      balls: vec![SslDetectionBall {
        confidence: 1.0,
        area: None,
        x: (state.ball.x * 1000.0) as f32,
        y: (state.ball.y * 1000.0) as f32,
        z: Some((state.ball.z * 1000.0) as f32),
        pixel_x: 0.0,
        pixel_y: 0.0,
      }],
      robots_yellow: detection_robots(&state.yellow_robots),
      robots_blue: detection_robots(&state.blue_robots),
    };

    let packet = SslWrapperPacket {
      detection: Some(detection),
      geometry: Some(geometry_from_config(config)),
    };

    let mut out = Vec::new();
    packet.encode(&mut out).map_err(decode_error)?;
    self.vision_socket.send_to(
      &out,
      SocketAddr::new(IpAddr::V4(self.config.vision_addr), self.config.vision_port),
    )?;
    Ok(())
  }
}

fn bind_socket(bind_ip: IpAddr, port: u16, timeout: Duration) -> Result<UdpSocket> {
  let socket = UdpSocket::bind(SocketAddr::new(bind_ip, port))?;
  socket.set_nonblocking(false)?;
  socket.set_read_timeout(Some(timeout))?;
  Ok(socket)
}

fn would_block(err: &std::io::Error) -> bool {
  matches!(err.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut)
}

fn decode_error(err: impl std::error::Error) -> std::io::Error {
  std::io::Error::new(ErrorKind::InvalidData, err.to_string())
}

fn unix_time_seconds() -> f64 {
  SystemTime::now()
    .duration_since(UNIX_EPOCH)
    .map(|d| d.as_secs_f64())
    .unwrap_or(0.0)
}

fn world_commands_from_grsim(commands: GrSimCommands) -> (TeamColor, Vec<RobotCommand>) {
  let team = if commands.isteamyellow {
    TeamColor::Yellow
  } else {
    TeamColor::Blue
  };
  let robot_commands = commands
    .robot_commands
    .into_iter()
    .map(robot_command_from_grsim)
    .collect();
  (team, robot_commands)
}

fn robot_command_from_grsim(command: GrSimRobotCommand) -> RobotCommand {
  let move_command = if command.wheelsspeed {
    Some(MoveCommand::WheelVelocity([
      command.wheel1.unwrap_or(0.0) as f64,
      command.wheel2.unwrap_or(0.0) as f64,
      command.wheel3.unwrap_or(0.0) as f64,
      command.wheel4.unwrap_or(0.0) as f64,
    ]))
  } else {
    Some(MoveCommand::LocalVelocity {
      forward: command.veltangent as f64,
      left: command.velnormal as f64,
      angular: command.velangular as f64,
    })
  };

  let kick_speed_xy = command.kickspeedx as f64;
  let kick_speed_z = command.kickspeedz as f64;
  let kick_speed = (kick_speed_xy * kick_speed_xy + kick_speed_z * kick_speed_z).sqrt();
  let kick_angle = if kick_speed > 0.0 {
    kick_speed_z.atan2(kick_speed_xy).to_degrees()
  } else {
    0.0
  };

  RobotCommand {
    id: command.id as usize,
    move_command,
    kick_speed,
    kick_angle,
    dribbler_on: command.spinner,
  }
}

fn robot_has_ball_contact(state: &WorldState, team: TeamColor, id: usize) -> bool {
  let robots = match team {
    TeamColor::Blue => &state.blue_robots,
    TeamColor::Yellow => &state.yellow_robots,
  };
  robots
    .iter()
    .find(|robot| robot.id == id)
    .is_some_and(|robot| robot.infrared)
}

fn apply_grsim_replacement(replacement: GrSimReplacement, frame_command: &mut WorldCommand) {
  if let Some(ball) = replacement.ball {
    frame_command.teleport_ball = Some(WorldTeleportBall {
      x: ball.x,
      y: ball.y,
      z: Some(0.0),
      vx: ball.vx,
      vy: ball.vy,
      vz: Some(0.0),
    });
  }

  frame_command
    .teleport_robots
    .extend(
      replacement
        .robots
        .into_iter()
        .map(|robot| WorldTeleportRobot {
          id: robot.id as usize,
          team: if robot.yellowteam {
            TeamColor::Yellow
          } else {
            TeamColor::Blue
          },
          x: Some(robot.x),
          y: Some(robot.y),
          orientation: Some(robot.dir),
          vx: Some(0.0),
          vy: Some(0.0),
          v_angular: Some(0.0),
          present: robot.turnon,
        }),
    );
}

fn world_commands_from_robot_control(
  control: RobotControl,
  robot_cfg: &RobotConfig,
) -> Vec<RobotCommand> {
  control
    .robot_commands
    .into_iter()
    .map(|command| {
      let move_command = command
        .move_command
        .and_then(|move_command| move_command_from_robot_control(move_command, robot_cfg));
      RobotCommand {
        id: command.id as usize,
        move_command,
        kick_speed: command.kick_speed.unwrap_or(0.0) as f64,
        kick_angle: command.kick_angle.unwrap_or(0.0) as f64,
        dribbler_on: command.dribbler_speed.unwrap_or(0.0) > 0.0,
      }
    })
    .collect()
}

fn move_command_from_robot_control(
  command: crate::proto::RobotMoveCommand,
  robot_cfg: &RobotConfig,
) -> Option<MoveCommand> {
  match command.command {
    Some(robot_move_command::Command::WheelVelocity(wheels)) => Some(MoveCommand::WheelVelocity([
      wheels.front_right as f64 / robot_cfg.wheel_radius,
      wheels.front_left as f64 / robot_cfg.wheel_radius,
      wheels.back_left as f64 / robot_cfg.wheel_radius,
      wheels.back_right as f64 / robot_cfg.wheel_radius,
    ])),
    Some(robot_move_command::Command::LocalVelocity(vel)) => Some(MoveCommand::LocalVelocity {
      forward: vel.forward as f64,
      left: vel.left as f64,
      angular: vel.angular as f64,
    }),
    Some(robot_move_command::Command::GlobalVelocity(vel)) => Some(MoveCommand::GlobalVelocity {
      vx: vel.x as f64,
      vy: vel.y as f64,
      angular: vel.angular as f64,
    }),
    None => None,
  }
}

fn process_simulator_command(
  engine: &mut SimulationEngine,
  command: SimulatorCommand,
  frame_command: &mut WorldCommand,
  compat_config: &mut GrSimCompatConfig,
) -> SimulatorResponse {
  let mut errors = Vec::new();

  if let Some(control) = command.control {
    if let Some(ball) = control.teleport_ball {
      if ball.teleport_safely.unwrap_or(false) {
        errors.push(sim_error(
          "GRSIM_UNSUPPORTED_TELEPORT_SAFELY",
          "teleport_safely is not supported",
        ));
      }
      if ball.roll.unwrap_or(false) {
        errors.push(sim_error(
          "GRSIM_UNSUPPORTED_ROLL_BALL",
          "roll is not supported",
        ));
      }
      frame_command.teleport_ball = Some(WorldTeleportBall {
        x: ball.x.map(f64::from),
        y: ball.y.map(f64::from),
        z: ball.z.map(f64::from),
        vx: ball.vx.map(f64::from),
        vy: ball.vy.map(f64::from),
        vz: ball.vz.map(f64::from),
      });
    }

    frame_command
      .teleport_robots
      .extend(control.teleport_robot.into_iter().filter_map(|robot| {
        let id = robot.id;
        Some(WorldTeleportRobot {
          id: id.id.unwrap_or(0) as usize,
          team: map_team(id.team),
          x: robot.x.map(f64::from),
          y: robot.y.map(f64::from),
          orientation: robot.orientation.map(f64::from),
          vx: robot.v_x.map(f64::from),
          vy: robot.v_y.map(f64::from),
          v_angular: robot.v_angular.map(f64::from),
          present: robot.present,
        })
      }));

    if control.simulation_speed.is_some() {
      errors.push(sim_error(
        "GRSIM_UNSUPPORTED_SIMULATION_SPEED",
        "simulation_speed is not supported",
      ));
    }
  }

  if let Some(config) = command.config {
    if config.geometry.is_some() {
      errors.push(sim_error(
        "GRSIM_UNSUPPORTED_CONFIG_GEOMETRY",
        "geometry updates are not supported",
      ));
    }
    if config.realism_config.is_some() {
      errors.push(sim_error(
        "GRSIM_UNSUPPORTED_CONFIG_REALISM",
        "realism_config updates are not supported",
      ));
    }
    if let Some(port) = config.vision_port {
      compat_config.vision_port = port as u16;
    }
    if !config.robot_specs.is_empty() {
      let world_count = engine.count();
      let mut base_config = engine.world(0).config.clone();
      for robot_spec in config.robot_specs {
        apply_robot_spec(&mut base_config, robot_spec);
      }
      rebuild_engine(engine, world_count, base_config);
    }
  }

  SimulatorResponse { errors }
}

fn rebuild_engine(engine: &mut SimulationEngine, world_count: usize, config: WorldConfig) {
  *engine = SimulationEngine::new(world_count, config);
}

fn apply_robot_spec(config: &mut WorldConfig, robot_spec: RobotSpecs) {
  let team = map_team(robot_spec.id.team);
  let robot_cfg = match team {
    TeamColor::Blue => &mut config.blue_robots,
    TeamColor::Yellow => &mut config.yellow_robots,
  };

  if let Some(radius) = robot_spec.radius {
    robot_cfg.radius = radius as f64;
  }
  if let Some(height) = robot_spec.height {
    robot_cfg.height = height as f64;
  }
  if let Some(mass) = robot_spec.mass {
    robot_cfg.body_mass = mass as f64;
  }
  if let Some(max_linear_kick_speed) = robot_spec.max_linear_kick_speed {
    robot_cfg.max_linear_kick_speed = max_linear_kick_speed as f64;
  }
  if let Some(max_chip_kick_speed) = robot_spec.max_chip_kick_speed {
    robot_cfg.max_chip_kick_speed = max_chip_kick_speed as f64;
  }
  if let Some(center_to_dribbler) = robot_spec.center_to_dribbler {
    robot_cfg.center_from_kicker = center_to_dribbler as f64;
  }
  if let Some(limits) = robot_spec.limits {
    apply_robot_limits(robot_cfg, limits);
  }
  if let Some(angles) = robot_spec.wheel_angles {
    robot_cfg.wheel_angles = [
      angles.front_right.to_degrees() as f64,
      angles.front_left.to_degrees() as f64,
      angles.back_left.to_degrees() as f64,
      angles.back_right.to_degrees() as f64,
    ];
  }
}

fn apply_robot_limits(robot_cfg: &mut RobotConfig, limits: RobotLimits) {
  if let Some(value) = limits.acc_speedup_absolute_max {
    robot_cfg.acc_speedup_absolute_max = value as f64;
  }
  if let Some(value) = limits.acc_speedup_angular_max {
    robot_cfg.acc_speedup_angular_max = value as f64;
  }
  if let Some(value) = limits.acc_brake_absolute_max {
    robot_cfg.acc_brake_absolute_max = value as f64;
  }
  if let Some(value) = limits.acc_brake_angular_max {
    robot_cfg.acc_brake_angular_max = value as f64;
  }
  if let Some(value) = limits.vel_absolute_max {
    robot_cfg.vel_absolute_max = value as f64;
  }
  if let Some(value) = limits.vel_angular_max {
    robot_cfg.vel_angular_max = value as f64;
  }
}

fn sim_error(code: &str, message: &str) -> SimulatorError {
  SimulatorError {
    code: Some(code.to_string()),
    message: Some(message.to_string()),
  }
}

fn map_team(team: Option<i32>) -> TeamColor {
  match team {
    Some(value) if value == Team::Yellow as i32 => TeamColor::Yellow,
    _ => TeamColor::Blue,
  }
}

fn detection_robots(robots: &[crate::state::RobotState]) -> Vec<SslDetectionRobot> {
  robots
    .iter()
    .filter(|robot| robot.is_on)
    .map(|robot| SslDetectionRobot {
      confidence: 1.0,
      robot_id: Some(robot.id as u32),
      x: (robot.x * 1000.0) as f32,
      y: (robot.y * 1000.0) as f32,
      orientation: Some(robot.orientation as f32),
      pixel_x: 0.0,
      pixel_y: 0.0,
      height: Some((robot.z * 1000.0) as f32),
    })
    .collect()
}

pub fn geometry_from_config(config: &WorldConfig) -> SslGeometryData {
  SslGeometryData {
    field: geometry_field_from_config(&config.field),
    calib: Vec::new(),
    models: None,
  }
}

fn geometry_field_from_config(field: &FieldConfig) -> SslGeometryFieldSize {
  SslGeometryFieldSize {
    field_length: (field.field_length * 1000.0) as i32,
    field_width: (field.field_width * 1000.0) as i32,
    goal_width: (field.goal_width * 1000.0) as i32,
    goal_depth: (field.goal_depth * 1000.0) as i32,
    boundary_width: (field.margin_touch_line * 1000.0) as i32,
    boundary_width_goal_line: Some((field.margin_goal_line * 1000.0) as i32),
    field_lines: Vec::<SslFieldLineSegment>::new(),
    field_arcs: Vec::<SslFieldCircularArc>::new(),
    penalty_area_depth: Some((field.penalty_depth * 1000.0) as i32),
    penalty_area_width: Some((field.penalty_width * 1000.0) as i32),
    goal_substitution_area_width: Some((field.goal_substitution_area_width * 1000.0) as i32),
  }
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::proto::{MoveWheelVelocity, RobotMoveCommand};

  #[test]
  fn translates_legacy_grsim_commands() {
    let (team, commands) = world_commands_from_grsim(GrSimCommands {
      timestamp: 1.0,
      isteamyellow: true,
      robot_commands: vec![GrSimRobotCommand {
        id: 3,
        kickspeedx: 4.0,
        kickspeedz: 3.0,
        veltangent: 1.5,
        velnormal: -0.5,
        velangular: 2.0,
        spinner: true,
        wheelsspeed: false,
        wheel1: None,
        wheel2: None,
        wheel3: None,
        wheel4: None,
      }],
    });

    assert_eq!(team, TeamColor::Yellow);
    assert_eq!(commands.len(), 1);
    assert!(commands[0].dribbler_on);
    assert!((commands[0].kick_speed - 5.0).abs() < 1e-9);
    assert!((commands[0].kick_angle - 36.86989764584402).abs() < 1e-9);
    assert!(matches!(
      commands[0].move_command,
      Some(MoveCommand::LocalVelocity {
        forward: 1.5,
        left: -0.5,
        angular: 2.0,
      })
    ));
  }

  #[test]
  fn imports_robot_specs_via_simulator_config() {
    let mut engine = SimulationEngine::new(2, WorldConfig::division_a());
    let mut frame_command = WorldCommand::default();
    let mut compat_config = GrSimCompatConfig::default();

    let response = process_simulator_command(
      &mut engine,
      SimulatorCommand {
        control: None,
        config: Some(crate::proto::SimulatorConfig {
          geometry: None,
          robot_specs: vec![RobotSpecs {
            id: crate::proto::RobotId {
              id: Some(0),
              team: Some(Team::Blue as i32),
            },
            radius: Some(0.11),
            height: Some(0.14),
            mass: Some(3.2),
            max_linear_kick_speed: Some(8.0),
            max_chip_kick_speed: Some(7.0),
            center_to_dribbler: Some(0.081),
            limits: Some(RobotLimits {
              acc_speedup_absolute_max: Some(6.0),
              acc_speedup_angular_max: Some(60.0),
              acc_brake_absolute_max: Some(5.0),
              acc_brake_angular_max: Some(55.0),
              vel_absolute_max: Some(4.5),
              vel_angular_max: Some(18.0),
            }),
            wheel_angles: None,
            custom: None,
          }],
          realism_config: None,
          vision_port: Some(12000),
        }),
      },
      &mut frame_command,
      &mut compat_config,
    );

    assert!(response.errors.is_empty());
    assert_eq!(engine.count(), 2);
    assert_eq!(compat_config.vision_port, 12000);

    let cfg = &engine.world(0).config.blue_robots;
    assert!((cfg.radius - 0.11).abs() < 1e-6);
    assert!((cfg.height - 0.14).abs() < 1e-6);
    assert!((cfg.body_mass - 3.2).abs() < 1e-6);
    assert!((cfg.max_linear_kick_speed - 8.0).abs() < 1e-6);
    assert!((cfg.max_chip_kick_speed - 7.0).abs() < 1e-6);
    assert!((cfg.center_from_kicker - 0.081).abs() < 1e-6);
    assert!((cfg.acc_speedup_absolute_max - 6.0).abs() < 1e-6);
    assert!((cfg.acc_speedup_angular_max - 60.0).abs() < 1e-6);
    assert!((cfg.acc_brake_absolute_max - 5.0).abs() < 1e-6);
    assert!((cfg.acc_brake_angular_max - 55.0).abs() < 1e-6);
    assert!((cfg.vel_absolute_max - 4.5).abs() < 1e-6);
    assert!((cfg.vel_angular_max - 18.0).abs() < 1e-6);
  }

  #[test]
  fn robot_control_wheel_velocity_uses_linear_units_and_internal_order() {
    let robot_cfg = RobotConfig::default();
    let move_command = move_command_from_robot_control(
      RobotMoveCommand {
        command: Some(robot_move_command::Command::WheelVelocity(
          MoveWheelVelocity {
            front_right: 0.54,
            back_right: 1.08,
            back_left: 1.62,
            front_left: 2.16,
          },
        )),
      },
      &robot_cfg,
    );

    assert!(matches!(move_command, Some(MoveCommand::WheelVelocity(_))));
    let Some(MoveCommand::WheelVelocity(wheels)) = move_command else {
      unreachable!("asserted wheel velocity above");
    };

    let expected = [
      0.54_f32 as f64 / robot_cfg.wheel_radius,
      2.16_f32 as f64 / robot_cfg.wheel_radius,
      1.62_f32 as f64 / robot_cfg.wheel_radius,
      1.08_f32 as f64 / robot_cfg.wheel_radius,
    ];
    for (got, want) in wheels.into_iter().zip(expected) {
      assert!((got - want).abs() < 1e-9);
    }
  }

  #[test]
  fn geometry_matches_world_config() {
    let mut config = WorldConfig::division_b();
    config.field.field_length = 7.5;
    config.field.field_width = 5.5;
    let geometry = geometry_from_config(&config);
    let field = geometry.field;
    assert_eq!(field.field_length, 7500);
    assert_eq!(field.field_width, 5500);
    assert_eq!(field.goal_width, (config.field.goal_width * 1000.0) as i32);
  }
}
