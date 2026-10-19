//! Standard SSL/grSim clients around the optional backend. Live datagrams are
//! timestamped at ingestion and recorded as simulator inputs. Feedback comes
//! from delivered telemetry; vision comes from delivered camera frames.
use super::{RealisticWorld, sensors::VisionFrame, transport::Safety};
use crate::{TeamColor, WorldCommand, WorldConfig, proto::*};
use prost::Message;
use std::{
  collections::BTreeMap,
  io,
  net::{SocketAddr, UdpSocket},
};

/// Capture actual detector output for a bounded integration check. This uses
/// wall time and is deliberately outside scheduled simulation.
pub fn capture_report(address: SocketAddr, seconds: f64) -> io::Result<serde_json::Value> {
  if !seconds.is_finite() || seconds <= 0.0 || seconds > 3600.0 {
    return Err(io::Error::other("invalid capture duration"));
  }
  let socket = UdpSocket::bind(address)?;
  socket.join_multicast_v4(
    &"224.5.23.2".parse().unwrap(),
    &std::net::Ipv4Addr::UNSPECIFIED,
  )?;
  socket.set_read_timeout(Some(std::time::Duration::from_millis(50)))?;
  let start = std::time::Instant::now();
  let mut buffer = [0; 65536];
  let (mut packets, mut detections, mut balls, mut blue, mut yellow, mut malformed) =
    (0_u64, 0_u64, 0_u64, 0_u64, 0_u64, 0_u64);
  let mut examples = Vec::new();
  let mut ids = std::collections::BTreeSet::new();
  while start.elapsed().as_secs_f64() < seconds {
    let n = match socket.recv(&mut buffer) {
      Ok(n) => n,
      Err(e)
        if matches!(
          e.kind(),
          io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
        ) =>
      {
        continue;
      }
      Err(e) => return Err(e),
    };
    packets += 1;
    let Ok(packet) = SslWrapperPacket::decode(&buffer[..n]) else {
      malformed += 1;
      continue;
    };
    if let Some(f) = packet.detection {
      detections += 1;
      balls += f.balls.len() as u64;
      blue += f.robots_blue.len() as u64;
      yellow += f.robots_yellow.len() as u64;
      for (color, robots) in [(0, &f.robots_blue), (1, &f.robots_yellow)] {
        for r in robots {
          if let Some(id) = r.robot_id {
            ids.insert((color, id));
          }
        }
      }
      if examples.len() < 10 {
        examples.push(serde_json::json!({"camera":f.camera_id,"frame":f.frame_number,"t_capture":f.t_capture,"t_sent":f.t_sent,
        "balls":f.balls.iter().map(|b|[b.x,b.y,b.z.unwrap_or(0.0)]).collect::<Vec<_>>(),
        "blue":f.robots_blue.iter().map(|r|serde_json::json!({"id":r.robot_id,"x_mm":r.x,"y_mm":r.y,"yaw":r.orientation})).collect::<Vec<_>>(),
        "yellow":f.robots_yellow.iter().map(|r|serde_json::json!({"id":r.robot_id,"x_mm":r.x,"y_mm":r.y,"yaw":r.orientation})).collect::<Vec<_>>() }));
      }
    }
  }
  Ok(
    serde_json::json!({"live_nondeterministic":true,"packets":packets,"detection_frames":detections,"balls":balls,"blue_robots":blue,"yellow_robots":yellow,"robot_ids":ids,"malformed":malformed,"examples":examples}),
  )
}

pub fn detection_packet(frame: &VisionFrame) -> SslWrapperPacket {
  let robot = |r: &super::sensors::RobotDetection| SslDetectionRobot {
    confidence: 1.0,
    robot_id: Some(r.id as u32),
    x: (r.position_m[0] * 1000.0) as f32,
    y: (r.position_m[1] * 1000.0) as f32,
    orientation: Some(r.orientation_rad as f32),
    pixel_x: 0.0,
    pixel_y: 0.0,
    height: None,
  };
  SslWrapperPacket {
    geometry: None,
    detection: Some(SslDetectionFrame {
      frame_number: frame.frame_number as u32,
      t_capture: frame.image_received_at as f64 * 1e-9,
      t_sent: frame.sent_at as f64 * 1e-9,
      camera_id: frame.camera_id,
      balls: frame
        .ball_position_m
        .into_iter()
        .map(|p| SslDetectionBall {
          confidence: 1.0,
          area: None,
          x: (p[0] * 1000.0) as f32,
          y: (p[1] * 1000.0) as f32,
          z: Some((p[2] * 1000.0) as f32),
          pixel_x: 0.0,
          pixel_y: 0.0,
        })
        .collect(),
      robots_blue: frame
        .robots
        .iter()
        .filter(|r| r.team == TeamColor::Blue)
        .map(robot)
        .collect(),
      robots_yellow: frame
        .robots
        .iter()
        .filter(|r| r.team == TeamColor::Yellow)
        .map(robot)
        .collect(),
    }),
  }
}
pub struct StandardServer {
  setup: Option<UdpSocket>,
  grsim: UdpSocket,
  blue: UdpSocket,
  yellow: UdpSocket,
  vision: UdpSocket,
  target: SocketAddr,
  base: WorldConfig,
  last_frames: BTreeMap<u32, u64>,
  /// Malformed input never panics or takes down the simulator.
  pub malformed: u64,
}

/// Actual SSL-Vision UDP ingress with an explicitly provisioned host clock
/// mapping. Receipt time and decoded detections are replay inputs.
pub struct ExternalVisionReceiver {
  socket: UdpSocket,
  pub host_epoch_s: f64,
  pub sim_epoch_ns: u64,
  pub rejected: u64,
}
impl ExternalVisionReceiver {
  pub fn bind(address: SocketAddr, host_epoch_s: f64, sim_epoch_ns: u64) -> io::Result<Self> {
    if !host_epoch_s.is_finite() {
      return Err(io::Error::other("invalid external host clock epoch"));
    }
    let socket = UdpSocket::bind(address)?;
    socket.set_nonblocking(true)?;
    Ok(Self {
      socket,
      host_epoch_s,
      sim_epoch_ns,
      rejected: 0,
    })
  }
  pub fn join_multicast(
    &self,
    group: std::net::Ipv4Addr,
    interface: std::net::Ipv4Addr,
  ) -> io::Result<()> {
    self.socket.join_multicast_v4(&group, &interface)
  }
  pub fn poll(&mut self, sim: &mut RealisticWorld) -> io::Result<usize> {
    let mut buffer = [0; 65536];
    let mut count = 0;
    for _ in 0..256 {
      let n = match self.socket.recv(&mut buffer) {
        Ok(n) => n,
        Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
        Err(e) => return Err(e),
      };
      let Some(frame) = SslWrapperPacket::decode(&buffer[..n])
        .ok()
        .and_then(|p| p.detection)
      else {
        self.rejected += 1;
        continue;
      };
      let time = |t: f64| -> Option<u64> {
        let ns = (t - self.host_epoch_s) * 1e9 + self.sim_epoch_ns as f64;
        (ns.is_finite() && ns >= 0.0 && ns <= sim.now() as f64).then_some(ns.round() as u64)
      };
      let (Some(captured), Some(sent)) = (time(frame.t_capture), time(frame.t_sent)) else {
        self.rejected += 1;
        continue;
      };
      let mut robots = Vec::new();
      for (team, detections) in [
        (TeamColor::Blue, frame.robots_blue),
        (TeamColor::Yellow, frame.robots_yellow),
      ] {
        for r in detections {
          if r.confidence <= 0.0 {
            continue;
          }
          if let (Some(id), Some(yaw)) = (r.robot_id, r.orientation) {
            robots.push(super::sensors::RobotDetection {
              id: id as usize,
              team,
              position_m: [f64::from(r.x) * 0.001, f64::from(r.y) * 0.001],
              orientation_rad: f64::from(yaw),
            });
          }
        }
      }
      let ball = frame
        .balls
        .into_iter()
        .filter(|b| b.confidence > 0.0)
        .max_by(|a, b| a.confidence.total_cmp(&b.confidence))
        .map(|b| {
          [
            f64::from(b.x) * 0.001,
            f64::from(b.y) * 0.001,
            f64::from(b.z.unwrap_or(0.0)) * 0.001,
          ]
        });
      let input = VisionFrame {
        camera_id: frame.camera_id,
        frame_number: u64::from(frame.frame_number),
        camera_capture_ticks: None,
        image_received_at: captured,
        sent_at: sent,
        captured_at: captured,
        physical_state_at: 0,
        robots,
        ball_position_m: ball,
      };
      if sim.inject_vision(input).is_ok() {
        count += 1;
      } else {
        self.rejected += 1;
      }
    }
    Ok(count)
  }
}
impl StandardServer {
  pub fn bind(
    bind_ip: std::net::IpAddr,
    grsim_port: u16,
    blue_port: u16,
    yellow_port: u16,
    vision_target: SocketAddr,
    base: WorldConfig,
  ) -> io::Result<Self> {
    let socket = |port| -> io::Result<UdpSocket> {
      let s = UdpSocket::bind((bind_ip, port))?;
      s.set_nonblocking(true)?;
      Ok(s)
    };
    let vision = socket(0)?;
    vision.set_multicast_ttl_v4(1)?;
    let geometry = SslWrapperPacket {
      detection: None,
      geometry: Some(crate::grsim::geometry_from_config(&base)),
    }
    .encode_to_vec();
    vision.send_to(&geometry, vision_target)?;
    Ok(Self {
      setup: None,
      grsim: socket(grsim_port)?,
      blue: socket(blue_port)?,
      yellow: socket(yellow_port)?,
      vision,
      target: vision_target,
      base,
      last_frames: BTreeMap::new(),
      malformed: 0,
    })
  }
  /// Setup/control is a separate administrator socket, never a team command.
  pub fn bind_setup(&mut self, address: SocketAddr) -> io::Result<()> {
    let socket = UdpSocket::bind(address)?;
    socket.set_nonblocking(true)?;
    self.setup = Some(socket);
    Ok(())
  }
  pub fn setup_address(&self) -> io::Result<Option<SocketAddr>> {
    self.setup.as_ref().map(UdpSocket::local_addr).transpose()
  }
  pub fn addresses(&self) -> io::Result<[SocketAddr; 3]> {
    Ok([
      self.grsim.local_addr()?,
      self.blue.local_addr()?,
      self.yellow.local_addr()?,
    ])
  }
  pub fn poll(&mut self, sim: &mut RealisticWorld) -> io::Result<()> {
    let mut buffer = [0; 65536];
    if let Some(socket) = &self.setup {
      for _ in 0..256 {
        let (n, peer) = match socket.recv_from(&mut buffer) {
          Ok(v) => v,
          Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
          Err(e) => return Err(e),
        };
        let result = SimulatorCommand::decode(&buffer[..n])
          .map_err(|e| e.to_string())
          .and_then(|command| setup_command(command, &self.base));
        let errors = match result {
          Ok(commands) => {
            sim.teleport(&commands);
            Vec::new()
          }
          Err(message) => {
            self.malformed += 1;
            vec![SimulatorError {
              code: Some("INVALID_SETUP".into()),
              message: Some(message),
            }]
          }
        };
        socket.send_to(&SimulatorResponse { errors }.encode_to_vec(), peer)?;
      }
    }
    for channel in 0..3 {
      let socket = match channel {
        0 => &self.grsim,
        1 => &self.blue,
        _ => &self.yellow,
      };
      // A bounded poll prevents a network flood from stopping logical time.
      for _ in 0..256 {
        let (n, peer) = match socket.recv_from(&mut buffer) {
          Ok(v) => v,
          Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
          Err(e) => return Err(e),
        };
        let decoded = if channel == 0 {
          GrSimPacket::decode(&buffer[..n])
            .ok()
            .and_then(|p| p.commands)
            .map(crate::grsim::realistic_codec::grsim)
        } else {
          RobotControl::decode(&buffer[..n]).ok().map(|p| {
            let (team, geometry) = if channel == 1 {
              (TeamColor::Blue, &self.base.blue_robots)
            } else {
              (TeamColor::Yellow, &self.base.yellow_robots)
            };
            (team, crate::grsim::realistic_codec::ssl(p, geometry))
          })
        };
        let Some((team, commands)) = decoded else {
          self.malformed += 1;
          continue;
        };
        let command = if team == TeamColor::Blue {
          WorldCommand {
            blue: commands,
            ..Default::default()
          }
        } else {
          WorldCommand {
            yellow: commands,
            ..Default::default()
          }
        };
        let result = sim.publish(command, sim.now(), 1, Safety::Run);
        if result.is_err() {
          self.malformed += 1;
        }
        if channel != 0 {
          let feedback = (0..self.base.robots_per_team)
            .filter_map(|id| {
              sim.host_feedback(team, id).map(|(_, ir, _)| RobotFeedback {
                id: id as u32,
                dribbler_ball_contact: ir
                  .filter(|v| sim.now().saturating_sub(v.sampled_at) <= 100_000_000)
                  .map(|v| v.detected),
                custom: None,
              })
            })
            .collect();
          let errors = result
            .err()
            .map(|message| SimulatorError {
              code: Some("INVALID_COMMAND".into()),
              message: Some(message),
            })
            .into_iter()
            .collect();
          socket.send_to(
            &RobotControlResponse { errors, feedback }.encode_to_vec(),
            peer,
          )?;
        }
      }
    }
    Ok(())
  }
  /// Send each delivered camera frame once. No truth snapshot is accepted here.
  pub fn vision(&mut self, frames: &[VisionFrame]) -> io::Result<()> {
    for frame in frames {
      if self.last_frames.get(&frame.camera_id) == Some(&frame.frame_number) {
        continue;
      }
      self
        .vision
        .send_to(&detection_packet(frame).encode_to_vec(), self.target)?;
      self.last_frames.insert(frame.camera_id, frame.frame_number);
    }
    Ok(())
  }
}

fn setup_command(command: SimulatorCommand, base: &WorldConfig) -> Result<WorldCommand, String> {
  if command.config.is_some() {
    return Err("hardware/venue configuration is immutable during a recorded run; use a profile before startup".into());
  }
  let control = command.control.ok_or("missing simulator control")?;
  if control.simulation_speed.is_some() {
    return Err("live simulation speed is controlled by the runner".into());
  }
  let mut out = WorldCommand::default();
  if let Some(ball) = control.teleport_ball {
    if ball.teleport_safely.unwrap_or(false) || ball.roll.unwrap_or(false) {
      return Err("safe teleport and roll are unsupported".into());
    }
    if [ball.x, ball.y, ball.z, ball.vx, ball.vy, ball.vz]
      .into_iter()
      .flatten()
      .any(|x| !x.is_finite())
    {
      return Err("nonfinite ball setup".into());
    }
    out.teleport_ball = Some(crate::TeleportBall {
      x: ball.x.map(f64::from),
      y: ball.y.map(f64::from),
      z: ball.z.map(f64::from),
      vx: ball.vx.map(f64::from),
      vy: ball.vy.map(f64::from),
      vz: ball.vz.map(f64::from),
    });
  }
  for r in control.teleport_robot {
    let id = r.id.id.ok_or("missing robot ID")? as usize;
    if id >= base.robots_per_team {
      return Err("robot ID outside configured roster".into());
    }
    let team = match r.id.team {
      n if n == Some(Team::Blue as i32) => TeamColor::Blue,
      n if n == Some(Team::Yellow as i32) => TeamColor::Yellow,
      _ => return Err("unknown robot team".into()),
    };
    if [r.x, r.y, r.orientation, r.v_x, r.v_y, r.v_angular]
      .into_iter()
      .flatten()
      .any(|x| !x.is_finite())
    {
      return Err("nonfinite robot setup".into());
    }
    out.teleport_robots.push(crate::TeleportRobot {
      id,
      team,
      x: r.x.map(f64::from),
      y: r.y.map(f64::from),
      orientation: r.orientation.map(f64::from),
      vx: r.v_x.map(f64::from),
      vy: r.v_y.map(f64::from),
      v_angular: r.v_angular.map(f64::from),
      present: r.present,
    });
  }
  Ok(out)
}
