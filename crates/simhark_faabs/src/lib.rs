mod conv;
#[cfg(feature = "viewer-debug")]
mod debug;
#[cfg(feature = "ssl_game_controller")]
pub mod game_controller;
#[cfg(feature = "interface")]
mod interface;
mod run;
pub mod synth;

use crate::conv::world_state_to_cp_events;
#[cfg(feature = "interface")]
use crate::interface::EventShare;
pub use crate::run::run_sim_action;
use core_dump::types::{Ai, DummyAi};
use crashpilot::CrashPilot;
use crashpilot::communication::RobotHeartbeat;
use crashpilot::config::{LoggingConfig, RobotConfig, ServerConfig, SslConfig};
#[cfg(feature = "viewer")]
use prost::Message;
use simhark::{TeamColor, WorldCommand, WorldState};
use std::collections::HashMap;
use std::mem;
use std::net::Ipv4Addr;
use tf_jetsoncode::Robot;

pub use crashpilot;

#[cfg(feature = "ssl_game_controller")]
pub type FaabsCommunication = game_controller::GameControllerCommunication;
#[cfg(not(feature = "ssl_game_controller"))]
pub type FaabsCommunication = ();

pub struct Faabs<A: Ai = DummyAi> {
  pub robots: Vec<Robot<()>>,
  pub crash_pilot: CrashPilot<FaabsCommunication, A>,
  pub feedback_robot: u32,
  pub team: TeamColor,
  pub events: crashpilot::Events,
  #[cfg(feature = "ssl_game_controller")]
  pub game_controller: crashpilot::SslGameController,
  #[cfg(feature = "ssl_game_controller")]
  game_controller_events: crashpilot::communication::EventShare,
  #[cfg(feature = "viewer-debug")]
  latest_debug: Option<simhark::viewer::ViewerDebugSnapshot>,
  #[cfg(feature = "interface")]
  pub interface: EventShare,
  #[cfg(feature = "interface")]
  pub ws_out: crashpilot::communication::WebsocketOut,
  #[cfg(feature = "interface")]
  interface_thread: Option<std::thread::JoinHandle<()>>,
  #[cfg(feature = "viewer")]
  shared_interface: Option<webinterface_crashpilot_bridge::CrashPilotAdapter>,
}

impl<A: Ai + Default + Send> Faabs<A> {
  pub fn with_interface(num_robots: u8, team: TeamColor) -> Self {
    Self::new(num_robots, team)
  }

  pub fn new(num_robots: u8, team: TeamColor) -> Self {
    Self::with_ai(num_robots, team, A::default())
  }
}

impl<A: Ai> Faabs<A> {
  fn start_interface(&mut self) {
    #[cfg(feature = "interface")]
    {
      let cfg = get_config(self.robots.len() as u8);
      let tx = self.interface.clone();
      let ws_out = self.ws_out.clone();
      let websocket_url = format!("ws://127.0.0.1:{}/ws", cfg.server.websocket_port);

      self.interface_thread = Some(
        crashpilot::interface::spawn_interface(websocket_url)
          .expect("failed to start Rust CrashPilot interface owner"),
      );

      tokio::spawn(async move {
        crate::interface::spawn_websocket(&cfg, tx, ws_out).await;
      });
    }
  }
}

impl<A: Ai + Send> Faabs<A> {
  pub fn with_ai(num_robots: u8, team: TeamColor, ai: A) -> Self {
    let mut robots = Vec::with_capacity(num_robots as usize);

    for i in 0..num_robots {
      let mut config = tf_jetsoncode::Config::default();
      config.robot_id = i;

      robots.push(Robot::new(config));
    }

    let cp_config = get_config(num_robots);
    #[cfg(feature = "ssl_game_controller")]
    let game_controller_events = game_controller::event_share();
    #[cfg(feature = "ssl_game_controller")]
    let comm = game_controller::GameControllerCommunication::spawn(
      &cp_config,
      game_controller_events.clone(),
    );
    #[cfg(feature = "ssl_game_controller")]
    let game_controller = comm.controller().clone();
    #[cfg(not(feature = "ssl_game_controller"))]
    let comm = ();

    let mut this = Self {
      robots,
      crash_pilot: CrashPilot::from_parts(
        cp_config,
        comm,
        ai,
        RobotHeartbeat::default(),
        std::time::Instant::now(),
      ),
      feedback_robot: 0,
      team,
      events: crashpilot::Events::default(),
      #[cfg(feature = "ssl_game_controller")]
      game_controller,
      #[cfg(feature = "ssl_game_controller")]
      game_controller_events,
      #[cfg(feature = "viewer-debug")]
      latest_debug: None,
      #[cfg(feature = "interface")]
      interface: EventShare::default(),
      #[cfg(feature = "interface")]
      ws_out: crashpilot::communication::WebsocketOut::new(),
      #[cfg(feature = "interface")]
      interface_thread: None,
      #[cfg(feature = "viewer")]
      shared_interface: None,
    };

    this.start_interface();

    this
  }

  #[cfg(feature = "viewer")]
  pub fn attach_shared_interface(
    &mut self,
    handle: &webinterface_core::InterfaceHandle,
    session_id: webinterface_protocol::SessionId,
  ) -> Result<(), webinterface_core::InterfaceError> {
    let suffix = match self.team {
      TeamColor::Blue => "blue",
      TeamColor::Yellow => "yellow",
    };
    let system_id = format!("crashpilot-{suffix}");
    handle.unregister_system(&system_id);
    self.shared_interface = Some(
      webinterface_crashpilot_bridge::CrashPilotAdapter::register_with_id(
        handle,
        session_id,
        system_id,
        format!("CrashPilot ({suffix})"),
      )?,
    );
    Ok(())
  }

  pub fn step(
    &mut self,
    state: &WorldState,
    command: &mut WorldCommand,
    referee: Option<core_dump::proto::Referee>,
  ) {
    world_state_to_cp_events(&mut self.events, state);
    self.events.gc = referee;

    #[cfg(feature = "viewer")]
    if let Some(interface) = self.shared_interface.as_mut() {
      while let Ok(Some((_command_id, payload))) = interface.try_next_command_bytes() {
        match crashpilot::core_dump::proto::CrashpilotInterfaceInput::decode(payload.as_slice()) {
          Ok(command) => self.events.ws = Some(command),
          Err(error) => eprintln!("shared interface command decode failed: {error}"),
        }
      }
    }

    #[cfg(feature = "interface")]
    {
      self.events.ws = self.interface.blocking_lock().clone();
    }

    #[cfg(feature = "ssl_game_controller")]
    self.drain_game_controller_events();

    let ws = self.events.ws.clone();
    #[cfg(feature = "viewer-debug")]
    let debug_referee = self.events.gc.clone();

    let (interface, robots) = self.crash_pilot.step_with_data(mem::take(&mut self.events));
    #[cfg(not(feature = "interface"))]
    let _ = &interface;
    #[cfg(feature = "viewer-debug")]
    let debug_ai_commands = *self.crash_pilot.ai_commands();
    #[cfg(feature = "viewer-debug")]
    let mut debug_robots = Vec::new();
    #[cfg(feature = "viewer-debug")]
    let mut debug_overlays = Vec::new();

    #[cfg(feature = "interface")]
    {
      self.ws_out.publish_sync(interface.clone());
    }

    #[cfg(feature = "viewer")]
    if let Some(shared_interface) = self.shared_interface.as_mut() {
      if let Err(error) = shared_interface.ingest_bytes(&interface.encode_to_vec()) {
        eprintln!("shared CrashPilot interface publish failed: {error}");
      }
    }

    self.events.ws = ws;

    for (id, data) in robots {
      let Some(robot) = self.robots.get_mut(id as usize) else {
        panic!(
          "Received data for robot with id {}, but only {} robots are configured",
          id,
          self.robots.len()
        );
      };

      #[cfg(feature = "viewer-debug")]
      let cp_robot = data.msg.clone();
      let events = conv::robot_events(id, data, state, self.team);

      let (teensy, robot_cp) = robot.step_with_data(events);

      #[cfg(feature = "viewer-debug")]
      let ai_command = debug_ai_commands.get(id as usize).copied().flatten();
      #[cfg(feature = "viewer-debug")]
      debug_robots.push(debug::robot_debug_info(
        id, self.team, ai_command, &cp_robot, &teensy, state,
      ));
      #[cfg(feature = "viewer-debug")]
      debug_overlays.extend(debug::robot_debug_overlays(
        id,
        self.team,
        ai_command,
        &cp_robot.cmd,
        state,
      ));

      run_sim_action(id, teensy, command, self.team);

      if self.feedback_robot == id {
        self.events.rf = Some(robot_cp);
      }
    }

    self.feedback_robot += 1;
    self.feedback_robot %= self.robots.len() as u32;

    #[cfg(feature = "viewer-debug")]
    {
      let ai_debug = self.crash_pilot.get_ai().debug();
      self.latest_debug = Some(debug::snapshot(
        state.world_id,
        self.team,
        state,
        debug_referee.as_ref(),
        Some(ai_debug),
        debug_robots,
        debug_overlays,
      ));
    }
  }

  #[cfg(feature = "viewer-debug")]
  pub fn debug_snapshot(&self) -> Option<simhark::viewer::ViewerDebugSnapshot> {
    self.latest_debug.clone()
  }

  #[cfg(feature = "ssl_game_controller")]
  fn drain_game_controller_events(&mut self) {
    let Ok(mut pending) = self.game_controller_events.try_write() else {
      return;
    };

    let mut events = pending.take();
    if events.gc.is_some() {
      self.events.gc = events.gc.take();
    }
    self
      .events
      .gc_team_messages
      .append(&mut events.gc_team_messages);
  }
}

fn get_config(num_robots: u8) -> crashpilot::Config {
  let mut config =
    crashpilot::config::load_or_create_config(concat!(env!("CARGO_MANIFEST_DIR"), "/config.toml"))
      .unwrap_or_else(|err| {
        eprintln!("Failed to load simhark_faabs config.toml, using defaults: {err}");
        crashpilot::Config {
          ssl: SslConfig::default(),
          server: ServerConfig::default(),
          logging: LoggingConfig::default(),
          robots: HashMap::new(),
          world_model: crashpilot::config::WorldModelConfig::default(),
        }
      });
  let mut robots = HashMap::new();

  for i in 0..num_robots as u32 {
    robots.insert(
      i,
      RobotConfig {
        ip: Ipv4Addr::new(10, 0, 64, 101 + i as u8),
        port: None,
        substitution_pos: Default::default(),
      },
    );
  }

  config.robots = robots;
  config
}
