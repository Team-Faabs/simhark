//! CLI entry point for the RoboCup SSL Simulator.
//!
//! Supports headless mode for training and a debug viewer mode.

use simhark::{
  GrSimCompatConfig, GrSimCompatServer, MoveCommand, RobotCommand, SimulationEngine, TeamColor,
  TeleportBall, TeleportRobot, WorldCommand, WorldConfig,
  domain_randomization::RandomizationConfig,
};
use std::time::Instant;

#[cfg(feature = "viewer")]
use simhark::viewer::{ViewerConfig, ViewerServer};

fn main() {
  env_logger::init();

  let args: Vec<String> = std::env::args().collect();
  let num_worlds: usize = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(512);
  let num_steps: usize = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(1000);
  let randomize = args.iter().any(|a| a == "--randomize");
  let grsim_api = args.iter().any(|a| a == "--grsim-api");
  let viewer_enabled = args.iter().any(|a| a == "--viewer");

  #[cfg(feature = "viewer")]
  let viewer_port = args
    .windows(2)
    .find(|window| window[0] == "--viewer-port")
    .and_then(|window| window[1].parse::<u16>().ok())
    .unwrap_or(8315);

  #[cfg(not(feature = "viewer"))]
  if viewer_enabled {
    eprintln!("viewer support is not compiled in; rebuild with `--features viewer`");
    std::process::exit(2);
  }

  println!("RoboCup SSL Simulator (Rust)");
  println!("  Worlds: {num_worlds}");
  println!("  Steps:  {num_steps}");
  println!("  Domain randomization: {randomize}");
  println!();

  let config = WorldConfig::division_a();

  let start = Instant::now();
  let mut engine = if randomize {
    SimulationEngine::new_randomized(num_worlds, config, RandomizationConfig::moderate())
  } else {
    SimulationEngine::new(num_worlds, config)
  };
  let init_time = start.elapsed();
  println!("Initialization: {init_time:.2?}");

  #[cfg(feature = "viewer")]
  let viewer = if viewer_enabled {
    let config = ViewerConfig {
      http_port: viewer_port,
      ..ViewerConfig::default()
    };
    let viewer = ViewerServer::bind(config, num_worlds, &engine.world(0).config)
      .expect("failed to start viewer");
    println!("Viewer: {}", config.http_url());
    println!();
    Some(viewer)
  } else {
    None
  };

  if grsim_api {
    let mut server = GrSimCompatServer::bind(GrSimCompatConfig::default())
      .expect("failed to bind grSim compatibility sockets");
    println!("grSim compatibility API enabled");
    println!("  Legacy command port: 20011");
    println!("  Simulation control: 10300");
    println!("  Blue robot control:  10301");
    println!("  Yellow robot control: 10302");
    println!("  Vision multicast: 224.5.23.2:10020");
    println!();

    #[cfg(feature = "viewer")]
    if let Some(viewer) = viewer.as_ref() {
      loop {
        viewer.apply_robot_move_requests(&mut engine);
        server
          .step(&mut engine)
          .expect("grSim compatibility server step failed");
        publish_viewer_frame(viewer, &engine);
      }
    }

    loop {
      server
        .step(&mut engine)
        .expect("grSim compatibility server step failed");
    }
  }

  let start = Instant::now();

  #[cfg(feature = "viewer")]
  if let Some(viewer) = viewer.as_ref() {
    for step in 0..num_steps {
      viewer.apply_robot_move_requests(&mut engine);
      let command = demo_command(step, num_worlds);
      engine.advance_with_commands(&command);

      publish_viewer_frame(viewer, &engine);

      if step == 0 || step == num_steps - 1 {
        log_sample_state(step, &engine.world(0).get_state());
      }
    }
  } else {
    for step in 0..num_steps {
      let command = demo_command(step, num_worlds);
      engine.advance_with_commands(&command);
      if step == 0 || step == num_steps - 1 {
        log_sample_state(step, &engine.world(0).get_state());
      }
    }
  }

  #[cfg(not(feature = "viewer"))]
  for step in 0..num_steps {
    let command = demo_command(step, num_worlds);
    engine.advance_with_commands(&command);
    if step == 0 || step == num_steps - 1 {
      log_sample_state(step, &engine.world(0).get_state());
    }
  }

  let sim_time = start.elapsed();

  let total_steps = num_worlds as u64 * num_steps as u64;
  let steps_per_sec = total_steps as f64 / sim_time.as_secs_f64();
  println!();
  println!("Simulation: {sim_time:.2?}");
  println!("Total world-steps: {total_steps}");
  println!("Throughput: {steps_per_sec:.0} world-steps/sec");
  println!(
    "Real-time factor per world: {:.1}x (at 60 FPS)",
    steps_per_sec / num_worlds as f64 / 60.0
  );
}

#[cfg(feature = "viewer")]
fn publish_viewer_frame(viewer: &ViewerServer, engine: &SimulationEngine) {
  if viewer.selected_worlds().len() > 1 {
    let states = engine.get_all_states();
    viewer.publish_states(&states);
  } else {
    let state = engine.world(viewer.selected_world()).get_state();
    viewer.publish(&state);
  }
}

fn log_sample_state(step: usize, state: &simhark::WorldState) {
  let robot = state
    .blue_robots
    .get(5)
    .or_else(|| state.blue_robots.iter().find(|robot| robot.is_on))
    .unwrap_or(&state.blue_robots[0]);
  println!(
    "Step {step:>5}: ball=({:.3}, {:.3}, {:.3}) blue[{}]=({:.3}, {:.3}) frame={}",
    state.ball.x, state.ball.y, state.ball.z, robot.id, robot.x, robot.y, state.frame,
  );
}

fn demo_command(step: usize, num_worlds: usize) -> Vec<WorldCommand> {
  (0..num_worlds)
    .map(|world_index| demo_world_command(step, world_index))
    .collect()
}

fn demo_world_command(step: usize, world_index: usize) -> WorldCommand {
  let phase = step % 360;
  let offset = world_index as f64 * 0.12;
  let wave = ((step as f64) * 0.05 + offset).sin();
  let sweep = ((step as f64) * 0.03 + offset * 0.5).cos();

  let mut command = WorldCommand::default();

  command.blue = vec![
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
      move_command: Some(MoveCommand::LocalVelocity {
        forward: 0.75,
        left: -0.6 * sweep,
        angular: 0.8,
      }),
      id: 5,
      kick_speed: 0.0,
      kick_angle: 0.0,
      dribbler_on: false,
    },
    RobotCommand {
      id: 6,
      move_command: Some(MoveCommand::LocalVelocity {
        forward: 0.45 + 0.15 * wave,
        left: 0.2,
        angular: -1.1,
      }),
      kick_speed: 0.0,
      kick_angle: 0.0,
      dribbler_on: false,
    },
  ];

  command.yellow = vec![
    RobotCommand {
      id: 4,
      move_command: Some(MoveCommand::LocalVelocity {
        forward: 0.9,
        left: -0.3 * sweep,
        angular: -0.35,
      }),
      kick_speed: 0.0,
      kick_angle: 0.0,
      dribbler_on: false,
    },
    RobotCommand {
      id: 5,
      move_command: Some(MoveCommand::LocalVelocity {
        forward: 0.6,
        left: 0.55 * wave,
        angular: 1.1,
      }),
      kick_speed: 0.0,
      kick_angle: 0.0,
      dribbler_on: false,
    },
    RobotCommand {
      id: 6,
      move_command: Some(MoveCommand::LocalVelocity {
        forward: 0.55,
        left: -0.45 * wave,
        angular: -0.9,
      }),
      kick_speed: 0.0,
      kick_angle: 0.0,
      dribbler_on: false,
    },
  ];

  if phase == 0 {
    command.teleport_ball = Some(TeleportBall {
      x: Some(-0.3 + offset * 0.2),
      y: Some(0.0),
      z: Some(0.0),
      vx: Some(2.2),
      vy: Some(0.9 * sweep),
      vz: Some(0.0),
    });
    command.teleport_robots.extend(disable_extra_robots());
  }

  if phase == 90 {
    command.teleport_ball = Some(TeleportBall {
      x: Some(0.0),
      y: Some(0.5 * wave),
      z: Some(0.16),
      vx: Some(-1.7),
      vy: Some(1.1 * sweep),
      vz: Some(1.0),
    });
  }

  if phase == 180 {
    command.teleport_ball = Some(TeleportBall {
      x: Some(-1.8 + offset * 0.2),
      y: Some(-1.0),
      z: Some(0.0),
      vx: Some(2.8),
      vy: Some(0.5),
      vz: Some(0.0),
    });
  }

  if phase == 270 {
    command.teleport_ball = Some(TeleportBall {
      x: Some(5.9),
      y: Some(0.0),
      z: Some(0.0),
      vx: Some(0.0),
      vy: Some(0.0),
      vz: Some(0.0),
    });
  }

  if phase == 300 {
    command.teleport_ball = Some(TeleportBall {
      x: Some(-0.2),
      y: Some(-0.6),
      z: Some(0.08),
      vx: Some(-1.2),
      vy: Some(1.8),
      vz: Some(0.7),
    });
  }

  command
}

fn disable_extra_robots() -> Vec<TeleportRobot> {
  let active_ids = [4, 5, 6];
  let mut teleports = Vec::new();

  for team in [TeamColor::Blue, TeamColor::Yellow] {
    for id in 0..11 {
      if active_ids.contains(&id) {
        continue;
      }

      teleports.push(TeleportRobot {
        id,
        team,
        x: None,
        y: None,
        orientation: None,
        vx: Some(0.0),
        vy: Some(0.0),
        v_angular: Some(0.0),
        present: Some(false),
      });
    }
  }

  teleports
}
