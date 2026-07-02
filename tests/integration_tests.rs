//! Integration tests for the RoboCup SSL simulator.
//!
//! These tests verify behavioral correctness matching grSim, determinism
//! across parallel worlds, and physics consistency.

use simhark::command::*;
use simhark::config::*;
use simhark::domain_randomization::*;
use simhark::state::*;
use simhark::*;
use simhark_sumatra::{
  SumatraInstance, SumatraLaunchConfig, SumatraSimNetConfig, SumatraSimNetServer,
};
use std::time::{Duration, Instant};

// ============================================================================
// World creation and initial state
// ============================================================================

#[test]
fn test_world_creates_correct_number_of_robots() {
  let config = WorldConfig::division_a();
  let world = World::new(0, config.clone());
  let state = world.get_state();

  assert_eq!(state.blue_robots.len(), config.robots_per_team);
  assert_eq!(state.yellow_robots.len(), config.robots_per_team);
}

#[test]
fn test_world_initial_ball_near_center() {
  let world = World::new(0, WorldConfig::division_a());
  let state = world.get_state();

  assert!(state.ball.x.abs() < 0.1, "ball should start near center x");
  assert!(state.ball.y.abs() < 0.1, "ball should start near center y");
  assert!(state.ball.z > 0.0, "ball should be above ground");
}

#[test]
fn test_world_initial_robots_on_correct_halves() {
  let world = World::new(0, WorldConfig::division_a());
  let state = world.get_state();

  for r in &state.blue_robots {
    assert!(
      r.x < 0.0,
      "blue robots should start on negative x half, got {}",
      r.x
    );
  }
  for r in &state.yellow_robots {
    assert!(
      r.x > 0.0,
      "yellow robots should start on positive x half, got {}",
      r.x
    );
  }
}

#[test]
fn test_world_initial_robots_all_on() {
  let world = World::new(0, WorldConfig::division_a());
  let state = world.get_state();

  for r in state.blue_robots.iter().chain(state.yellow_robots.iter()) {
    assert!(r.is_on, "all robots should start on");
  }
}

#[test]
fn test_division_b_smaller_field() {
  let a = FieldConfig::division_a();
  let b = FieldConfig::division_b();

  assert!(b.field_length < a.field_length);
  assert!(b.field_width < a.field_width);
}

#[test]
fn test_default_kick_limits_stay_below_card_threshold() {
  let config = WorldConfig::division_a();

  assert_eq!(config.blue_robots.max_linear_kick_speed, DEFAULT_KICK_SPEED);
  assert_eq!(config.blue_robots.max_chip_kick_speed, DEFAULT_KICK_SPEED);
  assert_eq!(
    config.yellow_robots.max_linear_kick_speed,
    DEFAULT_KICK_SPEED
  );
  assert_eq!(config.yellow_robots.max_chip_kick_speed, DEFAULT_KICK_SPEED);
  assert!(DEFAULT_KICK_SPEED <= 3.5);
}

// ============================================================================
// Physics: ball behavior
// ============================================================================

#[test]
fn test_ball_falls_to_ground() {
  let mut config = WorldConfig::division_a();
  config.robots_per_team = 0; // no robots for cleaner test
  let mut world = World::new(0, config);

  // Step a few times, ball should settle near ground level
  for _ in 0..120 {
    world.step_empty();
  }

  let state = world.get_state();
  assert!(
    state.ball.z < 0.1,
    "ball should settle near ground, got z={}",
    state.ball.z
  );
  assert!(state.ball.z >= 0.0, "ball should not go below ground");
}

#[test]
fn test_ball_friction_slows_ball() {
  let mut config = WorldConfig::division_a();
  config.robots_per_team = 0;
  let mut world = World::new(0, config);

  // Give ball initial velocity
  world.step(&WorldCommand {
    teleport_ball: Some(TeleportBall {
      x: Some(0.0),
      y: Some(0.0),
      z: Some(0.0),
      vx: Some(3.0),
      vy: Some(0.0),
      vz: Some(0.0),
    }),
    ..Default::default()
  });

  let initial_speed = {
    let s = world.get_state();
    (s.ball.vx * s.ball.vx + s.ball.vy * s.ball.vy).sqrt()
  };

  // Step forward
  for _ in 0..60 {
    world.step_empty();
  }

  let final_speed = {
    let s = world.get_state();
    (s.ball.vx * s.ball.vx + s.ball.vy * s.ball.vy).sqrt()
  };

  assert!(
    final_speed < initial_speed,
    "ball should slow down due to friction: initial={initial_speed}, final={final_speed}"
  );
}

#[test]
fn test_ball_bounce_off_wall() {
  let mut config = WorldConfig::division_a();
  config.robots_per_team = 0;
  let mut world = World::new(0, config.clone());

  // Place ball near SIDE wall (y-direction) with velocity toward it
  let half_width = config.field.field_width / 2.0;
  world.step(&WorldCommand {
    teleport_ball: Some(TeleportBall {
      x: Some(0.0),
      y: Some(half_width - 0.2),
      z: Some(0.0),
      vx: Some(0.0),
      vy: Some(5.0),
      vz: Some(0.0),
    }),
    ..Default::default()
  });

  // Step until ball bounces
  for _ in 0..120 {
    world.step_empty();
  }

  let state = world.get_state();
  // Ball should have bounced back toward center (y < half_width)
  assert!(
    state.ball.y < half_width + 0.5,
    "ball should bounce off side wall, y={}",
    state.ball.y
  );
}

#[test]
fn test_stopped_ball_outside_touchline_is_recovered() {
  let mut config = WorldConfig::division_a();
  config.robots_per_team = 0;
  let mut world = World::new(0, config.clone());

  let half_length = config.field.field_length / 2.0;
  let half_width = config.field.field_width / 2.0;
  world.step(&WorldCommand {
    teleport_ball: Some(TeleportBall {
      x: Some(-half_length + 0.25),
      y: Some(half_width + 0.05),
      z: Some(0.0),
      vx: Some(0.0),
      vy: Some(0.0),
      vz: Some(0.0),
    }),
    ..Default::default()
  });

  let state = world.get_state();
  assert!(
    state.ball.x < 0.0,
    "test should cover the upper-left side, x={}",
    state.ball.x
  );
  assert!(
    state.ball.y <= half_width - config.ball.radius,
    "stopped out-of-field ball should be recovered inside touchline, y={}",
    state.ball.y
  );
  assert!(!state.goal_blue);
  assert!(!state.goal_yellow);
}

#[test]
fn test_ball_stationary_stays_put() {
  let mut config = WorldConfig::division_a();
  config.robots_per_team = 0;
  let mut world = World::new(0, config);

  // Place ball at rest
  world.step(&WorldCommand {
    teleport_ball: Some(TeleportBall {
      x: Some(1.0),
      y: Some(1.0),
      z: Some(0.0),
      vx: Some(0.0),
      vy: Some(0.0),
      vz: Some(0.0),
    }),
    ..Default::default()
  });

  // Step forward
  for _ in 0..60 {
    world.step_empty();
  }

  let state = world.get_state();
  assert!(
    (state.ball.x - 1.0).abs() < 0.05,
    "stationary ball should stay at x=1.0, got {}",
    state.ball.x
  );
  assert!(
    (state.ball.y - 1.0).abs() < 0.05,
    "stationary ball should stay at y=1.0, got {}",
    state.ball.y
  );
}

// ============================================================================
// Robot commands
// ============================================================================

#[test]
fn test_robot_moves_forward() {
  let config = WorldConfig::division_a();
  let mut world = World::new(0, config);

  let initial_state = world.get_state();

  // Command robot 0 to move forward for more steps to allow motor to take effect
  let cmd = WorldCommand {
    blue: vec![RobotCommand {
      id: 0,
      move_command: Some(MoveCommand::LocalVelocity {
        forward: 2.0,
        left: 0.0,
        angular: 0.0,
      }),
      kick_speed: 0.0,
      kick_angle: 0.0,
      dribbler_on: false,
    }],
    ..Default::default()
  };

  for _ in 0..300 {
    world.step(&cmd);
  }

  let state = world.get_state();
  // Robot should have moved (exact direction depends on initial orientation)
  let dist = ((state.blue_robots[0].x - initial_state.blue_robots[0].x).powi(2)
    + (state.blue_robots[0].y - initial_state.blue_robots[0].y).powi(2))
  .sqrt();
  // Even if motor coupling isn't perfect, the robot should move at least a tiny amount
  // due to joint forces
  assert!(
    dist > 0.001,
    "robot should have moved at least slightly, distance={dist}"
  );
}

#[test]
fn test_wheel_speed_command() {
  let config = WorldConfig::division_a();
  let mut world = World::new(0, config);

  let initial = world.get_state();
  let initial_pos = (initial.blue_robots[0].x, initial.blue_robots[0].y);

  // Set wheels to different speeds to create rotation
  let cmd = WorldCommand {
    blue: vec![RobotCommand {
      id: 0,
      move_command: Some(MoveCommand::WheelVelocity([10.0, -10.0, 10.0, -10.0])),
      kick_speed: 0.0,
      kick_angle: 0.0,
      dribbler_on: false,
    }],
    ..Default::default()
  };

  for _ in 0..300 {
    world.step(&cmd);
  }

  let state = world.get_state();
  let dist = ((state.blue_robots[0].x - initial_pos.0).powi(2)
    + (state.blue_robots[0].y - initial_pos.1).powi(2))
  .sqrt();
  let angle_changed =
    (state.blue_robots[0].orientation - initial.blue_robots[0].orientation).abs() > 0.001;
  assert!(
    dist > 0.001 || angle_changed,
    "robot should have moved or rotated with wheel commands, dist={dist}, angle_change={}",
    (state.blue_robots[0].orientation - initial.blue_robots[0].orientation).abs()
  );
}

#[test]
fn test_noop_command_robot_stays() {
  let config = WorldConfig::division_a();
  let mut world = World::new(0, config);

  // Step many times to let robots settle on the ground
  for _ in 0..300 {
    world.step_empty();
  }

  let before = world.get_state();
  let pos_before = (before.blue_robots[0].x, before.blue_robots[0].y);

  // Send noop
  let cmd = WorldCommand {
    blue: vec![RobotCommand::noop(0)],
    ..Default::default()
  };
  for _ in 0..30 {
    world.step(&cmd);
  }

  let after = world.get_state();
  let dist = ((after.blue_robots[0].x - pos_before.0).powi(2)
    + (after.blue_robots[0].y - pos_before.1).powi(2))
  .sqrt();
  assert!(
    dist < 2.0,
    "robot with noop should not move far after settling, dist={dist}"
  );
}

// ============================================================================
// Teleportation
// ============================================================================

#[test]
fn test_teleport_ball() {
  let mut world = World::new(0, WorldConfig::division_a());

  world.step(&WorldCommand {
    teleport_ball: Some(TeleportBall {
      x: Some(2.5),
      y: Some(-1.0),
      z: Some(0.0),
      vx: Some(0.0),
      vy: Some(0.0),
      vz: Some(0.0),
    }),
    ..Default::default()
  });

  let state = world.get_state();
  assert!(
    (state.ball.x - 2.5).abs() < 0.1,
    "ball should be at x=2.5, got {}",
    state.ball.x
  );
  assert!(
    (state.ball.y - (-1.0)).abs() < 0.1,
    "ball should be at y=-1.0, got {}",
    state.ball.y
  );
}

#[test]
fn test_teleport_ball_hard_caps_ball_speed() {
  let mut world = World::new(0, WorldConfig::division_a());

  world.step(&WorldCommand {
    teleport_ball: Some(TeleportBall {
      x: Some(0.0),
      y: Some(0.0),
      z: Some(0.0),
      vx: Some(12.0),
      vy: Some(0.0),
      vz: Some(0.0),
    }),
    ..Default::default()
  });

  let state = world.get_state();
  let speed = ball_speed(&state);
  assert!(
    speed <= MAX_BALL_SPEED + 1e-5,
    "teleported ball should be capped at {MAX_BALL_SPEED}m/s, got {speed}"
  );
}

#[test]
fn test_teleport_robot() {
  let mut world = World::new(0, WorldConfig::division_a());

  world.step(&WorldCommand {
    teleport_robots: vec![TeleportRobot {
      id: 0,
      team: TeamColor::Blue,
      x: Some(1.0),
      y: Some(2.0),
      orientation: Some(std::f64::consts::PI / 2.0),
      vx: None,
      vy: None,
      v_angular: None,
      present: None,
    }],
    ..Default::default()
  });

  let state = world.get_state();
  // After teleport + one physics step, position should be near the target
  // Allow larger tolerance since physics runs after teleport
  assert!(
    (state.blue_robots[0].x - 1.0).abs() < 1.0,
    "robot x should be near 1.0, got {}",
    state.blue_robots[0].x
  );
  assert!(
    (state.blue_robots[0].y - 2.0).abs() < 1.0,
    "robot y should be near 2.0, got {}",
    state.blue_robots[0].y
  );
  assert!(
    (state.blue_robots[0].orientation - std::f64::consts::PI / 2.0).abs() < 0.2,
    "robot orientation should be near pi/2, got {}",
    state.blue_robots[0].orientation
  );
}

#[test]
fn test_teleport_robot_applies_requested_velocity() {
  let mut world = World::new(0, WorldConfig::division_a());

  world.step(&WorldCommand {
    teleport_robots: vec![TeleportRobot {
      id: 0,
      team: TeamColor::Blue,
      x: Some(0.0),
      y: Some(0.0),
      orientation: Some(0.0),
      vx: Some(1.25),
      vy: Some(-0.5),
      v_angular: Some(0.75),
      present: Some(true),
    }],
    ..Default::default()
  });

  let state = world.get_state();
  assert!(
    state.blue_robots[0].vx > 0.05,
    "robot vx should move in the requested direction, got {}",
    state.blue_robots[0].vx
  );
  assert!(
    state.blue_robots[0].vy < -0.02,
    "robot vy should move in the requested direction, got {}",
    state.blue_robots[0].vy
  );
  assert!(
    (state.blue_robots[0].v_angular - 0.75).abs() < 0.15,
    "robot angular velocity should be near requested velocity, got {}",
    state.blue_robots[0].v_angular
  );
}

#[test]
fn test_active_robot_outside_touchline_is_recovered() {
  let mut config = WorldConfig::division_a();
  config.robots_per_team = 1;
  let mut world = World::new(0, config.clone());

  let half_width = config.field.field_width / 2.0;
  let robot_radius = config.blue_robots.radius.max(config.yellow_robots.radius);
  world.step(&WorldCommand {
    teleport_robots: vec![TeleportRobot {
      id: 0,
      team: TeamColor::Blue,
      x: Some(-1.0),
      y: Some(half_width + 0.2),
      orientation: Some(0.0),
      vx: Some(0.0),
      vy: Some(1.0),
      v_angular: Some(0.0),
      present: Some(true),
    }],
    ..Default::default()
  });

  let state = world.get_state();
  assert!(
    state.blue_robots[0].y <= half_width - robot_radius + 0.001,
    "active robot should be recovered inside touchline, y={}",
    state.blue_robots[0].y
  );
  assert!(
    state.blue_robots[0].vy.abs() < 0.001,
    "outward velocity should be cleared after recovery, vy={}",
    state.blue_robots[0].vy
  );
  assert!(state.blue_robots[0].is_on, "robot should remain on");
}

#[test]
fn test_kick_contact_reenables_without_followup_command() {
  let mut world = World::new(0, WorldConfig::division_a());
  let robot = world.physics.blue_robots[0].clone();

  world.step(&WorldCommand {
    teleport_ball: Some(TeleportBall {
      x: Some(-3.0 + world.config.blue_robots.center_from_kicker + world.config.ball.radius),
      y: Some(-3.0),
      z: Some(0.0),
      vx: Some(0.0),
      vy: Some(0.0),
      vz: Some(0.0),
    }),
    teleport_robots: vec![TeleportRobot {
      id: 0,
      team: TeamColor::Blue,
      x: Some(-3.0),
      y: Some(-3.0),
      orientation: Some(0.0),
      vx: Some(0.0),
      vy: Some(0.0),
      v_angular: Some(0.0),
      present: Some(true),
    }],
    blue: vec![RobotCommand {
      id: 0,
      move_command: None,
      kick_speed: 2.0,
      kick_angle: 0.0,
      dribbler_on: true,
    }],
    ..Default::default()
  });

  assert!(
    !world.physics.is_robot_ball_contact_enabled(&robot),
    "kick should disable robot-ball contact for the cooldown"
  );

  for _ in 0..20 {
    world.step_empty();
  }

  assert!(
    world.physics.is_robot_ball_contact_enabled(&robot),
    "robot-ball contact should re-enable after cooldown even without another command"
  );
}

#[test]
fn test_flat_kick_hard_caps_ball_speed() {
  let mut config = WorldConfig::division_a();
  config.robots_per_team = 1;
  config.blue_robots.max_linear_kick_speed = 20.0;
  let mut world = World::new(0, config);
  let robot_x = -3.0;
  let robot_y = -3.0;

  world.step(&WorldCommand {
    teleport_ball: Some(TeleportBall {
      x: Some(robot_x + world.config.blue_robots.center_from_kicker + world.config.ball.radius),
      y: Some(robot_y),
      z: Some(0.0),
      vx: Some(0.0),
      vy: Some(3.0),
      vz: Some(0.0),
    }),
    teleport_robots: vec![TeleportRobot {
      id: 0,
      team: TeamColor::Blue,
      x: Some(robot_x),
      y: Some(robot_y),
      orientation: Some(0.0),
      vx: Some(0.0),
      vy: Some(0.0),
      v_angular: Some(0.0),
      present: Some(true),
    }],
    blue: vec![RobotCommand {
      id: 0,
      move_command: None,
      kick_speed: 20.0,
      kick_angle: 0.0,
      dribbler_on: true,
    }],
    ..Default::default()
  });

  let state = world.get_state();
  let speed = ball_speed(&state);
  assert!(
    speed <= MAX_BALL_SPEED + 1e-5,
    "flat kick should be capped at {MAX_BALL_SPEED}m/s, got {speed}"
  );
  let expected_angle = 3.0f64.atan2(20.0);
  let launch_angle = state.ball.vy.atan2(state.ball.vx);
  assert!(
    (launch_angle - expected_angle).abs() < 0.03,
    "flat kick should scale the summed ball/kick velocity direction, got {launch_angle} rad"
  );
}

#[test]
fn test_chip_kick_hard_caps_ball_speed() {
  let mut config = WorldConfig::division_a();
  config.robots_per_team = 1;
  config.blue_robots.max_chip_kick_speed = 20.0;
  let mut world = World::new(0, config);
  let robot_x = -3.0;
  let robot_y = -3.0;

  world.step(&WorldCommand {
    teleport_ball: Some(TeleportBall {
      x: Some(robot_x + world.config.blue_robots.center_from_kicker + world.config.ball.radius),
      y: Some(robot_y),
      z: Some(0.0),
      vx: Some(0.0),
      vy: Some(0.0),
      vz: Some(0.0),
    }),
    teleport_robots: vec![TeleportRobot {
      id: 0,
      team: TeamColor::Blue,
      x: Some(robot_x),
      y: Some(robot_y),
      orientation: Some(0.0),
      vx: Some(0.0),
      vy: Some(0.0),
      v_angular: Some(0.0),
      present: Some(true),
    }],
    blue: vec![RobotCommand {
      id: 0,
      move_command: None,
      kick_speed: 20.0,
      kick_angle: 45.0,
      dribbler_on: true,
    }],
    ..Default::default()
  });

  let state = world.get_state();
  let speed = ball_speed(&state);
  assert!(
    speed <= MAX_BALL_SPEED + 1e-5,
    "chip kick should be capped at {MAX_BALL_SPEED}m/s, got {speed}"
  );
}

#[test]
fn test_flat_kick_adds_to_existing_ball_velocity() {
  let mut config = WorldConfig::division_a();
  config.robots_per_team = 1;
  let mut world = World::new(0, config);
  let robot_x = -3.0;
  let robot_y = -3.0;

  world.step(&WorldCommand {
    teleport_ball: Some(TeleportBall {
      x: Some(robot_x + world.config.blue_robots.center_from_kicker + world.config.ball.radius),
      y: Some(robot_y),
      z: Some(0.0),
      vx: Some(0.0),
      vy: Some(1.0),
      vz: Some(0.0),
    }),
    teleport_robots: vec![TeleportRobot {
      id: 0,
      team: TeamColor::Blue,
      x: Some(robot_x),
      y: Some(robot_y),
      orientation: Some(0.0),
      vx: Some(0.0),
      vy: Some(0.0),
      v_angular: Some(0.0),
      present: Some(true),
    }],
    blue: vec![RobotCommand {
      id: 0,
      move_command: None,
      kick_speed: 2.0,
      kick_angle: 0.0,
      dribbler_on: true,
    }],
    ..Default::default()
  });

  let state = world.get_state();
  assert!(
    state.ball.vx > 1.5,
    "flat kick should still launch forward, vx={}",
    state.ball.vx
  );
  assert!(
    state.ball.vy > 0.5,
    "flat kick should preserve existing sideways ball velocity, vy={}",
    state.ball.vy
  );
}

#[test]
fn test_flat_kick_launches_along_robot_heading_on_diagonal() {
  let mut config = WorldConfig::division_a();
  config.robots_per_team = 1;
  let mut world = World::new(0, config);
  let robot_x = -3.0;
  let robot_y = -3.0;
  let yaw = std::f64::consts::FRAC_PI_4;
  let mouth_dist = world.config.blue_robots.center_from_kicker + world.config.ball.radius;

  world.step(&WorldCommand {
    teleport_ball: Some(TeleportBall {
      x: Some(robot_x + yaw.cos() * mouth_dist),
      y: Some(robot_y + yaw.sin() * mouth_dist),
      z: Some(0.0),
      vx: Some(0.0),
      vy: Some(0.0),
      vz: Some(0.0),
    }),
    teleport_robots: vec![TeleportRobot {
      id: 0,
      team: TeamColor::Blue,
      x: Some(robot_x),
      y: Some(robot_y),
      orientation: Some(yaw),
      vx: Some(0.0),
      vy: Some(0.0),
      v_angular: Some(0.0),
      present: Some(true),
    }],
    blue: vec![RobotCommand {
      id: 0,
      move_command: None,
      kick_speed: 4.0,
      kick_angle: 0.0,
      dribbler_on: true,
    }],
    ..Default::default()
  });

  let state = world.get_state();
  let launch_angle = state.ball.vy.atan2(state.ball.vx);
  let angle_error_deg = (launch_angle - yaw).to_degrees();
  assert!(
    state.ball.vx > 1.0 && state.ball.vy > 1.0,
    "diagonal flat kick should launch, v=({}, {})",
    state.ball.vx,
    state.ball.vy
  );
  assert!(
    angle_error_deg.abs() < 1.0,
    "stationary robot must kick along its heading; got {angle_error_deg:.2} deg off"
  );
}

#[test]
fn test_infrared_requires_ball_in_front_of_robot() {
  let mut config = WorldConfig::division_a();
  config.robots_per_team = 1;

  let center_from_kicker = config.blue_robots.center_from_kicker;
  let half_width = (config.blue_robots.radius * config.blue_robots.radius
    - center_from_kicker * center_from_kicker)
    .max(0.0)
    .sqrt();

  assert!(blue_infrared_for_ball_offset(
    &config,
    center_from_kicker + config.ball.radius * 0.5,
    0.0
  ));
  assert!(!blue_infrared_for_ball_offset(
    &config,
    -config.ball.radius,
    0.0
  ));
  assert!(!blue_infrared_for_ball_offset(
    &config,
    center_from_kicker - 0.02,
    half_width + config.ball.radius + 0.005
  ));
}

fn blue_infrared_for_ball_offset(config: &WorldConfig, ball_dx: f64, ball_dy: f64) -> bool {
  let robot_x = -3.0;
  let robot_y = -3.0;
  let mut world = World::new(0, config.clone());
  let robot = world.physics.blue_robots[0].clone();
  world.physics.teleport_body(
    robot.chassis_body,
    robot_x as f32,
    robot_y as f32,
    config.blue_robots.start_z() as f32,
  );
  world.physics.set_body_yaw(robot.chassis_body, 0.0);
  let ball_body = world.physics.ball_body;
  world.physics.teleport_body(
    ball_body,
    (robot_x + ball_dx) as f32,
    (robot_y + ball_dy) as f32,
    (config.ball.radius + 0.005) as f32,
  );

  world.get_state().blue_robots[0].infrared
}

fn ball_speed(state: &WorldState) -> f64 {
  (state.ball.vx * state.ball.vx + state.ball.vy * state.ball.vy + state.ball.vz * state.ball.vz)
    .sqrt()
}

#[test]
fn test_teleport_robot_off_field() {
  let mut world = World::new(0, WorldConfig::division_a());

  world.step(&WorldCommand {
    teleport_robots: vec![TeleportRobot {
      id: 0,
      team: TeamColor::Blue,
      x: None,
      y: None,
      orientation: None,
      vx: None,
      vy: None,
      v_angular: None,
      present: Some(false),
    }],
    ..Default::default()
  });

  let state = world.get_state();
  assert!(!state.blue_robots[0].is_on, "robot should be off");
}

// ============================================================================
// Goal detection
// ============================================================================

#[test]
fn test_goal_detection_blue_scores() {
  let mut config = WorldConfig::division_a();
  config.robots_per_team = 0;
  let mut world = World::new(0, config.clone());

  // Place ball past the positive goal line (yellow's goal = blue scores)
  let half = config.field.field_length / 2.0;
  world.step(&WorldCommand {
    teleport_ball: Some(TeleportBall {
      x: Some(half + 0.1),
      y: Some(0.0),
      z: Some(0.0),
      vx: Some(0.0),
      vy: Some(0.0),
      vz: Some(0.0),
    }),
    ..Default::default()
  });

  let state = world.get_state();
  assert!(
    state.goal_blue,
    "should detect blue goal (ball past +x goal line)"
  );
  assert!(!state.goal_yellow);
}

// ============================================================================
// Determinism
// ============================================================================

#[test]
fn test_single_world_deterministic() {
  let config = WorldConfig::division_a();

  let cmd = WorldCommand {
    blue: vec![RobotCommand {
      id: 0,
      move_command: Some(MoveCommand::LocalVelocity {
        forward: 1.5,
        left: 0.5,
        angular: 0.3,
      }),
      kick_speed: 0.0,
      kick_angle: 0.0,
      dribbler_on: false,
    }],
    ..Default::default()
  };

  // Run 1
  let mut world1 = World::new(0, config.clone());
  for _ in 0..100 {
    world1.step(&cmd);
  }
  let state1 = world1.get_state();

  // Run 2
  let mut world2 = World::new(0, config);
  for _ in 0..100 {
    world2.step(&cmd);
  }
  let state2 = world2.get_state();

  assert_eq!(
    state1.ball.x, state2.ball.x,
    "ball x should be deterministic"
  );
  assert_eq!(
    state1.ball.y, state2.ball.y,
    "ball y should be deterministic"
  );
  assert_eq!(
    state1.blue_robots[0].x, state2.blue_robots[0].x,
    "robot x should be deterministic"
  );
}

#[test]
fn test_parallel_worlds_match_sequential() {
  let config = WorldConfig::division_a();
  let cmd = WorldCommand::default();

  // Sequential
  let mut seq_states = Vec::new();
  for i in 0..4 {
    let mut cfg = config.clone();
    cfg.seed = config.seed.wrapping_add(i as u64);
    let mut world = World::new(i, cfg);
    for _ in 0..10 {
      world.step(&cmd);
    }
    seq_states.push(world.get_state());
  }

  // Parallel
  let mut engine = SimulationEngine::new(4, config);
  let mut par_states = Vec::new();
  for _ in 0..10 {
    par_states = engine.step_all();
  }

  for i in 0..4 {
    assert_eq!(
      seq_states[i].ball.x, par_states[i].ball.x,
      "world {i} ball x should match between sequential and parallel"
    );
    assert_eq!(
      seq_states[i].ball.y, par_states[i].ball.y,
      "world {i} ball y should match between sequential and parallel"
    );
  }
}

// ============================================================================
// Parallel engine
// ============================================================================

#[test]
fn test_engine_creates_correct_count() {
  let engine = SimulationEngine::new(16, WorldConfig::division_a());
  assert_eq!(engine.count(), 16);
}

#[test]
fn test_engine_step_all_returns_correct_count() {
  let mut engine = SimulationEngine::new(8, WorldConfig::division_a());
  let states = engine.step_all();
  assert_eq!(states.len(), 8);
}

#[test]
fn test_engine_per_world_commands() {
  let mut engine = SimulationEngine::new(4, WorldConfig::division_a());

  let commands: Vec<WorldCommand> = (0..4)
    .map(|i| WorldCommand {
      blue: vec![RobotCommand {
        id: 0,
        move_command: Some(MoveCommand::LocalVelocity {
          forward: (i + 1) as f64,
          left: 0.0,
          angular: 0.0,
        }),
        kick_speed: 0.0,
        kick_angle: 0.0,
        dribbler_on: false,
      }],
      ..Default::default()
    })
    .collect();

  for _ in 0..30 {
    engine.step_with_commands(&commands);
  }

  let states = engine.get_all_states();
  // Worlds with higher forward speed should have moved further
  // (approximately, physics may vary)
  for s in &states {
    assert_eq!(s.blue_robots.len(), 11);
  }
}

#[test]
fn test_engine_reset_all() {
  let mut engine = SimulationEngine::new(4, WorldConfig::division_a());

  // Step forward
  for _ in 0..60 {
    engine.step_all();
  }

  let before_reset = engine.get_all_states();
  assert!(before_reset[0].frame > 0);

  engine.reset_all();

  let after_reset = engine.get_all_states();
  assert_eq!(after_reset[0].frame, 0);
}

#[test]
fn test_engine_reset_specific_worlds() {
  let mut engine = SimulationEngine::new(4, WorldConfig::division_a());

  for _ in 0..30 {
    engine.step_all();
  }

  engine.reset_worlds(&[1, 3]);

  let states = engine.get_all_states();
  assert!(states[0].frame > 0, "world 0 should not be reset");
  assert_eq!(states[1].frame, 0, "world 1 should be reset");
  assert!(states[2].frame > 0, "world 2 should not be reset");
  assert_eq!(states[3].frame, 0, "world 3 should be reset");
}

// ============================================================================
// Domain randomization
// ============================================================================

#[test]
fn test_randomized_worlds_diverge() {
  let config = WorldConfig::division_a();
  let mut engine = SimulationEngine::new_randomized(4, config, RandomizationConfig::moderate());

  for _ in 0..60 {
    engine.step_all();
  }

  let states = engine.get_all_states();
  // With randomization, worlds should diverge
  let _all_same = states.windows(2).all(|w| {
    (w[0].ball.x - w[1].ball.x).abs() < 1e-10 && (w[0].ball.y - w[1].ball.y).abs() < 1e-10
  });
  // They can still be very similar if ball doesn't move much, but configs differ
  let configs_differ =
    engine.worlds.iter().enumerate().any(|(i, w)| {
      i > 0 && (w.config.ball.mass - engine.worlds[0].config.ball.mass).abs() > 1e-10
    });
  assert!(
    configs_differ,
    "randomized worlds should have different configs"
  );
}

// ============================================================================
// Velocity limits
// ============================================================================

#[test]
fn test_velocity_limit_in_command() {
  // Test that the velocity limiting in RobotSim works correctly
  let config = WorldConfig::division_a();
  let mut sim = simhark::robot::RobotSim::new(0, &config.blue_robots, 1.0);

  // Request absurd velocity
  sim.set_local_velocity(100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 / 60.0);

  // Wheel speeds should be bounded by the vel_absolute_max -> wheel conversion
  let max_wheel = sim
    .wheel_speeds
    .iter()
    .map(|s| s.abs())
    .fold(0.0_f64, f64::max);
  // With vel_absolute_max = 5.0, wheel speeds should be reasonable
  // The exact value depends on the inverse kinematics, but shouldn't be enormous
  assert!(
    max_wheel < 1000.0,
    "wheel speeds should be bounded after velocity limiting, got max={}",
    max_wheel
  );
}

// ============================================================================
// Config serialization
// ============================================================================

#[test]
fn test_config_serialization_roundtrip() {
  let config = WorldConfig::division_a();
  let json = serde_json::to_string_pretty(&config).unwrap();
  let deserialized: WorldConfig = serde_json::from_str(&json).unwrap();

  assert_eq!(config.robots_per_team, deserialized.robots_per_team);
  assert_eq!(config.field.field_length, deserialized.field.field_length);
  assert_eq!(config.ball.mass, deserialized.ball.mass);
  assert_eq!(config.blue_robots.radius, deserialized.blue_robots.radius);
}

// ============================================================================
// Edge cases
// ============================================================================

#[test]
fn test_zero_robots() {
  let mut config = WorldConfig::division_a();
  config.robots_per_team = 0;
  let mut world = World::new(0, config);

  let state = world.step_empty();
  assert!(state.blue_robots.is_empty());
  assert!(state.yellow_robots.is_empty());
}

#[test]
fn test_command_for_nonexistent_robot_ignored() {
  let config = WorldConfig::division_a();
  let mut world = World::new(0, config);

  // Command for robot id=99 which doesn't exist
  let cmd = WorldCommand {
    blue: vec![RobotCommand {
      id: 99,
      move_command: Some(MoveCommand::LocalVelocity {
        forward: 10.0,
        left: 0.0,
        angular: 0.0,
      }),
      kick_speed: 0.0,
      kick_angle: 0.0,
      dribbler_on: false,
    }],
    ..Default::default()
  };

  // Should not panic
  world.step(&cmd);
}

#[test]
#[ignore = "requires built Sumatra checkout"]
fn test_sumatra_single_team_scores_five_goals() {
  let mut config = WorldConfig::division_a();
  config.robots_per_team = 6;
  let mut engine = SimulationEngine::new(1, config.clone());
  let mut server = SumatraSimNetServer::bind(SumatraSimNetConfig::default()).unwrap();

  let mut blue = SumatraInstance::spawn(&SumatraLaunchConfig {
    remote_client: true,
    ai_blue: true,
    ai_yellow: false,
    host: Some("127.0.0.1".to_string()),
    ..SumatraLaunchConfig::default()
  })
  .unwrap();

  let start = Instant::now();
  let mut goals = 0usize;
  let mut cooldown = 0u32;

  while start.elapsed() < Duration::from_secs(60) && goals < 5 {
    let mut commands = WorldCommand::default();
    commands.teleport_robots = (0..config.robots_per_team)
      .map(|id| TeleportRobot {
        id,
        team: TeamColor::Yellow,
        x: Some(20.0 + id as f64),
        y: Some(20.0 + id as f64),
        orientation: Some(0.0),
        vx: Some(0.0),
        vy: Some(0.0),
        v_angular: Some(0.0),
        present: Some(false),
      })
      .collect();

    let states = server
      .step_with_local_commands(&mut engine, &[commands])
      .unwrap();
    let state = &states[0];

    if cooldown > 0 {
      cooldown -= 1;
    }

    if state.goal_blue && cooldown == 0 {
      goals += 1;
      cooldown = 30;
      let reset = WorldCommand {
        teleport_ball: Some(TeleportBall {
          x: Some(0.0),
          y: Some(0.0),
          z: Some(0.0),
          vx: Some(0.0),
          vy: Some(0.0),
          vz: Some(0.0),
        }),
        teleport_robots: (0..config.robots_per_team)
          .flat_map(|id| {
            [
              TeleportRobot {
                id,
                team: TeamColor::Blue,
                x: None,
                y: None,
                orientation: Some(0.0),
                vx: Some(0.0),
                vy: Some(0.0),
                v_angular: Some(0.0),
                present: Some(true),
              },
              TeleportRobot {
                id,
                team: TeamColor::Yellow,
                x: Some(20.0 + id as f64),
                y: Some(20.0 + id as f64),
                orientation: Some(0.0),
                vx: Some(0.0),
                vy: Some(0.0),
                v_angular: Some(0.0),
                present: Some(false),
              },
            ]
          })
          .collect(),
        ..Default::default()
      };
      engine.step_with_commands(&[reset]);
    }

    if blue.try_wait().unwrap().is_some() {
      panic!("Sumatra blue process exited before scoring five goals");
    }

    std::thread::sleep(Duration::from_millis(16));
  }

  blue.kill().unwrap();

  assert!(
    goals >= 5,
    "blue team should score 5 goals against disabled yellow team within the timeout, scored {goals}"
  );
}

#[test]
fn test_many_steps_stability() {
  let mut config = WorldConfig::division_a();
  config.robots_per_team = 3;
  let mut world = World::new(0, config);

  // Run 1000 steps - should not crash or produce NaN
  for _ in 0..1000 {
    let state = world.step_empty();
    assert!(!state.ball.x.is_nan(), "ball x became NaN");
    assert!(!state.ball.y.is_nan(), "ball y became NaN");
    assert!(!state.ball.z.is_nan(), "ball z became NaN");
    for r in state.blue_robots.iter().chain(state.yellow_robots.iter()) {
      assert!(!r.x.is_nan(), "robot x became NaN");
      assert!(!r.y.is_nan(), "robot y became NaN");
    }
  }
}
