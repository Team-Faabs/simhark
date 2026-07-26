use std::thread;
use std::time::{Duration, Instant};

use anyhow::Result;
#[cfg(feature = "motion-audit")]
use simhark::motion_audit::MotionAuditor;
use simhark::viewer::{ViewerConfig, ViewerServer};
use simhark::{
  MoveCommand, RobotCommand, SimulationEngine, TeamColor, TeleportBall, TeleportRobot,
  WorldCommand, WorldConfig, WorldState,
};

const ROBOT_ID: usize = 0;
const POSITION_TOLERANCE: f64 = 0.12;
const ORIENTATION_TOLERANCE: f64 = 0.12;
const WAYPOINT_SPEED: f64 = 1.4;
const CIRCLE_RADIUS: f64 = 0.9;
const CIRCLE_LINEAR_SPEED: f64 = 1.3;
const CIRCLE_DURATION_SECS: f64 = 7.0;
const DRIBBLE_SPEED: f64 = 0.9;
const BALL_APPROACH_OFFSET: f64 = 0.32;
const BALL_SHOOT_OFFSET: f64 = 0.12;
const BALL_STAGE_TOLERANCE: f64 = 0.08;
const SHOOT_ALIGNMENT_TOLERANCE: f64 = 0.08;
const POSSESSION_FRAMES_REQUIRED: u32 = 5;
const KICK_SPEED: f64 = 6.0;
const MAX_RUNTIME: Duration = Duration::from_secs(45);
const STEP_SLEEP: Duration = Duration::from_millis(16);

fn main() -> Result<()> {
  let config = WorldConfig::division_b();
  let viewer_config = ViewerConfig::default();
  let mut engine = SimulationEngine::new(1, config.clone());
  let viewer = ViewerServer::bind(viewer_config, 1, &config)?;
  println!("viewer: {}", viewer_config.http_url());
  let goal = Point {
    x: config.field.field_length * 0.5,
    y: 0.0,
  };

  let setup = initial_command(&config);
  let mut state = engine.step_with_commands(&[setup]).remove(0);
  viewer.publish(&state);

  let corner_x = config.field.field_length * 0.5 - 0.45;
  let corner_y = config.field.field_width * 0.5 - 0.45;
  let ball_buffer = config.blue_robots.radius + config.ball.radius + 0.18;
  let path = vec![
    PoseTarget::at(-ball_buffer, 0.45),
    PoseTarget::at(corner_x, corner_y),
    PoseTarget::at(corner_x, -corner_y),
    PoseTarget::at(-corner_x, -corner_y),
    PoseTarget::at(-corner_x, corner_y),
  ];
  let mut waypoint_index = 0usize;
  let mut phase = Phase::Waypoints;
  let mut possession_frames = 0u32;
  let mut previous_state: Option<WorldState> = None;
  #[cfg(feature = "motion-audit")]
  let mut motion_auditor = MotionAuditor::default();
  let start = Instant::now();

  while start.elapsed() < MAX_RUNTIME {
    if viewer.apply_robot_move_requests(&mut engine) > 0 {
      state = engine.world(0).get_state();
    }

    possession_frames = if robot(&state).infrared {
      possession_frames + 1
    } else {
      0
    };

    let command = match phase {
      Phase::Waypoints => {
        let target = path[waypoint_index];
        if pose_reached(&state, target) {
          waypoint_index += 1;
          if waypoint_index == path.len() {
            let robot = robot(&state);
            phase = Phase::Circle {
              start_time: state.sim_time,
              angle_offset: robot.y.atan2(robot.x),
            };
          }
        }

        if let Phase::Waypoints = phase {
          drive_to_pose(&state, path[waypoint_index], WAYPOINT_SPEED, false, None)
        } else {
          circle_command(&state, 0.0, 0.0)
        }
      }
      Phase::Circle {
        start_time,
        angle_offset,
      } => {
        let elapsed = state.sim_time - start_time;
        if elapsed >= CIRCLE_DURATION_SECS {
          phase = Phase::CollectBall;
          collect_ball_command(&state, goal)
        } else {
          circle_command(&state, elapsed, angle_offset)
        }
      }
      Phase::CollectBall => {
        if possession_frames >= POSSESSION_FRAMES_REQUIRED {
          phase = Phase::Shoot;
          shoot_command(&state, goal)
        } else {
          collect_ball_command(&state, goal)
        }
      }
      Phase::Shoot => {
        if !robot(&state).infrared {
          phase = Phase::CollectBall;
          collect_ball_command(&state, goal)
        } else {
          let cmd = shoot_command(&state, goal);
          if state.goal_blue {
            println!("goal at t={:.2}", state.sim_time);
            break;
          }
          cmd
        }
      }
    };

    state = engine.step_with_commands(&[command]).remove(0);
    viewer.publish(&state);

    log_motion(
      &state,
      previous_state.as_ref(),
      &config,
      #[cfg(feature = "motion-audit")]
      &mut motion_auditor,
    );
    previous_state = Some(state.clone());

    if state.goal_blue {
      println!("goal at t={:.2}", state.sim_time);
      break;
    }

    if state.frame % 30 == 0 {
      let robot = robot(&state);
      println!(
        "t={:.2} phase={} robot=({:.2},{:.2}) ball=({:.2},{:.2}) ir={} kick={:?}",
        state.sim_time,
        phase.label(),
        robot.x,
        robot.y,
        state.ball.x,
        state.ball.y,
        robot.infrared,
        robot.kick_status,
      );
    }

    thread::sleep(STEP_SLEEP);
  }

  if !state.goal_blue {
    println!("finished without a goal after {:.2}s", state.sim_time);
  }

  Ok(())
}

#[derive(Clone, Copy)]
struct PoseTarget {
  x: f64,
  y: f64,
  orientation: Option<f64>,
}

#[derive(Clone, Copy)]
struct Point {
  x: f64,
  y: f64,
}

impl PoseTarget {
  fn at(x: f64, y: f64) -> Self {
    Self {
      x,
      y,
      orientation: None,
    }
  }

  fn face(x: f64, y: f64, orientation: f64) -> Self {
    Self {
      x,
      y,
      orientation: Some(orientation),
    }
  }
}

#[derive(Clone, Copy)]
enum Phase {
  Waypoints,
  Circle { start_time: f64, angle_offset: f64 },
  CollectBall,
  Shoot,
}

impl Phase {
  fn label(self) -> &'static str {
    match self {
      Phase::Waypoints => "waypoints",
      Phase::Circle { .. } => "circle",
      Phase::CollectBall => "collect",
      Phase::Shoot => "shoot",
    }
  }
}

fn initial_command(config: &WorldConfig) -> WorldCommand {
  let robot_x = -config.field.field_length * 0.5 + 0.7;
  let mut teleport_robots = Vec::with_capacity(config.robots_per_team * 2);
  teleport_robots.push(TeleportRobot {
    id: ROBOT_ID,
    team: TeamColor::Blue,
    x: Some(robot_x),
    y: Some(0.0),
    orientation: Some(0.0),
    vx: Some(0.0),
    vy: Some(0.0),
    v_angular: Some(0.0),
    present: Some(true),
  });
  for id in 0..config.robots_per_team {
    if id == ROBOT_ID {
      continue;
    }
    teleport_robots.push(TeleportRobot {
      id,
      team: TeamColor::Blue,
      x: None,
      y: None,
      orientation: None,
      vx: None,
      vy: None,
      v_angular: None,
      present: Some(false),
    });
  }
  for id in 0..config.robots_per_team {
    teleport_robots.push(TeleportRobot {
      id,
      team: TeamColor::Yellow,
      x: None,
      y: None,
      orientation: None,
      vx: None,
      vy: None,
      v_angular: None,
      present: Some(false),
    });
  }

  WorldCommand {
    blue: vec![RobotCommand::noop(ROBOT_ID)],
    teleport_ball: Some(TeleportBall {
      x: Some(0.0),
      y: Some(0.0),
      z: Some(0.0),
      vx: Some(0.0),
      vy: Some(0.0),
      vz: Some(0.0),
    }),
    teleport_robots,
    ..Default::default()
  }
}

fn pose_reached(state: &WorldState, target: PoseTarget) -> bool {
  let robot = robot(state);
  let dx = target.x - robot.x;
  let dy = target.y - robot.y;
  let position_ok = (dx * dx + dy * dy).sqrt() <= POSITION_TOLERANCE;
  let orientation_ok = target
    .orientation
    .map(|orientation| angle_error(orientation, robot.orientation).abs() <= ORIENTATION_TOLERANCE)
    .unwrap_or(true);
  position_ok && orientation_ok
}

fn drive_to_pose(
  state: &WorldState,
  target: PoseTarget,
  max_speed: f64,
  dribbler_on: bool,
  kick_speed: Option<f64>,
) -> WorldCommand {
  let robot = robot(state);
  let dx = target.x - robot.x;
  let dy = target.y - robot.y;
  let distance = (dx * dx + dy * dy).sqrt();
  let desired_speed = if distance < 0.03 {
    0.0
  } else {
    (distance * 1.8).min(max_speed)
  };
  let (vx, vy) = if distance > 1e-6 {
    (dx / distance * desired_speed, dy / distance * desired_speed)
  } else {
    (0.0, 0.0)
  };
  let angular = target
    .orientation
    .map(|orientation| {
      let error = angle_error(orientation, robot.orientation);
      if error.abs() <= ORIENTATION_TOLERANCE * 0.5 {
        0.0
      } else {
        (error * 4.0).clamp(-3.5, 3.5)
      }
    })
    .unwrap_or(0.0);

  global_velocity_command(vx, vy, angular, dribbler_on, kick_speed.unwrap_or(0.0))
}

fn circle_command(state: &WorldState, elapsed: f64, angle_offset: f64) -> WorldCommand {
  let angle = angle_offset + elapsed * (CIRCLE_LINEAR_SPEED / CIRCLE_RADIUS);
  let target_x = CIRCLE_RADIUS * angle.cos();
  let target_y = CIRCLE_RADIUS * angle.sin();
  let tangent_heading = angle + std::f64::consts::FRAC_PI_2;
  drive_to_pose(
    state,
    PoseTarget::face(target_x, target_y, tangent_heading),
    CIRCLE_LINEAR_SPEED,
    false,
    None,
  )
}

fn collect_ball_command(state: &WorldState, goal: Point) -> WorldCommand {
  let robot = robot(state);
  let goal_heading = heading_to_point(state.ball.x, state.ball.y, goal.x, goal.y);
  let behind_target = ball_control_target(state, goal, BALL_APPROACH_OFFSET);
  let behind_dx = behind_target.x - robot.x;
  let behind_dy = behind_target.y - robot.y;
  let behind_distance = (behind_dx * behind_dx + behind_dy * behind_dy).sqrt();
  let heading_error = angle_error(goal_heading, robot.orientation);

  if behind_distance > BALL_STAGE_TOLERANCE {
    return drive_to_pose_local(state, behind_target, DRIBBLE_SPEED, true, None);
  }

  if heading_error.abs() > SHOOT_ALIGNMENT_TOLERANCE {
    return local_velocity_command(0.0, 0.0, (heading_error * 3.5).clamp(-1.8, 1.8), true, 0.0);
  }

  local_velocity_command(0.45, 0.0, 0.0, true, 0.0)
}

fn shoot_command(state: &WorldState, goal: Point) -> WorldCommand {
  let robot = robot(state);
  if !robot.infrared {
    return drive_to_pose_local(
      state,
      ball_control_target(state, goal, BALL_SHOOT_OFFSET),
      0.6,
      true,
      None,
    );
  }

  let goal_heading = heading_to_point(state.ball.x, state.ball.y, goal.x, goal.y);
  let heading_error = angle_error(goal_heading, robot.orientation);
  let aligned = heading_error.abs() <= SHOOT_ALIGNMENT_TOLERANCE;

  local_velocity_command(
    0.08,
    0.0,
    (heading_error * 3.5).clamp(-1.6, 1.6),
    true,
    if aligned { KICK_SPEED } else { 0.0 },
  )
}

fn ball_control_target(state: &WorldState, goal: Point, offset: f64) -> PoseTarget {
  let ball = &state.ball;
  let dx = goal.x - ball.x;
  let dy = goal.y - ball.y;
  let distance = (dx * dx + dy * dy).sqrt().max(1e-6);
  let dir_x = dx / distance;
  let dir_y = dy / distance;

  PoseTarget::face(
    ball.x - dir_x * offset,
    ball.y - dir_y * offset,
    heading_to_point(ball.x, ball.y, goal.x, goal.y),
  )
}

fn global_velocity_command(
  vx: f64,
  vy: f64,
  angular: f64,
  dribbler_on: bool,
  kick_speed: f64,
) -> WorldCommand {
  WorldCommand {
    blue: vec![RobotCommand {
      id: ROBOT_ID,
      move_command: Some(MoveCommand::GlobalVelocity { vx, vy, angular }),
      kick_speed,
      kick_angle: 0.0,
      dribbler_on,
    }],
    ..Default::default()
  }
}

fn drive_to_pose_local(
  state: &WorldState,
  target: PoseTarget,
  max_speed: f64,
  dribbler_on: bool,
  kick_speed: Option<f64>,
) -> WorldCommand {
  let robot = robot(state);
  let dx = target.x - robot.x;
  let dy = target.y - robot.y;
  let forward_error = dx * robot.orientation.cos() + dy * robot.orientation.sin();
  let left_error = -dx * robot.orientation.sin() + dy * robot.orientation.cos();
  let forward = (forward_error * 2.0).clamp(-max_speed, max_speed);
  let left = (left_error * 2.0).clamp(-max_speed, max_speed);
  let angular = target
    .orientation
    .map(|orientation| (angle_error(orientation, robot.orientation) * 4.0).clamp(-3.0, 3.0))
    .unwrap_or(0.0);

  local_velocity_command(
    forward,
    left,
    angular,
    dribbler_on,
    kick_speed.unwrap_or(0.0),
  )
}

fn local_velocity_command(
  forward: f64,
  left: f64,
  angular: f64,
  dribbler_on: bool,
  kick_speed: f64,
) -> WorldCommand {
  WorldCommand {
    blue: vec![RobotCommand {
      id: ROBOT_ID,
      move_command: Some(MoveCommand::LocalVelocity {
        forward,
        left,
        angular,
      }),
      kick_speed,
      kick_angle: 0.0,
      dribbler_on,
    }],
    ..Default::default()
  }
}

fn robot(state: &WorldState) -> &simhark::RobotState {
  &state.blue_robots[ROBOT_ID]
}

fn heading_to_point(from_x: f64, from_y: f64, to_x: f64, to_y: f64) -> f64 {
  (to_y - from_y).atan2(to_x - from_x)
}

fn angle_error(target: f64, current: f64) -> f64 {
  let two_pi = std::f64::consts::TAU;
  (target - current + std::f64::consts::PI).rem_euclid(two_pi) - std::f64::consts::PI
}

fn log_motion(
  state: &WorldState,
  previous_state: Option<&WorldState>,
  config: &WorldConfig,
  #[cfg(feature = "motion-audit")] motion_auditor: &mut MotionAuditor,
) {
  let robot = robot(state);
  let ball_speed = speed3(state.ball.vx, state.ball.vy, state.ball.vz);
  let robot_speed = speed2(robot.vx, robot.vy);
  let (ball_acc, robot_acc) = previous_state
    .map(|previous| {
      let dt = (state.sim_time - previous.sim_time).max(f64::EPSILON);
      let ball_acc = speed3(
        (state.ball.vx - previous.ball.vx) / dt,
        (state.ball.vy - previous.ball.vy) / dt,
        (state.ball.vz - previous.ball.vz) / dt,
      );
      let previous_robot = &previous.blue_robots[ROBOT_ID];
      let robot_acc = speed2(
        (robot.vx - previous_robot.vx) / dt,
        (robot.vy - previous_robot.vy) / dt,
      );
      (ball_acc, robot_acc)
    })
    .unwrap_or((0.0, 0.0));

  if state.frame % 15 == 0 {
    println!(
      "motion t={:.2} robot pos=({:.3},{:.3}) vel=({:.2},{:.2}) speed={:.2} acc={:.2} ball pos=({:.3},{:.3}) vel=({:.2},{:.2}) speed={:.2} acc={:.2}",
      state.sim_time,
      robot.x,
      robot.y,
      robot.vx,
      robot.vy,
      robot_speed,
      robot_acc,
      state.ball.x,
      state.ball.y,
      state.ball.vx,
      state.ball.vy,
      ball_speed,
      ball_acc,
    );
  }

  #[cfg(feature = "motion-audit")]
  {
    let findings = motion_auditor.audit(state, previous_state, config);
    for finding in findings {
      println!("motion-audit {} {}", finding.kind, finding.detail);
    }
  }
}

fn speed2(x: f64, y: f64) -> f64 {
  (x * x + y * y).sqrt()
}

fn speed3(x: f64, y: f64, z: f64) -> f64 {
  (x * x + y * y + z * z).sqrt()
}
