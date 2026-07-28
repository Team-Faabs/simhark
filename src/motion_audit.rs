use std::collections::{HashMap, VecDeque};

use crate::config::WorldConfig;
use crate::state::{RobotState, TeamColor, WorldState};

const HISTORY_LEN: usize = 6;
const POSITION_EPSILON: f64 = 1e-4;
const OSCILLATION_AXIS_DELTA: f64 = 0.015;
const OSCILLATION_ORTHOGONAL_RANGE: f64 = 0.04;
const TWO_POINT_MATCH_DISTANCE: f64 = 0.02;
const TWO_POINT_SEPARATION: f64 = 0.05;
const BALL_SETTLE_SPEED: f64 = 0.20;
const BALL_SETTLE_ACCEL: f64 = 1.5;
const BALL_SETTLE_JERK: f64 = 40.0;
const ROBOT_SETTLE_SPEED: f64 = 0.35;
const ROBOT_SETTLE_ACCEL: f64 = 2.0;
const ROBOT_SETTLE_JERK: f64 = 50.0;
const BALL_ENERGY_GAIN_EPSILON: f64 = 0.05;
const ROBOT_ENERGY_GAIN_EPSILON: f64 = 0.08;
const WOBBLE_SIGN_FLIP_MIN_SPEED: f64 = 0.08;
const WOBBLE_SIGN_FLIP_MIN_ACCEL: f64 = 0.8;
const MAX_FREE_BALL_SPEED_GAIN: f64 = 0.75;
const MAX_FREE_BALL_ACCEL_GAIN: f64 = 4.0;
const BALL_FORCE_EPSILON: f64 = 0.35;

#[derive(Debug, Clone)]
pub struct MotionFinding {
  pub kind: &'static str,
  pub detail: String,
}

#[derive(Clone, Copy)]
struct MotionSample {
  t: f64,
  x: f64,
  y: f64,
  vx: f64,
  vy: f64,
  speed: f64,
  acc: f64,
}

#[derive(Default)]
pub struct MotionAuditor {
  ball_history: VecDeque<MotionSample>,
  robot_history: HashMap<(TeamColor, usize), VecDeque<MotionSample>>,
}

impl MotionAuditor {
  pub fn audit(
    &mut self,
    current: &WorldState,
    previous: Option<&WorldState>,
    config: &WorldConfig,
  ) -> Vec<MotionFinding> {
    let mut findings = audit_world(current, previous, config);
    self.audit_ball_history(current, config, &mut findings);
    self.audit_robot_history(current, &mut findings);
    self.push_state(current);
    findings
  }

  fn audit_ball_history(
    &self,
    current: &WorldState,
    config: &WorldConfig,
    findings: &mut Vec<MotionFinding>,
  ) {
    if let Some(detail) = detect_line_oscillation(&self.ball_history) {
      findings.push(MotionFinding {
        kind: "ball-line-jitter",
        detail: format!(
          "ball {detail} near=({}, {}) pos=({:.3},{:.3}) vel=({:.2},{:.2})",
          near_vertical_wall(current, config),
          near_horizontal_wall(current, config),
          current.ball.x,
          current.ball.y,
          current.ball.vx,
          current.ball.vy,
        ),
      });
    }
    if let Some(detail) = detect_wobble(
      &self.ball_history,
      BALL_SETTLE_SPEED,
      BALL_SETTLE_ACCEL,
      BALL_SETTLE_JERK,
    ) {
      findings.push(MotionFinding {
        kind: "ball-wobble",
        detail: format!(
          "ball {detail} pos=({:.3},{:.3}) vel=({:.2},{:.2})",
          current.ball.x, current.ball.y, current.ball.vx, current.ball.vy,
        ),
      });
    }
  }

  fn audit_robot_history(&self, current: &WorldState, findings: &mut Vec<MotionFinding>) {
    for robot in current
      .blue_robots
      .iter()
      .chain(current.yellow_robots.iter())
    {
      let key = (robot.team, robot.id);
      let Some(history) = self.robot_history.get(&key) else {
        continue;
      };
      if let Some(detail) = detect_line_oscillation(history) {
        findings.push(MotionFinding {
          kind: "robot-line-jitter",
          detail: format!(
            "{}{} {detail} pos=({:.3},{:.3}) vel=({:.2},{:.2}) av={:.2}",
            team_label(robot.team),
            robot.id,
            robot.x,
            robot.y,
            robot.vx,
            robot.vy,
            robot.v_angular,
          ),
        });
      }
      let wobble_detail = detect_wobble(
        history,
        ROBOT_SETTLE_SPEED,
        ROBOT_SETTLE_ACCEL,
        ROBOT_SETTLE_JERK,
      );
      if let Some(detail) = wobble_detail {
        findings.push(MotionFinding {
          kind: "robot-wobble",
          detail: format!(
            "{}{} {detail} pos=({:.3},{:.3}) vel=({:.2},{:.2}) av={:.2}",
            team_label(robot.team),
            robot.id,
            robot.x,
            robot.y,
            robot.vx,
            robot.vy,
            robot.v_angular,
          ),
        });
      }
    }
  }

  fn push_state(&mut self, current: &WorldState) {
    push_history(
      &mut self.ball_history,
      MotionSample {
        t: current.sim_time,
        x: current.ball.x,
        y: current.ball.y,
        vx: current.ball.vx,
        vy: current.ball.vy,
        speed: speed3(current.ball.vx, current.ball.vy, current.ball.vz),
        acc: ball_acceleration(current, None),
      },
    );

    for robot in current
      .blue_robots
      .iter()
      .chain(current.yellow_robots.iter())
    {
      push_history(
        self
          .robot_history
          .entry((robot.team, robot.id))
          .or_default(),
        MotionSample {
          t: current.sim_time,
          x: robot.x,
          y: robot.y,
          vx: robot.vx,
          vy: robot.vy,
          speed: speed2(robot.vx, robot.vy),
          acc: 0.0,
        },
      );
    }

    for history in self.robot_history.values_mut() {
      backfill_acceleration(history);
    }
    backfill_acceleration(&mut self.ball_history);
  }
}

pub fn audit_world(
  current: &WorldState,
  previous: Option<&WorldState>,
  config: &WorldConfig,
) -> Vec<MotionFinding> {
  let mut findings = Vec::new();

  audit_robots(current, config, &mut findings);
  audit_ball(current, previous, config, &mut findings);

  if let Some(previous) = previous {
    audit_robot_continuity(current, previous, config, &mut findings);
  }

  findings
}

fn audit_robots(current: &WorldState, config: &WorldConfig, findings: &mut Vec<MotionFinding>) {
  for robot in current
    .blue_robots
    .iter()
    .chain(current.yellow_robots.iter())
  {
    let cfg = match robot.team {
      TeamColor::Blue => &config.blue_robots,
      TeamColor::Yellow => &config.yellow_robots,
    };
    let speed = speed2(robot.vx, robot.vy);
    if speed > cfg.vel_absolute_max + 0.25 {
      findings.push(MotionFinding {
        kind: "robot-speed",
        detail: format!(
          "{}{} speed {:.2} m/s exceeds limit {:.2}",
          team_label(robot.team),
          robot.id,
          speed,
          cfg.vel_absolute_max,
        ),
      });
    }
    if robot.v_angular.abs() > cfg.vel_angular_max + 1.0 {
      findings.push(MotionFinding {
        kind: "robot-spin",
        detail: format!(
          "{}{} angular {:.2} rad/s exceeds limit {:.2}",
          team_label(robot.team),
          robot.id,
          robot.v_angular,
          cfg.vel_angular_max,
        ),
      });
    }
  }
}

fn audit_ball(
  current: &WorldState,
  previous: Option<&WorldState>,
  config: &WorldConfig,
  findings: &mut Vec<MotionFinding>,
) {
  let Some(previous) = previous else {
    return;
  };

  let dt = current.sim_time - previous.sim_time;
  if dt <= f64::EPSILON {
    return;
  }

  let current_speed = speed3(current.ball.vx, current.ball.vy, current.ball.vz);
  let previous_speed = speed3(previous.ball.vx, previous.ball.vy, previous.ball.vz);
  let ball_acc = speed3(
    (current.ball.vx - previous.ball.vx) / dt,
    (current.ball.vy - previous.ball.vy) / dt,
    (current.ball.vz - previous.ball.vz) / dt,
  );

  let current_nearest = nearest_robot_distance(current);
  let previous_nearest = nearest_robot_distance(previous);
  let free_ball = current_nearest > config.blue_robots.radius + config.ball.radius + 0.08
    && previous_nearest > config.blue_robots.radius + config.ball.radius + 0.08;
  let near_wall = near_vertical_wall(current, config)
    || near_horizontal_wall(current, config)
    || near_vertical_wall(previous, config)
    || near_horizontal_wall(previous, config);
  let airborne = current.ball.z > config.ball.radius + 0.03
    || previous.ball.z > config.ball.radius + 0.03
    || current.ball.vz.abs() > 0.5
    || previous.ball.vz.abs() > 0.5;
  let auditable_free_ball = free_ball && !near_wall && !airborne;

  if auditable_free_ball && current_speed > previous_speed + MAX_FREE_BALL_SPEED_GAIN {
    findings.push(MotionFinding {
      kind: "ball-speed-gain",
      detail: format!(
        "ball speed increased from {:.2} to {:.2} m/s without nearby robot",
        previous_speed, current_speed,
      ),
    });
  }
  if auditable_free_ball
    && current_speed > previous_speed + 0.25
    && ball_acc > MAX_FREE_BALL_ACCEL_GAIN
  {
    findings.push(MotionFinding {
      kind: "ball-acc-gain",
      detail: format!(
        "ball acceleration {:.2} m/s^2 without nearby robot interaction",
        ball_acc,
      ),
    });
  }

  let energy_gain = current_speed * current_speed - previous_speed * previous_speed;
  if auditable_free_ball && energy_gain > BALL_ENERGY_GAIN_EPSILON {
    findings.push(MotionFinding {
      kind: "ball-wobble-energy",
      detail: format!(
        "ball kinetic energy increased unexpectedly: speed {:.2} -> {:.2} m/s, acc={:.2} m/s^2",
        previous_speed, current_speed, ball_acc,
      ),
    });
  }

  if auditable_free_ball
    && no_robot_should_affect_ball(current, config)
    && no_robot_should_affect_ball(previous, config)
  {
    let previous_planar_speed = speed2(previous.ball.vx, previous.ball.vy);
    let friction_decel = config.ball.friction * config.physics.gravity * dt;
    let predicted_planar_speed = (previous_planar_speed - friction_decel).max(0.0);
    let (predicted_vx, predicted_vy) = if previous_planar_speed <= f64::EPSILON {
      (0.0, 0.0)
    } else {
      let scale = predicted_planar_speed / previous_planar_speed;
      (previous.ball.vx * scale, previous.ball.vy * scale)
    };
    let predicted_vz = 0.0;
    let unexplained_delta = speed3(
      current.ball.vx - predicted_vx,
      current.ball.vy - predicted_vy,
      current.ball.vz - predicted_vz,
    );
    if unexplained_delta > BALL_FORCE_EPSILON {
      findings.push(MotionFinding {
                kind: "ball-random-force",
                detail: format!(
                    "ball velocity changed by {:.2} m/s without robot interaction: prev=({:.2},{:.2},{:.2}) now=({:.2},{:.2},{:.2})",
                    unexplained_delta,
                    previous.ball.vx,
                    previous.ball.vy,
                    previous.ball.vz,
                    current.ball.vx,
                    current.ball.vy,
                    current.ball.vz,
                ),
            });
    }
  }

  let dx = current.ball.x - previous.ball.x;
  let dy = current.ball.y - previous.ball.y;
  let dz = current.ball.z - previous.ball.z;
  let displacement = (dx * dx + dy * dy + dz * dz).sqrt();
  let step_speed = displacement / dt;
  if displacement > 0.03 && step_speed > current_speed + 1.0 {
    findings.push(MotionFinding {
      kind: "ball-teleport",
      detail: format!(
        "ball moved {:.3} m in one frame (step {:.2} m/s, reported {:.2} m/s)",
        displacement, step_speed, current_speed,
      ),
    });
  }

  if free_ball
    && (near_vertical_wall(current, config) || near_horizontal_wall(current, config))
    && current.ball.vx.abs() > 1.0
    && previous.ball.vx.abs() > 1.0
    && current.ball.vx.signum() != previous.ball.vx.signum()
    && (current.ball.y - previous.ball.y).abs() < 0.02
  {
    findings.push(MotionFinding {
      kind: "ball-wall-skitter",
      detail: format!(
        "ball reversed tangent velocity at boundary from {:.2} to {:.2} m/s",
        previous.ball.vx, current.ball.vx,
      ),
    });
  }
}

fn audit_robot_continuity(
  current: &WorldState,
  previous: &WorldState,
  config: &WorldConfig,
  findings: &mut Vec<MotionFinding>,
) {
  let dt = current.sim_time - previous.sim_time;
  if dt <= f64::EPSILON {
    return;
  }

  for (current_robot, previous_robot) in current
    .blue_robots
    .iter()
    .zip(previous.blue_robots.iter())
    .chain(
      current
        .yellow_robots
        .iter()
        .zip(previous.yellow_robots.iter()),
    )
  {
    if current_robot.id != previous_robot.id || current_robot.team != previous_robot.team {
      continue;
    }
    let step_dx = current_robot.x - previous_robot.x;
    let step_dy = current_robot.y - previous_robot.y;
    let displacement = (step_dx * step_dx + step_dy * step_dy).sqrt();
    let measured_speed = displacement / dt;
    let reported_speed = speed2(current_robot.vx, current_robot.vy);
    let cfg = match current_robot.team {
      TeamColor::Blue => &config.blue_robots,
      TeamColor::Yellow => &config.yellow_robots,
    };
    let current_acc = speed2(
      (current_robot.vx - previous_robot.vx) / dt,
      (current_robot.vy - previous_robot.vy) / dt,
    );
    let previous_speed = speed2(previous_robot.vx, previous_robot.vy);
    let current_speed = speed2(current_robot.vx, current_robot.vy);
    if displacement > POSITION_EPSILON && measured_speed > reported_speed + 1.0 {
      findings.push(MotionFinding {
        kind: "robot-step-mismatch",
        detail: format!(
          "{}{} moved {:.3} m in one frame (step {:.2} m/s, reported {:.2} m/s)",
          team_label(current_robot.team),
          current_robot.id,
          displacement,
          measured_speed,
          reported_speed,
        ),
      });
    }
    if measured_speed > cfg.vel_absolute_max + 0.5 {
      findings.push(MotionFinding {
        kind: "robot-teleport",
        detail: format!(
          "{}{} frame displacement implies {:.2} m/s",
          team_label(current_robot.team),
          current_robot.id,
          measured_speed,
        ),
      });
    }
    let _ = (current_acc, previous_speed, current_speed, displacement);
  }
}

fn ball_acceleration(current: &WorldState, previous: Option<&WorldState>) -> f64 {
  let Some(previous) = previous else {
    return 0.0;
  };
  let dt = current.sim_time - previous.sim_time;
  if dt <= f64::EPSILON {
    return 0.0;
  }
  speed3(
    (current.ball.vx - previous.ball.vx) / dt,
    (current.ball.vy - previous.ball.vy) / dt,
    (current.ball.vz - previous.ball.vz) / dt,
  )
}

fn nearest_robot_distance(state: &WorldState) -> f64 {
  state
    .blue_robots
    .iter()
    .chain(state.yellow_robots.iter())
    .map(|robot| {
      let dx = state.ball.x - robot.x;
      let dy = state.ball.y - robot.y;
      (dx * dx + dy * dy).sqrt()
    })
    .fold(f64::INFINITY, f64::min)
}

fn no_robot_should_affect_ball(state: &WorldState, config: &WorldConfig) -> bool {
  let interaction_distance = config.blue_robots.radius + config.ball.radius + 0.08;
  state
    .blue_robots
    .iter()
    .chain(state.yellow_robots.iter())
    .all(|robot| {
      let dx = state.ball.x - robot.x;
      let dy = state.ball.y - robot.y;
      let distance = (dx * dx + dy * dy).sqrt();
      distance > interaction_distance
        && !robot.infrared
        && robot.kick_status == crate::state::KickStatus::NoKick
    })
}

#[cfg(test)]
mod tests {
  use super::audit_world;
  use crate::config::WorldConfig;
  use crate::state::{BallState, KickStatus, RobotState, TeamColor, WorldState};

  fn robot(id: usize, team: TeamColor, x: f64, y: f64) -> RobotState {
    RobotState {
      id,
      team,
      x,
      y,
      z: 0.1,
      orientation: 0.0,
      vx: 0.0,
      vy: 0.0,
      vz: 0.0,
      v_angular: 0.0,
      infrared: false,
      dribbler_on: false,
      kick_status: KickStatus::NoKick,
      is_on: true,
      wheel_speeds: [0.0; 4],
    }
  }

  fn state(
    sim_time: f64,
    ball: BallState,
    blue_robots: Vec<RobotState>,
    yellow_robots: Vec<RobotState>,
  ) -> WorldState {
    WorldState {
      world_id: 0,
      sim_time,
      frame: (sim_time * 60.0) as u64,
      ball,
      blue_robots,
      yellow_robots,
      goal_blue: false,
      goal_yellow: false,
    }
  }

  #[test]
  fn detects_random_force_on_free_ball_even_if_far_robot_dribbles() {
    let config = WorldConfig::division_b();
    let previous = state(
      0.0,
      BallState {
        x: 0.0,
        y: 0.0,
        z: config.ball.radius,
        vx: 0.0,
        vy: 0.0,
        vz: 0.0,
      },
      {
        let mut bot = robot(0, TeamColor::Blue, 3.0, 3.0);
        bot.dribbler_on = true;
        vec![bot]
      },
      vec![],
    );
    let current = state(
      1.0 / 60.0,
      BallState {
        x: 0.05,
        y: 0.0,
        z: config.ball.radius,
        vx: 3.0,
        vy: 0.0,
        vz: 0.0,
      },
      {
        let mut bot = robot(0, TeamColor::Blue, 3.0, 3.0);
        bot.dribbler_on = true;
        vec![bot]
      },
      vec![],
    );

    let findings = audit_world(&current, Some(&previous), &config);
    assert!(
      findings
        .iter()
        .any(|finding| finding.kind == "ball-random-force"),
      "expected ball-random-force finding, got {findings:?}"
    );
  }

  #[test]
  fn does_not_flag_random_force_when_robot_is_close_enough_to_interact() {
    let config = WorldConfig::division_b();
    let previous = state(
      0.0,
      BallState {
        x: 0.0,
        y: 0.0,
        z: config.ball.radius,
        vx: 0.0,
        vy: 0.0,
        vz: 0.0,
      },
      vec![robot(0, TeamColor::Blue, 0.05, 0.0)],
      vec![],
    );
    let current = state(
      1.0 / 60.0,
      BallState {
        x: 0.05,
        y: 0.0,
        z: config.ball.radius,
        vx: 3.0,
        vy: 0.0,
        vz: 0.0,
      },
      vec![robot(0, TeamColor::Blue, 0.05, 0.0)],
      vec![],
    );

    let findings = audit_world(&current, Some(&previous), &config);
    assert!(
      findings
        .iter()
        .all(|finding| finding.kind != "ball-random-force"),
      "unexpected ball-random-force finding: {findings:?}"
    );
  }
}

fn speed2(x: f64, y: f64) -> f64 {
  (x * x + y * y).sqrt()
}

fn speed3(x: f64, y: f64, z: f64) -> f64 {
  (x * x + y * y + z * z).sqrt()
}

fn team_label(team: TeamColor) -> &'static str {
  match team {
    TeamColor::Blue => "B",
    TeamColor::Yellow => "Y",
  }
}

pub fn robot_motion_summary(robot: &RobotState) -> String {
  format!(
    "{}{} v={:.2} av={:.2} pos=({:.3},{:.3})",
    team_label(robot.team),
    robot.id,
    speed2(robot.vx, robot.vy),
    robot.v_angular,
    robot.x,
    robot.y,
  )
}

fn push_history(history: &mut VecDeque<MotionSample>, sample: MotionSample) {
  if history.len() == HISTORY_LEN {
    history.pop_front();
  }
  history.push_back(sample);
}

fn backfill_acceleration(history: &mut VecDeque<MotionSample>) {
  let len = history.len();
  if len < 2 {
    return;
  }

  let last = history[len - 1];
  let prev = history[len - 2];
  let dt = last.t - prev.t;
  if dt <= f64::EPSILON {
    return;
  }

  let acc = speed2((last.vx - prev.vx) / dt, (last.vy - prev.vy) / dt);
  if let Some(entry) = history.back_mut() {
    entry.acc = acc;
  }
}

fn detect_line_oscillation(history: &VecDeque<MotionSample>) -> Option<&'static str> {
  if history.len() < 4 {
    return None;
  }

  let samples = history.iter().rev().take(4).copied().collect::<Vec<_>>();
  let a = samples[3];
  let b = samples[2];
  let c = samples[1];
  let d = samples[0];

  if distance2(a, c) <= TWO_POINT_MATCH_DISTANCE
    && distance2(b, d) <= TWO_POINT_MATCH_DISTANCE
    && distance2(a, b) >= TWO_POINT_SEPARATION
  {
    return Some("is oscillating between two points");
  }

  let x_range = axis_range([a.x, b.x, c.x, d.x]);
  let y_range = axis_range([a.y, b.y, c.y, d.y]);
  let primary = if x_range >= y_range {
    [b.x - a.x, c.x - b.x, d.x - c.x]
  } else {
    [b.y - a.y, c.y - b.y, d.y - c.y]
  };
  let orthogonal_range = if x_range >= y_range { y_range } else { x_range };

  if orthogonal_range <= OSCILLATION_ORTHOGONAL_RANGE
    && primary
      .iter()
      .all(|delta| delta.abs() >= OSCILLATION_AXIS_DELTA)
    && primary[0].signum() != primary[1].signum()
    && primary[1].signum() != primary[2].signum()
  {
    return Some("is repeatedly reversing on a line");
  }

  None
}

fn detect_wobble(
  history: &VecDeque<MotionSample>,
  speed_limit: f64,
  acc_limit: f64,
  jerk_limit: f64,
) -> Option<String> {
  if history.len() < 4 {
    return None;
  }

  let samples = history.iter().rev().take(4).copied().collect::<Vec<_>>();
  let a = samples[3];
  let b = samples[2];
  let c = samples[1];
  let d = samples[0];

  let low_motion_zone = [a.speed, b.speed, c.speed, d.speed]
    .into_iter()
    .all(|speed| speed <= speed_limit);
  let acc_spike = [b.acc, c.acc, d.acc]
    .into_iter()
    .any(|acc| acc >= acc_limit);
  let jerk = compute_jerk([a, b, c, d]);
  let jerk_spike = jerk >= jerk_limit;
  let velocity_flip =
    sign_flip_count([a.vx, b.vx, c.vx, d.vx]) + sign_flip_count([a.vy, b.vy, c.vy, d.vy]) >= 2;
  let acceleration_flip = accel_sign_flip_count([b.vx - a.vx, c.vx - b.vx, d.vx - c.vx])
    + accel_sign_flip_count([b.vy - a.vy, c.vy - b.vy, d.vy - c.vy])
    >= 2;
  let confined =
    axis_range([a.x, b.x, c.x, d.x]) <= 0.12 && axis_range([a.y, b.y, c.y, d.y]) <= 0.12;

  if confined
    && low_motion_zone
    && (acc_spike || jerk_spike)
    && (velocity_flip || acceleration_flip)
  {
    return Some(format!(
      "settling wobble: speed=[{:.2},{:.2},{:.2},{:.2}] acc=[{:.2},{:.2},{:.2}] jerk={:.2}",
      a.speed, b.speed, c.speed, d.speed, b.acc, c.acc, d.acc, jerk,
    ));
  }

  None
}

fn accel_sign_flip_count(values: [f64; 3]) -> usize {
  values
    .windows(2)
    .filter(|pair| {
      let left = pair[0];
      let right = pair[1];
      left.abs() >= WOBBLE_SIGN_FLIP_MIN_ACCEL
        && right.abs() >= WOBBLE_SIGN_FLIP_MIN_ACCEL
        && left.signum() != right.signum()
    })
    .count()
}

fn compute_jerk(samples: [MotionSample; 4]) -> f64 {
  let mut max_jerk: f64 = 0.0;
  for pair in samples.windows(2) {
    let dt = pair[1].t - pair[0].t;
    if dt <= f64::EPSILON {
      continue;
    }
    let jerk = (pair[1].acc - pair[0].acc).abs() / dt;
    max_jerk = max_jerk.max(jerk);
  }
  max_jerk
}

fn sign_flip_count(values: [f64; 4]) -> usize {
  values
    .windows(2)
    .filter(|pair| {
      let left = pair[0];
      let right = pair[1];
      left.abs() >= WOBBLE_SIGN_FLIP_MIN_SPEED
        && right.abs() >= WOBBLE_SIGN_FLIP_MIN_SPEED
        && left.signum() != right.signum()
    })
    .count()
}

fn axis_range(values: [f64; 4]) -> f64 {
  let min = values.into_iter().fold(f64::INFINITY, f64::min);
  let max = values.into_iter().fold(f64::NEG_INFINITY, f64::max);
  max - min
}

fn distance2(a: MotionSample, b: MotionSample) -> f64 {
  let dx = a.x - b.x;
  let dy = a.y - b.y;
  (dx * dx + dy * dy).sqrt()
}

fn near_horizontal_wall(state: &WorldState, config: &WorldConfig) -> bool {
  let boundary =
    config.field.field_width * 0.5 + config.field.margin_touch_line - config.ball.radius;
  state.ball.y.abs() >= boundary - 0.03
}

fn near_vertical_wall(state: &WorldState, config: &WorldConfig) -> bool {
  let boundary =
    config.field.field_length * 0.5 + config.field.margin_goal_line - config.ball.radius;
  state.ball.x.abs() >= boundary - 0.03
}
