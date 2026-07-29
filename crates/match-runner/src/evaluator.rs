//! RL-style match evaluation. Accumulates per-team metrics every tick and
//! produces a composite score plus a human-readable breakdown — much like the
//! reward shaping you would write for an RL environment.

use serde::Serialize;
use simhark::{TeamColor, WorldConfig, WorldState};

#[derive(Debug, Clone, Default, Serialize)]
pub struct TeamMetrics {
  pub goals_for: u32,
  pub goals_against: u32,
  /// Ticks this team was the closest to the ball.
  pub possession_ticks: u64,
  /// Ticks the ball was in the opponent's half.
  pub attacking_ticks: u64,
  /// Metres of forward ball progress credited to this team.
  pub ball_progress: f64,
  /// Shots detected (ball accelerated toward the opponent goal).
  pub shots: u32,
  /// Shots whose line crossed the opponent goal mouth.
  pub shots_on_target: u32,
  /// Total distance travelled by this team's robots (activity).
  pub distance: f64,
  /// Accumulated time the ball spent very close to our own goal (danger).
  pub conceded_pressure: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct TeamReport {
  pub name: String,
  pub color: String,
  pub metrics: TeamMetrics,
  /// Composite RL-style reward.
  pub score: f64,
  pub possession_pct: f64,
  pub notes: Vec<String>,
}

#[derive(Debug, Clone, Serialize)]
pub struct MatchReport {
  pub blue: TeamReport,
  pub yellow: TeamReport,
  pub total_ticks: u64,
  pub sim_seconds: f64,
  pub winner: String,
}

pub struct Evaluator {
  cfg: WorldConfig,
  blue: TeamMetrics,
  yellow: TeamMetrics,
  blue_name: String,
  yellow_name: String,
  total_ticks: u64,
  // Shot cooldowns (sim time of last detected shot).
  last_shot_blue: f64,
  last_shot_yellow: f64,
}

impl Evaluator {
  pub fn new(cfg: WorldConfig, blue_name: String, yellow_name: String) -> Self {
    Self {
      cfg,
      blue: TeamMetrics::default(),
      yellow: TeamMetrics::default(),
      blue_name,
      yellow_name,
      total_ticks: 0,
      last_shot_blue: -10.0,
      last_shot_yellow: -10.0,
    }
  }

  pub fn record_goal(&mut self, scorer: TeamColor) {
    match scorer {
      TeamColor::Blue => {
        self.blue.goals_for += 1;
        self.yellow.goals_against += 1;
      }
      TeamColor::Yellow => {
        self.yellow.goals_for += 1;
        self.blue.goals_against += 1;
      }
    }
  }

  pub fn set_team_name(&mut self, color: TeamColor, name: String) {
    match color {
      TeamColor::Blue => self.blue_name = name,
      TeamColor::Yellow => self.yellow_name = name,
    }
  }

  pub fn tick(&mut self, state: &WorldState, prev: Option<&WorldState>) {
    self.total_ticks += 1;
    let half_l = self.cfg.field.field_length * 0.5;

    let bx = state.ball.x;
    let by = state.ball.y;

    // Possession: prefer the actual holder (infrared at the kicker, which is
    // what simhark uses for sticky possession); fall back to nearest robot
    // only when nobody holds the ball.
    let blue_holds = state.blue_robots.iter().any(|r| r.infrared);
    let yellow_holds = state.yellow_robots.iter().any(|r| r.infrared);
    let possessor = if blue_holds && !yellow_holds {
      Some(TeamColor::Blue)
    } else if yellow_holds && !blue_holds {
      Some(TeamColor::Yellow)
    } else if blue_holds && yellow_holds {
      None // contested
    } else {
      let blue_min = state
        .blue_robots
        .iter()
        .map(|r| ((r.x - bx).powi(2) + (r.y - by).powi(2)).sqrt())
        .fold(f64::INFINITY, f64::min);
      let yellow_min = state
        .yellow_robots
        .iter()
        .map(|r| ((r.x - bx).powi(2) + (r.y - by).powi(2)).sqrt())
        .fold(f64::INFINITY, f64::min);
      Some(if blue_min < yellow_min {
        TeamColor::Blue
      } else {
        TeamColor::Yellow
      })
    };
    match possessor {
      Some(TeamColor::Blue) => self.blue.possession_ticks += 1,
      Some(TeamColor::Yellow) => self.yellow.possession_ticks += 1,
      None => {}
    }
    let possessor = possessor.unwrap_or(TeamColor::Blue);

    // Attacking territory (blue attacks +x, yellow attacks -x).
    if bx > 0.0 {
      self.blue.attacking_ticks += 1;
    } else if bx < 0.0 {
      self.yellow.attacking_ticks += 1;
    }

    // Danger near own goal.
    if bx > half_l - 1.5 {
      self.yellow.conceded_pressure += 1.0; // ball near yellow goal (+x)
    } else if bx < -(half_l - 1.5) {
      self.blue.conceded_pressure += 1.0;
    }

    if let Some(prev) = prev {
      let dx = bx - prev.ball.x;
      // Credit forward progress to the possessing team.
      match possessor {
        TeamColor::Blue if dx > 0.0 => self.blue.ball_progress += dx,
        TeamColor::Yellow if dx < 0.0 => self.yellow.ball_progress += -dx,
        _ => {}
      }

      // Shot detection: ball speed jumped this tick and is heading at a goal.
      let dt = (state.sim_time - prev.sim_time).max(1e-3);
      let speed = (state.ball.vx.powi(2) + state.ball.vy.powi(2)).sqrt();
      let prev_speed = (prev.ball.vx.powi(2) + prev.ball.vy.powi(2)).sqrt();
      if speed > 2.5 && speed > prev_speed + 1.0 {
        // Toward +x goal => blue shot; toward -x => yellow shot.
        if state.ball.vx > 1.0 && state.sim_time - self.last_shot_blue > 0.4 {
          self.last_shot_blue = state.sim_time;
          self.blue.shots += 1;
          if self.shot_on_target(state, half_l) {
            self.blue.shots_on_target += 1;
          }
        } else if state.ball.vx < -1.0 && state.sim_time - self.last_shot_yellow > 0.4 {
          self.last_shot_yellow = state.sim_time;
          self.yellow.shots += 1;
          if self.shot_on_target(state, -half_l) {
            self.yellow.shots_on_target += 1;
          }
        }
      }
      let _ = dt;

      // Robot activity.
      self.blue.distance += team_distance(&state.blue_robots, &prev.blue_robots);
      self.yellow.distance += team_distance(&state.yellow_robots, &prev.yellow_robots);
    }
  }

  /// Would the ball, on its current heading, cross the goal line within the mouth?
  fn shot_on_target(&self, state: &WorldState, goal_x: f64) -> bool {
    let half_goal = self.cfg.field.goal_width * 0.5;
    if state.ball.vx.abs() < 1e-3 {
      return false;
    }
    let t = (goal_x - state.ball.x) / state.ball.vx;
    if t < 0.0 {
      return false;
    }
    let y_at_goal = state.ball.y + state.ball.vy * t;
    y_at_goal.abs() < half_goal + 0.1
  }

  pub fn finish(&self, sim_seconds: f64) -> MatchReport {
    let total = self.total_ticks.max(1) as f64;
    let blue = self.report(TeamColor::Blue, total);
    let yellow = self.report(TeamColor::Yellow, total);
    let winner = if blue.metrics.goals_for > yellow.metrics.goals_for {
      self.blue_name.clone()
    } else if yellow.metrics.goals_for > blue.metrics.goals_for {
      self.yellow_name.clone()
    } else if blue.score > yellow.score {
      format!("{} (on points)", self.blue_name)
    } else if yellow.score > blue.score {
      format!("{} (on points)", self.yellow_name)
    } else {
      "draw".to_string()
    };
    MatchReport {
      blue,
      yellow,
      total_ticks: self.total_ticks,
      sim_seconds,
      winner,
    }
  }

  fn report(&self, color: TeamColor, total: f64) -> TeamReport {
    let (m, name, other) = match color {
      TeamColor::Blue => (&self.blue, &self.blue_name, &self.yellow),
      TeamColor::Yellow => (&self.yellow, &self.yellow_name, &self.blue),
    };
    let _ = other;
    let possession_pct = 100.0 * m.possession_ticks as f64 / total;

    // RL-style composite reward.
    let score = 10.0 * m.goals_for as f64 - 8.0 * m.goals_against as f64
      + 1.0 * m.shots_on_target as f64
      + 0.2 * m.shots as f64
      + 0.04 * possession_pct
      + 0.03 * (m.attacking_ticks as f64 / total * 100.0)
      + 0.5 * m.ball_progress
      - 0.002 * m.conceded_pressure;

    let mut notes = Vec::new();
    if m.goals_for > m.goals_against {
      notes.push("outscored opponent".into());
    }
    if possession_pct > 55.0 {
      notes.push(format!("dominated possession ({possession_pct:.0}%)"));
    } else if possession_pct < 45.0 {
      notes.push(format!("lost the possession battle ({possession_pct:.0}%)"));
    }
    if m.shots_on_target == 0 {
      notes.push("created no shots on target".into());
    } else if m.shots > 0 {
      notes.push(format!(
        "{} shots, {} on target",
        m.shots, m.shots_on_target
      ));
    }
    if m.goals_against > 0 && m.shots_on_target as f64 / (m.goals_for.max(1) as f64) > 6.0 {
      notes.push("wasteful finishing".into());
    }

    TeamReport {
      name: name.clone(),
      color: format!("{color:?}"),
      metrics: m.clone(),
      score,
      possession_pct,
      notes,
    }
  }
}

fn team_distance(now: &[simhark::RobotState], prev: &[simhark::RobotState]) -> f64 {
  let mut d = 0.0;
  for r in now {
    if let Some(p) = prev.iter().find(|x| x.id == r.id) {
      d += ((r.x - p.x).powi(2) + (r.y - p.y).powi(2)).sqrt();
    }
  }
  d
}
