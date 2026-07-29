//! A lightweight match director: it owns the score, drives kickoffs, detects
//! goals, recovers stuck balls and decides the per-team [`GameCommand`].
//!
//! It is intentionally simpler than a full SSL game-controller so that self-play
//! matches flow continuously and produce a clean training signal.

use crate::controller::GameCommand;
use simhark::{TeamColor, TeleportBall, TeleportRobot, WorldCommand, WorldConfig, WorldState};

#[derive(Debug, Clone, Copy, PartialEq)]
enum Phase {
  /// Positioning before a kickoff by `kick`.
  Prepare {
    kick: TeamColor,
    until: f64,
  },
  Running,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Score {
  pub blue: u32,
  pub yellow: u32,
}

pub struct MatchDirector {
  cfg: WorldConfig,
  phase: Phase,
  pub score: Score,
  /// Rising-edge debounce for goal detection.
  goal_latched: bool,
  /// Frames the ball has been (almost) stationary and unreachable.
  stuck_frames: u64,
  /// Anchor position + frame count for position-based idle detection: a ball
  /// that barely moves for a sustained time (e.g. pinned in a corner by a
  /// robot, so the unreachable check below never fires) is recovered too.
  idle_anchor: (f64, f64),
  idle_frames: u64,
  /// Total goals/events for logging.
  pub last_event: Option<String>,
  /// The scorer of a goal detected this tick (consumed via `take_goal`).
  last_goal: Option<TeamColor>,
  duration: f64,
  teleport_ball_on_no_progress: bool,
  prepare_time: f64,
  blue_bots: usize,
  yellow_bots: usize,
}

impl MatchDirector {
  pub fn new(cfg: WorldConfig, duration_secs: f64) -> Self {
    let robots_per_team = cfg.robots_per_team;
    Self {
      cfg,
      phase: Phase::Prepare {
        kick: TeamColor::Blue,
        until: 1.0,
      },
      score: Score::default(),
      goal_latched: false,
      stuck_frames: 0,
      idle_anchor: (0.0, 0.0),
      idle_frames: 0,
      last_event: None,
      last_goal: None,
      duration: duration_secs,
      teleport_ball_on_no_progress: true,
      prepare_time: 0.8,
      blue_bots: robots_per_team,
      yellow_bots: robots_per_team,
    }
  }

  pub fn with_bot_counts(mut self, blue_bots: usize, yellow_bots: usize) -> Self {
    self.blue_bots = blue_bots.min(self.cfg.robots_per_team);
    self.yellow_bots = yellow_bots.min(self.cfg.robots_per_team);
    self
  }

  pub fn is_over(&self, state: &WorldState) -> bool {
    state.sim_time >= self.duration
  }

  pub fn teleport_ball_on_no_progress(&self) -> bool {
    self.teleport_ball_on_no_progress
  }

  pub fn set_teleport_ball_on_no_progress(&mut self, enabled: bool) {
    self.teleport_ball_on_no_progress = enabled;
    if !enabled {
      self.stuck_frames = 0;
      self.idle_frames = 0;
    }
  }

  /// The game command for a given team color this tick.
  pub fn command_for(&self, color: TeamColor) -> GameCommand {
    match self.phase {
      Phase::Running => GameCommand::Running,
      Phase::Prepare { kick, .. } => {
        if kick == color {
          GameCommand::PrepareKickoffUs
        } else {
          GameCommand::PrepareKickoffThem
        }
      }
    }
  }

  /// Advance the referee logic. Returns teleports/resets to apply this tick
  /// (ball + robots) and clears `last_event` after reading.
  /// Referee command code (SSL proto `Referee.Command`) for the current phase.
  pub fn referee_command_code(&self) -> i32 {
    match self.phase {
      Phase::Running => 3, // FORCE_START
      Phase::Prepare { kick, .. } => match kick {
        TeamColor::Yellow => 4, // PREPARE_KICKOFF_YELLOW
        TeamColor::Blue => 5,   // PREPARE_KICKOFF_BLUE
      },
    }
  }

  /// Consume the goal scored this tick, if any.
  pub fn take_goal(&mut self) -> Option<TeamColor> {
    self.last_goal.take()
  }

  pub fn update(&mut self, state: &WorldState) -> WorldCommand {
    self.last_event = None;
    let mut cmd = WorldCommand::default();

    // Phase transitions.
    if let Phase::Prepare { until, .. } = self.phase {
      if state.sim_time >= until {
        self.phase = Phase::Running;
      }
    }

    // Goal detection (rising edge).
    let goal = state.goal_blue || state.goal_yellow;
    if goal && !self.goal_latched {
      self.goal_latched = true;
      let scorer = if state.goal_blue {
        self.score.blue += 1;
        TeamColor::Blue
      } else {
        self.score.yellow += 1;
        TeamColor::Yellow
      };
      // The conceding team kicks off.
      let kick = match scorer {
        TeamColor::Blue => TeamColor::Yellow,
        TeamColor::Yellow => TeamColor::Blue,
      };
      self.last_event = Some(format!(
        "GOAL {:?} ({}-{})",
        scorer, self.score.blue, self.score.yellow
      ));
      self.last_goal = Some(scorer);
      self.reset_for_kickoff(state.sim_time, self.active_kickoff_team(kick), &mut cmd);
    } else if !goal {
      self.goal_latched = false;
    }

    // Stuck-ball recovery during running play. Two independent triggers:
    //  (a) unreachable & slow: no robot can play it (`ball_stuck`), or
    //  (b) idle: the ball barely moves for a sustained time — this catches a
    //      ball pinned in a corner or against the field edge by a robot, so
    //      (a) never fires because a robot is right on it. Without (b) a
    //      single corner-wedge stalls the whole match (observed: ball frozen
    //      out of bounds for the entire remainder of play).
    if matches!(self.phase, Phase::Running) && self.teleport_ball_on_no_progress {
      // (a) unreachable
      if self.ball_stuck(state) {
        self.stuck_frames += 1;
      } else {
        self.stuck_frames = 0;
      }

      // (b) position-idle: ball within `idle_radius` of its anchor.
      let (bx, by) = (state.ball.x, state.ball.y);
      let drift = ((bx - self.idle_anchor.0).powi(2) + (by - self.idle_anchor.1).powi(2)).sqrt();
      const IDLE_RADIUS: f64 = 0.1; // m — meaningful play moves the ball more
      if drift < IDLE_RADIUS {
        self.idle_frames += 1;
      } else {
        self.idle_anchor = (bx, by);
        self.idle_frames = 0;
      }

      // ~1.5 s unreachable, or ~4 s parked in essentially one spot.
      if self.stuck_frames > 90 || self.idle_frames > 250 {
        self.last_event = Some("ball-recovery".to_string());
        cmd.teleport_ball = Some(TeleportBall {
          x: Some(0.0),
          y: Some(0.0),
          z: Some(0.0),
          vx: Some(0.0),
          vy: Some(0.0),
          vz: Some(0.0),
        });
        self.stuck_frames = 0;
        self.idle_frames = 0;
        self.idle_anchor = (0.0, 0.0);
      }
    }

    cmd
  }

  fn reset_for_kickoff(&mut self, now: f64, kick: TeamColor, cmd: &mut WorldCommand) {
    self.phase = Phase::Prepare {
      kick,
      until: now + self.prepare_time,
    };
    self.goal_latched = true; // stay latched until ball leaves goal/reset
    self.stuck_frames = 0;
    self.idle_frames = 0;
    self.idle_anchor = (0.0, 0.0);
    cmd.teleport_ball = Some(TeleportBall {
      x: Some(0.0),
      y: Some(0.0),
      z: Some(0.0),
      vx: Some(0.0),
      vy: Some(0.0),
      vz: Some(0.0),
    });
    cmd.teleport_robots = formation(&self.cfg, TeamColor::Blue, self.blue_bots)
      .into_iter()
      .chain(formation(&self.cfg, TeamColor::Yellow, self.yellow_bots))
      .chain(disabled_robots(&self.cfg, TeamColor::Blue, self.blue_bots))
      .chain(disabled_robots(
        &self.cfg,
        TeamColor::Yellow,
        self.yellow_bots,
      ))
      .collect();
  }

  /// Place everyone for the opening kickoff.
  pub fn kickoff_reset(&mut self) -> WorldCommand {
    let mut cmd = WorldCommand::default();
    self.reset_for_kickoff(0.0, self.active_kickoff_team(TeamColor::Blue), &mut cmd);
    cmd
  }

  fn active_kickoff_team(&self, preferred: TeamColor) -> TeamColor {
    let preferred_bots = match preferred {
      TeamColor::Blue => self.blue_bots,
      TeamColor::Yellow => self.yellow_bots,
    };
    if preferred_bots > 0 {
      return preferred;
    }

    match preferred {
      TeamColor::Blue if self.yellow_bots > 0 => TeamColor::Yellow,
      TeamColor::Yellow if self.blue_bots > 0 => TeamColor::Blue,
      _ => preferred,
    }
  }

  fn ball_stuck(&self, state: &WorldState) -> bool {
    let speed = (state.ball.vx.powi(2) + state.ball.vy.powi(2) + state.ball.vz.powi(2)).sqrt();
    if speed > 0.3 {
      return false;
    }
    // Is any robot close enough to play it?
    let reach = self.cfg.blue_robots.radius + self.cfg.ball.radius + 0.12;
    let bx = state.ball.x;
    let by = state.ball.y;
    let nearest = state
      .blue_robots
      .iter()
      .chain(state.yellow_robots.iter())
      .map(|r| ((r.x - bx).powi(2) + (r.y - by).powi(2)).sqrt())
      .fold(f64::INFINITY, f64::min);
    nearest > reach + 0.4
  }
}

/// Deterministic [-1, 1] jitter from `(seed, id, axis)`. A cheap integer hash
/// (splitmix-ish) so each seed yields a different but reproducible formation.
/// Depends only on `(seed, id, axis)` — never on team color — so the two
/// color-swapped matches a mirrored bench runs on the same seed stay exact
/// reflections of each other and the blue-side advantage still cancels.
fn jitter(seed: u64, id: usize, axis: u64) -> f64 {
  let mut z = seed
    .wrapping_mul(0x9E3779B97F4A7C15)
    .wrapping_add((id as u64).wrapping_mul(0xBF58476D1CE4E5B9))
    .wrapping_add(axis.wrapping_mul(0x94D049BB133111EB));
  z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
  z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
  z ^= z >> 31;
  // Map to [-1, 1).
  (z as f64 / u64::MAX as f64) * 2.0 - 1.0
}

#[cfg(test)]
mod tests {
  use super::*;

  fn idle_state() -> WorldState {
    WorldState {
      world_id: 0,
      sim_time: 2.0,
      frame: 120,
      ball: simhark::BallState {
        x: 0.5,
        y: 0.0,
        z: 0.0,
        vx: 0.0,
        vy: 0.0,
        vz: 0.0,
      },
      blue_robots: Vec::new(),
      yellow_robots: Vec::new(),
      goal_blue: false,
      goal_yellow: false,
    }
  }

  #[test]
  fn no_progress_recovery_can_be_toggled_at_runtime() {
    let mut director = MatchDirector::new(WorldConfig::division_b(), 60.0);
    director.phase = Phase::Running;
    director.set_teleport_ball_on_no_progress(false);

    for _ in 0..300 {
      assert!(director.update(&idle_state()).teleport_ball.is_none());
    }

    director.set_teleport_ball_on_no_progress(true);
    assert!(
      (0..100).any(|_| director.update(&idle_state()).teleport_ball.is_some()),
      "re-enabling recovery should restore the existing stuck-ball behaviour"
    );
  }
}

/// A simple defensive formation for `color` on its own half. Positions are
/// jittered by `cfg.seed` so the (otherwise fully deterministic) sim explores
/// different opening states across seeds — making the mirrored bench a real
/// multi-sample A/B instead of one scenario counted N times.
fn formation(cfg: &WorldConfig, color: TeamColor, n: usize) -> Vec<TeleportRobot> {
  let l = cfg.field.field_length * 0.5;
  let w = cfg.field.field_width * 0.5;
  let seed = cfg.seed;
  // attack_dir: blue attacks +x, yellow attacks -x. Own goal at -attack_dir*l.
  let attack = match color {
    TeamColor::Blue => 1.0,
    TeamColor::Yellow => -1.0,
  };
  let own_goal_x = -attack * l;
  let face = if attack > 0.0 {
    0.0
  } else {
    std::f64::consts::PI
  };

  let mut robots = Vec::with_capacity(n);
  for id in 0..n {
    let (x, y) = if id == 0 {
      // Keeper just in front of own goal (only a small lateral jitter so it
      // stays in the mouth).
      (own_goal_x + attack * 0.15, jitter(seed, id, 1) * w * 0.1)
    } else {
      // Spread the rest across the own half at staggered depths, jittered.
      let frac = id as f64 / n as f64;
      let depth = (0.25 + 0.55 * frac + jitter(seed, id, 0) * 0.12).clamp(0.12, 0.92);
      let x = own_goal_x + attack * (l * depth);
      let lane =
        (((id as f64 * 2.7).sin()) * 0.6 + jitter(seed, id, 1) * 0.35).clamp(-0.9, 0.9) * w;
      (x, lane)
    };
    robots.push(TeleportRobot {
      id,
      team: color,
      x: Some(x),
      y: Some(y),
      orientation: Some(face),
      vx: Some(0.0),
      vy: Some(0.0),
      v_angular: Some(0.0),
      present: Some(true),
    });
  }
  robots
}

fn disabled_robots(
  cfg: &WorldConfig,
  color: TeamColor,
  first_disabled: usize,
) -> impl Iterator<Item = TeleportRobot> + '_ {
  (first_disabled..cfg.robots_per_team).map(move |id| TeleportRobot {
    id,
    team: color,
    x: None,
    y: None,
    orientation: None,
    vx: Some(0.0),
    vy: Some(0.0),
    v_angular: Some(0.0),
    present: Some(false),
  })
}
