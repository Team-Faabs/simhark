//! Team controllers. Legacy AIs run through the full CrashPilot/faabs stack.
//! Dehumanized can also run directly against simhark while the 2027
//! CrashPilot/robot protocol is still under development.

#[cfg(feature = "faabs")]
use core_dump::proto::Referee;
#[cfg(feature = "faabs")]
use core_dump::types::Ai;
#[cfg(feature = "faabs")]
use simhark::WorldCommand;
use simhark::{MoveCommand, RobotCommand, TeamColor, WorldConfig, WorldState};
#[cfg(feature = "faabs")]
use simhark_faabs::Faabs;
#[cfg(feature = "faabs")]
use simhark_faabs::crashpilot::{FieldSide, GameStartOptions, GameTeam};
#[cfg(feature = "faabs")]
use simhark_faabs::synth::{force_start_referee, referee_command};

#[cfg(feature = "dehumanized")]
use crate::direct_dehumanized::DirectDehumanizedController;

/// Referee state resolved relative to a team, as decided by the match director.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GameCommand {
  Halt,
  Stop,
  Running,
  FreeKickUs,
  FreeKickThem,
  PrepareKickoffUs,
  PrepareKickoffThem,
}

pub trait Controller {
  fn name(&self) -> &str;
  #[cfg(feature = "viewer")]
  fn developer_schema(&self) -> Option<serde_json::Value> {
    None
  }
  #[cfg(feature = "viewer")]
  fn apply_developer_request(
    &mut self,
    _request: &simhark::viewer::DeveloperRequest,
  ) -> Result<String, String> {
    Err("this controller does not expose developer actions".to_string())
  }
  #[cfg(feature = "viewer-debug")]
  fn debug_snapshot(&self) -> Option<simhark::viewer::ViewerDebugSnapshot> {
    None
  }
  fn act(
    &mut self,
    state: &WorldState,
    cfg: &WorldConfig,
    color: TeamColor,
    gc: GameCommand,
  ) -> Vec<RobotCommand>;
}

#[cfg(feature = "faabs")]
fn referee_for(gc: GameCommand) -> Option<Referee> {
  match gc {
    GameCommand::Halt => Some(referee_command(0)), // HALT
    // Everything else: keep CrashPilot in the Running phase so it plays. The
    // match director handles kickoff positioning via teleports.
    _ => Some(force_start_referee()),
  }
}

/// Wraps a `Faabs<A>` (a CrashPilot AI bound into simhark) as a `Controller`.
#[cfg(feature = "faabs")]
pub struct FaabsController<A: Ai> {
  faabs: Faabs<A>,
  name: String,
}

#[cfg(feature = "faabs")]
fn start_crash_pilot<A: Ai>(faabs: &mut Faabs<A>, color: TeamColor) {
  // Preserve simhark_faabs' existing side convention:
  // yellow uses x+, blue uses x-.
  let options = match color {
    TeamColor::Yellow => GameStartOptions::new(GameTeam::Yellow, FieldSide::PositiveX),
    TeamColor::Blue => GameStartOptions::new(GameTeam::Blue, FieldSide::NegativeX),
  };
  faabs.crash_pilot.start_game(options);
}

#[cfg(feature = "faabs")]
impl<A: Ai + Send> Controller for FaabsController<A> {
  fn name(&self) -> &str {
    &self.name
  }

  #[cfg(feature = "viewer-debug")]
  fn debug_snapshot(&self) -> Option<simhark::viewer::ViewerDebugSnapshot> {
    self.faabs.debug_snapshot()
  }

  fn act(
    &mut self,
    state: &WorldState,
    _cfg: &WorldConfig,
    color: TeamColor,
    gc: GameCommand,
  ) -> Vec<RobotCommand> {
    let mut scratch = WorldCommand::default();
    self.faabs.step(state, &mut scratch, referee_for(gc));
    match color {
      TeamColor::Blue => scratch.blue,
      TeamColor::Yellow => scratch.yellow,
    }
  }
}

/// Controller that keeps every robot in a zero-drive idle state.
pub struct DummyController {
  num_robots: u8,
}

impl Controller for DummyController {
  fn name(&self) -> &str {
    "dummy"
  }

  fn act(
    &mut self,
    _state: &WorldState,
    _cfg: &WorldConfig,
    _color: TeamColor,
    _gc: GameCommand,
  ) -> Vec<RobotCommand> {
    (0..self.num_robots as usize)
      .map(|id| RobotCommand {
        id,
        move_command: Some(MoveCommand::WheelVelocity([0.0; 4])),
        kick_speed: 0.0,
        kick_angle: 0.0,
        dribbler_on: false,
      })
      .collect()
  }
}

/// Identifies which AI a side should use.
#[derive(Debug, Clone)]
pub enum TeamKind {
  /// Bangka — the current non-ML role/skill AI, run inside CrashPilot.
  Bangka,
  /// Bongka — the tuned/legacy Bangka-line AI, run inside CrashPilot.
  Bongka { params: Option<String> },
  /// Ungabunga — a sibling Bangka-line AI, run inside CrashPilot.
  Ungabunga { params: Option<String> },
  /// Frozen snapshot of Bangka at Pass 5 (goal-shadow wall + far-post striker),
  /// used as a fixed sparring partner for deterministic benchmarking.
  Bangka1,
  /// Frozen snapshot of the original Bangka, used as a fixed sparring partner
  /// for deterministic benchmarking of new Bangka versions.
  BangkaLegacy,
  /// CrashPilot's machine-learning AI.
  CrashPilot { model: Option<String> },
  /// Dehumanized connected directly to simhark, without CrashPilot or ORCA.
  Dehumanized,
  /// No-op side: keeps robots idle with zero wheel velocity.
  Dummy,
  /// The real Sumatra (external Java AI), driven over the SimNet protocol.
  /// This side is *not* a faabs controller; `run_match` handles it specially.
  Sumatra,
}

impl TeamKind {
  pub fn parse(s: &str) -> Result<Self, String> {
    let (name, arg) = match s.split_once(':') {
      Some((n, a)) => (n, Some(a.to_string())),
      None => (s, None),
    };
    match name.to_ascii_lowercase().as_str() {
      "bangka" | "us" | "new" => Ok(TeamKind::Bangka),
      "bongka" => Ok(TeamKind::Bongka { params: arg }),
      "ungabunga" => Ok(TeamKind::Ungabunga { params: arg }),
      "bangka1" => Ok(TeamKind::Bangka1),
      "legacy" | "bangka0" | "baseline" => Ok(TeamKind::BangkaLegacy),
      "crashpilot" | "cp" | "ml" | "ai" => Ok(TeamKind::CrashPilot { model: arg }),
      "dehumanized" | "dehumanized-direct" | "deh" => Ok(TeamKind::Dehumanized),
      "dummy" | "noop" | "none" | "idle" => Ok(TeamKind::Dummy),
      "sumatra" | "real" | "tigers" => Ok(TeamKind::Sumatra),
      other => Err(format!("unknown team kind: {other}")),
    }
  }

  pub fn label(&self) -> &'static str {
    match self {
      TeamKind::Bangka => "bangka",
      TeamKind::Bongka { .. } => "bongka",
      TeamKind::Ungabunga { .. } => "ungabunga",
      TeamKind::Bangka1 => "bangka1",
      TeamKind::BangkaLegacy => "legacy",
      TeamKind::CrashPilot { .. } => "crashpilot",
      TeamKind::Dehumanized => "dehumanized",
      TeamKind::Dummy => "dummy",
      TeamKind::Sumatra => "sumatra",
    }
  }

  /// True for AIs that run externally (over SimNet) rather than as a faabs
  /// controller inside this process.
  pub fn is_external(&self) -> bool {
    matches!(self, TeamKind::Sumatra)
  }

  /// True for in-process sides that go through CrashPilot/faabs and therefore
  /// inherit CrashPilot's current robot-count limit.
  pub fn uses_crashpilot_binding(&self) -> bool {
    !matches!(
      self,
      TeamKind::Dehumanized | TeamKind::Dummy | TeamKind::Sumatra
    )
  }
}

/// Build a faabs controller for an in-process side. Panics for external kinds
/// (e.g. [`TeamKind::Sumatra`]); `run_match` must route those separately.
pub fn build_controller(kind: &TeamKind, color: TeamColor, num_robots: u8) -> Box<dyn Controller> {
  #[cfg(not(feature = "faabs"))]
  let _ = color;

  match kind {
    TeamKind::Dummy => Box::new(DummyController { num_robots }),
    #[cfg(not(feature = "dehumanized"))]
    TeamKind::Dehumanized => {
      panic!("Dehumanized is disabled; build with `--features dehumanized` to enable")
    }
    #[cfg(feature = "dehumanized")]
    TeamKind::Dehumanized => Box::new(DirectDehumanizedController::new(num_robots)),
    #[cfg(not(feature = "bangka"))]
    TeamKind::Bangka => panic!("Bangka is disabled; build with `--features bangka` to enable"),
    #[cfg(feature = "bangka")]
    TeamKind::Bangka => {
      let mut faabs = Faabs::with_ai(num_robots, color, bangka::Bangka::new());
      start_crash_pilot(&mut faabs, color);
      Box::new(FaabsController {
        faabs,
        name: "bangka".to_string(),
      })
    }
    #[cfg(not(feature = "bongka"))]
    TeamKind::Bongka { .. } => {
      panic!("Bongka is disabled; build with `--features bongka` to enable")
    }
    #[cfg(feature = "bongka")]
    TeamKind::Bongka { params } => {
      let p = params
        .as_ref()
        .and_then(|path| std::fs::read_to_string(path).ok())
        .and_then(|s| bongka::Params::from_json_str(&s))
        .unwrap_or_default();
      let mut faabs = Faabs::with_ai(num_robots, color, bongka::Bangka::with_params(p));
      start_crash_pilot(&mut faabs, color);
      Box::new(FaabsController {
        faabs,
        name: "bongka".to_string(),
      })
    }
    #[cfg(not(feature = "ungabunga"))]
    TeamKind::Ungabunga { .. } => {
      panic!("Ungabunga is disabled; build with `--features ungabunga` to enable")
    }

    #[cfg(feature = "ungabunga")]
    TeamKind::Ungabunga { params } => {
      let p = params
        .as_ref()
        .and_then(|path| std::fs::read_to_string(path).ok())
        .and_then(|s| ungabunga::Params::from_json_str(&s))
        .unwrap_or_default();
      let mut faabs = Faabs::with_ai(num_robots, color, ungabunga::Bangka::with_params(p));
      start_crash_pilot(&mut faabs, color);
      Box::new(FaabsController {
        faabs,
        name: "ungabunga".to_string(),
      })
    }

    #[cfg(not(feature = "ungabunga"))]
    TeamKind::Bangka1 { .. } => {
      panic!("Ungabunga is disabled; build with `--features ungabunga` to enable")
    }

    #[cfg(feature = "ungabunga")]
    TeamKind::Bangka1 => {
      let mut faabs = Faabs::with_ai(num_robots, color, ungabunga::Bangka1::new());
      start_crash_pilot(&mut faabs, color);
      Box::new(FaabsController {
        faabs,
        name: "bangka1".to_string(),
      })
    }

    #[cfg(not(feature = "ungabunga"))]
    TeamKind::BangkaLegacy { .. } => {
      panic!("Ungabunga is disabled; build with `--features ungabunga` to enable")
    }

    #[cfg(feature = "ungabunga")]
    TeamKind::BangkaLegacy => {
      let mut faabs = Faabs::with_ai(num_robots, color, ungabunga::LegacyBangka::new());
      start_crash_pilot(&mut faabs, color);
      Box::new(FaabsController {
        faabs,
        name: "legacy".to_string(),
      })
    }
    #[cfg(not(feature = "artificial_incompetence"))]
    TeamKind::CrashPilot { .. } => {
      panic!("CrashPilot is disabled; build with `--features artificial_incompetence` to enable")
    }
    #[cfg(feature = "artificial_incompetence")]
    TeamKind::CrashPilot { model } => {
      let path = model
        .as_deref()
        .unwrap_or(artificial_incompetence::DEFAULT_MODEL_PATH);
      let ai = MlAi::from_safetensors(path).unwrap_or_else(|err| {
        panic!("failed to load CrashPilot model from {path}: {err}");
      });
      let mut faabs = Faabs::with_ai(num_robots, color, ai);
      start_crash_pilot(&mut faabs, color);
      Box::new(FaabsController {
        faabs,
        name: "crashpilot".to_string(),
      })
    }
    TeamKind::Sumatra => {
      unreachable!("Sumatra is external; run_match drives it over SimNet")
    }
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn dummy_aliases_parse_to_dummy() {
    for name in ["dummy", "noop", "none", "idle"] {
      assert!(matches!(TeamKind::parse(name), Ok(TeamKind::Dummy)));
    }
  }

  #[cfg(feature = "dehumanized")]
  #[test]
  fn dehumanized_aliases_parse_to_direct_controller() {
    for name in ["dehumanized", "dehumanized-direct", "deh"] {
      assert!(matches!(TeamKind::parse(name), Ok(TeamKind::Dehumanized)));
    }
  }

  #[test]
  fn dummy_controller_sends_zero_wheel_commands_for_every_robot() {
    let mut controller = build_controller(&TeamKind::Dummy, TeamColor::Blue, 3);
    let state = WorldState {
      world_id: 0,
      sim_time: 0.0,
      frame: 0,
      ball: simhark::BallState {
        x: 0.0,
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
    };
    let commands = controller.act(
      &state,
      &WorldConfig::division_b(),
      TeamColor::Blue,
      GameCommand::Running,
    );

    assert_eq!(commands.len(), 3);
    for (id, command) in commands.iter().enumerate() {
      assert_eq!(command.id, id);
      assert!(matches!(
        command.move_command,
        Some(MoveCommand::WheelVelocity([0.0, 0.0, 0.0, 0.0]))
      ));
      assert_eq!(command.kick_speed, 0.0);
      assert_eq!(command.kick_angle, 0.0);
      assert!(!command.dribbler_on);
    }
  }
}
