use crate::director::Score;
use referris::domain::{TrackedBall, TrackedRobot};
use referris::{
  AutoRef, Command, FieldGeometry, GameEvent, InputEnvelope, RawDetectionFrame, RefereeSnapshot,
  Stage, Team, TeamInfo, TrackedFrame,
};
use simhark::{TeamColor, WorldConfig, WorldState};

pub struct ReferrisAutoref {
  autoref: AutoRef,
  enabled: bool,
  interface: Option<referris::webinterface::ReferrisInterfaceAdapter>,
  blue_yellow_cards: u32,
  yellow_yellow_cards: u32,
  blue_red_cards: u32,
  yellow_red_cards: u32,
}

#[derive(Debug, Clone)]
pub struct ReferrisTick {
  pub command_code: i32,
  pub command_counter: u32,
  pub command_label: &'static str,
}

impl ReferrisAutoref {
  pub fn new() -> Self {
    Self {
      autoref: AutoRef::default(),
      enabled: true,
      interface: None,
      blue_yellow_cards: 0,
      yellow_yellow_cards: 0,
      blue_red_cards: 0,
      yellow_red_cards: 0,
    }
  }

  pub fn step(
    &mut self,
    state: &WorldState,
    cfg: &WorldConfig,
    score: Score,
    command_code: i32,
    quiet: bool,
  ) -> ReferrisTick {
    self.apply_interface_commands();
    let command = command_from_ssl_code(command_code);
    let input = referris_input(state, cfg, score, command);
    let step = self.enabled.then(|| self.autoref.step(&input));

    if !quiet {
      for event in step
        .as_ref()
        .into_iter()
        .flat_map(|step| &step.events)
        .filter(|event| important_event(event))
      {
        println!(
          "  [{:6.1}s] referris: {}",
          state.sim_time,
          event_label(event)
        );
      }
    }

    let events = step.as_ref().map_or(&[][..], |step| step.events.as_slice());
    let snapshot = self.autoref.referee_state().map(|state| state.snapshot());
    if let Some(interface) = &self.interface
      && let Err(error) = interface.publish(snapshot, events, self.enabled)
    {
      eprintln!("[referris-interface] {error}");
    }

    let referee = self.autoref.referee_state().map(|state| {
      let snapshot = state.snapshot();
      (
        snapshot.command,
        snapshot.command_counter,
        snapshot.blue.yellow_cards,
        snapshot.yellow.yellow_cards,
        snapshot.blue.red_cards,
        snapshot.yellow.red_cards,
      )
    });
    if let Some((
      command,
      command_counter,
      blue_yellow_cards,
      yellow_yellow_cards,
      blue_red_cards,
      yellow_red_cards,
    )) = referee
    {
      if !quiet {
        self.print_card_changes(
          state.sim_time,
          blue_yellow_cards,
          yellow_yellow_cards,
          blue_red_cards,
          yellow_red_cards,
        );
      }
      ReferrisTick {
        command_code: ssl_code_from_command(command),
        command_counter,
        command_label: command_label(command),
      }
    } else {
      ReferrisTick {
        command_code,
        command_counter: state.frame as u32,
        command_label: command_label(command),
      }
    }
  }

  pub fn attach_interface(
    &mut self,
    handle: &webinterface_core::InterfaceHandle,
    session_id: webinterface_protocol::SessionId,
  ) -> Result<(), webinterface_core::InterfaceError> {
    self.interface = Some(referris::webinterface::ReferrisInterfaceAdapter::register(
      handle, session_id, true,
    )?);
    Ok(())
  }

  fn apply_interface_commands(&mut self) {
    loop {
      let Some(queued) = self
        .interface
        .as_mut()
        .and_then(|interface| interface.try_next_command())
      else {
        break;
      };
      let result = match queued.command {
        webinterface_protocol::ReferrisCommand::Start => {
          self.enabled = true;
          Ok("Referris started".to_string())
        }
        webinterface_protocol::ReferrisCommand::Stop => {
          self.enabled = false;
          Ok("Referris stopped".to_string())
        }
        webinterface_protocol::ReferrisCommand::Reset => {
          self.autoref = AutoRef::default();
          Ok("Referris reset".to_string())
        }
        webinterface_protocol::ReferrisCommand::SetConfiguration(value) => {
          serde_json::from_value::<referris::AutoRefConfig>(value)
            .map(|config| {
              self.autoref = AutoRef::new(config);
              "Referris configuration applied".to_string()
            })
            .map_err(|error| error.to_string())
        }
        webinterface_protocol::ReferrisCommand::EmitEvent(value) => self
          .interface
          .as_ref()
          .expect("interface exists while draining commands")
          .publish_operator_event(value)
          .map(|_| "Referris operator event emitted".to_string())
          .map_err(|error| error.to_string()),
      };
      let (status, message) = match result {
        Ok(message) => (webinterface_protocol::CommandStatus::Applied, message),
        Err(message) => (webinterface_protocol::CommandStatus::Rejected, message),
      };
      self
        .interface
        .as_ref()
        .expect("interface exists while acknowledging commands")
        .acknowledge(queued.id, status, message);
    }
  }

  fn print_card_changes(
    &mut self,
    sim_time: f64,
    blue_yellow_cards: u32,
    yellow_yellow_cards: u32,
    blue_red_cards: u32,
    yellow_red_cards: u32,
  ) {
    if blue_yellow_cards > self.blue_yellow_cards {
      println!(
        "  [{sim_time:6.1}s] referris: yellow-card Blue total={}",
        blue_yellow_cards
      );
    }
    if yellow_yellow_cards > self.yellow_yellow_cards {
      println!(
        "  [{sim_time:6.1}s] referris: yellow-card Yellow total={}",
        yellow_yellow_cards
      );
    }
    if blue_red_cards > self.blue_red_cards {
      println!(
        "  [{sim_time:6.1}s] referris: red-card Blue total={}",
        blue_red_cards
      );
    }
    if yellow_red_cards > self.yellow_red_cards {
      println!(
        "  [{sim_time:6.1}s] referris: red-card Yellow total={}",
        yellow_red_cards
      );
    }
    self.blue_yellow_cards = blue_yellow_cards;
    self.yellow_yellow_cards = yellow_yellow_cards;
    self.blue_red_cards = blue_red_cards;
    self.yellow_red_cards = yellow_red_cards;
  }
}

impl Default for ReferrisAutoref {
  fn default() -> Self {
    Self::new()
  }
}

fn referris_input(
  state: &WorldState,
  cfg: &WorldConfig,
  score: Score,
  command: Command,
) -> InputEnvelope {
  InputEnvelope {
    geometry: Some(FieldGeometry {
      field_length: cfg.field.field_length,
      field_width: cfg.field.field_width,
      goal_width: cfg.field.goal_width,
      goal_depth: cfg.field.goal_depth,
      boundary_width: cfg.field.margin_touch_line,
      boundary_width_goal_line: cfg.field.margin_goal_line,
      defense_area_depth: cfg.field.penalty_depth,
      defense_area_width: cfg.field.penalty_width,
      center_circle_radius: cfg.field.field_center_radius,
      line_thickness: cfg.field.field_line_width,
      goal_height: cfg.field.goal_height,
      ball_radius: cfg.ball.radius,
      max_robot_radius: cfg.blue_robots.radius.max(cfg.yellow_robots.radius),
    }),
    referee: Some(RefereeSnapshot {
      timestamp: state.sim_time,
      stage: Stage::NormalFirstHalf,
      stage_time_left: None,
      command,
      command_counter: state.frame as u32,
      command_timestamp: state.sim_time,
      blue_on_positive_half: Some(false),
      next_command: None,
      current_action_time_remaining: None,
      designated_position: None,
      yellow: team_info("Yellow", state.yellow_robots.len(), score.yellow),
      blue: team_info("Blue", state.blue_robots.len(), score.blue),
    }),
    detections: Vec::<RawDetectionFrame>::new(),
    tracked: Some(TrackedFrame {
      frame_number: state.frame as u32,
      timestamp: state.sim_time,
      ball: Some(TrackedBall {
        pos: referris::math::Vec3 {
          x: state.ball.x,
          y: state.ball.y,
          z: state.ball.z,
        },
        vel: referris::math::Vec3 {
          x: state.ball.vx,
          y: state.ball.vy,
          z: state.ball.vz,
        },
        visible: true,
      }),
      robots: state
        .blue_robots
        .iter()
        .map(|robot| tracked_robot(robot, Team::Blue))
        .chain(
          state
            .yellow_robots
            .iter()
            .map(|robot| tracked_robot(robot, Team::Yellow)),
        )
        .collect(),
      kicked_ball: None,
    }),
  }
}

fn team_info(name: &str, robots: usize, score: u32) -> TeamInfo {
  TeamInfo {
    name: name.into(),
    goalkeeper: Some(0),
    max_allowed_bots: Some(robots as u32),
    score,
    red_cards: 0,
    yellow_cards: 0,
    yellow_card_times: Vec::new(),
    timeouts: 0,
    timeout_time: 0.0,
  }
}

fn tracked_robot(robot: &simhark::RobotState, team: Team) -> TrackedRobot {
  let _ = match robot.team {
    TeamColor::Blue => Team::Blue,
    TeamColor::Yellow => Team::Yellow,
  };
  TrackedRobot {
    id: robot.id as u32,
    team,
    pos: referris::math::Vec2 {
      x: robot.x,
      y: robot.y,
    },
    orientation: robot.orientation,
    vel: referris::math::Vec2 {
      x: robot.vx,
      y: robot.vy,
    },
    angular_velocity: robot.v_angular,
    visible: robot.is_on,
  }
}

fn important_event(event: &GameEvent) -> bool {
  event.is_foul()
    || matches!(
      event,
      GameEvent::BallLeftFieldTouchLine { .. }
        | GameEvent::BallLeftFieldGoalLine { .. }
        | GameEvent::PossibleGoal { .. }
    )
}

fn event_label(event: &GameEvent) -> String {
  match event {
    GameEvent::BallLeftFieldTouchLine {
      by_team, by_bot, ..
    } => format!(
      "ball-left-touch-line by {}{}",
      team_label(*by_team),
      bot_suffix(*by_bot)
    ),
    GameEvent::BallLeftFieldGoalLine {
      by_team, by_bot, ..
    } => format!(
      "ball-left-goal-line by {}{}",
      team_label(*by_team),
      bot_suffix(*by_bot)
    ),
    GameEvent::AimlessKick {
      by_team, by_bot, ..
    } => format!(
      "aimless-kick by {}{}",
      team_label(*by_team),
      bot_suffix(*by_bot)
    ),
    GameEvent::AttackerTooCloseToDefenseArea {
      by_team,
      by_bot,
      distance,
      ..
    } => format!(
      "attacker-too-close-to-defense-area by {}{} distance={}",
      team_label(*by_team),
      bot_suffix(*by_bot),
      opt_f64(*distance)
    ),
    GameEvent::DefenderInDefenseArea {
      by_team,
      by_bot,
      distance,
      ..
    } => format!(
      "defender-in-defense-area by {}{} distance={}",
      team_label(*by_team),
      bot_suffix(*by_bot),
      opt_f64(*distance)
    ),
    GameEvent::AttackerTouchedBallInDefenseArea {
      by_team, by_bot, ..
    } => format!(
      "attacker-touched-ball-in-defense-area by {}{}",
      team_label(*by_team),
      bot_suffix(*by_bot)
    ),
    GameEvent::BotKickedBallTooFast {
      by_team,
      by_bot,
      initial_ball_speed,
      ..
    } => format!(
      "bot-kicked-ball-too-fast by {}{} speed={}",
      team_label(*by_team),
      bot_suffix(*by_bot),
      opt_f64(*initial_ball_speed)
    ),
    GameEvent::BotCrashUnique {
      by_team,
      violator,
      victim,
      crash_speed,
      ..
    } => format!(
      "bot-crash by {}{} victim={} speed={}",
      team_label(*by_team),
      bot_suffix(*violator),
      victim
        .map(|bot| bot.to_string())
        .unwrap_or_else(|| "?".to_string()),
      opt_f64(*crash_speed)
    ),
    GameEvent::BotCrashDrawn {
      bot_yellow,
      bot_blue,
      crash_speed,
      ..
    } => format!(
      "bot-crash-drawn yellow={} blue={} speed={}",
      bot_yellow
        .map(|bot| bot.to_string())
        .unwrap_or_else(|| "?".to_string()),
      bot_blue
        .map(|bot| bot.to_string())
        .unwrap_or_else(|| "?".to_string()),
      opt_f64(*crash_speed)
    ),
    GameEvent::DefenderTooCloseToKickPoint {
      by_team,
      by_bot,
      distance,
      ..
    } => format!(
      "defender-too-close-to-kick-point by {}{} distance={}",
      team_label(*by_team),
      bot_suffix(*by_bot),
      opt_f64(*distance)
    ),
    GameEvent::BotTooFastInStop {
      by_team,
      by_bot,
      speed,
      ..
    } => format!(
      "bot-too-fast-in-stop by {}{} speed={}",
      team_label(*by_team),
      bot_suffix(*by_bot),
      opt_f64(*speed)
    ),
    GameEvent::BotInterferedPlacement {
      by_team, by_bot, ..
    } => format!(
      "bot-interfered-placement by {}{}",
      team_label(*by_team),
      bot_suffix(*by_bot)
    ),
    GameEvent::PossibleGoal { by_team, .. } => {
      format!("possible-goal by {}", team_label(*by_team))
    }
    GameEvent::AttackerDoubleTouchedBall {
      by_team, by_bot, ..
    } => format!(
      "attacker-double-touched-ball by {}{}",
      team_label(*by_team),
      bot_suffix(*by_bot)
    ),
    GameEvent::PlacementSucceeded {
      by_team, precision, ..
    } => format!(
      "placement-succeeded by {} precision={}",
      team_label(*by_team),
      opt_f64(*precision)
    ),
    GameEvent::BotDribbledBallTooFar {
      by_team, by_bot, ..
    } => format!(
      "bot-dribbled-ball-too-far by {}{}",
      team_label(*by_team),
      bot_suffix(*by_bot)
    ),
  }
}

fn command_from_ssl_code(code: i32) -> Command {
  match code {
    0 => Command::Halt,
    1 => Command::Stop,
    2 => Command::NormalStart,
    3 => Command::ForceStart,
    4 => Command::PrepareKickoffYellow,
    5 => Command::PrepareKickoffBlue,
    6 => Command::PreparePenaltyYellow,
    7 => Command::PreparePenaltyBlue,
    8 => Command::DirectFreeYellow,
    9 => Command::DirectFreeBlue,
    10 => Command::IndirectFreeYellow,
    11 => Command::IndirectFreeBlue,
    12 => Command::TimeoutYellow,
    13 => Command::TimeoutBlue,
    16 => Command::BallPlacementYellow,
    17 => Command::BallPlacementBlue,
    _ => Command::Unknown,
  }
}

fn ssl_code_from_command(command: Command) -> i32 {
  match command {
    Command::Halt => 0,
    Command::Stop => 1,
    Command::NormalStart => 2,
    Command::ForceStart => 3,
    Command::PrepareKickoffYellow => 4,
    Command::PrepareKickoffBlue => 5,
    Command::PreparePenaltyYellow => 6,
    Command::PreparePenaltyBlue => 7,
    Command::DirectFreeYellow => 8,
    Command::DirectFreeBlue => 9,
    Command::IndirectFreeYellow => 10,
    Command::IndirectFreeBlue => 11,
    Command::TimeoutYellow => 12,
    Command::TimeoutBlue => 13,
    Command::BallPlacementYellow => 16,
    Command::BallPlacementBlue => 17,
    Command::Unknown => 3,
  }
}

pub fn command_label(command: Command) -> &'static str {
  match command {
    Command::Halt => "HALT",
    Command::Stop => "STOP",
    Command::NormalStart => "NORMAL_START",
    Command::ForceStart => "FORCE_START",
    Command::PrepareKickoffYellow => "PREPARE_KICKOFF_YELLOW",
    Command::PrepareKickoffBlue => "PREPARE_KICKOFF_BLUE",
    Command::PreparePenaltyYellow => "PREPARE_PENALTY_YELLOW",
    Command::PreparePenaltyBlue => "PREPARE_PENALTY_BLUE",
    Command::DirectFreeYellow => "DIRECT_FREE_YELLOW",
    Command::DirectFreeBlue => "DIRECT_FREE_BLUE",
    Command::IndirectFreeYellow => "INDIRECT_FREE_YELLOW",
    Command::IndirectFreeBlue => "INDIRECT_FREE_BLUE",
    Command::TimeoutYellow => "TIMEOUT_YELLOW",
    Command::TimeoutBlue => "TIMEOUT_BLUE",
    Command::BallPlacementYellow => "BALL_PLACEMENT_YELLOW",
    Command::BallPlacementBlue => "BALL_PLACEMENT_BLUE",
    Command::Unknown => "UNKNOWN",
  }
}

fn team_label(team: Team) -> &'static str {
  match team {
    Team::Blue => "Blue",
    Team::Yellow => "Yellow",
    Team::Unknown => "Unknown",
  }
}

fn bot_suffix(bot: Option<u32>) -> String {
  bot
    .map(|bot| format!("#{bot}"))
    .unwrap_or_else(|| "".to_string())
}

fn opt_f64(value: Option<f64>) -> String {
  value
    .map(|value| format!("{value:.2}"))
    .unwrap_or_else(|| "?".to_string())
}
