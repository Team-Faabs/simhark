use std::collections::BTreeMap;

use core_dump::proto::{
  CrashpilotCommand, CrashpilotRobot, CrashpilotState, CrashpilotTask, Referee,
};
use core_dump::types::RobotCommand as AiRobotCommand;
use core_dump::vec::types::Vec2;
use simhark::state::RobotState;
use simhark::viewer::{
  DebugHoloRobot, DebugKickLine, DebugOverlay, RobotDebugInfo, ViewerDebugSnapshot,
};
use simhark::{TeamColor, WorldState};
use tf_jetsoncode::{TeensySendMsg, send_flags};

pub fn robot_debug_info(
  id: u32,
  team: TeamColor,
  ai_command: Option<AiRobotCommand>,
  cp: &CrashpilotRobot,
  teensy: &TeensySendMsg,
  state: &WorldState,
) -> RobotDebugInfo {
  let command = &cp.cmd;
  let task = ai_command
    .map(ai_task_label)
    .unwrap_or_else(|| task_label(command));
  let sim_robot = team_robots(state, team)
    .iter()
    .find(|robot| robot.id == id as usize);

  RobotDebugInfo {
    team,
    id: id as usize,
    color: debug_color(ai_command, command),
    task,
    message: Some(robot_message(ai_command, command, teensy, sim_robot)),
  }
}

pub fn robot_debug_overlays(
  id: u32,
  team: TeamColor,
  ai_command: Option<AiRobotCommand>,
  command: &CrashpilotCommand,
  state: &WorldState,
) -> Vec<DebugOverlay> {
  let color = debug_color(ai_command, command);
  match CrashpilotTask::try_from(command.task).ok() {
    Some(CrashpilotTask::Pos) => target_overlay(id, team, ai_command, command, color)
      .into_iter()
      .collect(),
    Some(CrashpilotTask::Kick) | Some(CrashpilotTask::Chip) | Some(CrashpilotTask::RecKick) => {
      let mut overlays = kick_line_angle(id, team, ai_command, command, state)
        .map(|angle| {
          vec![DebugOverlay::KickLine(DebugKickLine {
            team,
            id: id as usize,
            from_x: state.ball.x,
            from_y: state.ball.y,
            angle,
            color: color.clone(),
            label: Some(
              ai_command
                .map(ai_task_label)
                .unwrap_or_else(|| task_label(command).to_lowercase()),
            ),
          })]
        })
        .unwrap_or_default();

      if matches!(ai_command, Some(AiRobotCommand::RecPass)) {
        if let Some(target) = target_overlay(id, team, ai_command, command, color) {
          overlays.push(target);
        }
      }

      overlays
    }
    _ => Vec::new(),
  }
}

fn target_overlay(
  id: u32,
  team: TeamColor,
  ai_command: Option<AiRobotCommand>,
  command: &CrashpilotCommand,
  color: String,
) -> Option<DebugOverlay> {
  let pos = command.pos.as_ref()?;
  let label = match ai_command {
    Some(AiRobotCommand::RecPass) => "AI RecPass target".to_string(),
    Some(command) => ai_task_label(command),
    None => "target".to_string(),
  };

  Some(DebugOverlay::HoloRobot(DebugHoloRobot {
    team,
    id: id as usize,
    x: pos.x as f64 / 1000.0,
    y: pos.y as f64 / 1000.0,
    orientation: command
      .orientation
      .map(|orientation| (orientation as f64).to_radians()),
    color,
    label: Some(label),
  }))
}

fn kick_line_angle(
  id: u32,
  team: TeamColor,
  ai_command: Option<AiRobotCommand>,
  command: &CrashpilotCommand,
  state: &WorldState,
) -> Option<f64> {
  if matches!(ai_command, Some(AiRobotCommand::RecPass)) {
    return moving_ball_angle(state).or_else(|| {
      command
        .kick_orient
        .map(|angle| (angle as f64).to_radians())
        .or_else(|| receive_pass_angle(id, team, state))
    });
  }

  command
    .kick_orient
    .map(|angle| (angle as f64).to_radians())
    .or_else(|| {
      if command.task == CrashpilotTask::RecKick as i32 {
        moving_ball_angle(state)
      } else {
        None
      }
    })
}

fn moving_ball_angle(state: &WorldState) -> Option<f64> {
  let ball_speed = (state.ball.vx * state.ball.vx + state.ball.vy * state.ball.vy).sqrt();
  (ball_speed > 0.05).then_some(state.ball.vy.atan2(state.ball.vx))
}

fn receive_pass_angle(id: u32, team: TeamColor, state: &WorldState) -> Option<f64> {
  let receiver = team_robots(state, team)
    .iter()
    .find(|robot| robot.id == id as usize)?;
  let dx = receiver.x - state.ball.x;
  let dy = receiver.y - state.ball.y;
  if dx.abs() <= f64::EPSILON && dy.abs() <= f64::EPSILON {
    return None;
  }

  Some(dy.atan2(dx))
}

pub fn snapshot(
  world_id: usize,
  team: TeamColor,
  state: &WorldState,
  referee: Option<&Referee>,
  ai_debug: Option<String>,
  robots: Vec<RobotDebugInfo>,
  overlays: Vec<DebugOverlay>,
) -> ViewerDebugSnapshot {
  ViewerDebugSnapshot {
    world_id,
    strategy: Some(strategy_message(
      team,
      state,
      referee,
      ai_debug.as_deref(),
      &robots,
    )),
    robots,
    overlays,
  }
}

fn strategy_message(
  team: TeamColor,
  state: &WorldState,
  referee: Option<&Referee>,
  ai_debug: Option<&str>,
  robots: &[RobotDebugInfo],
) -> String {
  let active = team_robots(state, team)
    .iter()
    .filter(|robot| robot.is_on)
    .count();
  let holders = ball_holders(state);
  let task_counts = task_counts(robots);
  let referee = referee
    .map(|referee| format!("referee={}", referee.command))
    .unwrap_or_else(|| "referee=none".to_string());
  let ball_speed = (state.ball.vx * state.ball.vx + state.ball.vy * state.ball.vy).sqrt();

  let summary = format!(
    "FAABS {team:?}: commanded {}/{} active robots | tasks: {} | ball=({:.2},{:.2}) v={:.2}m/s | holders: {} | {referee}",
    robots.len(),
    active,
    format_task_counts(&task_counts),
    state.ball.x,
    state.ball.y,
    ball_speed,
    holders.unwrap_or_else(|| "none".to_string()),
  );

  match ai_debug.map(str::trim).filter(|debug| !debug.is_empty()) {
    Some(debug) => format!("{summary} | ai: {debug}"),
    None => summary,
  }
}

fn task_counts(robots: &[RobotDebugInfo]) -> BTreeMap<&str, usize> {
  let mut counts = BTreeMap::new();
  for robot in robots {
    *counts.entry(robot.task.as_str()).or_insert(0) += 1;
  }
  counts
}

fn format_task_counts(counts: &BTreeMap<&str, usize>) -> String {
  if counts.is_empty() {
    return "none".to_string();
  }
  counts
    .iter()
    .map(|(task, count)| format!("{task} {count}"))
    .collect::<Vec<_>>()
    .join(", ")
}

fn robot_message(
  ai_command: Option<AiRobotCommand>,
  command: &CrashpilotCommand,
  teensy: &TeensySendMsg,
  robot: Option<&RobotState>,
) -> String {
  let state = state_label(command.state);
  let ai = ai_command
    .map(|command| format!("ai={}; ", ai_command_detail(command)))
    .unwrap_or_default();
  let cp = format!("cp={}; ", task_label(command));
  let doing = firmware_action(teensy);
  let target = command
    .pos
    .as_ref()
    .map(|pos| {
      format!(
        " target=({:.2},{:.2})m",
        pos.x as f64 / 1000.0,
        pos.y as f64 / 1000.0
      )
    })
    .unwrap_or_default();
  let cp_speed = command
    .speed
    .map(|speed| format!(" cp_speed={:.2}m/s", speed as f64 / 1000.0))
    .unwrap_or_default();
  let orientation = command
    .orientation
    .map(|orientation| format!(" orient={}deg", orientation))
    .unwrap_or_default();
  let kick_orient = command
    .kick_orient
    .map(|orientation| format!(" kick_orient={}deg", orientation))
    .unwrap_or_default();
  let kick_speed = command
    .kick_speed
    .map(|speed| format!(" kick_speed={}", speed))
    .unwrap_or_default();
  let enemy = command
    .enemy_id
    .map(|enemy| format!(" enemy={enemy}"))
    .unwrap_or_default();
  let possession = robot
    .map(|robot| {
      format!(
        " pos=({:.2},{:.2}) v={:.2}m/s ball={}",
        robot.x,
        robot.y,
        (robot.vx * robot.vx + robot.vy * robot.vy).sqrt(),
        if robot.infrared { "yes" } else { "no" },
      )
    })
    .unwrap_or_default();

  format!(
    "{ai}{cp}{state}; {doing}{target}{cp_speed}{orientation}{kick_orient}{kick_speed}{enemy}{possession}"
  )
}

fn firmware_action(teensy: &TeensySendMsg) -> String {
  if teensy.state == CrashpilotState::Halt as u8 {
    return "halted".to_string();
  }

  let speed = teensy.speed as f64 / 1000.0;
  let mut parts = Vec::new();
  if speed > 0.01 {
    parts.push(format!("move {:.2}m/s @ {}deg", speed, teensy.dir));
  } else {
    parts.push("hold position".to_string());
  }
  parts.push(format!("turn {}->{}deg", teensy.self_orient, teensy.orient));
  if teensy.flags & send_flags::DRIBBLER != 0 {
    parts.push(format!("dribbler {}", teensy.dribbler_pwr));
  }
  if teensy.flags & send_flags::KICK != 0 {
    parts.push(format!("kick {}", teensy.kick_pwr));
  }
  if teensy.flags & send_flags::CHIP != 0 {
    parts.push(format!("chip {}", teensy.kick_pwr));
  }

  parts.join("; ")
}

fn task_label(command: &CrashpilotCommand) -> String {
  CrashpilotTask::try_from(command.task)
    .map(|task| task.as_str_name())
    .unwrap_or("TASK_UNKNOWN")
    .trim_start_matches("TASK_")
    .trim_start_matches("STATE_")
    .replace('_', " ")
}

fn state_label(state: i32) -> &'static str {
  CrashpilotState::try_from(state)
    .map(|state| state.as_str_name())
    .unwrap_or("STATE_UNKNOWN")
}

fn task_color(command: &CrashpilotCommand) -> String {
  if command.state == CrashpilotState::Halt as i32 {
    return "#64748b".to_string();
  }

  let color = match CrashpilotTask::try_from(command.task).ok() {
    Some(CrashpilotTask::Pos) => "#38bdf8",
    Some(CrashpilotTask::Kick) => "#ef4444",
    Some(CrashpilotTask::Chip) => "#f97316",
    Some(CrashpilotTask::RecKick) => "#22c55e",
    Some(CrashpilotTask::Steal) => "#eab308",
    Some(CrashpilotTask::Dribble) => "#a855f7",
    Some(CrashpilotTask::PosBall) => "#14b8a6",
    Some(CrashpilotTask::Block) => "#6366f1",
    Some(CrashpilotTask::Kickoff) => "#ec4899",
    Some(CrashpilotTask::Freekick) => "#f59e0b",
    _ => "#94a3b8",
  };
  color.to_string()
}

fn debug_color(ai_command: Option<AiRobotCommand>, cp_command: &CrashpilotCommand) -> String {
  ai_command
    .map(ai_task_color)
    .unwrap_or_else(|| task_color(cp_command))
}

fn ai_task_label(command: AiRobotCommand) -> String {
  match command {
    AiRobotCommand::Pos(_) => "AI Pos",
    AiRobotCommand::PosSpeed(_, _) => "AI PosSpeed",
    AiRobotCommand::PosFace(_, _) => "AI PosFace",
    AiRobotCommand::PosFaceSpeed(_, _, _) => "AI PosFaceSpeed",
    AiRobotCommand::WallPos(_, _) => "AI WallPos",
    AiRobotCommand::Kick(_) => "AI Kick",
    AiRobotCommand::Chip(_) => "AI Chip",
    AiRobotCommand::RecKick(_) => "AI RecKick",
    AiRobotCommand::Steal => "AI Steal",
    AiRobotCommand::Dribble(_) => "AI Dribble",
    AiRobotCommand::PosBall(_) => "AI PosBall",
    AiRobotCommand::Kickoff(_) => "AI Kickoff",
    AiRobotCommand::FreeKick(_) => "AI FreeKick",
    AiRobotCommand::KickGoal => "AI KickGoal",
    AiRobotCommand::PassTo(robot) => return format!("AI PassTo {robot}"),
    AiRobotCommand::RecPass => "AI RecPass",
    AiRobotCommand::GoalWall => "AI GoalWall",
    AiRobotCommand::GoalieGuard => "AI GoalieGuard",
    AiRobotCommand::Hold => "AI Hold",
  }
  .to_string()
}

fn ai_command_detail(command: AiRobotCommand) -> String {
  match command {
    AiRobotCommand::Pos(pos) => format!("Pos {}", norm_pos(pos)),
    AiRobotCommand::PosSpeed(pos, speed) => format!("PosSpeed {} speed={speed}", norm_pos(pos)),
    AiRobotCommand::PosFace(pos, orient) => format!("PosFace {} face={orient}deg", norm_pos(pos)),
    AiRobotCommand::PosFaceSpeed(pos, orient, speed) => {
      format!(
        "PosFaceSpeed {} face={orient}deg speed={speed}",
        norm_pos(pos)
      )
    }
    AiRobotCommand::WallPos(pos, orient) => format!("WallPos {} face={orient}deg", norm_pos(pos)),
    AiRobotCommand::Kick(orient) => format!("Kick orient={orient:.0}deg"),
    AiRobotCommand::Chip(orient) => format!("Chip orient={orient:.0}deg"),
    AiRobotCommand::RecKick(power) => format!("RecKick power={power:.2}"),
    AiRobotCommand::Steal => "Steal".to_string(),
    AiRobotCommand::Dribble(pos) => format!("Dribble {}", norm_pos(pos)),
    AiRobotCommand::PosBall(pos) => format!("PosBall {}", norm_pos(pos)),
    AiRobotCommand::Kickoff(power) => format!("Kickoff power={power:.2}"),
    AiRobotCommand::FreeKick(power) => format!("FreeKick power={power:.2}"),
    AiRobotCommand::KickGoal => "KickGoal".to_string(),
    AiRobotCommand::PassTo(robot) => format!("PassTo {robot}"),
    AiRobotCommand::RecPass => "RecPass".to_string(),
    AiRobotCommand::GoalWall => "GoalWall".to_string(),
    AiRobotCommand::GoalieGuard => "GoalieGuard".to_string(),
    AiRobotCommand::Hold => "Hold".to_string(),
  }
}

fn norm_pos(pos: Vec2<f32>) -> String {
  format!("target=({:.2},{:.2})", pos.x, pos.y)
}

fn ai_task_color(command: AiRobotCommand) -> String {
  let color = match command {
    AiRobotCommand::Pos(_)
    | AiRobotCommand::PosSpeed(_, _)
    | AiRobotCommand::PosFace(_, _)
    | AiRobotCommand::PosFaceSpeed(_, _, _)
    | AiRobotCommand::WallPos(_, _) => "#38bdf8",
    AiRobotCommand::Kick(_) | AiRobotCommand::KickGoal => "#ef4444",
    AiRobotCommand::Chip(_) => "#f97316",
    AiRobotCommand::RecKick(_) | AiRobotCommand::RecPass => "#22c55e",
    AiRobotCommand::Steal => "#eab308",
    AiRobotCommand::Dribble(_) => "#a855f7",
    AiRobotCommand::PosBall(_) => "#14b8a6",
    AiRobotCommand::Kickoff(_) => "#ec4899",
    AiRobotCommand::FreeKick(_) => "#f59e0b",
    AiRobotCommand::PassTo(_) => "#0ea5e9",
    AiRobotCommand::GoalWall | AiRobotCommand::GoalieGuard => "#6366f1",
    AiRobotCommand::Hold => "#94a3b8",
  };
  color.to_string()
}

fn team_robots(state: &WorldState, team: TeamColor) -> &[RobotState] {
  match team {
    TeamColor::Blue => &state.blue_robots,
    TeamColor::Yellow => &state.yellow_robots,
  }
}

fn ball_holders(state: &WorldState) -> Option<String> {
  let mut holders = state
    .blue_robots
    .iter()
    .filter(|robot| robot.infrared)
    .map(|robot| format!("Blue {}", robot.id))
    .chain(
      state
        .yellow_robots
        .iter()
        .filter(|robot| robot.infrared)
        .map(|robot| format!("Yellow {}", robot.id)),
    )
    .collect::<Vec<_>>();
  holders.sort();
  if holders.is_empty() {
    None
  } else {
    Some(holders.join(", "))
  }
}
