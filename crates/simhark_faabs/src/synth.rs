use core_dump::proto::{
  CpMode, InterfaceCommandCp, InterfaceGameCp, InterfaceManualCp, InterfaceTestCp,
  InterfaceWrapperCp,
};
use simhark::TeamColor;
use std::time::{SystemTime, UNIX_EPOCH};

fn current_time_micros() -> u64 {
  SystemTime::now()
    .duration_since(UNIX_EPOCH)
    .unwrap_or_default()
    .as_micros() as u64
}

pub fn referee_command(command: i32) -> core_dump::proto::Referee {
  let now = current_time_micros();
  core_dump::proto::Referee {
    packet_timestamp: now,
    command,
    command_counter: 1,
    command_timestamp: 1,
    ..Default::default()
  }
}

pub fn force_start_referee() -> core_dump::proto::Referee {
  referee_command(3) // FORCE_START
}

pub fn interface_command(team: TeamColor) -> InterfaceWrapperCp {
  let (team_color, side) = match team {
    TeamColor::Yellow => (false, false),
    TeamColor::Blue => (true, true),
  };
  InterfaceWrapperCp {
    robot_commands: Vec::new(),
    interface_command: InterfaceCommandCp {
      team_color,
      side,
      mode: CpMode::ModeGame as i32,
      manual: InterfaceManualCp {
        ball_tracked: true,
        ..Default::default()
      },
      game: InterfaceGameCp {
        running: true,
        goalkeeper_id: 0,
        max_speed: 0,
      },
      test: InterfaceTestCp::default(),
    },
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn interface_command_uses_robot_code_team_color_convention() {
    let yellow = interface_command(TeamColor::Yellow);
    let blue = interface_command(TeamColor::Blue);

    assert!(!yellow.interface_command.team_color);
    assert!(blue.interface_command.team_color);
  }
}
