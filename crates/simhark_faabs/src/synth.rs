use core_dump::proto::{
  CpMode, InterfaceCommandCp, InterfaceGameCp, InterfaceManualCp, InterfaceTestCp,
  InterfaceWrapperCp,
};
use simhark::TeamColor;

pub fn force_start_referee() -> core_dump::proto::Referee {
  core_dump::proto::Referee {
    command: 3, // FORCE_START
    ..Default::default()
  }
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
