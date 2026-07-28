use crashpilot::core_dump::proto::CrashpilotInfos;
use simhark::{RobotState, TeamColor, WorldCommand, WorldConfig, WorldState};
use simhark_faabs::run_sim_action;
use simhark_testing::{
  BallInit, InitialWorld, RobotInit, SimTest, TeamCommands, TestCase, TestCli, TestOutcome,
  TestPlan, TestRunner, TestSuite,
};
use std::borrow::Cow;
use tf_jetsoncode::{
  Config, CrashpilotBall, CrashpilotCommand, CrashpilotRobot, CrashpilotState,
  CrashpilotTrackedRobot, CrashpilotVector2, Events, Robot, TeensyRecMSG,
};

const GOALIE_ID: usize = 0;
const ARC_RADIUS: f64 = 2.95;
const ARC_MIN_DEG: f64 = -82.0;
const ARC_MAX_DEG: f64 = 82.0;
const ARC_SAMPLES: usize = 32;
const SPEEDS: [f64; 4] = [2.5, 3.5, 4.5, 5.5];
const DEFENSE_PASS_FRAME: u64 = 260;

fn main() -> std::io::Result<()> {
  #[cfg(not(feature = "viewer"))]
  let cli = match TestCli::from_env() {
    Ok(cli) => cli,
    Err(error) => {
      eprintln!("{error}");
      std::process::exit(2);
    }
  };
  let runner = TestRunner::with_config(WorldConfig::division_b())
    .max_ticks(360)
    .concurrent_root_groups(1);

  #[cfg(not(feature = "viewer"))]
  runner.run_plan_cli(&cli, make_plan);
  #[cfg(feature = "viewer")]
  runner.run_plan_with_viewer(
    make_plan,
    simhark::viewer::ViewerConfig::default(),
    std::time::Duration::from_millis(15),
  )?;

  Ok(())
}

fn make_plan() -> TestPlan<GoalieShotTest> {
  let config = WorldConfig::division_b();
  let half_goal_width = config.field.goal_width * 0.5;
  let goal_x = -config.field.field_length * 0.5;
  let defense_front_x = goal_x + config.field.penalty_depth - config.ball.radius;
  let defense_mid_x = goal_x + config.field.penalty_depth * 0.58;
  let half_defense_width = config.field.penalty_width * 0.5;
  let arrivals = [
    Arrival::goal("goal lower post", -half_goal_width + 0.04),
    Arrival::goal("goal lower lane", -half_goal_width * 0.45),
    Arrival::goal("goal center", 0.0),
    Arrival::goal("goal upper lane", half_goal_width * 0.45),
    Arrival::goal("goal upper post", half_goal_width - 0.04),
    Arrival::defense(
      "defense lower side",
      defense_mid_x,
      -half_defense_width + 0.12,
    ),
    Arrival::defense("defense center", defense_front_x, 0.0),
    Arrival::defense(
      "defense upper side",
      defense_mid_x,
      half_defense_width - 0.12,
    ),
  ];

  let mut suite = TestSuite::new("goalie");
  for speed in SPEEDS {
    for arrival in arrivals {
      suite = suite.subtests(
        format!("{} @ {speed:.1}mps", arrival.name),
        arc_cases(&config, arrival, speed),
      );
    }
  }

  TestPlan::new().suite(suite)
}

fn arc_cases(
  config: &WorldConfig,
  arrival: Arrival,
  speed: f64,
) -> impl IntoIterator<Item = TestCase<GoalieShotTest>> {
  let goal_x = -config.field.field_length * 0.5;
  let target_x = match arrival.kind {
    ArrivalKind::Goal => goal_x - config.ball.radius,
    ArrivalKind::Defense => arrival.x,
  };

  (0..ARC_SAMPLES)
    .map(move |index| {
      let t = index as f64 / (ARC_SAMPLES - 1) as f64;
      let angle_deg = ARC_MIN_DEG + (ARC_MAX_DEG - ARC_MIN_DEG) * t;
      let angle = angle_deg.to_radians();
      let start_x = goal_x + ARC_RADIUS * angle.cos();
      let start_y = ARC_RADIUS * angle.sin();

      TestCase::new(
        format!("arc {angle_deg:+05.1}deg"),
        GoalieShotTest::new(
          start_x,
          start_y,
          target_x,
          arrival.y,
          speed,
          arrival.kind,
          angle_deg,
        ),
      )
    })
    .collect::<Vec<_>>()
}

#[derive(Clone, Copy)]
struct Arrival {
  name: &'static str,
  x: f64,
  y: f64,
  kind: ArrivalKind,
}

impl Arrival {
  fn goal(name: &'static str, y: f64) -> Self {
    Self {
      name,
      x: 0.0,
      y,
      kind: ArrivalKind::Goal,
    }
  }

  fn defense(name: &'static str, x: f64, y: f64) -> Self {
    Self {
      name,
      x,
      y,
      kind: ArrivalKind::Defense,
    }
  }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum ArrivalKind {
  Goal,
  Defense,
}

struct GoalieShotTest {
  start_x: f64,
  start_y: f64,
  target_x: f64,
  target_y: f64,
  speed: f64,
  arrival_kind: ArrivalKind,
  arc_angle_deg: f64,
  goalie: Robot<()>,
}

impl GoalieShotTest {
  fn new(
    start_x: f64,
    start_y: f64,
    target_x: f64,
    target_y: f64,
    speed: f64,
    arrival_kind: ArrivalKind,
    arc_angle_deg: f64,
  ) -> Self {
    let mut robot_config = Config::default();
    robot_config.robot_id = GOALIE_ID as u8;

    Self {
      start_x,
      start_y,
      target_x,
      target_y,
      speed,
      arrival_kind,
      arc_angle_deg,
      goalie: Robot::new(robot_config),
    }
  }
}

impl SimTest for GoalieShotTest {
  fn name(&self) -> Cow<'_, str> {
    Cow::Borrowed("goalie shot")
  }

  fn initial_state(&self) -> InitialWorld {
    let config = WorldConfig::division_b();
    let goal_x = -config.field.field_length * 0.5;
    let dx = self.target_x - self.start_x;
    let dy = self.target_y - self.start_y;
    let len = (dx * dx + dy * dy).sqrt().max(f64::EPSILON);

    let mut ball = BallInit::at(self.start_x, self.start_y);
    ball.vx = self.speed * dx / len;
    ball.vy = self.speed * dy / len;

    InitialWorld::new(
      [],
      [RobotInit::at(GOALIE_ID, goal_x + 0.22, 0.0, 0.0)],
      ball,
    )
  }

  fn drive(&mut self, state: &WorldState) -> TeamCommands {
    let (teensy, _) = self.goalie.step_with_data(goalie_events(state));
    let mut command = WorldCommand::default();
    run_sim_action(GOALIE_ID as u32, teensy, &mut command, TeamColor::Yellow);

    TeamCommands {
      blue: command.blue,
      yellow: command.yellow,
    }
  }

  fn validate(&mut self, state: &WorldState) -> TestOutcome {
    if state.yellow_robots[0].infrared {
      return TestOutcome::Passed;
    }

    if state.goal_yellow || state.ball.x < -4.6 {
      TestOutcome::Failed(format!(
        "goal conceded for target=({:.2},{:.2}), arc={:+.1}deg, speed={:.1}mps",
        self.target_x, self.target_y, self.arc_angle_deg, self.speed
      ))
    } else if self.arrival_kind == ArrivalKind::Defense && state.frame >= DEFENSE_PASS_FRAME {
      TestOutcome::Passed
    } else {
      TestOutcome::Running
    }
  }
}

fn goalie_events(state: &WorldState) -> Events {
  Events {
    cp: Some(cp_robot(state)),
    vis: None,
    teensy: Some(TeensyRecMSG {
      flags: (1 << 2) | (1 << 3),
      batt_level: 0,
      current: 0,
    }),
  }
}

fn cp_robot(state: &WorldState) -> CrashpilotRobot {
  let config = WorldConfig::division_b();

  CrashpilotRobot {
    robot_id: GOALIE_ID as u32,
    timestamp: state.sim_time,
    packet_id: state.frame as u32,
    ball: CrashpilotBall {
      pos: cp_vec(state.ball.x, state.ball.y),
      vel: Some(cp_vec(state.ball.vx, state.ball.vy)),
    },
    robots_yellow: state
      .yellow_robots
      .iter()
      .map(cp_tracked_robot)
      .collect::<Vec<_>>(),
    robots_blue: state
      .blue_robots
      .iter()
      .map(cp_tracked_robot)
      .collect::<Vec<_>>(),
    cmd: CrashpilotCommand {
      state: CrashpilotState::Goalie as i32,
      ..CrashpilotCommand::default()
    },
    infos: CrashpilotInfos {
      team_color: false,
      team_site: true,
      width: meters_to_mm_u32(config.field.field_length),
      height: meters_to_mm_u32(config.field.field_width),
      runoff_width: meters_to_mm_u32(config.field.margin_touch_line),
      penalty_area_width: meters_to_mm_u32(config.field.penalty_width),
      penalty_area_height: meters_to_mm_u32(config.field.penalty_depth),
      goal_width: meters_to_mm_u32(config.field.goal_width),
    },
  }
}

fn cp_tracked_robot(robot: &RobotState) -> CrashpilotTrackedRobot {
  CrashpilotTrackedRobot {
    robot_id: robot.id as u32,
    pos: cp_vec(robot.x, robot.y),
    orientation: robot.orientation.to_degrees() as i32,
    vel: Some(cp_vec(robot.vx, robot.vy)),
    visibility: if robot.is_on { 255 } else { 0 },
  }
}

fn cp_vec(x: f64, y: f64) -> CrashpilotVector2 {
  CrashpilotVector2 {
    x: meters_to_mm_i32(x),
    y: meters_to_mm_i32(y),
  }
}

fn meters_to_mm_i32(value: f64) -> i32 {
  (value * 1000.0).round() as i32
}

fn meters_to_mm_u32(value: f64) -> u32 {
  meters_to_mm_i32(value).max(0) as u32
}
