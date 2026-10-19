#![cfg(feature = "realistic")]
use simhark::{
  TeamColor, WorldConfig,
  realistic::{
    RealisticConfig, RealisticWorld,
    sensors::{RobotDetection, VisionFrame},
    vision::VisionTracker,
  },
};

fn detection(at: u64, x: f64) -> VisionFrame {
  VisionFrame {
    camera_id: 0,
    frame_number: at,
    camera_capture_ticks: None,
    captured_at: at,
    image_received_at: at,
    sent_at: at,
    physical_state_at: 0,
    robots: vec![RobotDetection {
      id: 0,
      team: TeamColor::Blue,
      position_m: [x, 0.0],
      orientation_rad: 0.3,
    }],
    ball_position_m: Some([x, 0.0, 0.0215]),
  }
}
fn stationary_tracker() -> VisionTracker {
  let mut tracker = VisionTracker::new(Default::default());
  for i in 0..120 {
    tracker.ingest(&detection(i * 16_666_667, 0.0));
  }
  tracker
}
#[test]
fn kick_after_stationary_tracking_recovers_before_expiry() {
  let mut tracker = stationary_tracker();
  for i in 1..=30 {
    let at = (119 + i) * 16_666_667;
    let x = i as f64 * 0.1;
    tracker.ingest(&detection(at, x));
    let world = tracker.snapshot(at);
    let ball = world.ball.unwrap();
    assert!(at - ball.last_observed_at < 100_000_000);
    if i >= 6 {
      assert!((ball.position_m[0] - x).abs() < 0.05, "{i}: {ball:?}");
      assert!((ball.velocity_m_s[0] - 6.0).abs() < 0.5);
    }
  }
}
#[test]
fn teleport_recovers_ball_and_robot_but_isolated_outlier_is_rejected() {
  let mut tracker = stationary_tracker();
  let start = 120 * 16_666_667;
  tracker.ingest(&detection(start, 4.0));
  assert!(tracker.snapshot(start).ball.unwrap().position_m[0].abs() < 0.01);
  tracker.ingest(&detection(start + 16_666_667, 0.0));
  assert!(
    tracker
      .snapshot(start + 16_666_667)
      .ball
      .unwrap()
      .position_m[0]
      .abs()
      < 0.01
  );
  for i in 2..6 {
    tracker.ingest(&detection(start + i * 16_666_667, 4.0));
  }
  let world = tracker.snapshot(start + 5 * 16_666_667);
  assert!((world.ball.unwrap().position_m[0] - 4.0).abs() < 0.01);
  let robot = &world.robots[&(TeamColor::Blue as u8, 0)];
  assert!((robot.position_m[0] - 4.0).abs() < 0.01);
  assert!((robot.orientation_rad.unwrap() - 0.3).abs() < 1e-12);
  // An expired track is replaced by the next detection even after a long outage.
  let at = start + 700_000_000;
  tracker.ingest(&detection(at, -4.0));
  assert_eq!(tracker.snapshot(at).ball.unwrap().position_m[0], -4.0);
  tracker.ingest(&detection(start, 3.0)); // late frames cannot undo reacquisition
  assert_eq!(tracker.snapshot(at).ball.unwrap().position_m[0], -4.0);
}
fn sim(config: RealisticConfig) -> RealisticWorld {
  let mut base = WorldConfig::division_b();
  base.robots_per_team = 1;
  RealisticWorld::new(0, base, config).unwrap()
}
#[test]
fn vision_outage_expires_all_audiences_and_respects_processing_delay() {
  let mut cfg = RealisticConfig::default();
  cfg.external_vision = true;
  cfg.tracker.processing_ns = 50_000_000;
  let mut world = sim(cfg);
  world.inject_vision(detection(0, 0.0)).unwrap();
  world.advance_to(49_000_000).unwrap();
  assert!(world.official_observations().ball.is_none());
  world.advance_to(50_000_000).unwrap();
  assert!(world.official_observations().ball.is_some());
  for color in [TeamColor::Blue, TeamColor::Yellow] {
    assert!(world.team_observations(color).ball.is_some());
  }
  world.advance_to(200_000_000).unwrap();
  assert!(world.official_observations().ball.is_some());
  world.advance_to(200_000_001).unwrap();
  assert!(world.official_observations().ball.is_none());
  world.advance_to(700_000_000).unwrap();
  for observed in [
    world.official_observations(),
    world.team_observations(TeamColor::Blue),
    world.team_observations(TeamColor::Yellow),
  ] {
    assert_eq!(observed.at, 700_000_000);
    assert!(observed.ball.is_none());
    assert!(observed.robots.is_empty());
    assert!(
      observed
        .legacy_state(0, 0, &WorldConfig::division_b())
        .is_none()
    );
  }
}
#[test]
fn completed_observations_keep_predicting_between_frames() {
  let mut cfg = RealisticConfig::default();
  cfg.external_vision = true;
  let mut world = sim(cfg);
  for i in 0..20 {
    let at = i * 20_000_000;
    world.advance_to(at).unwrap();
    world.inject_vision(detection(at, i as f64 * 0.02)).unwrap();
    world.advance_to(at + 1_000_000).unwrap();
  }
  let before = world.official_observations().ball.clone().unwrap();
  world.advance_to(431_000_000).unwrap();
  let after = world.official_observations().ball.as_ref().unwrap();
  assert_eq!(before.last_observed_at, after.last_observed_at);
  assert!(
    (after.position_m[0] - before.position_m[0] - before.velocity_m_s[0] * 0.05).abs() < 1e-8
  );
}

#[test]
fn onboard_ir_reacts_locally_after_one_radio_delivery_and_stops_at_expiry() {
  use simhark::realistic::{controller::RobotController, sensors::Observations, transport::Safety};
  use simhark::{MoveCommand, RobotCommand, TeleportBall, TeleportRobot, WorldCommand};
  use std::sync::{Arc, Mutex};
  #[derive(Default)]
  struct Seen {
    first: Option<u64>,
    fired: Option<u64>,
    last: u64,
    samples: u32,
  }
  struct Local(Arc<Mutex<Seen>>);
  impl RobotController for Local {
    fn onboard_command(
      &mut self,
      now: u64,
      payload: &[u8],
      observations: &Observations,
      ready: bool,
    ) -> Option<RobotCommand> {
      assert_eq!(payload, b"kick when IR detects ball");
      let mut seen = self.0.lock().unwrap();
      seen.first.get_or_insert(now);
      seen.last = now;
      seen.samples += 1;
      let ir = observations.infrared.as_ref().is_some_and(|ir| ir.detected);
      let kick = ir && ready && seen.fired.is_none();
      if kick {
        seen.fired = Some(now);
      }
      Some(RobotCommand {
        kick_speed: if kick { 3.5 } else { 0.0 },
        ..RobotCommand::noop(0)
      })
    }
    fn tick(
      &mut self,
      _: u64,
      command: Option<&RobotCommand>,
      _: &Observations,
    ) -> [Option<f64>; 4] {
      assert!(!command.is_some_and(|c| matches!(c.move_command, Some(MoveCommand::Onboard(_)))));
      [None; 4]
    }
  }
  let mut cfg = RealisticConfig::default();
  cfg.transport.wifi_delay_trace_ns = vec![40_000_000];
  cfg.transport.uwb_host_delay_trace_ns = vec![40_000_000];
  cfg.command_ttl_ns = 150_000_000;
  cfg.watchdog_ns = 150_000_000;
  let mut world = sim(cfg);
  let seen = Arc::new(Mutex::new(Seen::default()));
  world
    .set_controller(TeamColor::Blue, 0, Box::new(Local(seen.clone())))
    .unwrap();
  world.teleport(&WorldCommand {
    teleport_robots: vec![TeleportRobot {
      id: 0,
      team: TeamColor::Blue,
      x: Some(0.0),
      y: Some(0.0),
      orientation: Some(0.0),
      vx: Some(0.0),
      vy: Some(0.0),
      v_angular: Some(0.0),
      present: Some(true),
    }],
    teleport_ball: Some(TeleportBall {
      x: Some(2.0),
      y: Some(0.0),
      z: Some(0.0215),
      vx: Some(0.0),
      vy: Some(0.0),
      vz: Some(0.0),
    }),
    ..Default::default()
  });
  world
    .publish(
      WorldCommand {
        blue: vec![RobotCommand {
          move_command: Some(MoveCommand::Onboard(b"kick when IR detects ball".to_vec())),
          ..RobotCommand::noop(0)
        }],
        ..Default::default()
      },
      0,
      1,
      Safety::Run,
    )
    .unwrap();
  world.advance_to(39_000_000).unwrap();
  assert!(seen.lock().unwrap().first.is_none());
  world.advance_to(60_000_000).unwrap();
  assert!(seen.lock().unwrap().first.unwrap() >= 40_000_000);
  assert!(seen.lock().unwrap().fired.is_none());
  world.teleport(&WorldCommand {
    teleport_ball: Some(TeleportBall {
      x: Some(0.095),
      y: Some(0.0),
      z: Some(0.0215),
      vx: Some(0.0),
      vy: Some(0.0),
      vz: Some(0.0),
    }),
    ..Default::default()
  });
  world.advance_to(70_000_000).unwrap();
  let fired = seen
    .lock()
    .unwrap()
    .fired
    .expect("local IR should trigger kick");
  assert!((60_000_000..70_000_000).contains(&fired));
  assert_eq!(world.diagnostics().actions_consumed, 1);
  world.advance_to(170_000_000).unwrap();
  let count = seen.lock().unwrap().samples;
  assert!(seen.lock().unwrap().last < 150_000_000);
  world.advance_to(200_000_000).unwrap();
  assert_eq!(seen.lock().unwrap().samples, count);
}
