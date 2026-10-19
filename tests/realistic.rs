#![cfg(feature = "realistic")]
use simhark::{
  MoveCommand, RobotCommand, TeamColor, WorldCommand, WorldConfig,
  realistic::{
    RealisticConfig, RealisticWorld,
    config::{MotorConfig, RadioSlot, SlotPurpose},
    controller::RobotController,
    hardware::{DriverState, MotorState},
    sensors::Observations,
    time::{Periodic, SECOND, Timeline},
    transport::{CommandPacket, Receiver, Rejection, Safety},
  },
};

fn command() -> RobotCommand {
  RobotCommand {
    move_command: Some(MoveCommand::LocalVelocity {
      forward: 1.0,
      left: 0.0,
      angular: 0.0,
    }),
    ..RobotCommand::noop(0)
  }
}
fn commands() -> WorldCommand {
  WorldCommand {
    blue: vec![command()],
    ..WorldCommand::default()
  }
}
fn world(config: RealisticConfig) -> RealisticWorld {
  let mut base = WorldConfig::division_b();
  base.robots_per_team = 1;
  RealisticWorld::new(0, base, config).unwrap()
}
fn packet(sequence: u64) -> CommandPacket {
  CommandPacket {
    session: 1,
    sequence,
    decision_at: 0,
    published_at: 0,
    expires_at: 50_000_000,
    safety_epoch: 1,
    safety: Safety::Run,
    action_id: None,
    command: command(),
  }
}

#[test]
fn simultaneous_events_are_stable_and_nonintegral_periods_do_not_drift() {
  let mut time = Timeline::default();
  time.schedule(5, "first");
  time.schedule(5, "second");
  time.schedule(3, "earlier");
  assert_eq!(time.pop_until(5), Some((3, "earlier")));
  assert_eq!(time.pop_until(5), Some((5, "first")));
  assert_eq!(time.pop_until(5), Some((5, "second")));
  assert_eq!(time.pop_until(8), None);
  assert_eq!(time.now(), 8);
  for rate in [1000, 1600, 1920] {
    let mut clock = Periodic::new(rate, 17);
    for _ in 0..rate {
      clock.next_time();
    }
    assert_eq!(clock.next_time(), SECOND + 17);
  }
}

#[test]
fn redundant_copies_cannot_regress_sequence_or_refresh_watchdog() {
  let mut receiver = Receiver::new(1, 10);
  receiver.accept(packet(2), 1).unwrap();
  assert_eq!(
    receiver.accept(packet(1), 2),
    Err(Rejection::DuplicateOrOlder)
  );
  assert_eq!(
    receiver.accept(packet(2), 9),
    Err(Rejection::DuplicateOrOlder)
  );
  assert_eq!(receiver.active(10).unwrap().sequence, 2);
  assert!(receiver.active(11).is_none());
}
#[test]
fn safety_is_ordered_independently_of_motion_sequence() {
  let mut receiver = Receiver::new(1, 100);
  receiver.accept(packet(3), 0).unwrap();
  let mut halt = packet(2);
  halt.safety_epoch = 2;
  halt.safety = Safety::Halt;
  assert_eq!(receiver.accept(halt, 1), Err(Rejection::DuplicateOrOlder));
  assert!(receiver.active(1).is_none());
  assert_eq!(
    receiver.accept(packet(4), 2),
    Err(Rejection::SafetyRegression)
  );
  let mut resume = packet(5);
  resume.safety_epoch = 3;
  receiver.accept(resume, 3).unwrap();
  assert!(receiver.active(3).is_some());
}
#[test]
fn kick_action_is_at_most_once_across_copies_and_republication() {
  let mut receiver = Receiver::new(1, 100);
  let mut kick = packet(1);
  kick.action_id = Some(7);
  kick.command.kick_speed = 3.0;
  receiver.accept(kick.clone(), 0).unwrap();
  assert!(receiver.take_kick(0).is_some());
  assert!(receiver.take_kick(0).is_none());
  assert_eq!(
    receiver.accept(kick.clone(), 1),
    Err(Rejection::DuplicateOrOlder)
  );
  kick.sequence = 2;
  receiver.accept(kick.clone(), 2).unwrap();
  assert!(receiver.take_kick(2).is_none());
  kick.sequence = 3;
  kick.action_id = Some(8);
  receiver.accept(kick, 3).unwrap();
  assert!(receiver.take_kick(3).is_some());
}
#[test]
fn old_sessions_expired_and_future_commands_are_rejected() {
  let mut receiver = Receiver::new(1, SECOND);
  assert_eq!(
    receiver.accept(packet(1), 50_000_000),
    Err(Rejection::Expired)
  );
  let mut future = packet(1);
  future.published_at = 10;
  assert_eq!(receiver.accept(future, 0), Err(Rejection::FutureTimestamp));
  receiver.accept(packet(1), 0).unwrap();
  receiver.begin_session(2);
  assert!(receiver.active(0).is_none());
  assert_eq!(
    receiver.accept(packet(100), 1),
    Err(Rejection::WrongSession)
  );
}
#[test]
fn profile_roundtrip_and_invalid_airtime_rates_and_noise_are_rejected() {
  let default = RealisticConfig::default();
  RealisticConfig::from_toml(&default.to_toml().unwrap()).unwrap();
  let mut config = default.clone();
  config.transport.slots.push(RadioSlot {
    every_superframes: 1,
    phase_superframe: 0,
    offset_ns: 10,
    occupied_ns: 500,
    purpose: SlotPurpose::Ranging,
  });
  assert!(config.validate().unwrap_err().contains("overlap"));
  config = default.clone();
  config.transport.slots[0].occupied_ns = SECOND;
  assert!(config.validate().is_err());
  config = default.clone();
  config.imu.rate_hz = 0;
  assert!(config.validate().is_err());
  config = default.clone();
  config.motor.inductance_h = f64::NAN;
  assert!(config.validate().is_err());
  config = default.clone();
  config.imu.gyro_noise_density_rad_s_sqrt_hz = -0.1;
  assert!(config.validate().is_err());
  config = default;
  config.transport.common_delay_trace_ns = vec![];
  assert!(config.validate().is_err());
  assert!(RealisticConfig::from_toml("unknown_key = 1").is_err());
}
#[test]
fn reversal_has_back_emf_current_and_fault_requires_explicit_restart() {
  let config = MotorConfig {
    trip_current_a: 1.0,
    trip_exposure_ns: 0,
    ..MotorConfig::default()
  };
  let mut motor = MotorState::default();
  motor.speed_rad_s = 100.0;
  motor.step(Some(-12.0), 0.0, 1_000_000, 0, &config);
  assert_eq!(motor.driver, DriverState::FaultLatched);
  assert_eq!(motor.trips, 1);
  motor.step(Some(0.0), 0.0, 1_000_000, SECOND, &config);
  assert_eq!(motor.driver, DriverState::FaultLatched);
  motor.request_reset(SECOND, &config);
  motor.step(None, 0.0, 1_000_000, SECOND, &config);
  assert!(matches!(motor.driver, DriverState::Resetting { .. }));
  motor.step(
    None,
    0.0,
    1_000_000,
    SECOND + config.reset_duration_ns,
    &config,
  );
  assert_eq!(motor.driver, DriverState::Enabled);
  let no_trip = MotorConfig {
    trip_current_a: 100.0,
    ..config
  };
  let mut stationary = MotorState::default();
  let mut spinning = MotorState::default();
  spinning.speed_rad_s = 100.0;
  stationary.step(Some(-12.0), 0.0, 250_000, 0, &no_trip);
  spinning.step(Some(-12.0), 0.0, 250_000, 0, &no_trip);
  assert!(spinning.current_a.abs() > stationary.current_a.abs());
}
#[test]
fn total_radio_outage_advances_time_without_motion() {
  let mut config = RealisticConfig::default();
  config.transport.wifi_delay_trace_ns = vec![-1];
  config.transport.uwb_host_delay_trace_ns = vec![-1];
  let mut sim = world(config);
  let initial = sim.truth().blue_robots[0].x;
  sim.publish(commands(), 0, 1, Safety::Run).unwrap();
  sim.advance_to(100_000_000).unwrap();
  assert_eq!(sim.now(), 100_000_000);
  assert_eq!(sim.diagnostics().accepted, 0);
  assert_eq!(sim.truth().blue_robots[0].x, initial);
  assert!(sim.diagnostics().dropped > 0);
}
#[test]
fn duplicates_and_reordering_are_observable_and_expiry_coasts() {
  let mut sim = world(RealisticConfig::default());
  sim.enable_recording();
  sim.publish(commands(), 0, 1, Safety::Run).unwrap();
  sim.advance_to(20_000_000).unwrap();
  assert_eq!(sim.diagnostics().accepted, 1);
  assert_eq!(sim.diagnostics().rejected, 1);
  assert!(
    sim.robot_diagnostics()[0]
      .motors
      .iter()
      .any(|m| m.speed_rad_s.abs() > 0.0)
  );
  sim.advance_to(60_000_000).unwrap();
  let recording = sim.recording("test".into());
  let expired = recording.events.iter().filter_map(|e| {
    if let simhark::realistic::TraceEvent::Control {
      at,
      team: TeamColor::Blue,
      voltage_v,
      ..
    } = e
    {
      (*at >= 50_000_000).then_some(voltage_v)
    } else {
      None
    }
  });
  for voltage in expired {
    assert_eq!(*voltage, [None; 4]);
  }
}
#[test]
fn uwb_slot_wait_and_latest_publication_replacement_are_real() {
  let mut config = RealisticConfig::default();
  config.transport.wifi_delay_trace_ns = vec![-1];
  let mut sim = world(config);
  sim.advance_to(10).unwrap();
  sim.publish(commands(), 10, 1, Safety::Run).unwrap();
  sim.advance_to(20).unwrap();
  sim.publish(commands(), 20, 1, Safety::Run).unwrap();
  sim.advance_to(1_299_999).unwrap();
  assert_eq!(sim.diagnostics().accepted, 0);
  sim.advance_to(1_300_000).unwrap();
  assert_eq!(sim.diagnostics().accepted, 1);
  assert_eq!(sim.diagnostics().coalesced, 1);
}
#[test]
fn common_stall_affects_both_radios_and_cannot_resurrect_expired_motion() {
  let mut config = RealisticConfig::default();
  config.transport.common_delay_trace_ns = vec![60_000_000];
  let mut sim = world(config);
  sim.publish(commands(), 0, 1, Safety::Run).unwrap();
  sim.advance_to(70_000_000).unwrap();
  assert_eq!(sim.diagnostics().accepted, 0);
  assert_eq!(sim.diagnostics().rejected, 2);
  assert!(
    sim.robot_diagnostics()[0]
      .motors
      .iter()
      .all(|m| m.speed_rad_s == 0.0)
  );
}

struct CheckObservations {
  seen: std::sync::Arc<std::sync::atomic::AtomicU64>,
}
impl RobotController for CheckObservations {
  fn tick(
    &mut self,
    now: u64,
    _command: Option<&RobotCommand>,
    observations: &Observations,
  ) -> [Option<f64>; 4] {
    assert!(observations.delivered_at <= now);
    for sample in &observations.imu {
      assert!(sample.physical_state_at <= sample.data_ready_at);
      assert!(sample.window_start_at <= sample.data_ready_at);
      assert!(sample.data_ready_at + 200_000 <= now);
      self.seen.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    }
    if let Some(v) = &observations.vision {
      assert!(v.captured_at + 15_000_000 <= now);
    }
    if let Some(e) = &observations.encoder {
      assert!(e.sampled_at + 200_000 <= now);
    }
    [Some(12.0); 4]
  }
}
#[test]
fn controller_sees_only_available_fifo_samples_and_receiver_gates_output() {
  let mut sim = world(RealisticConfig::default());
  let seen = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(0));
  sim
    .set_controller(
      TeamColor::Blue,
      0,
      Box::new(CheckObservations { seen: seen.clone() }),
    )
    .unwrap();
  sim.advance_to(30_000_000).unwrap();
  assert!(seen.load(std::sync::atomic::Ordering::Relaxed) > 30);
  assert!(
    sim.robot_diagnostics()[0]
      .motors
      .iter()
      .all(|m| m.speed_rad_s == 0.0)
  );
}
#[test]
fn identical_runs_replay_bitwise_even_when_advancement_is_partitioned() {
  let mut a = world(RealisticConfig::default());
  let mut b = world(RealisticConfig::default());
  a.enable_recording();
  b.enable_recording();
  a.publish(commands(), 0, 1, Safety::Run).unwrap();
  b.publish(commands(), 0, 1, Safety::Run).unwrap();
  a.advance_to(100_000_000).unwrap();
  for end in [123, 333_333, 20_555_555, 55_000_000, 100_000_000] {
    b.advance_to(end).unwrap();
  }
  assert_eq!(
    bincode::serialize(&a.truth()).unwrap(),
    bincode::serialize(&b.truth()).unwrap()
  );
  let left = a.recording("test".into());
  let right = b.recording("test".into());
  assert!(
    serde_json::to_vec(&left.events).unwrap() == serde_json::to_vec(&right.events).unwrap(),
    "event traces differ"
  );
  assert!(
    serde_json::to_vec(&left.diagnostics).unwrap()
      == serde_json::to_vec(&right.diagnostics).unwrap(),
    "diagnostics differ"
  );
}
#[test]
fn physics_step_halving_converges_for_straight_acceleration() {
  fn run(step_ns: u64) -> (f64, f64) {
    let mut config = RealisticConfig::default();
    config.physics_step_ns = step_ns;
    config.imu.gyro_noise_density_rad_s_sqrt_hz = 0.0;
    config.imu.accel_noise_density_m_s2_sqrt_hz = 0.0;
    let mut sim = world(config);
    for _ in 0..100 {
      let now = sim.now();
      sim.publish(commands(), now, 1, Safety::Run).unwrap();
      sim.advance_to(now + 1_000_000).unwrap();
    }
    (sim.truth().blue_robots[0].vx, sim.truth().blue_robots[0].x)
  }
  let coarse = run(250_000);
  let fine = run(125_000);
  assert!(
    (coarse.0 - fine.0).abs() < 0.03,
    "velocity coarse={coarse:?} fine={fine:?}"
  );
  assert!(
    (coarse.1 - fine.1).abs() < 0.002,
    "position coarse={coarse:?} fine={fine:?}"
  );
}
#[test]
fn setup_and_team_commands_have_separate_interfaces() {
  let mut sim = world(RealisticConfig::default());
  let mut bad = commands();
  bad.teleport_ball = Some(simhark::TeleportBall {
    x: Some(0.0),
    y: None,
    z: None,
    vx: None,
    vy: None,
    vz: None,
  });
  assert!(sim.publish(bad, 0, 1, Safety::Run).is_err());
  let mut bad = commands();
  bad.blue[0].kick_speed = f64::NAN;
  assert!(sim.publish(bad, 0, 1, Safety::Run).is_err());
  assert!(sim.publish(commands(), 1, 1, Safety::Run).is_err());
}

#[test]
fn recorded_inputs_reproduce_traces_and_final_physics() {
  let mut sim = world(RealisticConfig::default());
  sim.enable_recording();
  for _ in 0..10 {
    sim.step(commands()).unwrap();
  }
  let record = sim.recording("same-build".into());
  let record: simhark::realistic::Recording =
    serde_json::from_slice(&serde_json::to_vec(&record).unwrap()).unwrap();
  let replay = record.replay("same-build").unwrap();
  assert_eq!(
    bincode::serialize(&sim.truth()).unwrap(),
    bincode::serialize(&replay.truth()).unwrap()
  );
  assert_eq!(
    serde_json::to_vec(&record).unwrap(),
    serde_json::to_vec(&replay.recording("same-build".into())).unwrap()
  );
  assert!(record.replay("different-build").is_err());
}

struct ConstantVoltage;
impl RobotController for ConstantVoltage {
  fn tick(&mut self, _: u64, _: Option<&RobotCommand>, _: &Observations) -> [Option<f64>; 4] {
    [Some(12.0); 4]
  }
}
#[test]
fn unaligned_controller_updates_cannot_actuate_in_the_past() {
  let mut config = RealisticConfig::default();
  config.physics_step_ns = 500_000;
  config.control_rate_hz = 1600;
  config.transport.wifi_delay_trace_ns = vec![0];
  config.transport.uwb_host_delay_trace_ns = vec![-1];
  let mut sim = world(config);
  sim
    .set_controller(TeamColor::Blue, 0, Box::new(ConstantVoltage))
    .unwrap();
  sim.publish(commands(), 0, 1, Safety::Run).unwrap();
  sim.advance_to(1_000_000).unwrap();
  // Control tick at 625 us may first power the interval starting at 1 ms.
  assert!(
    sim.robot_diagnostics()[0]
      .motors
      .iter()
      .all(|m| m.speed_rad_s == 0.0)
  );
  sim.advance_to(1_500_000).unwrap();
  assert!(
    sim.robot_diagnostics()[0]
      .motors
      .iter()
      .all(|m| m.speed_rad_s > 0.0)
  );
  assert_eq!(sim.diagnostics().max_actuation_quantization_ns, 375_000);
}

#[test]
fn logical_frames_advance_without_the_frame_adapter() {
  let mut sim = world(RealisticConfig::default());
  sim.advance_to(SECOND).unwrap();
  assert_eq!(sim.truth().frame, 60);
  sim.step(commands()).unwrap();
  assert_eq!(sim.truth().frame, 61);
}

#[test]
fn radio_capacity_is_checked_against_team_size() {
  let config = RealisticConfig::default();
  let base = WorldConfig::division_a();
  RealisticWorld::new(0, base.clone(), config.clone()).unwrap();
  let mut oversized = base;
  oversized.robots_per_team = 16;
  assert!(
    RealisticWorld::new(0, oversized, config)
      .err()
      .unwrap()
      .contains("airtime")
  );
}
