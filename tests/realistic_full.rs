#![cfg(feature = "realistic")]
use simhark::{
  RobotCommand, TeamColor, WorldConfig,
  realistic::{
    RealisticConfig, RealisticWorld,
    clocks::{ClockMapper, DeviceClock},
    controller::RobotController,
    network::{DelayDistribution, LinkConfig, QueuePolicy, QueuedPacket, SharedLink, TrafficClass},
    profile::{ClockConfig, ImuStream},
    sensors::{ImuSample, Observations},
    vision::{CameraConfig, CameraModel},
  },
};
use std::sync::{Arc, Mutex};
fn world(config: RealisticConfig) -> RealisticWorld {
  let mut base = WorldConfig::division_b();
  base.robots_per_team = 1;
  RealisticWorld::new(0, base, config).unwrap()
}
struct Collect(Arc<Mutex<Vec<ImuSample>>>);
impl RobotController for Collect {
  fn tick(
    &mut self,
    now: u64,
    _: Option<&RobotCommand>,
    observations: &Observations,
  ) -> [Option<f64>; 4] {
    for sample in &observations.imu {
      assert!(sample.data_ready_at <= now);
      if let Some(t) = &sample.timing {
        assert!(t.available_at <= now);
      }
    }
    self
      .0
      .lock()
      .unwrap()
      .extend(observations.imu.iter().cloned());
    [None; 4]
  }
}

#[test]
fn dual_streams_are_native_rate_and_driver_timestamps_are_causal() {
  let mut sim = world(RealisticConfig::full());
  let samples = Arc::new(Mutex::new(Vec::new()));
  sim
    .set_controller(TeamColor::Blue, 0, Box::new(Collect(samples.clone())))
    .unwrap();
  sim.advance_to(1_000_000_000).unwrap();
  let samples = samples.lock().unwrap();
  for (stream, min) in [
    (ImuStream::X16, 1850),
    (ImuStream::BmiGyro, 950),
    (ImuStream::BmiAccel, 1550),
  ] {
    let data: Vec<_> = samples.iter().filter(|s| s.stream == stream).collect();
    assert!(data.len() > min, "{stream:?}: {}", data.len());
    assert!(
      data
        .windows(2)
        .all(|p| p[0].data_ready_at < p[1].data_ready_at)
    );
  }
}
#[test]
fn native_clock_mapping_handles_offset_skew_wrap_and_reset_without_truth_mapping() {
  let mut clock = DeviceClock::new(ClockConfig {
    tick_hz: 1_000_000,
    counter_bits: 16,
    offset_ticks: 65000,
    skew_ppm: 1000.0,
    ..Default::default()
  });
  let mut mapping = ClockMapper::new(1_000_000, 16);
  for step in 0..100_u64 {
    let at = step * 1_000_000;
    let estimate = mapping
      .observe(clock.counter(at), clock.epoch(), at + 1000, 2000)
      .unwrap();
    if step > 30 {
      assert!(estimate.common_time_ns.abs_diff(at) <= estimate.uncertainty_ns);
    }
  }
  clock.reset(100_000_000);
  let estimate = mapping
    .observe(clock.counter(101_000_000), clock.epoch(), 101_001_000, 2000)
    .unwrap();
  assert!(estimate.common_time_ns.abs_diff(101_000_000) <= estimate.uncertainty_ns);
}
#[test]
fn free_chassis_settles_on_compliant_supports_and_can_lift_wheels() {
  let mut sim = world(RealisticConfig::full());
  sim.advance_to(300_000_000).unwrap();
  let state = sim.robot_diagnostics();
  let normal: f64 = state[0].contacts.iter().map(|c| c.normal_load_n).sum();
  assert!((normal - 2.02 * 9.81).abs() < 1.0, "support load {normal}");
  assert!(sim.truth().blue_robots[0].vz.abs() < 0.01);
  sim
    .set_attitude(TeamColor::Blue, 0, [0.5, 0.0, 0.0], Some(0.14))
    .unwrap();
  sim.advance_to(301_000_000).unwrap();
  assert!(sim.robot_diagnostics()[0].contacts.iter().any(|c| c.lifted));
  assert!(sim.robot_diagnostics()[0].attitude_xyzw[0].abs() > 0.1);
}
#[test]
fn shared_airtime_is_bounded_and_latency_does_not_hold_the_radio() {
  let mut config = LinkConfig {
    bits_per_second: 1_000_000,
    overhead_bytes: 0,
    turnaround_ns: 0,
    capacity_bytes: 1000,
    latency: DelayDistribution::Constant {
      delay_ns: 50_000_000,
    },
    policy: QueuePolicy::Latest,
    ..Default::default()
  };
  let mut link = SharedLink::new(config.clone());
  let packet = |flow, payload| QueuedPacket {
    flow,
    bytes: 100,
    class: TrafficClass::Command,
    payload,
  };
  link.enqueue(packet(1, 1));
  link.enqueue(packet(1, 2));
  assert_eq!(link.diagnostics.coalesced, 1);
  let mut rng = rand_chacha::ChaCha8Rng::seed_from_u64(42);
  let tx = link.start(0, &mut rng).unwrap();
  assert_eq!(tx.payload, 2);
  assert_eq!(tx.finishes_at, 800_000);
  assert_eq!(tx.available_at, 50_800_000);
  link.enqueue(packet(2, 3));
  assert!(link.start(1, &mut rng).is_none());
  link.complete();
  assert_eq!(
    link.start(800_000, &mut rng).unwrap().finishes_at,
    1_600_000
  );
  config.capacity_bytes = 50;
  let mut full = SharedLink::new(config);
  assert!(!full.enqueue(packet(1, 0)));
}
#[test]
fn camera_projection_roundtrips_and_tracker_is_not_populated_without_detections() {
  let camera = CameraModel::new(
    CameraConfig {
      pixel_noise_std: 0.0,
      ..Default::default()
    },
    42,
    0,
  );
  let pixel = camera.project([1.0, 1.0, 0.0]).unwrap();
  let point = camera.unproject(pixel, 0.0).unwrap();
  assert!((point[0] - 1.0).abs() < 1e-10 && (point[1] - 1.0).abs() < 1e-10);
  let mut config = RealisticConfig::full();
  for camera in &mut config.cameras {
    camera.blue_enabled = false;
  }
  let mut sim = world(config);
  sim.advance_to(200_000_000).unwrap();
  assert!(sim.team_observations(TeamColor::Blue).robots.is_empty());
  assert!(!sim.official_observations().robots.is_empty());
  assert!(
    sim
      .observations(TeamColor::Blue, 0)
      .unwrap()
      .vision
      .is_none()
  );
}
use rand::SeedableRng;

#[test]
fn processor_keeps_running_snapshot_and_coalesces_waiting_jobs() {
  use simhark::realistic::processing::{ProcessingConfig, ProcessingResource};
  let mut processor = ProcessingResource::new(ProcessingConfig::default());
  let mut rng = rand_chacha::ChaCha8Rng::seed_from_u64(1);
  processor.submit(0, 100, 1);
  let first = processor.start(0, &mut rng).unwrap();
  processor.submit(10, 100, 2);
  processor.submit(20, 100, 3);
  assert!(processor.start(20, &mut rng).is_none());
  assert_eq!(first.input, 1);
  assert_eq!(first.completes_at, 100);
  processor.complete();
  let next = processor.start(100, &mut rng).unwrap();
  assert_eq!(next.input, 3);
  assert_eq!(next.completes_at, 200);
  assert_eq!(processor.diagnostics.coalesced, 1);
  assert_eq!(processor.diagnostics.max_wait_ns, 80);
}

#[test]
fn camera_overload_drops_stale_work_and_publishes_after_completion() {
  let mut config = RealisticConfig::full();
  config.cameras.truncate(1);
  config.cameras[0].processing_ns = 100_000_000;
  let mut sim = world(config);
  sim.advance_to(99_000_000).unwrap();
  assert!(sim.delivered_camera_frames().is_empty());
  sim.advance_to(350_000_000).unwrap();
  let (camera, _) = sim.processing_diagnostics();
  assert!(camera[0].coalesced > 10);
  assert_eq!(camera[0].completed, 3);
  assert!(sim.delivered_camera_frames()[0].sent_at >= 303_000_000);
}

#[test]
fn radio_exchanges_produce_fine_native_ranges_and_observation_only_pose() {
  use simhark::realistic::localization::RangingProtocol;
  for protocol in [RangingProtocol::SingleSided, RangingProtocol::DoubleSided] {
    let mut config = RealisticConfig::full();
    config.uwb.protocol = protocol;
    config.uwb.range_noise_std_m = 0.0;
    config.uwb.bearing_noise_std_rad = 0.0;
    config.uwb.nlos_bias_m = 0.0;
    let mut sim = world(config.clone());
    sim.advance_to(100_000_000).unwrap();
    let r = &sim.truth().blue_robots[0];
    let observations = sim.observations(TeamColor::Blue, 0).unwrap();
    assert_eq!(observations.radio.len(), 4);
    for raw in &observations.radio {
      let anchor = config
        .uwb
        .anchors
        .iter()
        .find(|a| a.id == raw.anchor_id)
        .unwrap();
      let expected = ((r.x - anchor.position_m[0]).powi(2)
        + (r.y - anchor.position_m[1]).powi(2)
        + (r.z - anchor.position_m[2]).powi(2))
      .sqrt();
      assert!(
        (raw.range_m().unwrap() - expected).abs() < 0.03,
        "{protocol:?}: {:?} expected {expected}",
        raw.range_m()
      );
      assert!(raw.timestamps[0] != raw.timestamps[1]);
    }
    let pose = observations.pose.as_ref().unwrap();
    let p = pose.position_m.unwrap();
    assert!((p[0] - r.x).hypot(p[1] - r.y) < 0.05);
    assert!(!pose.ambiguous);
    assert_eq!(sim.diagnostics().radio.collisions, 0);
  }
}

#[test]
fn radio_collisions_and_synchronization_loss_are_observable() {
  use simhark::realistic::radio::MacArbiter;
  let mut mac = MacArbiter::default();
  let a = mac.reserve(0, 100);
  let b = mac.reserve(50, 100);
  assert!(!mac.valid(a) && !mac.valid(b));
  assert_eq!(mac.diagnostics.collisions, 1);
  assert!(!mac.finish(a));
  assert!(!mac.finish(b));
  let c = mac.reserve(150, 1);
  assert!(mac.finish(c));
  let mut config = RealisticConfig::full();
  config.uwb.synchronization_loss_every = Some(1);
  let mut sim = world(config);
  sim.advance_to(50_000_000).unwrap();
  assert!(
    sim
      .observations(TeamColor::Blue, 0)
      .unwrap()
      .radio
      .is_empty()
  );
  assert!(sim.diagnostics().radio.missed_sync > 0);
}

fn setup_at_origin(sim: &mut RealisticWorld, ball_x: f64) {
  sim.teleport(&simhark::WorldCommand {
    teleport_robots: vec![
      simhark::TeleportRobot {
        id: 0,
        team: TeamColor::Blue,
        x: Some(0.0),
        y: Some(0.0),
        orientation: Some(0.0),
        vx: None,
        vy: None,
        v_angular: None,
        present: Some(true),
      },
      simhark::TeleportRobot {
        id: 0,
        team: TeamColor::Yellow,
        x: None,
        y: None,
        orientation: None,
        vx: None,
        vy: None,
        v_angular: None,
        present: Some(false),
      },
    ],
    teleport_ball: Some(simhark::TeleportBall {
      x: Some(ball_x),
      y: Some(0.0),
      z: None,
      vx: Some(0.0),
      vy: Some(0.0),
      vz: Some(0.0),
    }),
    ..Default::default()
  });
}

#[test]
fn lidar_sees_physical_height_and_does_not_get_an_implicit_field_map() {
  use simhark::realistic::scene::LandmarkBox;
  for (height, expect_hit) in [(0.0, true), (0.5, false)] {
    let mut config = RealisticConfig::full();
    config.uwb.enabled = false;
    config.environment.standard_goals = false;
    config.lidar.range_noise_std_m = 0.0;
    config.lidar.divergence_rad = 0.0;
    config.environment.physical = vec![LandmarkBox {
      min_m: [1.0, -0.2, height],
      max_m: [1.2, 0.2, height + 0.2],
      reflectivity: 0.8,
    }];
    let mut sim = world(config);
    setup_at_origin(&mut sim, -2.0);
    sim.advance_to(105_000_000).unwrap();
    let o = sim.observations(TeamColor::Blue, 0).unwrap();
    let scan = o.lidar.as_ref().unwrap();
    assert_eq!(scan.beams.len(), 360);
    assert!(
      scan
        .beams
        .windows(2)
        .all(|p| p[0].sampled_at < p[1].sampled_at)
    );
    if expect_hit {
      assert!((scan.beams[0].range_m.unwrap() - 1.0).abs() < 0.01);
    } else {
      assert!(scan.beams[0].range_m.is_none());
    }
    assert!(o.pose.as_ref().unwrap().position_m.is_none());
  }
}

#[test]
fn local_ball_detection_survives_radio_loss_and_optional_streaming() {
  for streaming in [false, true] {
    let mut config = RealisticConfig::full();
    config.uwb.loss_probability = 1.0;
    config.onboard_camera.streaming = streaming;
    let mut sim = world(config);
    setup_at_origin(&mut sim, 0.5);
    sim.advance_to(100_000_000).unwrap();
    let ball = sim
      .observations(TeamColor::Blue, 0)
      .unwrap()
      .onboard_ball
      .as_ref()
      .unwrap();
    assert!(ball.bearing_rad.abs() < 0.03);
    assert!(ball.range_m > 0.3 && ball.range_m < 0.6);
    if streaming {
      assert!(sim.diagnostics().wifi_link.airtime_ns > 0);
    }
  }
}

#[test]
fn current_limit_and_latched_trip_are_distinct_and_monitored_current_is_explicit() {
  use simhark::realistic::{
    config::{MotorConfig, TripCurrent},
    hardware::{DriverState, MotorState},
  };
  let mut config = MotorConfig {
    current_limit_a: Some(0.5),
    trip_current_a: 1.0,
    ..Default::default()
  };
  let mut motor = MotorState::default();
  for i in 0..100 {
    motor.step(Some(12.0), 0.0, 250_000, i * 250_000, &config);
  }
  assert_eq!(motor.driver, DriverState::CurrentLimited);
  assert!(motor.current_a <= 0.501);
  assert_eq!(motor.trips, 0);
  config.current_limit_a = None;
  config.trip_current_type = TripCurrent::PhasePeak;
  motor.step(Some(-12.0), 0.0, 1_000_000, 25_000_000, &config);
  assert_eq!(motor.driver, DriverState::FaultLatched);
  motor.step(None, 0.0, 1_000_000, 26_000_000, &config);
  assert_eq!(motor.driver, DriverState::FaultLatched);
}

#[test]
fn recording_profile_digest_rejects_tampering_and_observed_maps_serialize() {
  let mut sim = world(RealisticConfig::full());
  sim.enable_recording();
  sim.advance_to(20_000_000).unwrap();
  assert!(serde_json::to_string(sim.official_observations()).is_ok());
  let recording = sim.recording("test".into());
  assert!(recording.replay("test").is_ok());
  let mut altered = recording;
  altered.config.motor.resistance_ohm += 0.1;
  assert!(altered.replay("test").err().unwrap().contains("digest"));
}

fn metadata(id: &str) -> simhark::realistic::calibration_fit::Metadata {
  use simhark::realistic::calibration_fit::*;
  Metadata {
    recording_id: id.into(),
    kind: DatasetKind::Synthetic,
    hardware_id: "synthetic test fixture".into(),
    firmware_id: "test-v1".into(),
    source: "analytic fixture, not robot measurements".into(),
    clock_boundary: "aligned plant input and sensor sample".into(),
    common_clock_verified: true,
  }
}
#[test]
fn motor_calibration_identifies_excited_parameters_and_checks_a_separate_recording() {
  use simhark::realistic::calibration_fit::*;
  let dataset = |id, phase: f64, resistance: f64| MotorDataset {
    schema_version: 1,
    metadata: metadata(id),
    samples: (0..2000)
      .map(|i| {
        let t = i as f64 * 1e-5;
        let a = 200.0 * t + phase;
        let b = 73.0 * t + phase;
        let current = a.sin() + 0.2 * b.cos();
        let derivative = 200.0 * a.cos() - 14.6 * b.sin();
        let speed = 10.0 + (35.0 * t + phase).sin() * 20.0;
        MotorSample {
          at_ns: i * 10_000,
          voltage_v: resistance * current + 0.001 * derivative + 0.025 * speed,
          phase_current_a: current,
          wheel_speed_rad_s: speed,
          enabled: true,
        }
      })
      .collect(),
  };
  let train = dataset("train", 0.0, 2.0);
  let held = dataset("held", 0.7, 2.0);
  let fit = fit_motor(&train, &held, 0.0001).unwrap();
  assert!(fit.passed);
  assert!(!fit.hardware_calibration);
  assert!((fit.resistance_ohm - 2.0).abs() < 1e-4);
  assert!((fit.inductance_h - 0.001).abs() < 1e-7);
  assert!((fit.back_emf_v_per_rad_s - 0.025).abs() < 1e-5);
  assert!(
    !fit_motor(&train, &dataset("different-load", 0.7, 2.4), 0.01)
      .unwrap()
      .passed
  );
  assert!(fit_motor(&train, &train, 0.01).is_err());
  let mut idle = train.clone();
  for s in &mut idle.samples {
    s.phase_current_a = 0.0;
    s.wheel_speed_rad_s = 0.0;
  }
  assert!(fit_motor(&idle, &held, 0.01).is_err());
  let directory = std::env::temp_dir().join(format!("simhark-cal-fit-{}", std::process::id()));
  std::fs::create_dir_all(&directory).unwrap();
  let base = directory.join("base.toml");
  let overlay = directory.join("fit.toml");
  std::fs::write(&base, RealisticConfig::full().to_toml().unwrap()).unwrap();
  std::fs::write(&overlay, fit.profile_overlay_toml).unwrap();
  let resolved = simhark::realistic::profiles::resolve(&[base, overlay]).unwrap();
  assert!((resolved.config.motor.resistance_ohm - 2.0).abs() < 1e-4);
  assert_eq!(resolved.layers.len(), 2);
  std::fs::remove_dir_all(directory).unwrap();
}
#[test]
fn timing_calibration_preserves_joint_copies_and_rejects_unknown_boundaries() {
  use simhark::realistic::calibration_fit::*;
  let dataset = |id| TimingDataset {
    schema_version: 1,
    metadata: metadata(id),
    samples: (0..100)
      .map(|i| {
        let source = i * 100_000_000;
        let common = if i % 10 == 0 { 50_000_000 } else { 0 };
        TimingSample {
          source_ns: source,
          common_ready_ns: Some(source + common),
          wifi_available_ns: Some(source + common + 1_000_000),
          uwb_air_end_ns: Some(source + common + 200_000),
          uwb_available_ns: Some(source + common + 300_000),
        }
      })
      .collect(),
  };
  let train = dataset("train");
  let held = dataset("held");
  let fit = fit_timing(&train, &held, 0.05, 0.01).unwrap();
  assert!(fit.passed && !fit.hardware_calibration);
  assert_eq!(fit.joint_trace[0].common_delay_ns, 50_000_000);
  assert_eq!(fit.joint_trace[0].wifi_post_split_ns, 1_000_000);
  assert_eq!(fit.joint_trace[0].uwb_post_airtime_ns, 100_000);
  let mut invalid = held.clone();
  invalid.samples[0].uwb_air_end_ns = None;
  assert!(fit_timing(&train, &invalid, 0.05, 0.01).is_err());
  let mut changed = held;
  for s in &mut changed.samples {
    s.wifi_available_ns = s.wifi_available_ns.map(|at| at + 20_000_000);
  }
  assert!(!fit_timing(&train, &changed, 0.05, 0.01).unwrap().passed);
}
#[test]
fn sensor_response_calibration_fits_delay_response_scale_and_bias() {
  use simhark::realistic::calibration_fit::*;
  let dataset = |id, phase: f64| {
    let input = |t: f64| (600.0 * t + phase).sin() + 0.3 * (113.0 * t + phase).cos();
    let mut y = input(0.0);
    ResponseDataset {
      schema_version: 1,
      metadata: metadata(id),
      units: "rad/s".into(),
      samples: (0..2000)
        .map(|i| {
          let t = i as f64 * 1e-4;
          let x = input((t - 0.0007).max(0.0));
          y += (x - y) * (1.0 - (-1e-4_f64 / 0.0012).exp());
          ResponseSample {
            at_ns: i * 100_000,
            reference: input(t),
            sensor: y * 1.1 + 0.02,
          }
        })
        .collect(),
    }
  };
  let fit = fit_response(
    &dataset("train", 0.0),
    &dataset("held", 0.7),
    &ResponseSearch {
      max_delay_ns: 1_000_000,
      max_time_constant_ns: 2_000_000,
      grid_step_ns: 100_000,
      held_out_rmse_tolerance: 0.001,
    },
  )
  .unwrap();
  assert!(fit.passed && !fit.hardware_calibration);
  assert_eq!(fit.delay_ns, 700_000);
  assert!((fit.time_constant_s - 0.0012).abs() < 1e-8);
  assert!((fit.scale - 1.1).abs() < 0.001);
  assert!((fit.bias - 0.02).abs() < 0.001);
}
#[test]
fn planner_completes_before_publication_and_repeated_setpoints_keep_one_action_id() {
  let mut config = RealisticConfig::full();
  config.ai_processing_ns = 5_000_000;
  let mut sim = world(config);
  sim.enable_recording();
  let command = simhark::WorldCommand {
    blue: vec![RobotCommand {
      kick_speed: 1.0,
      ..RobotCommand::noop(0)
    }],
    ..Default::default()
  };
  sim
    .submit_decision(command, 0, 1, simhark::realistic::transport::Safety::Run)
    .unwrap();
  sim.advance_to(4_000_000).unwrap();
  assert_eq!(sim.diagnostics().accepted, 0);
  sim.advance_to(50_000_000).unwrap();
  assert!(sim.diagnostics().accepted > 20);
  assert_eq!(sim.diagnostics().actions_consumed, 1);
  let recording = sim.recording("same-build".into());
  let replay = recording.replay("same-build").unwrap();
  assert_eq!(
    serde_json::to_vec(&recording.events).unwrap(),
    serde_json::to_vec(&replay.recording("same-build".into()).events).unwrap()
  );
}
#[test]
fn custom_controller_boundary_can_be_replayed_without_rerunning_the_process() {
  struct Voltage;
  impl RobotController for Voltage {
    fn tick(&mut self, _: u64, _: Option<&RobotCommand>, _: &Observations) -> [Option<f64>; 4] {
      [Some(0.3); 4]
    }
  }
  let mut sim = world(RealisticConfig::full());
  sim.enable_recording();
  sim
    .set_controller(TeamColor::Blue, 0, Box::new(Voltage))
    .unwrap();
  sim
    .publish(
      simhark::WorldCommand {
        blue: vec![RobotCommand::noop(0)],
        ..Default::default()
      },
      0,
      1,
      simhark::realistic::transport::Safety::Run,
    )
    .unwrap();
  sim.advance_to(30_000_000).unwrap();
  let record = sim.recording("native-build".into());
  assert!(record.replay("native-build").is_err());
  let replay = record.replay_recorded_controls("native-build").unwrap();
  assert_eq!(
    serde_json::to_vec(&record.final_truth).unwrap(),
    serde_json::to_vec(&replay.truth()).unwrap()
  );
  assert_eq!(
    serde_json::to_vec(&record.events).unwrap(),
    serde_json::to_vec(&replay.recording("native-build".into()).events).unwrap()
  );
}
#[test]
fn physical_camera_occlusion_uses_configured_venue_objects() {
  let mut config = RealisticConfig::full();
  config.cameras.truncate(1);
  config.cameras[0].position_world_m = [0.0, 0.0, 5.0];
  config
    .environment
    .physical
    .push(simhark::realistic::scene::LandmarkBox {
      min_m: [-0.2, -0.2, 2.0],
      max_m: [0.2, 0.2, 3.0],
      reflectivity: 0.5,
    });
  let mut sim = world(config);
  setup_at_origin(&mut sim, 1.0);
  sim.advance_to(50_000_000).unwrap();
  assert!(
    !sim.delivered_camera_frames()[0]
      .robots
      .iter()
      .any(|r| r.team == TeamColor::Blue)
  );
  assert!(sim.delivered_camera_frames()[0].ball_position_m.is_some());
}
#[test]
fn stereo_ball_height_comes_from_detections_and_preserves_single_camera_ambiguity() {
  use simhark::realistic::vision::VisionTracker;
  let cameras = [
    CameraConfig {
      position_world_m: [-2.0, 0.0, 5.0],
      pixel_noise_std: 0.0,
      ..Default::default()
    },
    CameraConfig {
      id: 1,
      position_world_m: [2.0, 0.0, 5.0],
      pixel_noise_std: 0.0,
      ..Default::default()
    },
  ];
  let mut tracker = VisionTracker::new(Default::default()).with_cameras(&cameras);
  for (index, c) in cameras.iter().enumerate() {
    let model = CameraModel::new(c.clone(), 42, 0);
    let pixel = model.project([0.3, 0.2, 0.7]).unwrap();
    let p = model.unproject(pixel, 0.0215).unwrap();
    tracker.ingest(&simhark::realistic::sensors::VisionFrame {
      camera_id: c.id,
      frame_number: 0,
      camera_capture_ticks: None,
      image_received_at: 0,
      sent_at: 0,
      captured_at: 0,
      physical_state_at: 0,
      robots: vec![],
      ball_position_m: Some([p[0], p[1], 0.0215]),
    });
    if index == 0 {
      assert!(tracker.snapshot(0).ball_height_m.is_none());
    }
  }
  assert!((tracker.snapshot(0).ball_height_m.unwrap() - 0.7).abs() < 1e-8);
}

#[test]
fn streaming_has_distinct_encode_network_and_decode_boundaries() {
  let mut config = RealisticConfig::full();
  config.onboard_camera.streaming = true;
  config.onboard_camera.decode_delay_ns = 20_000_000;
  let mut sim = world(config);
  setup_at_origin(&mut sim, 0.5);
  sim.advance_to(10_000_000).unwrap();
  assert!(sim.host_video(TeamColor::Blue, 0).is_none());
  assert!(
    sim
      .observations(TeamColor::Blue, 0)
      .unwrap()
      .onboard_ball
      .is_some()
  );
  sim.advance_to(30_000_000).unwrap();
  let frame = sim.host_video(TeamColor::Blue, 0).unwrap();
  assert_eq!(frame.encoded_at - frame.captured_at, 2_000_000);
  assert!(frame.received_at > frame.encoded_at);
  assert_eq!(frame.decoded_at - frame.received_at, 20_000_000);
}

#[test]
fn delayed_kick_is_cancelled_by_halt_or_original_expiry() {
  use simhark::{WorldCommand, realistic::transport::Safety};
  fn run(halt: bool, ttl: u64) -> f64 {
    let mut config = RealisticConfig::default();
    config.hardware.auxiliaries.kick_delay_ns = 20_000_000;
    config.command_ttl_ns = ttl;
    config.transport.wifi_delay_trace_ns = vec![0];
    let mut sim = world(config);
    setup_at_origin(&mut sim, 0.10);
    sim
      .publish(
        WorldCommand {
          blue: vec![RobotCommand {
            kick_speed: 3.0,
            ..RobotCommand::noop(0)
          }],
          ..Default::default()
        },
        0,
        1,
        Safety::Run,
      )
      .unwrap();
    sim.advance_to(5_000_000).unwrap();
    assert_eq!(sim.diagnostics().actions_consumed, 1);
    if halt {
      sim
        .publish(
          WorldCommand {
            blue: vec![RobotCommand::noop(0)],
            ..Default::default()
          },
          sim.now(),
          2,
          Safety::Halt,
        )
        .unwrap();
    }
    sim.advance_to(25_000_000).unwrap();
    sim.truth().ball.vx
  }
  assert!(run(false, 100_000_000) > 1.0);
  assert!(run(true, 100_000_000).abs() < 0.1);
  assert!(run(false, 10_000_000).abs() < 0.1);
}

#[test]
fn physical_venue_objects_collide_with_the_ball() {
  let mut config = RealisticConfig::full();
  config.environment.standard_goals = false;
  config
    .environment
    .physical
    .push(simhark::realistic::scene::LandmarkBox {
      min_m: [0.5, -0.5, 0.0],
      max_m: [0.6, 0.5, 0.4],
      reflectivity: 0.5,
    });
  let mut sim = world(config);
  setup_at_origin(&mut sim, 0.2);
  sim.teleport(&simhark::WorldCommand {
    teleport_ball: Some(simhark::TeleportBall {
      x: None,
      y: None,
      z: None,
      vx: Some(2.0),
      vy: Some(0.0),
      vz: Some(0.0),
    }),
    ..Default::default()
  });
  sim.advance_to(250_000_000).unwrap();
  assert!(sim.truth().ball.x < 0.5);
  assert!(sim.truth().ball.vx < 0.0);
}

#[test]
fn power_sag_restarts_and_thermal_protection_latches() {
  use simhark::realistic::{
    hardware::{DriverState, MotorState},
    power::PowerState,
    profile::{AuxiliaryConfig, BatteryConfig},
  };
  let mut config = BatteryConfig::default();
  config.internal_resistance_ohm = 8.0;
  config.restart_ns = 10_000_000;
  let mut state = PowerState::new(&config);
  let mut motors = std::array::from_fn(|_| MotorState::default());
  state.step(
    0,
    0.001,
    &config,
    &AuxiliaryConfig::default(),
    &mut motors,
    [None; 4],
    2.0,
    false,
  );
  assert!(state.brownout);
  assert_eq!(state.brownout_count, 1);
  config.internal_resistance_ohm = 0.01;
  state.step(
    5_000_000,
    0.001,
    &config,
    &AuxiliaryConfig::default(),
    &mut motors,
    [None; 4],
    2.0,
    false,
  );
  assert!(state.brownout);
  state.step(
    11_000_000,
    0.001,
    &config,
    &AuxiliaryConfig::default(),
    &mut motors,
    [None; 4],
    2.0,
    false,
  );
  assert!(!state.brownout);
  config.motor_thermal_capacity_j_k = 0.001;
  motors[0].current_a = 2.0;
  state.step(
    12_000_000,
    0.1,
    &config,
    &AuxiliaryConfig::default(),
    &mut motors,
    [Some(12.0); 4],
    2.0,
    false,
  );
  assert_eq!(motors[0].driver, DriverState::FaultLatched);
}

#[test]
fn magnetic_rotor_field_persists_unpowered_and_changes_with_mount_distance() {
  use rand::SeedableRng;
  use simhark::realistic::{
    hardware::MotorState,
    magnetic,
    profile::{MagneticDipole, MagnetometerConfig},
  };
  let mut config = MagnetometerConfig::default();
  config.noise_std_ut = 0.0;
  config.motor_dipoles.push(MagneticDipole {
    wheel: 0,
    position_body_m: [0.1, 0.0, 0.0],
    current_moment_am2_per_a: [0.0, 0.0, 0.0],
    rotor_moment_am2: [0.0, 0.001, 0.0],
    rotation_axis_body: [0.0, 0.0, 1.0],
  });
  let mut motors = std::array::from_fn(|_| MotorState::default());
  let mut rng = rand_chacha::ChaCha8Rng::seed_from_u64(1);
  let sample = |config: &MagnetometerConfig, motors: &_| {
    magnetic::sample(
      config,
      0,
      nalgebra::UnitQuaternion::identity(),
      [0.0; 3],
      motors,
      0.0,
      false,
      &mut rand_chacha::ChaCha8Rng::seed_from_u64(1),
    )
    .field_ut
  };
  let first = sample(&config, &motors);
  motors[0].angle_rad = std::f64::consts::PI;
  let second = sample(&config, &motors);
  assert!((first[1] - second[1]).abs() > 0.1);
  assert_eq!(motors[0].current_a, 0.0);
  config.position_body_m = [0.05, 0.0, 0.0];
  let near = sample(&config, &motors);
  assert!(near[1].abs() > second[1].abs() * 7.0);
  config.ambient_gradient_ut_per_m[0][0] = 10.0;
  let shifted = magnetic::sample(
    &config,
    0,
    nalgebra::UnitQuaternion::identity(),
    [1.0, 0.0, 0.0],
    &motors,
    0.0,
    false,
    &mut rng,
  );
  assert!((shifted.field_ut[0] - near[0] - 10.5).abs() < 1e-8);
}

#[test]
fn tdoa_and_anchor_bearings_do_not_invent_robot_yaw() {
  use simhark::realistic::localization::{BearingFrame, RangingProtocol};
  let mut config = RealisticConfig::full();
  config.uwb.protocol = RangingProtocol::Tdoa;
  config.uwb.bearing_frame = BearingFrame::AnchorArray;
  config.uwb.range_noise_std_m = 0.0;
  config.uwb.bearing_noise_std_rad = 0.0;
  let mut sim = world(config);
  sim.advance_to(150_000_000).unwrap();
  let observed = sim.observations(TeamColor::Blue, 0).unwrap();
  assert!(observed.radio.iter().any(|o| o.tdoa_reference.is_some()));
  let pose = observed.pose.as_ref().unwrap();
  assert!(pose.position_m.is_some());
  assert!(pose.yaw_rad.is_none());
  assert!(pose.ambiguous);
}

#[test]
fn free_chassis_step_halving_converges_during_drive_and_reversal() {
  fn run(step: u64) -> [f64; 3] {
    let mut config = RealisticConfig::full();
    config.physics_step_ns = step;
    let mut sim = world(config);
    setup_at_origin(&mut sim, 2.0);
    for i in 0..600 {
      let now = sim.now();
      sim
        .publish(
          simhark::WorldCommand {
            blue: vec![RobotCommand {
              move_command: Some(simhark::MoveCommand::LocalVelocity {
                forward: if i < 300 { 0.4 } else { -0.4 },
                left: 0.1,
                angular: 0.2,
              }),
              ..RobotCommand::noop(0)
            }],
            ..Default::default()
          },
          now,
          1,
          simhark::realistic::transport::Safety::Run,
        )
        .unwrap();
      sim.advance_to(now + 1_000_000).unwrap();
    }
    let r = &sim.truth().blue_robots[0];
    [r.x, r.y, r.z]
  }
  let coarse = run(250_000);
  let fine = run(125_000);
  let finer = run(62_500);
  let distance = |a: [f64; 3], b: [f64; 3]| {
    a.iter()
      .zip(b)
      .map(|(x, y)| (x - y).powi(2))
      .sum::<f64>()
      .sqrt()
  };
  assert!(
    distance(coarse, finer) < 0.01,
    "coarse {coarse:?}, fine {fine:?}, finer {finer:?}"
  );
  assert!(distance(fine, finer) < 0.005);
}

#[test]
fn standard_udp_rejects_malformed_input_and_keeps_sensor_outputs_separate() {
  use prost::Message;
  use simhark::{proto, realistic::protocol::StandardServer};
  use std::net::UdpSocket;
  let receiver = UdpSocket::bind("127.0.0.1:0").unwrap();
  receiver
    .set_read_timeout(Some(std::time::Duration::from_millis(100)))
    .unwrap();
  let mut base = WorldConfig::division_b();
  base.robots_per_team = 1;
  let mut server = StandardServer::bind(
    "127.0.0.1".parse().unwrap(),
    0,
    0,
    0,
    receiver.local_addr().unwrap(),
    base,
  )
  .unwrap();
  let sender = UdpSocket::bind("127.0.0.1:0").unwrap();
  let blue = server.addresses().unwrap()[1];
  sender.send_to(&[255, 255], blue).unwrap();
  let command = proto::RobotControl {
    robot_commands: vec![proto::RobotCommand {
      id: 0,
      move_command: Some(proto::RobotMoveCommand {
        command: Some(proto::robot_move_command::Command::LocalVelocity(
          proto::MoveLocalVelocity {
            forward: 0.5,
            left: 0.0,
            angular: 0.0,
          },
        )),
      }),
      kick_speed: None,
      kick_angle: None,
      dribbler_speed: None,
    }],
  };
  sender.send_to(&command.encode_to_vec(), blue).unwrap();
  let mut sim = world(RealisticConfig::full());
  server.poll(&mut sim).unwrap();
  assert_eq!(server.malformed, 1);
  sim.advance_to(20_000_000).unwrap();
  assert!(sim.diagnostics().accepted > 0);
  let frames = sim.delivered_camera_frames();
  assert!(!frames.is_empty());
  server.vision(&frames).unwrap();
  let mut buffer = [0; 65536];
  let mut detected = false;
  for _ in 0..=frames.len() {
    let n = receiver.recv(&mut buffer).unwrap();
    let packet = proto::SslWrapperPacket::decode(&buffer[..n]).unwrap();
    if let Some(frame) = packet.detection {
      detected = true;
      let source = frames
        .iter()
        .find(|f| f.camera_id == frame.camera_id)
        .unwrap();
      assert_eq!(
        frame.robots_blue.len(),
        source
          .robots
          .iter()
          .filter(|r| r.team == TeamColor::Blue)
          .count()
      );
      assert!(frame.t_sent > frame.t_capture);
    }
  }
  assert!(detected);
}

#[test]
fn localization_recovers_from_kidnapping_and_explicit_restart_without_vision() {
  let mut config = RealisticConfig::full();
  config.cameras.iter_mut().for_each(|c| {
    c.blue_enabled = false;
    c.yellow_enabled = false;
  });
  config.uwb.range_noise_std_m = 0.0;
  config.uwb.bearing_noise_std_rad = 0.0;
  let mut sim = world(config);
  sim.advance_to(150_000_000).unwrap();
  let before = sim
    .observations(TeamColor::Blue, 0)
    .unwrap()
    .pose
    .as_ref()
    .unwrap()
    .position_m
    .unwrap();
  sim.teleport(&simhark::WorldCommand {
    teleport_robots: vec![simhark::TeleportRobot {
      id: 0,
      team: TeamColor::Blue,
      x: Some(1.0),
      y: Some(1.0),
      orientation: Some(0.5),
      vx: None,
      vy: None,
      v_angular: None,
      present: None,
    }],
    ..Default::default()
  });
  assert_eq!(
    before,
    sim
      .observations(TeamColor::Blue, 0)
      .unwrap()
      .pose
      .as_ref()
      .unwrap()
      .position_m
      .unwrap()
  );
  sim.advance_to(300_000_000).unwrap();
  let recovered = sim
    .observations(TeamColor::Blue, 0)
    .unwrap()
    .pose
    .as_ref()
    .unwrap();
  let p = recovered.position_m.unwrap();
  assert!((p[0] - 1.0).hypot(p[1] - 1.0) < 0.05);
  sim.reset_localization(TeamColor::Blue, 0, None).unwrap();
  assert!(
    sim
      .observations(TeamColor::Blue, 0)
      .unwrap()
      .pose
      .as_ref()
      .unwrap()
      .position_m
      .is_none()
  );
  sim.advance_to(450_000_000).unwrap();
  let p = sim
    .observations(TeamColor::Blue, 0)
    .unwrap()
    .pose
    .as_ref()
    .unwrap()
    .position_m
    .unwrap();
  assert!((p[0] - 1.0).hypot(p[1] - 1.0) < 0.05);
  assert!(sim.team_observations(TeamColor::Blue).robots.is_empty());
}

#[test]
fn standard_setup_socket_is_separate_from_radio_control_and_validates_before_mutation() {
  use prost::Message;
  use simhark::{proto, realistic::protocol::StandardServer};
  let mut base = WorldConfig::division_b();
  base.robots_per_team = 1;
  let receiver = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
  let mut server = StandardServer::bind(
    "127.0.0.1".parse().unwrap(),
    0,
    0,
    0,
    receiver.local_addr().unwrap(),
    base,
  )
  .unwrap();
  server.bind_setup("127.0.0.1:0".parse().unwrap()).unwrap();
  let sender = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
  let mut sim = world(RealisticConfig::full());
  let packet = |x| {
    proto::SimulatorCommand {
      config: None,
      control: Some(proto::SimulatorControl {
        teleport_ball: Some(proto::TeleportBall {
          x: Some(x),
          y: Some(0.3),
          z: None,
          vx: None,
          vy: None,
          vz: None,
          teleport_safely: None,
          roll: None,
        }),
        teleport_robot: vec![],
        simulation_speed: None,
      }),
    }
    .encode_to_vec()
  };
  sender
    .send_to(&packet(0.7), server.setup_address().unwrap().unwrap())
    .unwrap();
  server.poll(&mut sim).unwrap();
  assert!((sim.truth().ball.x - 0.7).abs() < 1e-6);
  assert_eq!(sim.diagnostics().accepted, 0);
  sender
    .send_to(&packet(f32::NAN), server.setup_address().unwrap().unwrap())
    .unwrap();
  server.poll(&mut sim).unwrap();
  assert!((sim.truth().ball.x - 0.7).abs() < 1e-6);
  assert_eq!(server.malformed, 1);
}

#[test]
fn gravity_tips_a_chassis_whose_com_leaves_the_wheel_support_polygon() {
  fn run(x: f64) -> (f64, usize) {
    let mut config = RealisticConfig::full();
    config.hardware.center_of_mass_body_m = Some([x, 0.0, -0.05]);
    let mut sim = world(config);
    setup_at_origin(&mut sim, 2.0);
    sim.advance_to(500_000_000).unwrap();
    let d = &sim.robot_diagnostics()[0];
    (
      d.attitude_xyzw[0].hypot(d.attitude_xyzw[1]),
      d.contacts.iter().filter(|c| c.lifted).count(),
    )
  }
  let centered = run(0.0);
  let outside = run(0.08);
  assert!(
    outside.0 > centered.0 + 0.05,
    "centered {centered:?}, outside {outside:?}"
  );
  assert!(outside.1 > 0);
}

#[test]
fn layered_profiles_keep_raw_file_digests_inside_recorded_configuration() {
  let directory = std::env::temp_dir().join(format!(
    "simhark-profile-{}-{}",
    std::process::id(),
    std::time::SystemTime::now()
      .duration_since(std::time::UNIX_EPOCH)
      .unwrap()
      .as_nanos()
  ));
  std::fs::create_dir_all(&directory).unwrap();
  let base = directory.join("base.toml");
  let overlay = directory.join("overlay.toml");
  std::fs::write(&base, "schema_version = 1\n").unwrap();
  std::fs::write(&overlay, "[motor]\nresistance_ohm = 3.0\n").unwrap();
  let resolved = simhark::realistic::profiles::resolve(&[base, overlay]).unwrap();
  assert_eq!(
    resolved.layers[0].content_sha256,
    "4e1a53d3a13edf322c7d195c0bb546d0b29b969f74f6427a6edf05fa54ebafef"
  );
  assert_eq!(resolved.config.motor.resistance_ohm, 3.0);
  assert_eq!(resolved.config.profile_layers.len(), 2);
  let mut sim = world(resolved.config);
  sim.enable_recording();
  assert_eq!(
    sim
      .recording("profile-test".into())
      .config
      .profile_layers
      .len(),
    2
  );
  std::fs::remove_dir_all(directory).unwrap();
}
