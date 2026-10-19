//! Runs actual Crashanter fusion against delivered synthetic dual-IMU FIFO samples.
//! Drive control remains an explicitly labeled surrogate. No truth in tick().
use core_dump::vec::types::Vec3;
use crashanter::vimu::{FusionMode, RawImuSample, Vimu, VimuConfig};
use simhark::{
  MoveCommand, RobotCommand, TeamColor, WorldCommand, WorldConfig,
  realistic::{
    RealisticConfig, RealisticWorld,
    controller::{EncoderVelocityController, RobotController},
    profile::ImuStream,
    sensors::Observations,
    time::SECOND,
    transport::Safety,
  },
};
use std::{
  collections::BTreeMap,
  error::Error,
  sync::{Arc, Mutex},
};

#[derive(Default)]
struct FusionMetrics {
  raw_samples: usize,
  rejected: usize,
  calibrated: bool,
  samples: Vec<(u64, f32, u64)>,
  health: BTreeMap<String, usize>,
  peak_gyro_variance: f32,
  recovered_fused: usize,
}
struct FusedController {
  fusion: Vimu,
  drive: EncoderVelocityController,
  metrics: Arc<Mutex<FusionMetrics>>,
  last_yaw_rate: Option<f64>,
  calibration_attempted: bool,
  calibration_started: bool,
}
impl RobotController for FusedController {
  fn tick(
    &mut self,
    now: u64,
    command: Option<&RobotCommand>,
    observations: &Observations,
  ) -> [Option<f64>; 4] {
    let mut metrics = self.metrics.lock().unwrap();
    if !self.calibration_started && now >= 400_000_000 {
      self.fusion.start_calibration();
      self.calibration_started = true;
    }
    for sample in &observations.imu {
      let vec3 = |v: [f64; 3]| Vec3::new(v[0] as f32, v[1] as f32, v[2] as f32);
      let timestamp_us = sample.data_ready_at / 1000;
      let raw = match sample.stream {
        ImuStream::X16 => RawImuSample::X16 {
          timestamp_us,
          angular: vec3(sample.angular_rad_s),
          acc: vec3(sample.specific_force_m_s2),
        },
        ImuStream::BmiGyro => RawImuSample::Bmi088Gyro {
          timestamp_us,
          angular: vec3(sample.angular_rad_s),
        },
        ImuStream::BmiAccel => RawImuSample::Bmi088Acc {
          timestamp_us,
          acc: vec3(sample.specific_force_m_s2),
        },
      };
      let accepted = self.fusion.push(&raw);
      metrics.raw_samples += 1;
      if !accepted {
        metrics.rejected += 1;
      }
    }
    if now >= 800_000_000 && !self.calibration_attempted {
      metrics.calibrated = self.fusion.finish_calibration().is_ok();
      self.calibration_attempted = true;
    }
    while let Some(sample) = self.fusion.pop() {
      let measured_at = sample.timestamp_us * 1000;
      assert!(measured_at <= now, "fusion published a future measurement");
      *metrics
        .health
        .entry(format!("{:?}/{:?}", sample.gyro_health, sample.acc_health))
        .or_default() += 1;
      metrics.peak_gyro_variance = metrics.peak_gyro_variance.max(sample.angular_variance.z);
      if measured_at > 2_200_000_000
        && matches!(sample.gyro_health, crashanter::vimu::ImuHealth::Fused)
      {
        metrics.recovered_fused += 1;
      }
      self.last_yaw_rate = Some(f64::from(sample.angular.z));
      metrics
        .samples
        .push((measured_at, sample.angular.z, now - measured_at));
    }
    drop(metrics);
    let mut adjusted = command.cloned();
    // This controller is a surrogate; the actual production vIMU is the unit
    // under test. Correct yaw rate using only its available fused measurement.
    if let Some(RobotCommand {
      move_command: Some(MoveCommand::LocalVelocity { angular, .. }),
      ..
    }) = &mut adjusted
      && let Some(measured) = self.last_yaw_rate
    {
      *angular += 0.3 * (*angular - measured);
    }
    self.drive.tick(now, adjusted.as_ref(), observations)
  }
}

fn main() -> Result<(), Box<dyn Error>> {
  let scenario = std::env::args().nth(1).unwrap_or_else(|| "healthy".into());
  let scenarios = if scenario == "all" {
    vec![
      "healthy",
      "missing",
      "frozen",
      "clipped",
      "clock-reset",
      "duplicate",
      "reordered",
      "wrong-delay",
    ]
  } else {
    vec![scenario.as_str()]
  };
  let mut results = Vec::new();
  for scenario in scenarios {
    results.push(run(scenario)?);
  }
  println!("{}", serde_json::to_string_pretty(&results)?);
  Ok(())
}
fn run(scenario: &str) -> Result<serde_json::Value, Box<dyn Error>> {
  let mut base = WorldConfig::division_b();
  base.robots_per_team = 1;
  let mut profile = RealisticConfig::full();
  use simhark::realistic::profile::{SensorFault, SensorFaultKind, TimestampSource};
  let kind=match scenario {"healthy"|"wrong-delay"=>None,"missing"=>Some(SensorFaultKind::Missing),"frozen"=>Some(SensorFaultKind::Frozen),"clipped"=>Some(SensorFaultKind::Clipped),"clock-reset"=>Some(SensorFaultKind::ClockReset),"duplicate"=>Some(SensorFaultKind::Duplicate),"reordered"=>Some(SensorFaultKind::Reordered),_=>return Err("scenario must be healthy, missing, frozen, clipped, clock-reset, duplicate, reordered, wrong-delay, or all".into())};
  if let Some(kind) = kind {
    profile.devices.imus[0].faults.push(SensorFault {
      start_ns: 1_200_000_000,
      end_ns: 1_800_000_000,
      kind,
    });
  }
  if scenario == "clock-reset" {
    profile.devices.imus[0].timing.source = TimestampSource::NativeCounter;
    profile.devices.imus[0].timing.clock.counter_bits = 16;
  }
  if scenario == "reordered" {
    profile.devices.imus[0].physical.fifo_watermark = 4;
  }

  // Mounting is centered/aligned in this experiment; timestamps mark data-ready.
  // The 1 ms compensation is a provisional low-frequency approximation of the
  // configured causal response, not a fitted device/filter calibration.
  let mut config = VimuConfig {
    mode: FusionMode::Adaptive,
    bmi088_position: Vec3::new(0.03, 0.0, 0.0),
    ..VimuConfig::default()
  };
  config.x16_gyro.sample_rate = profile.imu.rate_hz as f32;
  config.x16_acc.sample_rate = profile.imu.rate_hz as f32;
  config.x16_gyro.delay_us = (profile.imu.filter_time_constant_s * 1e6).round() as u32;
  config.x16_acc.delay_us = config.x16_gyro.delay_us;
  if scenario == "wrong-delay" {
    config.x16_gyro.delay_us = 8000;
    config.x16_acc.delay_us = 8000;
  }
  let fusion = Vimu::new(config);
  let metrics = Arc::new(Mutex::new(FusionMetrics::default()));
  let controller = FusedController {
    fusion,
    drive: EncoderVelocityController::new(
      base.blue_robots.clone(),
      profile.motor.clone(),
      TeamColor::Blue,
      0,
    ),
    metrics: metrics.clone(),
    last_yaw_rate: None,
    calibration_attempted: false,
    calibration_started: false,
  };
  let mut sim = RealisticWorld::new(0, base, profile)?;
  sim.set_controller(TeamColor::Blue, 0, Box::new(controller))?;
  let mut truth_history = BTreeMap::from([(0_u64, 0.0_f64)]);
  while sim.now() < 4 * SECOND {
    let now = sim.now();
    let angular = if now < SECOND {
      0.0
    } else if now < 2 * SECOND {
      1.0
    } else if now < 3 * SECOND {
      -1.0
    } else {
      0.0
    };
    sim.publish(
      WorldCommand {
        blue: vec![RobotCommand {
          move_command: Some(MoveCommand::LocalVelocity {
            forward: 0.0,
            left: 0.0,
            angular,
          }),
          ..RobotCommand::noop(0)
        }],
        ..WorldCommand::default()
      },
      now,
      1,
      Safety::Run,
    )?;
    sim.advance_to(now + 1_000_000)?;
    truth_history.insert(sim.now(), sim.truth().blue_robots[0].v_angular);
  }
  let metrics = metrics.lock().unwrap();
  let errors: Vec<f64> = metrics
    .samples
    .iter()
    .filter(|(at, _, _)| *at >= SECOND)
    .filter_map(|(at, rate, _)| truth_history.get(at).map(|truth| f64::from(*rate) - truth))
    .collect();
  let rms = (errors.iter().map(|e| e * e).sum::<f64>() / errors.len() as f64).sqrt();
  let report = serde_json::json!({
      "scenario":scenario,"experiment":"actual Crashanter adaptive dual-IMU; surrogate drive; provisional plant",
      "raw_samples":metrics.raw_samples,"rejected_samples":metrics.rejected,"calibrated":metrics.calibrated,"fused_samples":metrics.samples.len(),
      "health":metrics.health,"recovered_fused_outputs":metrics.recovered_fused,"peak_gyro_variance":metrics.peak_gyro_variance,
      "max_availability_age_ns":metrics.samples.iter().map(|(_,_,age)|age).max(),"yaw_rate_rms_error_rad_s":rms,"diagnostics":sim.diagnostics()
  });
  if !metrics.calibrated
    || (scenario == "healthy" && metrics.rejected != 0)
    || metrics.samples.len() < 3000
    || !rms.is_finite()
    || rms > 0.2
    || metrics.recovered_fused < 1000
  {
    return Err(
      "vIMU experiment failed its synthetic integration check (not a hardware fidelity gate)"
        .into(),
    );
  }
  Ok(report)
}
