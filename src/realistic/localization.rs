//! Geometry-derived observations and observation-only pose estimation.
use super::{
  scene::{LandmarkBox, Scene},
  sensors::{self, EncoderSample, ImuSample},
  time::TimeNs,
};
use nalgebra::{UnitQuaternion, Vector3};
use rand::RngExt;
use rand_chacha::ChaCha8Rng;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct LidarConfig {
  pub enabled: bool,
  pub scan_rate_hz: u32,
  pub beams: u32,
  pub position_body_m: [f64; 3],
  pub mounting_rpy_rad: [f64; 3],
  pub min_range_m: f64,
  pub max_range_m: f64,
  pub range_resolution_m: f64,
  pub range_noise_std_m: f64,
  pub angular_resolution_rad: f64,
  pub divergence_rad: f64,
  pub dropout_probability: f64,
  pub delivery_ns: TimeNs,
  pub initial_pose: Option<[f64; 3]>,
}
impl Default for LidarConfig {
  fn default() -> Self {
    Self {
      enabled: false,
      scan_rate_hz: 10,
      beams: 360,
      position_body_m: [0.0, 0.0, -0.03],
      mounting_rpy_rad: [0.0; 3],
      min_range_m: 0.05,
      max_range_m: 12.0,
      range_resolution_m: 0.001,
      range_noise_std_m: 0.005,
      angular_resolution_rad: 0.0,
      divergence_rad: 0.003,
      dropout_probability: 0.0,
      delivery_ns: 1_000_000,
      initial_pose: None,
    }
  }
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LidarBeam {
  pub sampled_at: TimeNs,
  pub azimuth_rad: f64,
  pub range_m: Option<f64>,
  pub intensity: f64,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LidarScan {
  pub started_at: TimeNs,
  pub completed_at: TimeNs,
  pub beams: Vec<LidarBeam>,
}
pub(crate) fn lidar_beam(
  config: &LidarConfig,
  at: TimeNs,
  index: u64,
  position: Vector3<f64>,
  orientation: UnitQuaternion<f64>,
  scene: &Scene,
  rng: &mut ChaCha8Rng,
) -> LidarBeam {
  let mut angle =
    std::f64::consts::TAU * (index % u64::from(config.beams)) as f64 / f64::from(config.beams);
  if config.angular_resolution_rad > 0.0 {
    angle = (angle / config.angular_resolution_rad).round() * config.angular_resolution_rad;
  }
  let sensor = orientation
    * UnitQuaternion::from_euler_angles(
      config.mounting_rpy_rad[0],
      config.mounting_rpy_rad[1],
      config.mounting_rpy_rad[2],
    );
  let origin = position + orientation * Vector3::from(config.position_body_m);
  let angular_spread = (rng.random::<f64>() - 0.5) * config.divergence_rad;
  let direction = sensor
    * Vector3::new(
      (angle + angular_spread).cos(),
      (angle + angular_spread).sin(),
      0.0,
    );
  let hit = scene.cast(origin, direction, config.max_range_m);
  let (mut range, intensity) = hit.map_or((None, 0.0), |(r, intensity)| (Some(r), intensity));
  if rng.random::<f64>() < config.dropout_probability || intensity < rng.random::<f64>() * 0.05 {
    range = None;
  }
  if let Some(value) = &mut range {
    *value += sensors::normal(rng) * config.range_noise_std_m;
    if config.range_resolution_m > 0.0 {
      *value = (*value / config.range_resolution_m).round() * config.range_resolution_m;
    }
    if *value < config.min_range_m || *value > config.max_range_m {
      range = None;
    }
  }
  LidarBeam {
    sampled_at: at,
    azimuth_rad: angle,
    range_m: range,
    intensity,
  }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct OnboardCameraConfig {
  pub enabled: bool,
  pub rate_hz: u32,
  pub position_body_m: [f64; 3],
  pub yaw_body_rad: f64,
  pub horizontal_fov_rad: f64,
  pub max_range_m: f64,
  pub bearing_noise_std_rad: f64,
  pub range_fractional_noise: f64,
  pub detection_delay_ns: TimeNs,
  pub streaming: bool,
  pub stream_bytes_per_frame: usize,
  pub encode_delay_ns: TimeNs,
  pub decode_delay_ns: TimeNs,
}
impl Default for OnboardCameraConfig {
  fn default() -> Self {
    Self {
      enabled: false,
      rate_hz: 30,
      position_body_m: [0.07, 0.0, -0.05],
      yaw_body_rad: 0.0,
      horizontal_fov_rad: 1.7,
      max_range_m: 5.0,
      bearing_noise_std_rad: 0.005,
      range_fractional_noise: 0.05,
      detection_delay_ns: 5_000_000,
      streaming: false,
      stream_bytes_per_frame: 50_000,
      encode_delay_ns: 2_000_000,
      decode_delay_ns: 3_000_000,
    }
  }
}
/// Timing of an optional encoded camera frame after shared WiFi and host decoding.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StreamedFrame {
  pub captured_at: TimeNs,
  pub encoded_at: TimeNs,
  pub received_at: TimeNs,
  pub decoded_at: TimeNs,
  pub bytes: usize,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BallObservation {
  pub captured_at: TimeNs,
  pub bearing_rad: f64,
  pub range_m: f64,
  pub confidence: f64,
}
pub(crate) fn onboard_ball(
  config: &OnboardCameraConfig,
  at: TimeNs,
  position: Vector3<f64>,
  orientation: UnitQuaternion<f64>,
  scene: &Scene,
  rng: &mut ChaCha8Rng,
) -> Option<BallObservation> {
  let origin = position + orientation * Vector3::from(config.position_body_m);
  let delta = scene.ball_position - origin;
  let local = orientation.inverse() * delta;
  let angle = super::vision::wrap(local.y.atan2(local.x) - config.yaw_body_rad);
  let range = delta.norm();
  if range > config.max_range_m || angle.abs() > config.horizontal_fov_rad / 2.0 {
    return None;
  }
  if scene
    .cast(
      origin,
      delta / range,
      (range - scene.ball_radius * 1.1).max(0.0),
    )
    .is_some()
  {
    return None;
  }
  Some(BallObservation {
    captured_at: at,
    bearing_rad: angle + sensors::normal(rng) * config.bearing_noise_std_rad,
    range_m: (range * (1.0 + sensors::normal(rng) * config.range_fractional_noise)).max(0.0),
    confidence: 0.9,
  })
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RangingProtocol {
  SingleSided,
  #[default]
  DoubleSided,
  Tdoa,
}
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BearingFrame {
  #[default]
  RobotArray,
  AnchorArray,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Anchor {
  pub id: u32,
  pub position_m: [f64; 3],
  #[serde(default)]
  pub estimated_position_m: Option<[f64; 3]>,
  #[serde(default)]
  pub clock: super::profile::ClockConfig,
  #[serde(default)]
  pub antenna_delay_ns: f64,
  #[serde(default)]
  pub yaw_rad: f64,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct UwbConfig {
  pub tx_firmware_delay_ns: TimeNs,
  pub slot_jitter_ns: TimeNs,
  pub require_sync_for_commands: bool,
  pub command_clock_uncertainty_ns: TimeNs,
  pub enabled: bool,
  pub protocol: RangingProtocol,
  pub anchors: Vec<Anchor>,
  pub clock: super::profile::ClockConfig,
  pub turnaround_ns: TimeNs,
  pub frame_airtime_ns: TimeNs,
  pub host_delay_ns: TimeNs,
  pub ranging_payload_bytes: u32,
  pub telemetry_payload_bytes: u32,
  pub range_noise_std_m: f64,
  pub nlos_bias_m: f64,
  pub loss_probability: f64,
  pub antenna_delay_ns: f64,
  pub assumed_antenna_delay_ns: f64,
  pub bearing_frame: BearingFrame,
  pub bearing_noise_std_rad: f64,
  pub bearing_bias_rad: f64,
  pub antenna_yaw_body_rad: f64,
  pub bearing_ambiguity_pi: bool,
  pub synchronization_timeout_ns: TimeNs,
  pub synchronization_loss_every: Option<u64>,
}
impl Default for UwbConfig {
  fn default() -> Self {
    Self {
      tx_firmware_delay_ns: 0,
      slot_jitter_ns: 0,
      require_sync_for_commands: false,
      command_clock_uncertainty_ns: 0,
      enabled: false,
      protocol: RangingProtocol::DoubleSided,
      anchors: Vec::new(),
      clock: super::profile::ClockConfig {
        tick_hz: 64_000_000_000,
        counter_bits: 40,
        ..Default::default()
      },
      turnaround_ns: 20_000,
      frame_airtime_ns: 110_000,
      host_delay_ns: 100_000,
      ranging_payload_bytes: 16,
      telemetry_payload_bytes: 48,
      range_noise_std_m: 0.05,
      nlos_bias_m: 0.3,
      loss_probability: 0.0,
      antenna_delay_ns: 0.0,
      assumed_antenna_delay_ns: 0.0,
      bearing_frame: BearingFrame::RobotArray,
      bearing_noise_std_rad: 0.03,
      bearing_bias_rad: 0.0,
      antenna_yaw_body_rad: 0.0,
      bearing_ambiguity_pi: false,
      synchronization_timeout_ns: 100_000_000,
      synchronization_loss_every: None,
    }
  }
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UwbObservation {
  pub sampled_at: TimeNs,
  pub anchor_id: u32,
  pub protocol: RangingProtocol,
  pub tick_hz: u64,
  pub counter_bits: u8,
  pub timestamps: [u64; 6],
  pub antenna_correction_ns: f64,
  pub range_std_m: f64,
  pub bearing_rad: Option<f64>,
  pub bearing_frame: BearingFrame,
  pub bearing_std_rad: f64,
  pub ambiguous_pi: bool,
  pub quality: f64,
  pub tdoa_reference: Option<(u32, f64)>,
}
impl UwbObservation {
  pub fn range_m(&self) -> Option<f64> {
    let mask = (1_u128 << self.counter_bits) - 1;
    let delta = |a: u64, b: u64| ((u128::from(a) + (mask + 1) - u128::from(b)) & mask) as f64;
    let [t1, t2, t3, t4, t5, t6] = self.timestamps;
    let ticks = match self.protocol {
      RangingProtocol::SingleSided => (delta(t4, t1) - delta(t3, t2)) / 2.0,
      RangingProtocol::DoubleSided => {
        let ra = delta(t4, t1);
        let rb = delta(t6, t3);
        let da = delta(t5, t4);
        let db = delta(t3, t2);
        (ra * rb - da * db) / (ra + rb + da + db)
      }
      RangingProtocol::Tdoa => return None,
    };
    let value = (ticks / self.tick_hz as f64 - self.antenna_correction_ns * 1e-9) * 299_792_458.0;
    (value.is_finite() && value >= 0.0).then_some(value)
  }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PoseEstimate {
  pub estimated_at: TimeNs,
  pub position_m: Option<[f64; 2]>,
  pub yaw_rad: Option<f64>,
  pub covariance: [[f64; 3]; 3],
  pub ambiguous: bool,
  pub source: String,
}
pub struct PoseEstimator {
  body_velocity: [f64; 3],
  known_height_m: f64,
  pub estimate: PoseEstimate,
  pub known_x16_mounting_rpy_rad: [f64; 3],
  last_encoder: Option<TimeNs>,
  last_gyro: Option<(TimeNs, f64)>,
  radius: f64,
  wheel_radius: f64,
  angles: [f64; 4],
  map: Vec<LandmarkBox>,
  pub rejected_scans: u64,
}
impl PoseEstimator {
  pub fn new(robot: &crate::RobotConfig, map: Vec<LandmarkBox>, initial: Option<[f64; 3]>) -> Self {
    Self {
      body_velocity: [0.0; 3],
      known_height_m: robot.start_z(),
      estimate: PoseEstimate {
        estimated_at: 0,
        position_m: initial.map(|p| [p[0], p[1]]),
        yaw_rad: initial.map(|p| p[2]),
        covariance: [[100.0, 0.0, 0.0], [0.0, 100.0, 0.0], [0.0, 0.0, 100.0]],
        ambiguous: initial.is_none(),
        source: "uninitialized".into(),
      },
      known_x16_mounting_rpy_rad: [0.0; 3],
      last_encoder: None,
      last_gyro: None,
      radius: robot.radius,
      wheel_radius: robot.wheel_radius,
      angles: robot.wheel_angles.map(f64::to_radians),
      map,
      rejected_scans: 0,
    }
  }
  /// Explicit estimator restart. A seed must come from an observable/manual
  /// initialization; the simulator never supplies its current pose here.
  pub fn reset(&mut self, at: TimeNs, initial: Option<[f64; 3]>) -> Result<(), String> {
    if initial.is_some_and(|p| p.iter().any(|x| !x.is_finite())) {
      return Err("nonfinite localization seed".into());
    }
    self.body_velocity = [0.0; 3];
    self.last_encoder = None;
    self.last_gyro = None;
    self.estimate = PoseEstimate {
      estimated_at: at,
      position_m: initial.map(|p| [p[0], p[1]]),
      yaw_rad: initial.map(|p| p[2]),
      covariance: [[100.0, 0.0, 0.0], [0.0, 100.0, 0.0], [0.0, 0.0, 100.0]],
      ambiguous: initial.is_none(),
      source: "explicit estimator restart".into(),
    };
    Ok(())
  }
  pub fn imu(&mut self, samples: &[ImuSample]) {
    for sample in samples {
      if sample.stream != super::profile::ImuStream::X16 {
        continue;
      }
      let r = self.known_x16_mounting_rpy_rad;
      let rate_z = (UnitQuaternion::from_euler_angles(r[0], r[1], r[2])
        * Vector3::from(sample.angular_rad_s))
      .z;
      if let Some((at, rate)) = self.last_gyro
        && sample.data_ready_at > at
        && let Some(yaw) = &mut self.estimate.yaw_rad
      {
        *yaw = super::vision::wrap(
          *yaw + 0.5 * (rate + rate_z) * (sample.data_ready_at - at) as f64 * 1e-9,
        );
      }
      if self
        .last_gyro
        .is_none_or(|(at, _)| sample.data_ready_at > at)
      {
        self.last_gyro = Some((sample.data_ready_at, rate_z));
      }
    }
  }
  pub fn encoder(&mut self, sample: &EncoderSample) {
    if let Some(last) = self.last_encoder
      && sample.sampled_at > last
    {
      let a = nalgebra::SMatrix::<f64, 4, 3>::from_fn(|r, c| match c {
        0 => -self.angles[r].sin(),
        1 => self.angles[r].cos(),
        _ => self.radius,
      });
      if let Some(inverse) = (a.transpose() * a).try_inverse() {
        let v = inverse
          * a.transpose()
          * nalgebra::SVector::<f64, 4>::from(sample.speed_rad_s)
          * self.wheel_radius;
        self.body_velocity = [v[0], v[1], v[2]];
        let dt = (sample.sampled_at - last) as f64 * 1e-9;
        if let (Some(position), Some(yaw)) = (&mut self.estimate.position_m, self.estimate.yaw_rad)
        {
          position[0] += (v[0] * yaw.cos() - v[1] * yaw.sin()) * dt;
          position[1] += (v[0] * yaw.sin() + v[1] * yaw.cos()) * dt;
          for i in 0..3 {
            self.estimate.covariance[i][i] += 0.05 * dt;
          }
        }
      }
    }
    self.last_encoder = Some(sample.sampled_at);
    self.estimate.estimated_at = sample.sampled_at;
  }
  pub fn ranges(&mut self, observations: &[UwbObservation], anchors: &[Anchor], at: TimeNs) {
    let valid: Vec<_> = observations
      .iter()
      .filter(|o| at.saturating_sub(o.sampled_at) < 200_000_000 && o.quality > 0.0)
      .filter_map(|o| Some((o, anchors.iter().find(|a| a.id == o.anchor_id)?)))
      .collect();
    if valid.len() < 3 {
      return;
    }
    let mut state = self.estimate.position_m.map_or(
      Vector3::new(0.0, 0.0, self.estimate.yaw_rad.unwrap_or(0.0)),
      |p| Vector3::new(p[0], p[1], self.estimate.yaw_rad.unwrap_or(0.0)),
    );
    let mut hessian = nalgebra::Matrix3::zeros();
    for _ in 0..15 {
      hessian.fill(0.0);
      let mut gradient = Vector3::zeros();
      for (obs, anchor) in &valid {
        let delta = nalgebra::Vector2::new(
          state.x - anchor.estimated_position_m.unwrap_or(anchor.position_m)[0],
          state.y - anchor.estimated_position_m.unwrap_or(anchor.position_m)[1],
        );
        let horizontal = delta.norm().max(1e-6);
        let distance = (horizontal.powi(2)
          + (anchor.estimated_position_m.unwrap_or(anchor.position_m)[2] - self.known_height_m)
            .powi(2))
        .sqrt();
        if let Some(range) = obs.range_m() {
          let j = Vector3::new(delta.x / distance, delta.y / distance, 0.0);
          let weight = 1.0 / obs.range_std_m.powi(2).max(1e-6);
          hessian += j * j.transpose() * weight;
          gradient += j * (range - distance) * weight;
        }
        if let Some((reference, difference)) = obs.tdoa_reference
          && let Some(reference) = anchors.iter().find(|a| a.id == reference)
        {
          let d = nalgebra::Vector2::new(
            state.x
              - reference
                .estimated_position_m
                .unwrap_or(reference.position_m)[0],
            state.y
              - reference
                .estimated_position_m
                .unwrap_or(reference.position_m)[1],
          );
          let range = (d.norm_squared()
            + (reference
              .estimated_position_m
              .unwrap_or(reference.position_m)[2]
              - self.known_height_m)
              .powi(2))
          .sqrt()
          .max(1e-6);
          let j = Vector3::new(
            delta.x / distance - d.x / range,
            delta.y / distance - d.y / range,
            0.0,
          );
          let weight = 1.0 / (2.0 * obs.range_std_m.powi(2)).max(1e-6);
          hessian += j * j.transpose() * weight;
          gradient += j * (difference - (distance - range)) * weight;
        }
        if let Some(bearing) = obs.bearing_rad
          && obs.bearing_frame == BearingFrame::RobotArray
          && !obs.ambiguous_pi
        {
          let expected = (-delta.y).atan2(-delta.x) - state.z;
          let residual = super::vision::wrap(bearing - expected);
          let r2 = horizontal * horizontal;
          let j = Vector3::new(-delta.y / r2, delta.x / r2, -1.0);
          let weight = 1.0 / obs.bearing_std_rad.powi(2).max(1e-8);
          hessian += j * j.transpose() * weight;
          gradient += j * residual * weight;
        }
        if let Some(bearing) = obs.bearing_rad
          && obs.bearing_frame == BearingFrame::AnchorArray
          && !obs.ambiguous_pi
        {
          let residual = super::vision::wrap(bearing - (delta.y.atan2(delta.x) - anchor.yaw_rad));
          let j = Vector3::new(
            -delta.y / horizontal.powi(2),
            delta.x / horizontal.powi(2),
            0.0,
          );
          let weight = 1.0 / obs.bearing_std_rad.powi(2).max(1e-8);
          hessian += j * j.transpose() * weight;
          gradient += j * residual * weight;
        }
      }
      let regularized = hessian + nalgebra::Matrix3::identity() * 1e-8;
      let Some(step) = regularized.lu().solve(&gradient) else {
        return;
      };
      state += step;
      state.z = super::vision::wrap(state.z);
      if step.norm() < 1e-5 {
        break;
      }
    }
    let eigen = hessian.symmetric_eigen().eigenvalues;
    let observable = eigen.min() > 1e-4;
    let position_observable = (hessian.fixed_view::<2, 2>(0, 0)).determinant() > 1e-6;
    if position_observable {
      self.estimate.position_m = Some([state.x, state.y]);
    }
    if observable {
      self.estimate.yaw_rad = Some(state.z);
      self.estimate.ambiguous = false;
    }
    self.estimate.covariance = (hessian + nalgebra::Matrix3::identity() * 1e-8)
      .try_inverse()
      .map_or([[1e8; 3]; 3], |p| {
        std::array::from_fn(|r| std::array::from_fn(|c| p[(r, c)]))
      });
    self.estimate.estimated_at = at;
    self.estimate.source = "radio ranges/bearings plus inertial/encoder prediction".into();
  }
  pub fn lidar(&mut self, scan: &LidarScan, config: &LidarConfig) {
    let (Some(position), Some(yaw)) = (self.estimate.position_m, self.estimate.yaw_rad) else {
      self.rejected_scans += 1;
      return;
    };
    if self.map.is_empty() {
      self.rejected_scans += 1;
      return;
    }
    let points = deskew_scan(scan, self.body_velocity, config);
    let mut state = Vector3::new(position[0], position[1], yaw);
    let mut hessian = nalgebra::Matrix3::zeros();
    let mut count = 0;
    for _ in 0..10 {
      hessian.fill(0.0);
      let mut gradient = Vector3::zeros();
      count = 0;
      for p in &points {
        let rotated = nalgebra::Vector2::new(
          state.z.cos() * p[0] - state.z.sin() * p[1],
          state.z.sin() * p[0] + state.z.cos() * p[1],
        );
        let point = nalgebra::Vector2::new(state.x, state.y) + rotated;
        let closest = self
          .map
          .iter()
          .flat_map(|b| {
            let min = nalgebra::Vector2::new(b.min_m[0], b.min_m[1]);
            let max = nalgebra::Vector2::new(b.max_m[0], b.max_m[1]);
            [
              (
                nalgebra::Vector2::new(min.x, point.y.clamp(min.y, max.y)),
                nalgebra::Vector2::new(-1.0, 0.0),
              ),
              (
                nalgebra::Vector2::new(max.x, point.y.clamp(min.y, max.y)),
                nalgebra::Vector2::new(1.0, 0.0),
              ),
              (
                nalgebra::Vector2::new(point.x.clamp(min.x, max.x), min.y),
                nalgebra::Vector2::new(0.0, -1.0),
              ),
              (
                nalgebra::Vector2::new(point.x.clamp(min.x, max.x), max.y),
                nalgebra::Vector2::new(0.0, 1.0),
              ),
            ]
          })
          .min_by(|(a, _), (b, _)| {
            (point - a)
              .norm_squared()
              .total_cmp(&(point - b).norm_squared())
          });
        if let Some((landmark, normal)) = closest {
          let residual = (point - landmark).dot(&normal);
          if (point - landmark).norm() > 0.3 {
            continue;
          }
          let j = Vector3::new(
            normal.x,
            normal.y,
            normal.dot(&nalgebra::Vector2::new(-rotated.y, rotated.x)),
          );
          let weight = 1.0 / residual.abs().max(0.02);
          hessian += j * j.transpose() * weight;
          gradient -= j * residual * weight;
          count += 1;
        }
      }
      if count < 6 {
        self.rejected_scans += 1;
        return;
      }
      let eigen = hessian.symmetric_eigen().eigenvalues;
      if eigen.min() < 1e-4 {
        self.estimate.ambiguous = true;
        self.rejected_scans += 1;
        return;
      }
      let Some(step) = hessian.lu().solve(&gradient) else {
        return;
      };
      state += step;
      state.z = super::vision::wrap(state.z);
      if step.norm() < 1e-5 {
        break;
      }
    }
    if count >= 6 {
      self.estimate.position_m = Some([state.x, state.y]);
      self.estimate.yaw_rad = Some(state.z);
      self.estimate.estimated_at = scan.completed_at;
      if let Some(cov) = hessian.try_inverse() {
        self.estimate.covariance = std::array::from_fn(|r| std::array::from_fn(|c| cov[(r, c)]));
      }
      self.estimate.source = "timestamped LiDAR scan matching with encoder/gyro deskew".into();
    }
  }
}

pub fn deskew_scan(
  scan: &LidarScan,
  body_velocity: [f64; 3],
  config: &LidarConfig,
) -> Vec<[f64; 2]> {
  let [vx, vy, omega] = body_velocity;
  let mount = config.mounting_rpy_rad[2];
  scan
    .beams
    .iter()
    .filter_map(|beam| {
      let range = beam.range_m?;
      let dt = (beam.sampled_at as i128 - scan.completed_at as i128) as f64 * 1e-9;
      let theta = omega * dt;
      let (dx, dy) = if omega.abs() > 1e-6 {
        (
          (theta.sin() * vx - (1.0 - theta.cos()) * vy) / omega,
          ((1.0 - theta.cos()) * vx + theta.sin() * vy) / omega,
        )
      } else {
        (vx * dt, vy * dt)
      };
      let x = config.position_body_m[0] + range * (beam.azimuth_rad + mount).cos();
      let y = config.position_body_m[1] + range * (beam.azimuth_rad + mount).sin();
      Some([
        theta.cos() * x - theta.sin() * y + dx,
        theta.sin() * x + theta.cos() * y + dy,
      ])
    })
    .collect()
}
