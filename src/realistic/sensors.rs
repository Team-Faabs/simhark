//! Observation generation is the only sensor code allowed to see physical truth.
use super::{config::ImuConfig, time::TimeNs};
use nalgebra::{UnitQuaternion, Vector3};
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImuSample {
  pub stream: super::profile::ImuStream,
  pub timing: Option<SampleTiming>,
  /// Data-ready timestamp in the modeled common clock; causal filtering has
  /// already been applied to values, not compensated out of this timestamp.
  pub data_ready_at: TimeNs,
  pub window_start_at: TimeNs,
  /// Last completed physics grid boundary used in generating this sample.
  pub physical_state_at: TimeNs,
  pub angular_rad_s: [f64; 3],
  pub specific_force_m_s2: [f64; 3],
  pub clipped: bool,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SampleTiming {
  pub source: super::profile::TimestampSource,
  pub native_counter: Option<u64>,
  pub counter_epoch: u64,
  pub kernel_capture_at: TimeNs,
  pub available_at: TimeNs,
  pub uncertainty_ns: TimeNs,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EncoderSample {
  pub sampled_at: TimeNs,
  pub ticks: [i64; 4],
  pub speed_rad_s: [f64; 4],
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotDetection {
  pub id: usize,
  pub team: crate::TeamColor,
  pub position_m: [f64; 2],
  pub orientation_rad: f64,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VisionFrame {
  pub camera_id: u32,
  pub frame_number: u64,
  pub camera_capture_ticks: Option<u64>,
  pub image_received_at: TimeNs,
  pub sent_at: TimeNs,
  pub captured_at: TimeNs,
  pub physical_state_at: TimeNs,
  /// Raw detections only: no perfect tracked velocity alongside noisy positions.
  pub robots: Vec<RobotDetection>,
  pub ball_position_m: Option<[f64; 3]>,
}
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Observations {
  pub motor_adc: Option<MotorAdcSample>,
  pub lidar: Option<super::localization::LidarScan>,
  pub radio: Vec<super::localization::UwbObservation>,
  pub pose: Option<super::localization::PoseEstimate>,
  pub onboard_ball: Option<super::localization::BallObservation>,
  pub delivered_at: TimeNs,
  pub imu: Vec<ImuSample>,
  pub encoder: Option<EncoderSample>,
  pub vision: Option<VisionFrame>,
  pub magnetic: Option<MagneticSample>,
  pub telemetry: Option<RobotTelemetry>,
  pub infrared: Option<InfraredSample>,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotorAdcSample {
  pub sampled_at: TimeNs,
  pub phase_current_a: [f64; 4],
  pub bus_voltage_v: f64,
  pub clipped: bool,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MagneticSample {
  pub sampled_at: TimeNs,
  pub field_ut: [f64; 3],
  pub clipped: bool,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InfraredSample {
  pub sampled_at: TimeNs,
  pub detected: bool,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotTelemetry {
  pub sampled_at: TimeNs,
  pub voltage_v: f64,
  pub phase_current_a: [f64; 4],
  pub supply_current_a: f64,
  pub motor_temperature_c: [f64; 4],
  pub driver_fault: [bool; 4],
  pub brownout: bool,
}

pub(crate) fn rng(seed: u64, world: usize, robot: usize, device: u64) -> ChaCha8Rng {
  fn mix(mut x: u64) -> u64 {
    x = (x ^ (x >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
    x = (x ^ (x >> 27)).wrapping_mul(0x94d049bb133111eb);
    x ^ (x >> 31)
  }
  // Mix domains in order; an XOR of independently mixed IDs would alias when
  // world/device IDs are exchanged.
  let world_seed = mix(seed ^ (world as u64).wrapping_add(0x9e3779b97f4a7c15));
  let robot_seed = mix(world_seed ^ (robot as u64).wrapping_add(0xd1b54a32d192ed03));
  ChaCha8Rng::seed_from_u64(mix(robot_seed ^ device.wrapping_add(0x94d049bb133111eb)))
}
pub(crate) fn normal(rng: &mut ChaCha8Rng) -> f64 {
  let u = rng.random::<f64>().max(f64::MIN_POSITIVE);
  (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * rng.random::<f64>()).cos()
}

pub(crate) struct ImuModel {
  config: ImuConfig,
  rng: ChaCha8Rng,
  last_at: TimeNs,
  last_velocity: [f64; 3],
  last_omega: Vector3<f64>,
  filtered_gyro: Vector3<f64>,
  filtered_accel: Vector3<f64>,
  initialized: bool,
  pub fifo: Vec<ImuSample>,
  errors: Option<super::profile::ImuDevice>,
}
pub(crate) struct ImuMotion {
  pub body_to_world: UnitQuaternion<f64>,
  pub velocity_world: [f64; 3],
  pub angular_world: [f64; 3],
  pub temperature_c: f64,
}
impl ImuModel {
  pub fn new(config: ImuConfig, rng: ChaCha8Rng) -> Self {
    Self {
      config,
      rng,
      last_at: 0,
      last_velocity: [0.0; 3],
      last_omega: Vector3::zeros(),
      filtered_gyro: Vector3::zeros(),
      filtered_accel: Vector3::zeros(),
      initialized: false,
      fifo: Vec::new(),
      errors: None,
    }
  }
  pub fn with_device(mut self, device: super::profile::ImuDevice) -> Self {
    self.errors = Some(device);
    self
  }
  #[cfg(test)]
  pub fn sample(
    &mut self,
    at: TimeNs,
    physical_state_at: TimeNs,
    yaw: f64,
    velocity: [f64; 3],
    omega: f64,
    gravity: f64,
  ) {
    self.sample_motion(
      at,
      physical_state_at,
      ImuMotion {
        body_to_world: UnitQuaternion::from_euler_angles(0.0, 0.0, yaw),
        velocity_world: velocity,
        angular_world: [0.0, 0.0, omega],
        temperature_c: 25.0,
      },
      gravity,
    );
  }
  pub fn sample_motion(
    &mut self,
    at: TimeNs,
    physical_state_at: TimeNs,
    motion: ImuMotion,
    gravity: f64,
  ) {
    let ImuMotion {
      body_to_world,
      velocity_world: velocity,
      angular_world,
      temperature_c,
    } = motion;
    let c = &self.config;
    let dt = if self.initialized {
      (at - self.last_at) as f64 * 1e-9
    } else {
      1.0 / f64::from(c.rate_hz)
    };
    let acceleration = if self.initialized {
      Vector3::from_fn(|i, _| (velocity[i] - self.last_velocity[i]) / dt)
    } else {
      Vector3::zeros()
    };
    let omega = Vector3::from(angular_world);
    let body_from_world = body_to_world.inverse();
    let alpha = if self.initialized {
      body_from_world * (omega - self.last_omega) / dt
    } else {
      Vector3::zeros()
    };
    let sensor_from_body = UnitQuaternion::from_euler_angles(
      c.mounting_rpy_rad[0],
      c.mounting_rpy_rad[1],
      c.mounting_rpy_rad[2],
    )
    .inverse();
    let r = Vector3::from(c.position_body_m);
    let w1 = body_from_world * omega;
    let w0 = if self.initialized {
      body_from_world * self.last_omega
    } else {
      w1
    };
    let wm = (w0 + w1) * 0.5;
    let lever = alpha.cross(&r)
      + (w0.cross(&w0.cross(&r)) + 4.0 * wm.cross(&wm.cross(&r)) + w1.cross(&w1.cross(&r))) / 6.0;
    let mut acceleration = sensor_from_body
      * (body_from_world * (acceleration + Vector3::new(0.0, 0.0, gravity)) + lever);
    let mut gyro = sensor_from_body * w1;
    if let Some(device) = &self.errors {
      for vector in [&mut gyro, &mut acceleration] {
        let original = *vector;
        for axis in 0..3 {
          vector[axis] *= 1.0 + device.scale_error[axis];
          for other in 0..3 {
            vector[axis] += device.cross_axis[axis][other] * original[other];
          }
          vector[axis] += device.temperature_bias_per_c[axis] * (temperature_c - 25.0)
            + device.vibration_amplitude[axis]
              * (std::f64::consts::TAU * device.vibration_frequency_hz * at as f64 * 1e-9).sin();
          if device.quantization_step > 0.0 {
            vector[axis] =
              (vector[axis] / device.quantization_step).round() * device.quantization_step;
          }
        }
      }
    }
    let noise_bandwidth = (0.5 / dt).sqrt();
    let mut clipped = false;
    for i in 0..3 {
      gyro[i] += c.gyro_bias_rad_s[i]
        + normal(&mut self.rng) * c.gyro_noise_density_rad_s_sqrt_hz * noise_bandwidth;
      acceleration[i] += c.accel_bias_m_s2[i]
        + normal(&mut self.rng) * c.accel_noise_density_m_s2_sqrt_hz * noise_bandwidth;
      clipped |= gyro[i].abs() >= c.gyro_range_rad_s || acceleration[i].abs() >= c.accel_range_m_s2;
      gyro[i] = gyro[i].clamp(-c.gyro_range_rad_s, c.gyro_range_rad_s);
      acceleration[i] = acceleration[i].clamp(-c.accel_range_m_s2, c.accel_range_m_s2);
    }
    let gain = if c.filter_time_constant_s == 0.0 {
      1.0
    } else {
      1.0 - (-dt / c.filter_time_constant_s).exp()
    };
    if self.initialized {
      self.filtered_gyro += (gyro - self.filtered_gyro) * gain;
      self.filtered_accel += (acceleration - self.filtered_accel) * gain;
    } else {
      self.filtered_gyro = gyro;
      self.filtered_accel = acceleration;
    }
    self.fifo.push(ImuSample {
      stream: self
        .errors
        .as_ref()
        .map_or(super::profile::ImuStream::X16, |d| d.stream),
      timing: None,
      data_ready_at: at,
      window_start_at: self.last_at,
      physical_state_at,
      angular_rad_s: self.filtered_gyro.into(),
      specific_force_m_s2: self.filtered_accel.into(),
      clipped,
    });
    self.last_at = at;
    self.last_velocity = velocity;
    self.last_omega = omega;
    self.initialized = true;
  }
}

/// Actual ODR error is applied to the absolute sample index, preserving
/// fractional native periods without repeated-rounded-period drift.
pub(crate) fn imu_time(config: &ImuConfig, index: u64) -> TimeNs {
  let numerator = u128::from(index) * 1_000_000_000_000_000;
  let denominator =
    u128::from(config.rate_hz) * (1_000_000_i64 + i64::from(config.odr_error_ppm)) as u128;
  config
    .phase_ns
    .checked_add(u64::try_from(numerator / denominator).expect("IMU time overflow"))
    .expect("IMU time overflow")
}

#[cfg(test)]
mod tests {
  use super::*;

  fn noiseless() -> ImuConfig {
    ImuConfig {
      gyro_noise_density_rad_s_sqrt_hz: 0.0,
      accel_noise_density_m_s2_sqrt_hz: 0.0,
      filter_time_constant_s: 0.0,
      ..ImuConfig::default()
    }
  }
  #[test]
  fn accelerometer_observes_specific_force_in_mounted_axes() {
    let mut config = noiseless();
    config.mounting_rpy_rad = [std::f64::consts::FRAC_PI_2, 0.0, 0.0];
    let mut imu = ImuModel::new(config, rng(42, 0, 0, 1));
    imu.sample(0, 0, 0.0, [0.0; 3], 0.0, 9.81);
    let sample = &imu.fifo[0];
    assert!(sample.specific_force_m_s2[0].abs() < 1e-12);
    assert!((sample.specific_force_m_s2[1] - 9.81).abs() < 1e-12);
    assert!(sample.specific_force_m_s2[2].abs() < 1e-12);
  }
  #[test]
  fn off_center_imu_observes_centripetal_acceleration() {
    let mut config = noiseless();
    config.position_body_m = [0.05, 0.0, 0.0];
    let mut imu = ImuModel::new(config, rng(42, 0, 0, 1));
    imu.sample(0, 0, 0.0, [0.0; 3], 10.0, 9.81);
    let sample = &imu.fifo[0];
    assert!((sample.specific_force_m_s2[0] + 5.0).abs() < 1e-12);
    assert_eq!(sample.angular_rad_s, [0.0, 0.0, 10.0]);
  }
  #[test]
  fn odr_error_uses_native_period_without_cumulative_rounding_drift() {
    let mut config = noiseless();
    config.odr_error_ppm = 10_000;
    assert_eq!(imu_time(&config, 193_920), 100 * super::super::time::SECOND);
    assert_eq!(
      imu_time(&ImuConfig::default(), 1920),
      super::super::time::SECOND
    );
  }
}
