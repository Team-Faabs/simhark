//! Hardware instances and infrastructure, independent of compatibility WorldConfig.
use super::{
  config::{ImuConfig, MotorConfig, Provenance},
  time::TimeNs,
};
pub use crate::config::WheelProfile;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Dynamics {
  #[default]
  Planar,
  Free3d,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct MotorAdcConfig {
  pub enabled: bool,
  pub rate_hz: u32,
  pub delivery_ns: TimeNs,
  pub current_resolution_a: f64,
  pub current_noise_std_a: f64,
  pub current_range_a: f64,
  pub voltage_resolution_v: f64,
}
impl Default for MotorAdcConfig {
  fn default() -> Self {
    Self {
      enabled: false,
      rate_hz: 20_000,
      delivery_ns: 10_000,
      current_resolution_a: 0.001,
      current_noise_std_a: 0.002,
      current_range_a: 12.0,
      voltage_resolution_v: 0.01,
    }
  }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct HardwareProfile {
  pub dynamics: Dynamics,
  /// Total simulated robot mass, including the kicker (kg). None uses team geometry.
  pub mass_kg: Option<f64>,
  pub wheel: WheelProfile,
  pub support_stiffness_n_m: f64,
  pub support_damping_n_s_m: f64,
  pub rolling_resistance: f64,
  pub transverse_friction: f64,
  pub center_of_mass_body_m: Option<[f64; 3]>,
  pub inertia_diagonal_kg_m2: Option<[f64; 3]>,
  pub battery: BatteryConfig,
  pub auxiliaries: AuxiliaryConfig,
}
impl Default for HardwareProfile {
  fn default() -> Self {
    Self {
      dynamics: Dynamics::Planar,
      mass_kg: None,
      wheel: WheelProfile::default(),
      support_stiffness_n_m: 25000.0,
      support_damping_n_s_m: 120.0,
      rolling_resistance: 0.01,
      transverse_friction: 0.04,
      center_of_mass_body_m: None,
      inertia_diagonal_kg_m2: None,
      battery: BatteryConfig::default(),
      auxiliaries: AuxiliaryConfig::default(),
    }
  }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct BatteryConfig {
  pub enabled: bool,
  pub capacity_coulomb: f64,
  pub open_circuit_voltage_v: f64,
  pub internal_resistance_ohm: f64,
  pub brownout_voltage_v: f64,
  pub recovery_voltage_v: f64,
  pub restart_ns: TimeNs,
  pub electronics_power_w: f64,
  pub thermal_capacity_j_k: f64,
  pub cooling_w_k: f64,
  pub ambient_temperature_c: f64,
  pub motor_thermal_capacity_j_k: f64,
  pub motor_cooling_w_k: f64,
  pub motor_shutdown_temperature_c: f64,
  pub regenerative: bool,
}
impl Default for BatteryConfig {
  fn default() -> Self {
    Self {
      enabled: true,
      capacity_coulomb: 7200.0,
      open_circuit_voltage_v: 12.0,
      internal_resistance_ohm: 0.08,
      brownout_voltage_v: 8.0,
      recovery_voltage_v: 9.0,
      restart_ns: 100_000_000,
      electronics_power_w: 8.0,
      thermal_capacity_j_k: 100.0,
      cooling_w_k: 0.2,
      ambient_temperature_c: 25.0,
      motor_thermal_capacity_j_k: 15.0,
      motor_cooling_w_k: 0.3,
      motor_shutdown_temperature_c: 100.0,
      regenerative: false,
    }
  }
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct AuxiliaryConfig {
  pub kick_charge_time_ns: TimeNs,
  pub kick_delay_ns: TimeNs,
  pub kick_energy_j: f64,
  pub dribbler_time_constant_s: f64,
  pub dribbler_power_w: f64,
  pub dribbler_pull_n: f64,
  pub dribbler_max_force_n: f64,
  pub dribbler_stiffness_n_m: f64,
  pub dribbler_damping_n_s_m: f64,
  pub dribbler_spin_torque_nm: f64,
  pub charging_power_w: f64,
  pub ir_delay_ns: TimeNs,
  pub ir_debounce_ns: TimeNs,
}
impl Default for AuxiliaryConfig {
  fn default() -> Self {
    Self {
      kick_charge_time_ns: 1_000_000_000,
      kick_delay_ns: 2_000_000,
      kick_energy_j: 8.0,
      dribbler_time_constant_s: 0.04,
      dribbler_power_w: 6.0,
      dribbler_pull_n: 0.3,
      dribbler_max_force_n: 1.0,
      dribbler_stiffness_n_m: 30.0,
      dribbler_damping_n_s_m: 0.15,
      dribbler_spin_torque_nm: 0.001,
      charging_power_w: 10.0,
      ir_delay_ns: 1_000_000,
      ir_debounce_ns: 1_000_000,
    }
  }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ImuStream {
  X16,
  BmiGyro,
  BmiAccel,
}
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TimestampSource {
  #[default]
  HostInterrupt,
  NativeCounter,
  SampleCount,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct ClockConfig {
  pub tick_hz: u64,
  pub counter_bits: u8,
  pub offset_ticks: u64,
  pub skew_ppm: f64,
  pub drift_ppm_per_s: f64,
}
impl Default for ClockConfig {
  fn default() -> Self {
    Self {
      tick_hz: 1_000_000,
      counter_bits: 32,
      offset_ticks: 0,
      skew_ppm: 0.0,
      drift_ppm_per_s: 0.0,
    }
  }
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct SensorTiming {
  pub source: TimestampSource,
  pub clock: ClockConfig,
  pub interrupt_delay_ns: TimeNs,
  pub kernel_capture_jitter_ns: TimeNs,
  pub wakeup_trace_ns: Vec<i64>,
  pub bus_transaction_ns: TimeNs,
  pub missed_interrupt_every: Option<u64>,
  pub timestamp_uncertainty_ns: TimeNs,
}
impl Default for SensorTiming {
  fn default() -> Self {
    Self {
      source: TimestampSource::SampleCount,
      clock: ClockConfig::default(),
      interrupt_delay_ns: 1000,
      kernel_capture_jitter_ns: 1000,
      wakeup_trace_ns: vec![100_000],
      bus_transaction_ns: 100_000,
      missed_interrupt_every: None,
      timestamp_uncertainty_ns: 100_000,
    }
  }
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct SensorFault {
  pub start_ns: TimeNs,
  pub end_ns: TimeNs,
  pub kind: SensorFaultKind,
}
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SensorFaultKind {
  #[default]
  Missing,
  Frozen,
  Clipped,
  Duplicate,
  Reordered,
  ClockReset,
}
impl Default for SensorFault {
  fn default() -> Self {
    Self {
      start_ns: 0,
      end_ns: 0,
      kind: SensorFaultKind::Missing,
    }
  }
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ImuDevice {
  pub stream: ImuStream,
  pub physical: ImuConfig,
  #[serde(default)]
  pub timing: SensorTiming,
  #[serde(default)]
  pub scale_error: [f64; 3],
  #[serde(default)]
  pub cross_axis: [[f64; 3]; 3],
  #[serde(default)]
  pub quantization_step: f64,
  #[serde(default)]
  pub temperature_bias_per_c: [f64; 3],
  #[serde(default)]
  pub vibration_amplitude: [f64; 3],
  #[serde(default)]
  pub vibration_frequency_hz: f64,
  #[serde(default)]
  pub faults: Vec<SensorFault>,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct MagnetometerConfig {
  pub ambient_gradient_ut_per_m: [[f64; 3]; 3],
  pub motor_dipoles: Vec<MagneticDipole>,
  pub dribbler_field_ut: [f64; 3],
  pub charging_field_ut: [f64; 3],
  pub enabled: bool,
  pub rate_hz: u32,
  pub position_body_m: [f64; 3],
  pub mounting_rpy_rad: [f64; 3],
  pub earth_field_world_ut: [f64; 3],
  pub hard_iron_ut: [f64; 3],
  pub soft_iron: [[f64; 3]; 3],
  pub current_field_ut_per_a: [[f64; 3]; 4],
  pub rotor_field_ut: [[f64; 3]; 4],
  pub rotor_harmonic: u32,
  pub noise_std_ut: f64,
  pub range_ut: f64,
  pub delay_ns: TimeNs,
}
impl Default for MagnetometerConfig {
  fn default() -> Self {
    Self {
      ambient_gradient_ut_per_m: [[0.0; 3]; 3],
      motor_dipoles: Vec::new(),
      dribbler_field_ut: [0.0; 3],
      charging_field_ut: [0.0; 3],
      enabled: true,
      rate_hz: 100,
      position_body_m: [0.0; 3],
      mounting_rpy_rad: [0.0; 3],
      earth_field_world_ut: [20.0, 0.0, 40.0],
      hard_iron_ut: [0.0; 3],
      soft_iron: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
      current_field_ut_per_a: [[0.0; 3]; 4],
      rotor_field_ut: [[0.0; 3]; 4],
      rotor_harmonic: 1,
      noise_std_ut: 0.1,
      range_ut: 1500.0,
      delay_ns: 1_000_000,
    }
  }
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MagneticDipole {
  pub wheel: usize,
  pub position_body_m: [f64; 3],
  pub current_moment_am2_per_a: [f64; 3],
  pub rotor_moment_am2: [f64; 3],
  pub rotation_axis_body: [f64; 3],
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct DeviceProfiles {
  pub imus: Vec<ImuDevice>,
  pub magnetometer: MagnetometerConfig,
}
impl Default for DeviceProfiles {
  fn default() -> Self {
    Self {
      imus: Vec::new(),
      magnetometer: MagnetometerConfig::default(),
    }
  }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RobotOverride {
  pub team: crate::TeamColor,
  pub id: usize,
  pub motor: Option<MotorConfig>,
  pub hardware: Option<HardwareProfile>,
  pub devices: Option<DeviceProfiles>,
  pub provenance: Provenance,
}

impl DeviceProfiles {
  pub fn dual_imu(base: &ImuConfig) -> Self {
    let device = |stream, physical| ImuDevice {
      stream,
      physical,
      timing: SensorTiming::default(),
      scale_error: [0.0; 3],
      cross_axis: [[0.0; 3]; 3],
      quantization_step: 0.0,
      temperature_bias_per_c: [0.0; 3],
      vibration_amplitude: [0.0; 3],
      vibration_frequency_hz: 0.0,
      faults: Vec::new(),
    };
    let mut gyro = base.clone();
    gyro.delivery_delay_ns = 0;
    gyro.rate_hz = 1000;
    gyro.filter_time_constant_s = 0.0025;
    gyro.position_body_m = [0.03, 0.0, 0.0];
    gyro.gyro_noise_density_rad_s_sqrt_hz = 0.000244;
    let mut accel = gyro.clone();
    accel.rate_hz = 1600;
    accel.filter_time_constant_s = 0.0011;
    accel.accel_range_m_s2 = 58.84;
    Self {
      imus: vec![
        device(ImuStream::X16, {
          let mut x = base.clone();
          x.delivery_delay_ns = 0;
          x
        }),
        device(ImuStream::BmiGyro, gyro),
        device(ImuStream::BmiAccel, accel),
      ],
      magnetometer: MagnetometerConfig::default(),
    }
  }
}

impl HardwareProfile {
  pub fn validate(&self) -> Result<(), String> {
    self.wheel.validate()?;
    if self
      .mass_kg
      .is_some_and(|mass| !mass.is_finite() || mass <= 0.0)
    {
      return Err("hardware mass_kg must be finite and positive".into());
    }
    let positive = [
      self.support_stiffness_n_m,
      self.support_damping_n_s_m,
      self.battery.capacity_coulomb,
      self.battery.open_circuit_voltage_v,
      self.battery.thermal_capacity_j_k,
      self.battery.motor_thermal_capacity_j_k,
      self.auxiliaries.dribbler_time_constant_s,
    ];
    if positive.iter().any(|v| !v.is_finite() || *v <= 0.0) {
      return Err(
        "hardware stiffness, damping, capacities, and time constants must be positive".into(),
      );
    }
    let nonnegative = [
      self.rolling_resistance,
      self.transverse_friction,
      self.battery.internal_resistance_ohm,
      self.battery.electronics_power_w,
      self.battery.cooling_w_k,
      self.battery.motor_cooling_w_k,
      self.auxiliaries.kick_energy_j,
      self.auxiliaries.dribbler_pull_n,
      self.auxiliaries.dribbler_max_force_n,
      self.auxiliaries.dribbler_stiffness_n_m,
      self.auxiliaries.dribbler_damping_n_s_m,
      self.auxiliaries.dribbler_spin_torque_nm,
      self.auxiliaries.dribbler_power_w,
      self.auxiliaries.charging_power_w,
    ];
    if nonnegative.iter().any(|v| !v.is_finite() || *v < 0.0) {
      return Err("hardware dissipation/energy values must be nonnegative".into());
    }
    if !self.battery.ambient_temperature_c.is_finite()
      || !self.battery.motor_shutdown_temperature_c.is_finite()
      || self.battery.motor_shutdown_temperature_c <= self.battery.ambient_temperature_c
    {
      return Err("invalid thermal limits".into());
    }
    if !self.battery.brownout_voltage_v.is_finite()
      || !self.battery.recovery_voltage_v.is_finite()
      || self.battery.brownout_voltage_v < 0.0
      || self.battery.recovery_voltage_v <= self.battery.brownout_voltage_v
      || self.battery.recovery_voltage_v >= self.battery.open_circuit_voltage_v
    {
      return Err("invalid battery brownout/recovery thresholds".into());
    }
    if self
      .center_of_mass_body_m
      .is_some_and(|p| p.iter().any(|v| !v.is_finite()))
      || self
        .inertia_diagonal_kg_m2
        .is_some_and(|p| p.iter().any(|v| !v.is_finite() || *v <= 0.0))
    {
      return Err("invalid COM/inertia".into());
    }
    Ok(())
  }
}
impl DeviceProfiles {
  pub fn validate(&self) -> Result<(), String> {
    let mut streams = std::collections::HashSet::new();
    for device in &self.imus {
      if !streams.insert(device.stream as u8) {
        return Err("duplicate physical IMU stream".into());
      }
      let p = &device.physical;
      let t = &device.timing;
      if p.rate_hz == 0
        || p.rate_hz > 100000
        || p.odr_error_ppm.abs_diff(0) > 100000
        || p.fifo_watermark == 0
        || p.fifo_watermark > 128
        || t.clock.counter_bits == 0
        || t.clock.counter_bits > 64
        || t.clock.tick_hz == 0
        || t.wakeup_trace_ns.is_empty()
        || t.wakeup_trace_ns.iter().any(|v| *v < -1)
      {
        return Err("invalid IMU rate/FIFO/clock/readout profile".into());
      }
      if device.stream == ImuStream::BmiGyro && t.source == TimestampSource::NativeCounter {
        return Err(
          "BMI gyro native counter is unavailable in this backend; use host/sample-count timing"
            .into(),
        );
      }
      if !t.clock.skew_ppm.is_finite()
        || t.clock.skew_ppm.abs() > 100000.0
        || !t.clock.drift_ppm_per_s.is_finite()
      {
        return Err("invalid sensor clock skew/drift".into());
      }
      if [
        p.filter_time_constant_s,
        p.gyro_noise_density_rad_s_sqrt_hz,
        p.accel_noise_density_m_s2_sqrt_hz,
        device.quantization_step,
        device.vibration_frequency_hz,
      ]
      .iter()
      .any(|v| !v.is_finite() || *v < 0.0)
        || [p.gyro_range_rad_s, p.accel_range_m_s2]
          .iter()
          .any(|v| !v.is_finite() || *v <= 0.0)
      {
        return Err("invalid sensor response/noise/range".into());
      }
      for values in [
        p.position_body_m,
        p.mounting_rpy_rad,
        p.gyro_bias_rad_s,
        p.accel_bias_m_s2,
        device.scale_error,
        device.temperature_bias_per_c,
        device.vibration_amplitude,
      ] {
        if values.iter().any(|v| !v.is_finite()) {
          return Err("nonfinite device transform/error".into());
        }
      }
      if device.cross_axis.iter().flatten().any(|v| !v.is_finite())
        || device.faults.iter().any(|f| f.end_ns <= f.start_ns)
      {
        return Err("invalid cross-axis/fault configuration".into());
      }
    }
    let m = &self.magnetometer;
    if m
      .position_body_m
      .iter()
      .chain(&m.mounting_rpy_rad)
      .chain(&m.earth_field_world_ut)
      .chain(&m.hard_iron_ut)
      .chain(m.soft_iron.iter().flatten())
      .chain(m.current_field_ut_per_a.iter().flatten())
      .chain(m.rotor_field_ut.iter().flatten())
      .chain(&m.dribbler_field_ut)
      .chain(&m.charging_field_ut)
      .any(|v| !v.is_finite())
    {
      return Err("nonfinite magnetic field/geometry/calibration".into());
    }
    if m
      .ambient_gradient_ut_per_m
      .iter()
      .flatten()
      .any(|v| !v.is_finite())
    {
      return Err("nonfinite ambient magnetic gradient".into());
    }
    for d in &m.motor_dipoles {
      if d.wheel >= 4
        || d
          .position_body_m
          .iter()
          .chain(&d.current_moment_am2_per_a)
          .chain(&d.rotor_moment_am2)
          .chain(&d.rotation_axis_body)
          .any(|v| !v.is_finite())
        || d.rotation_axis_body.iter().map(|v| v * v).sum::<f64>() < 1e-9
        || (0..3)
          .map(|i| (m.position_body_m[i] - d.position_body_m[i]).powi(2))
          .sum::<f64>()
          < 1e-6
      {
        return Err("invalid magnetic source geometry/dipole".into());
      }
    }
    if m.rate_hz == 0
      || m.rate_hz > 100
      || !m.noise_std_ut.is_finite()
      || m.noise_std_ut < 0.0
      || !m.range_ut.is_finite()
      || m.range_ut <= 0.0
    {
      return Err("invalid LIS2MDL rate/noise/range".into());
    }
    Ok(())
  }
}
