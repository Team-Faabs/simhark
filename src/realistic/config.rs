//! Versioned profiles. Defaults are exploratory parameters, not hardware calibration.
use super::time::SECOND;
pub use crate::config::FloorConfig;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RealisticConfig {
  pub schema_version: u32,
  pub profile_layers: Vec<super::profiles::ProfileLayer>,
  pub provenance: Provenance,
  pub parameter_provenance: std::collections::BTreeMap<String, Provenance>,
  pub known_calibration: super::profiles::KnownCalibration,
  pub physics_step_ns: u64,
  pub control_rate_hz: u32,
  pub publication_rate_hz: u32,
  pub ai_rate_hz: u32,
  pub ai_processing_ns: u64,
  pub ai_processing: super::processing::ProcessingConfig,
  pub command_ttl_ns: u64,
  pub watchdog_ns: u64,
  pub max_pending_events: usize,
  pub record_raw_sensors: bool,
  pub record_raw_packets: bool,
  pub transport: TransportConfig,
  pub motor: MotorConfig,
  pub motor_adc: super::profile::MotorAdcConfig,
  pub contact: ContactConfig,
  pub imu: ImuConfig,
  pub vision: VisionConfig,
  pub hardware: super::profile::HardwareProfile,
  pub devices: super::profile::DeviceProfiles,
  pub robot_overrides: Vec<super::profile::RobotOverride>,
  pub cameras: Vec<super::vision::CameraConfig>,
  pub camera_timing_priors: Vec<super::calibration::CameraTimingPrior>,
  pub external_vision: bool,
  pub tracker: super::vision::TrackerConfig,
  pub camera_processing: super::processing::ProcessingConfig,
  pub tracker_processing: super::processing::ProcessingConfig,
  pub network: super::network::NetworkConfig,
  pub environment: super::scene::Environment,
  pub lidar: super::localization::LidarConfig,
  pub uwb: super::localization::UwbConfig,
  pub onboard_camera: super::localization::OnboardCameraConfig,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Provenance {
  pub classification: ParameterSource,
  pub source: String,
  pub owner: String,
}
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ParameterSource {
  Measured,
  Datasheet,
  Provisional,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct TransportConfig {
  /// Joint samples indexed by logical publication, preserving correlation
  /// across radio copies. Empty selects the separate exploratory traces.
  pub joint_delay_trace: Vec<JointDelay>,
  pub superframe_ns: u64,
  pub guard_ns: u64,
  pub slots: Vec<RadioSlot>,
  pub command_format: CommandFormat,
  /// Empirical end-to-end traces: -1 means loss. Repeated cyclically in time order.
  pub wifi_delay_trace_ns: Vec<i64>,
  pub uwb_host_delay_trace_ns: Vec<i64>,
  /// Common processing before the radio split, preserving correlated stalls.
  pub common_delay_trace_ns: Vec<i64>,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct JointDelay {
  pub common_delay_ns: i64,
  pub wifi_post_split_ns: i64,
  pub uwb_post_airtime_ns: i64,
}

/// Explicit compact synthetic format used for capacity checks; this is not an
/// assertion about a QM35825 firmware API or the size of SSL protobuf packets.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct CommandFormat {
  pub header_bytes: u32,
  pub bytes_per_robot: u32,
  pub bitrate_bits_per_second: u64,
  pub preamble_and_phy_header_ns: u64,
}
impl Default for CommandFormat {
  fn default() -> Self {
    Self {
      header_bytes: 8,
      bytes_per_robot: 8,
      bitrate_bits_per_second: 6_800_000,
      preamble_and_phy_header_ns: 80_000,
    }
  }
}
impl CommandFormat {
  pub fn airtime_ns(&self, robots: usize) -> Result<u64, String> {
    if self.bitrate_bits_per_second == 0 || self.bytes_per_robot == 0 {
      return Err("command bitrate and bytes_per_robot must be positive".into());
    }
    let bytes = u128::from(self.header_bytes) + robots as u128 * u128::from(self.bytes_per_robot);
    let payload_ns =
      (bytes * 8 * u128::from(SECOND)).div_ceil(u128::from(self.bitrate_bits_per_second));
    u64::try_from(payload_ns + u128::from(self.preamble_and_phy_header_ns))
      .map_err(|_| "command airtime overflow".into())
  }
}

impl MotorConfig {
  pub fn validate(&self) -> Result<(), String> {
    if [
      self.supply_voltage_v,
      self.resistance_ohm,
      self.inductance_h,
      self.torque_constant_nm_per_a,
      self.back_emf_v_per_rad_s,
      self.wheel_inertia_kg_m2,
      self.trip_current_a,
      self.current_filter_time_constant_s,
    ]
    .iter()
    .any(|v| !v.is_finite() || *v <= 0.0)
      || [
        self.viscous_drag_nm_per_rad_s,
        self.speed_gain_v_per_rad_s,
        self.peak_margin_a,
      ]
      .iter()
      .any(|v| !v.is_finite() || *v < 0.0)
      || self
        .current_limit_a
        .is_some_and(|v| !v.is_finite() || v <= 0.0)
      || self.encoder_ticks_per_revolution == 0
      || self.reset_duration_ns == 0
    {
      return Err("invalid motor/driver calibration".into());
    }
    Ok(())
  }
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RadioSlot {
  #[serde(default = "one_slot_period")]
  pub every_superframes: u32,
  #[serde(default)]
  pub phase_superframe: u32,
  pub offset_ns: u64,
  /// Includes preamble, headers, payload, turnaround, and response when applicable.
  pub occupied_ns: u64,
  pub purpose: SlotPurpose,
}
fn one_slot_period() -> u32 {
  1
}
impl Default for RadioSlot {
  fn default() -> Self {
    Self {
      every_superframes: 1,
      phase_superframe: 0,
      offset_ns: 0,
      occupied_ns: 1,
      purpose: SlotPurpose::Telemetry,
    }
  }
}
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SlotPurpose {
  BlueCommand,
  YellowCommand,
  Telemetry,
  Ranging,
  Synchronization,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TripCurrent {
  #[default]
  PhasePeak,
  PhaseFiltered,
  Supply,
}
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FaultScope {
  #[default]
  Motor,
  AllWheels,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct MotorConfig {
  pub supply_voltage_v: f64,
  pub resistance_ohm: f64,
  pub inductance_h: f64,
  /// Effective constants and inertia referred to the wheel shaft (includes gearing).
  pub torque_constant_nm_per_a: f64,
  pub back_emf_v_per_rad_s: f64,
  pub wheel_inertia_kg_m2: f64,
  pub viscous_drag_nm_per_rad_s: f64,
  pub speed_gain_v_per_rad_s: f64,
  pub trip_current_a: f64,
  pub trip_current_type: TripCurrent,
  pub current_filter_time_constant_s: f64,
  pub current_limit_a: Option<f64>,
  pub fault_scope: FaultScope,
  pub trip_exposure_ns: u64,
  pub reset_duration_ns: u64,
  pub peak_margin_a: f64,
  pub encoder_ticks_per_revolution: u32,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct ContactConfig {
  pub floor: FloorConfig,
  pub friction: f64,
  pub slip_stiffness_n_per_m_s: f64,
  pub center_of_mass_height_m: f64,
  /// Planar load-transfer surrogate relaxation, not a full 3D tip model.
  pub load_transfer_time_constant_s: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct ImuConfig {
  pub rate_hz: u32,
  pub phase_ns: u64,
  pub odr_error_ppm: i32,
  pub fifo_watermark: usize,
  pub delivery_delay_ns: u64,
  /// Causal first-order response; do not also subtract a nominal group delay.
  pub filter_time_constant_s: f64,
  pub gyro_noise_density_rad_s_sqrt_hz: f64,
  pub accel_noise_density_m_s2_sqrt_hz: f64,
  pub gyro_bias_rad_s: [f64; 3],
  pub accel_bias_m_s2: [f64; 3],
  pub gyro_range_rad_s: f64,
  pub accel_range_m_s2: f64,
  pub position_body_m: [f64; 3],
  /// Rotation from sensor coordinates to chassis coordinates (roll, pitch, yaw).
  pub mounting_rpy_rad: [f64; 3],
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct VisionConfig {
  pub rate_hz: u32,
  pub delay_ns: u64,
  pub position_noise_std_m: f64,
  pub orientation_noise_std_rad: f64,
  pub dropout_probability: f64,
}

impl Default for RealisticConfig {
  fn default() -> Self {
    Self {
      schema_version: 1,
      parameter_provenance: Default::default(),
      profile_layers: Vec::new(),
      known_calibration: Default::default(),
      provenance: Provenance {
        classification: ParameterSource::Provisional,
        source: "Exploratory surrogate; no assembled-robot measurements".into(),
        owner: "teamfaabs".into(),
      },
      physics_step_ns: 250_000,
      control_rate_hz: 1000,
      publication_rate_hz: 1000,
      ai_rate_hz: 60,
      ai_processing_ns: 1_000_000,
      ai_processing: Default::default(),
      command_ttl_ns: 50_000_000,
      watchdog_ns: 100_000_000,
      record_raw_sensors: false,
      record_raw_packets: false,
      max_pending_events: 100_000,
      transport: TransportConfig::default(),
      motor: MotorConfig::default(),
      motor_adc: Default::default(),
      contact: ContactConfig::default(),
      imu: ImuConfig::default(),
      vision: VisionConfig::default(),
      hardware: super::profile::HardwareProfile::default(),
      devices: super::profile::DeviceProfiles::default(),
      robot_overrides: Vec::new(),
      cameras: Vec::new(),
      camera_timing_priors: Vec::new(),
      external_vision: false,
      tracker: super::vision::TrackerConfig::default(),
      camera_processing: Default::default(),
      tracker_processing: Default::default(),
      network: super::network::NetworkConfig::default(),
      environment: Default::default(),
      lidar: Default::default(),
      uwb: Default::default(),
      onboard_camera: Default::default(),
    }
  }
}
impl Default for TransportConfig {
  fn default() -> Self {
    Self {
      joint_delay_trace: Vec::new(),
      superframe_ns: 1_000_000,
      guard_ns: 20_000,
      slots: vec![
        RadioSlot {
          every_superframes: 1,
          phase_superframe: 0,
          offset_ns: 0,
          occupied_ns: 200_000,
          purpose: SlotPurpose::BlueCommand,
        },
        RadioSlot {
          every_superframes: 1,
          phase_superframe: 0,
          offset_ns: 250_000,
          occupied_ns: 200_000,
          purpose: SlotPurpose::YellowCommand,
        },
        RadioSlot {
          every_superframes: 1,
          phase_superframe: 0,
          offset_ns: 500_000,
          occupied_ns: 300_000,
          purpose: SlotPurpose::Telemetry,
        },
      ],
      wifi_delay_trace_ns: vec![1_000_000],
      uwb_host_delay_trace_ns: vec![100_000],
      common_delay_trace_ns: vec![0],
      command_format: CommandFormat::default(),
    }
  }
}
impl Default for MotorConfig {
  fn default() -> Self {
    Self {
      supply_voltage_v: 12.0,
      resistance_ohm: 2.0,
      inductance_h: 0.001,
      torque_constant_nm_per_a: 0.025,
      back_emf_v_per_rad_s: 0.025,
      wheel_inertia_kg_m2: 0.0002,
      viscous_drag_nm_per_rad_s: 0.0001,
      speed_gain_v_per_rad_s: 0.2,
      trip_current_a: 8.0,
      trip_current_type: TripCurrent::PhasePeak,
      current_filter_time_constant_s: 0.002,
      current_limit_a: None,
      fault_scope: FaultScope::Motor,
      trip_exposure_ns: 500_000,
      reset_duration_ns: 10_000_000,
      peak_margin_a: 0.0,
      encoder_ticks_per_revolution: 2048,
    }
  }
}
impl Default for ContactConfig {
  fn default() -> Self {
    Self {
      floor: FloorConfig::default(),
      friction: 0.8,
      slip_stiffness_n_per_m_s: 25.0,
      center_of_mass_height_m: 0.05,
      load_transfer_time_constant_s: 0.02,
    }
  }
}
impl Default for ImuConfig {
  fn default() -> Self {
    Self {
      rate_hz: 1920,
      phase_ns: 0,
      odr_error_ppm: 0,
      fifo_watermark: 4,
      delivery_delay_ns: 200_000,
      filter_time_constant_s: 0.001,
      gyro_noise_density_rad_s_sqrt_hz: 0.00005,
      accel_noise_density_m_s2_sqrt_hz: 0.0006,
      gyro_bias_rad_s: [0.0; 3],
      accel_bias_m_s2: [0.0; 3],
      gyro_range_rad_s: 35.0,
      accel_range_m_s2: 78.48,
      position_body_m: [0.0; 3],
      mounting_rpy_rad: [0.0; 3],
    }
  }
}
impl Default for VisionConfig {
  fn default() -> Self {
    Self {
      rate_hz: 60,
      delay_ns: 15_000_000,
      position_noise_std_m: 0.002,
      orientation_noise_std_rad: 0.005,
      dropout_probability: 0.0,
    }
  }
}

impl RealisticConfig {
  pub fn full() -> Self {
    let mut config = Self::default();
    config.hardware.dynamics = super::profile::Dynamics::Free3d;
    config.devices = super::profile::DeviceProfiles::dual_imu(&config.imu);
    config.cameras = vec![
      super::vision::CameraConfig {
        position_world_m: [-2.5, 0.0, 5.0],
        ..Default::default()
      },
      super::vision::CameraConfig {
        id: 1,
        phase_ns: 8_000_000,
        position_world_m: [2.5, 0.0, 5.0],
        ..Default::default()
      },
    ];
    config.network.wifi = Some(super::network::LinkConfig::default());
    config.network.lan = Some(super::network::LinkConfig {
      bits_per_second: 1_000_000_000,
      latency: super::network::DelayDistribution::Constant { delay_ns: 50_000 },
      ..Default::default()
    });
    config.uwb.enabled = true;
    config.uwb.require_sync_for_commands = true;
    config.uwb.anchors = [
      [-4.5, -3.0, 3.0],
      [-4.5, 3.0, 3.0],
      [4.5, -3.0, 3.0],
      [4.5, 3.0, 3.0],
    ]
    .into_iter()
    .enumerate()
    .map(|(i, p)| super::localization::Anchor {
      id: i as u32,
      position_m: p,
      estimated_position_m: None,
      clock: config.uwb.clock.clone(),
      antenna_delay_ns: 0.0,
      yaw_rad: 0.0,
    })
    .collect();
    config.transport.slots.retain(|s| {
      matches!(
        s.purpose,
        SlotPurpose::BlueCommand | SlotPurpose::YellowCommand
      )
    });
    config.transport.slots.push(RadioSlot {
      offset_ns: 500_000,
      occupied_ns: 400_000,
      purpose: SlotPurpose::Ranging,
      every_superframes: 2,
      phase_superframe: 0,
    });
    config.transport.slots.push(RadioSlot {
      offset_ns: 500_000,
      occupied_ns: 300_000,
      purpose: SlotPurpose::Telemetry,
      every_superframes: 2,
      phase_superframe: 1,
    });
    config.environment.legacy_field_colliders = false;
    config.environment.standard_goals = true;
    config.environment.known_map =
      super::scene::Environment::goal_boxes(&crate::WorldConfig::division_b().field);
    config.lidar.enabled = true;
    config.onboard_camera.enabled = true;
    config
  }
  pub fn full_for_world(world: &crate::WorldConfig) -> Self {
    let mut config = Self::full();
    config.environment.known_map = super::scene::Environment::goal_boxes(&world.field);
    let x = world.field.field_length * 0.5;
    let y = world.field.field_width * 0.5;
    for (anchor, p) in
      config
        .uwb
        .anchors
        .iter_mut()
        .zip([[-x, -y, 3.0], [-x, y, 3.0], [x, -y, 3.0], [x, y, 3.0]])
    {
      anchor.position_m = p;
    }
    for (i, c) in config.cameras.iter_mut().enumerate() {
      c.position_world_m = [
        if i == 0 { -x * 0.55 } else { x * 0.55 },
        0.0,
        (world.field.field_width * 0.9).max(5.0),
      ];
    }
    config
  }
  pub fn from_toml(text: &str) -> Result<Self, String> {
    let config: Self = toml::from_str(text).map_err(|e| e.to_string())?;
    config.validate()?;
    Ok(config)
  }
  pub fn to_toml(&self) -> Result<String, String> {
    toml::to_string_pretty(self).map_err(|e| e.to_string())
  }
  pub fn validate(&self) -> Result<(), String> {
    if self.ai_rate_hz == 0 || self.ai_rate_hz > 10000 {
      return Err("invalid AI rate".into());
    }
    if self.publication_rate_hz == 0 || self.publication_rate_hz > 1_000_000 {
      return Err("invalid publication rate".into());
    }
    let adc = &self.motor_adc;
    if adc.rate_hz == 0
      || adc.rate_hz > 1_000_000
      || [
        adc.current_resolution_a,
        adc.current_noise_std_a,
        adc.voltage_resolution_v,
      ]
      .iter()
      .any(|v| !v.is_finite() || *v < 0.0)
      || !adc.current_range_a.is_finite()
      || adc.current_range_a <= 0.0
    {
      return Err("invalid motor ADC profile".into());
    }
    self.ai_processing.extra_service.validate()?;
    self.camera_processing.extra_service.validate()?;
    self.tracker_processing.extra_service.validate()?;
    if self.camera_processing.queued_capacity > 10000
      || self.tracker_processing.queued_capacity > 10000
    {
      return Err("processing queue too large".into());
    }
    if self
      .known_calibration
      .x16_mounting_rpy_rad
      .iter()
      .any(|v| !v.is_finite())
    {
      return Err("invalid known IMU mounting".into());
    }
    for p in self.parameter_provenance.values() {
      if p.source.trim().is_empty() || p.owner.trim().is_empty() {
        return Err("parameter provenance requires source and owner".into());
      }
    }
    self.motor.validate()?;
    if let Some(m) = &self.known_calibration.motor {
      m.validate()?;
    }
    self.uwb.validate()?;
    if self.uwb.enabled {
      let required = self.uwb.exchange_duration_ns();
      if !self
        .transport
        .slots
        .iter()
        .any(|s| s.purpose == SlotPurpose::Ranging)
      {
        return Err("UWB localization requires a ranging slot".into());
      }
      if self
        .transport
        .slots
        .iter()
        .any(|s| s.purpose == SlotPurpose::Ranging && s.occupied_ns < required)
      {
        return Err("UWB exchange frames/turnaround exceed ranging slot".into());
      }
      let f = &self.transport.command_format;
      if f.bitrate_bits_per_second == 0 {
        return Err("invalid radio bitrate".into());
      }
      let needed = f.preamble_and_phy_header_ns
        + ((u128::from(self.uwb.ranging_payload_bytes + f.header_bytes) * 8 * 1_000_000_000)
          .div_ceil(u128::from(f.bitrate_bits_per_second))) as u64;
      if self.uwb.frame_airtime_ns < needed {
        return Err("ranging PHY frame is shorter than configured payload/headers".into());
      }
      let telemetry = f.preamble_and_phy_header_ns
        + ((u128::from(self.uwb.telemetry_payload_bytes) + u128::from(f.header_bytes))
          * 8
          * 1_000_000_000)
          .div_ceil(u128::from(f.bitrate_bits_per_second)) as u64;
      if self.transport.slots.iter().any(|s| {
        matches!(
          s.purpose,
          SlotPurpose::Telemetry | SlotPurpose::Synchronization
        ) && s.occupied_ns < telemetry
      }) {
        return Err("telemetry/sync payload exceeds reserved slot".into());
      }
    }
    if self.lidar.enabled
      && (self.lidar.scan_rate_hz == 0
        || self.lidar.beams < 2
        || self.lidar.beams > 100000
        || self.lidar.scan_rate_hz > 1000
        || !self.lidar.min_range_m.is_finite()
        || !self.lidar.max_range_m.is_finite()
        || self.lidar.min_range_m < 0.0
        || self.lidar.max_range_m <= self.lidar.min_range_m)
    {
      return Err("invalid LiDAR scan geometry/rate".into());
    }
    if self.onboard_camera.enabled
      && (self.onboard_camera.rate_hz == 0 || self.onboard_camera.rate_hz > 1000)
    {
      return Err("invalid onboard camera rate".into());
    }
    for landmark in self
      .environment
      .physical
      .iter()
      .chain(&self.environment.known_map)
    {
      if !landmark.reflectivity.is_finite()
        || !(0.0..=1.0).contains(&landmark.reflectivity)
        || (0..3).any(|a| {
          !landmark.min_m[a].is_finite()
            || !landmark.max_m[a].is_finite()
            || landmark.min_m[a] >= landmark.max_m[a]
        })
      {
        return Err("invalid physical/known landmark geometry".into());
      }
    }
    if [
      self.tracker.measurement_variance_m2,
      self.tracker.acceleration_variance,
      self.tracker.gate_sigma,
      self.tracker.initial_velocity_variance,
    ]
    .iter()
    .any(|v| !v.is_finite() || *v <= 0.0)
      || self.tracker.max_age_ns == 0
    {
      return Err("invalid tracker noise/age/gate".into());
    }
    for value in self
      .lidar
      .position_body_m
      .iter()
      .chain(&self.lidar.mounting_rpy_rad)
      .chain(&self.onboard_camera.position_body_m)
    {
      if !value.is_finite() {
        return Err("nonfinite sensor mounting".into());
      }
    }
    if [
      self.lidar.range_resolution_m,
      self.lidar.range_noise_std_m,
      self.lidar.angular_resolution_rad,
      self.lidar.divergence_rad,
      self.onboard_camera.bearing_noise_std_rad,
      self.onboard_camera.range_fractional_noise,
    ]
    .iter()
    .any(|v| !v.is_finite() || *v < 0.0)
      || !self.lidar.dropout_probability.is_finite()
      || !(0.0..=1.0).contains(&self.lidar.dropout_probability)
      || self
        .lidar
        .initial_pose
        .is_some_and(|p| p.iter().any(|v| !v.is_finite()))
      || !self.onboard_camera.yaw_body_rad.is_finite()
      || !self.onboard_camera.horizontal_fov_rad.is_finite()
      || self.onboard_camera.horizontal_fov_rad <= 0.0
      || self.onboard_camera.horizontal_fov_rad > std::f64::consts::TAU
      || !self.onboard_camera.max_range_m.is_finite()
      || self.onboard_camera.max_range_m <= 0.0
    {
      return Err("invalid localization/ball sensor errors, field of view, or range".into());
    }
    self.hardware.validate()?;
    self.contact.floor.validate()?;
    self.devices.validate()?;
    let mut robot_ids = std::collections::HashSet::new();
    for robot in &self.robot_overrides {
      if !robot_ids.insert((robot.team, robot.id))
        || robot.provenance.source.trim().is_empty()
        || robot.provenance.owner.trim().is_empty()
      {
        return Err("duplicate robot override or missing provenance".into());
      }
      if let Some(m) = &robot.motor {
        m.validate()?;
      }
      if let Some(h) = &robot.hardware {
        h.validate()?;
      }
      if let Some(d) = &robot.devices {
        d.validate()?;
      }
    }
    if let Some(link) = &self.network.wifi {
      link.validate()?;
    }
    if let Some(link) = &self.network.lan {
      link.validate()?;
    }
    if !self.network.bulk.is_empty() && self.network.wifi.is_none() {
      return Err("bulk traffic requires a shared WiFi resource".into());
    }
    for traffic in &self.network.bulk {
      if traffic.rate_hz == 0
        || traffic.rate_hz > 100000
        || traffic.end_ns <= traffic.start_ns
        || traffic.bytes_per_burst == 0
      {
        return Err("invalid bulk traffic burst".into());
      }
    }
    let mut prior_ids = std::collections::HashSet::new();
    for prior in &self.camera_timing_priors {
      if !prior_ids.insert(prior.camera_id)
        || !self.cameras.iter().any(|c| c.id == prior.camera_id)
        || prior.samples.is_empty()
        || prior.provenance.source.trim().is_empty()
        || prior.provenance.owner.trim().is_empty()
        || prior.samples.iter().any(|s| {
          s.interval_ns.is_some_and(|v| v == 0 || v > 10_000_000_000)
            || s.image_receive_to_send_ns > 10_000_000_000
        })
      {
        return Err("invalid camera timing prior/identity/boundary".into());
      }
    }
    let mut camera_ids = std::collections::HashSet::new();
    for camera in &self.cameras {
      camera.validate()?;
      if !camera_ids.insert(camera.id)
        || camera.rate_hz == 0
        || camera.rate_hz > 100000
        || camera.width == 0
        || camera.height == 0
        || camera.focal_px.iter().any(|v| !v.is_finite() || *v <= 0.0)
        || camera.position_world_m.iter().any(|v| !v.is_finite())
        || [
          camera.dropout_probability,
          camera.id_swap_probability,
          camera.heading_flip_probability,
          camera.false_detection_probability,
        ]
        .iter()
        .any(|v| !v.is_finite() || !(0.0..=1.0).contains(v))
      {
        return Err("invalid/duplicate camera configuration".into());
      }
    }
    fn positive(name: &str, value: f64) -> Result<(), String> {
      if value.is_finite() && value > 0.0 {
        Ok(())
      } else {
        Err(format!("{name} must be finite and positive"))
      }
    }
    fn nonnegative(name: &str, value: f64) -> Result<(), String> {
      if value.is_finite() && value >= 0.0 {
        Ok(())
      } else {
        Err(format!("{name} must be finite and nonnegative"))
      }
    }
    if self.schema_version != 1 {
      return Err("unsupported realism schema_version".into());
    }
    if self.provenance.source.trim().is_empty() || self.provenance.owner.trim().is_empty() {
      return Err("provenance needs a source and owner".into());
    }
    if self.physics_step_ns == 0 || self.physics_step_ns > 1_000_000 {
      return Err(
        "physics_step_ns must be in 1..=1000000; validate convergence for the selected step".into(),
      );
    }
    for (name, rate) in [
      ("control", self.control_rate_hz),
      ("IMU", self.imu.rate_hz),
      ("vision", self.vision.rate_hz),
    ] {
      if rate == 0 || rate > 100_000 {
        return Err(format!("{name} rate must be in 1..=100000 Hz"));
      }
    }
    if self.command_ttl_ns == 0 || self.watchdog_ns == 0 || self.max_pending_events == 0 {
      return Err("TTL, watchdog, and event capacity must be positive".into());
    }
    let t = &self.transport;
    for sample in &t.joint_delay_trace {
      if [
        sample.common_delay_ns,
        sample.wifi_post_split_ns,
        sample.uwb_post_airtime_ns,
      ]
      .iter()
      .any(|v| *v < -1 || *v > 10_000_000_000)
      {
        return Err("joint delay samples must be -1..=10000000000 ns".into());
      }
    }
    if !t.joint_delay_trace.is_empty()
      && self.network.wifi.is_some()
      && !self.network.empirical_commands
    {
      return Err("joint timing traces require network.empirical_commands=true when a shared WiFi model is configured".into());
    }

    t.command_format.airtime_ns(0)?;
    if t.superframe_ns == 0 || t.superframe_ns > SECOND {
      return Err("invalid UWB superframe".into());
    }
    for (i, slot) in t.slots.iter().enumerate() {
      if slot.occupied_ns == 0
        || slot.every_superframes == 0
        || slot.every_superframes > 1024
        || slot.phase_superframe >= slot.every_superframes
      {
        return Err("invalid UWB slot duration/cadence".into());
      }
      let end = slot
        .offset_ns
        .checked_add(slot.occupied_ns)
        .and_then(|n| n.checked_add(t.guard_ns))
        .ok_or("UWB budget overflow")?;
      if end > t.superframe_ns {
        return Err("UWB airtime/guards exceed superframe".into());
      }
      for other in t.slots.iter().take(i) {
        let mut a = slot.every_superframes;
        let mut b = other.every_superframes;
        while b != 0 {
          let r = a % b;
          a = b;
          b = r;
        }
        let coincident = slot.phase_superframe.abs_diff(other.phase_superframe) % a == 0;
        if coincident
          && slot.offset_ns < other.offset_ns + other.occupied_ns + t.guard_ns
          && other.offset_ns < end
        {
          return Err("UWB slots/guards overlap".into());
        }
      }
    }
    for purpose in [SlotPurpose::BlueCommand, SlotPurpose::YellowCommand] {
      if t.slots.iter().filter(|s| s.purpose == purpose).count() != 1 {
        return Err("exactly one broadcast command slot per team is required".into());
      }
    }
    for trace in [
      &t.wifi_delay_trace_ns,
      &t.uwb_host_delay_trace_ns,
      &t.common_delay_trace_ns,
    ] {
      if trace.is_empty() || trace.iter().any(|&v| v < -1 || v > 10 * SECOND as i64) {
        return Err("delay traces need values in -1..=10000000000 ns (-1 is loss)".into());
      }
    }
    let m = &self.motor;
    for (name, value) in [
      ("supply voltage", m.supply_voltage_v),
      ("resistance", m.resistance_ohm),
      ("inductance", m.inductance_h),
      ("torque constant", m.torque_constant_nm_per_a),
      ("back EMF constant", m.back_emf_v_per_rad_s),
      ("wheel inertia", m.wheel_inertia_kg_m2),
      ("speed gain", m.speed_gain_v_per_rad_s),
      ("trip current", m.trip_current_a),
      ("current filter", m.current_filter_time_constant_s),
      ("friction", self.contact.friction),
      ("slip stiffness", self.contact.slip_stiffness_n_per_m_s),
      (
        "load transfer time",
        self.contact.load_transfer_time_constant_s,
      ),
      ("gyro range", self.imu.gyro_range_rad_s),
      ("accel range", self.imu.accel_range_m_s2),
    ] {
      positive(name, value)?;
    }
    for (name, value) in [
      ("motor drag", m.viscous_drag_nm_per_rad_s),
      ("peak margin", m.peak_margin_a),
      ("COM height", self.contact.center_of_mass_height_m),
      ("IMU filter", self.imu.filter_time_constant_s),
      ("gyro noise", self.imu.gyro_noise_density_rad_s_sqrt_hz),
      ("accel noise", self.imu.accel_noise_density_m_s2_sqrt_hz),
      ("vision position noise", self.vision.position_noise_std_m),
      ("vision angle noise", self.vision.orientation_noise_std_rad),
    ] {
      nonnegative(name, value)?;
    }
    if m
      .current_limit_a
      .is_some_and(|v| !v.is_finite() || v <= 0.0)
    {
      return Err("invalid current limit".into());
    }
    if m.encoder_ticks_per_revolution == 0
      || m.reset_duration_ns == 0
      || self.imu.fifo_watermark == 0
      || self.imu.fifo_watermark > 128
    {
      return Err("invalid encoder resolution, reset duration, or FIFO watermark".into());
    }
    if self.imu.odr_error_ppm.abs_diff(0) > 100_000 {
      return Err("ODR error exceeds 10%".into());
    }
    if !self.vision.dropout_probability.is_finite()
      || !(0.0..=1.0).contains(&self.vision.dropout_probability)
    {
      return Err("vision dropout_probability must be in 0..=1".into());
    }
    for values in [
      self.imu.gyro_bias_rad_s,
      self.imu.accel_bias_m_s2,
      self.imu.position_body_m,
      self.imu.mounting_rpy_rad,
    ] {
      if values.iter().any(|v| !v.is_finite()) {
        return Err("IMU mounting and biases must be finite".into());
      }
    }
    Ok(())
  }
}
