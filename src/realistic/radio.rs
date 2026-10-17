//! Half-duplex channel arbitration and timestamped UWB exchanges.
use super::{
  clocks::DeviceClock,
  localization::{BearingFrame, RangingProtocol, UwbConfig, UwbObservation},
  time::TimeNs,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct RadioDiagnostics {
  /// Reserved exchanges, including response/turnaround reservations.
  pub transmissions: u64,
  pub phy_frames: u64,
  pub rf_airtime_ns: u64,
  pub collisions: u64,
  pub missed_sync: u64,
  pub telemetry_delivered: u64,
  pub ranging_delivered: u64,
  pub responses_missed: u64,
  pub occupied_ns: u64,
}
#[derive(Default)]
pub struct MacArbiter {
  next: u64,
  active: Vec<(u64, TimeNs)>,
  valid: BTreeMap<u64, bool>,
  pub diagnostics: RadioDiagnostics,
}
impl MacArbiter {
  pub fn frame(&mut self, airtime_ns: u64) {
    self.diagnostics.phy_frames += 1;
    self.diagnostics.rf_airtime_ns += airtime_ns;
  }
  pub fn reserve(&mut self, at: TimeNs, duration: TimeNs) -> u64 {
    self.active.retain(|(_, end)| *end > at);
    let ticket = self.next;
    self.next = self.next.checked_add(1).expect("radio event ID exhausted");
    let mut valid = true;
    for (id, _) in &self.active {
      if let Some(valid) = self.valid.get_mut(id) {
        *valid = false;
      }
      valid = false;
    }
    if !valid {
      self.diagnostics.collisions += 1;
    }
    self.valid.insert(ticket, valid);
    self.active.push((ticket, at.saturating_add(duration)));
    self.diagnostics.transmissions += 1;
    self.diagnostics.occupied_ns += duration;
    ticket
  }
  pub fn valid(&self, ticket: u64) -> bool {
    self.valid.get(&ticket).copied().unwrap_or(false)
  }
  pub fn finish(&mut self, ticket: u64) -> bool {
    self.valid.remove(&ticket).unwrap_or(false)
  }
}
pub(crate) struct RangeJob {
  pub robot: usize,
  pub anchor: usize,
  pub ticket: u64,
  pub slot_end: TimeNs,
  pub bias_m: f64,
  pub observation: UwbObservation,
}
pub(crate) fn stamp(clock: &super::profile::ClockConfig, at: TimeNs, offset_s: f64) -> u64 {
  DeviceClock::new(clock.clone()).counter_fine(at, offset_s)
}
pub(crate) fn observation(config: &UwbConfig, at: TimeNs, anchor: u32) -> UwbObservation {
  UwbObservation {
    sampled_at: at,
    anchor_id: anchor,
    protocol: config.protocol,
    tick_hz: config.clock.tick_hz,
    counter_bits: config.clock.counter_bits,
    timestamps: [0; 6],
    antenna_correction_ns: config.assumed_antenna_delay_ns,
    range_std_m: config.range_noise_std_m,
    bearing_rad: None,
    bearing_frame: config.bearing_frame,
    bearing_std_rad: config.bearing_noise_std_rad,
    ambiguous_pi: config.bearing_ambiguity_pi,
    quality: 1.0,
    tdoa_reference: None,
  }
}
impl UwbConfig {
  pub fn exchange_duration_ns(&self) -> u64 {
    match self.protocol {
      RangingProtocol::SingleSided => 2 * self.frame_airtime_ns + self.turnaround_ns + 1000,
      RangingProtocol::DoubleSided => 3 * self.frame_airtime_ns + 2 * self.turnaround_ns + 1000,
      RangingProtocol::Tdoa => self.frame_airtime_ns + 1000,
    }
  }
  pub fn validate(&self) -> Result<(), String> {
    if self.clock.tick_hz == 0
      || self.clock.counter_bits == 0
      || self.clock.counter_bits > 64
      || self.frame_airtime_ns == 0
      || self.synchronization_timeout_ns == 0
    {
      return Err("invalid UWB clock/frame/synchronization".into());
    }
    if self.enabled && self.anchors.is_empty() {
      return Err("UWB localization needs explicit anchors".into());
    }
    let mut ids = std::collections::HashSet::new();
    for anchor in &self.anchors {
      if !ids.insert(anchor.id)
        || anchor.position_m.iter().any(|v| !v.is_finite())
        || anchor
          .estimated_position_m
          .is_some_and(|p| p.iter().any(|v| !v.is_finite()))
        || [
          anchor.antenna_delay_ns,
          anchor.yaw_rad,
          anchor.clock.skew_ppm,
          anchor.clock.drift_ppm_per_s,
        ]
        .iter()
        .any(|v| !v.is_finite())
        || anchor.clock.tick_hz != self.clock.tick_hz
        || anchor.clock.counter_bits != self.clock.counter_bits
      {
        return Err("invalid anchor geometry/native clock".into());
      }
    }
    if [
      self.antenna_delay_ns,
      self.assumed_antenna_delay_ns,
      self.bearing_bias_rad,
      self.antenna_yaw_body_rad,
      self.clock.skew_ppm,
      self.clock.drift_ppm_per_s,
    ]
    .iter()
    .any(|v| !v.is_finite())
    {
      return Err("invalid UWB clock/calibration".into());
    }
    if [
      self.range_noise_std_m,
      self.nlos_bias_m,
      self.bearing_noise_std_rad,
    ]
    .iter()
    .any(|v| !v.is_finite() || *v < 0.0)
      || !self.loss_probability.is_finite()
      || !(0.0..=1.0).contains(&self.loss_probability)
    {
      return Err("invalid UWB uncertainty/loss".into());
    }
    Ok(())
  }
}
pub(crate) fn bearing(
  config: &UwbConfig,
  robot_position: nalgebra::Vector3<f64>,
  robot_orientation: nalgebra::UnitQuaternion<f64>,
  anchor: &super::localization::Anchor,
) -> f64 {
  let delta = nalgebra::Vector3::from(anchor.position_m) - robot_position;
  let angle = match config.bearing_frame {
    BearingFrame::RobotArray => {
      let local = robot_orientation.inverse() * delta;
      local.y.atan2(local.x) - config.antenna_yaw_body_rad
    }
    BearingFrame::AnchorArray => (-delta.y).atan2(-delta.x) - anchor.yaw_rad,
  };
  super::vision::wrap(angle + config.bearing_bias_rad)
}
