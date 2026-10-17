//! Device clock truth and host-visible synchronization are deliberately separate.
use super::{profile::ClockConfig, time::TimeNs};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

pub struct DeviceClock {
  config: ClockConfig,
  reset_at: TimeNs,
  epoch: u64,
}
impl DeviceClock {
  pub fn new(config: ClockConfig) -> Self {
    Self {
      config,
      reset_at: 0,
      epoch: 0,
    }
  }
  pub fn reset(&mut self, at: TimeNs) {
    self.reset_at = at;
    self.epoch = self.epoch.wrapping_add(1);
  }
  pub fn epoch(&self) -> u64 {
    self.epoch
  }
  pub fn counter(&self, at: TimeNs) -> u64 {
    self.counter_fine(at, 0.0)
  }
  /// Fractional propagation offsets retain native fine resolution instead of
  /// rounding time-of-flight to the event queue's nanosecond grid.
  pub fn counter_fine(&self, at: TimeNs, offset_s: f64) -> u64 {
    let seconds = at.saturating_sub(self.reset_at) as f64 * 1e-9 + offset_s;
    let ticks = seconds
      * self.config.tick_hz as f64
      * (1.0 + self.config.skew_ppm * 1e-6 + 0.5 * self.config.drift_ppm_per_s * 1e-6 * seconds);
    let modulus = 1_u128 << self.config.counter_bits;
    ((ticks.max(0.0).floor() as u128 + u128::from(self.config.offset_ticks)) % modulus) as u64
  }
}
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ClockEstimate {
  pub common_time_ns: TimeNs,
  pub uncertainty_ns: TimeNs,
  pub epoch: u64,
}

/// Fits only (native counter, kernel timestamp) pairs a real driver could see.
/// Native resets are signaled by device status; wrap is unwrapped within half a
/// counter period. Long blackouts beyond that are reported as ambiguous.
pub struct ClockMapper {
  tick_hz: u64,
  bits: u8,
  last: Option<u64>,
  unwrapped: u128,
  epoch: Option<u64>,
  pairs: VecDeque<(f64, f64, u64)>,
  estimate: Option<ClockEstimate>,
  slope_ns_tick: f64,
}
impl ClockMapper {
  pub fn new(tick_hz: u64, bits: u8) -> Self {
    Self {
      tick_hz,
      bits,
      last: None,
      unwrapped: 0,
      epoch: None,
      pairs: VecDeque::new(),
      estimate: None,
      slope_ns_tick: 1e9 / tick_hz as f64,
    }
  }
  pub fn observe(
    &mut self,
    counter: u64,
    epoch: u64,
    kernel_at: TimeNs,
    uncertainty_ns: TimeNs,
  ) -> Result<ClockEstimate, String> {
    if self.epoch != Some(epoch) {
      self.last = None;
      self.unwrapped = 0;
      self.pairs.clear();
      self.epoch = Some(epoch);
    }
    let modulus = 1_u128 << self.bits;
    if let Some(last) = self.last {
      let delta = (u128::from(counter) + modulus - u128::from(last)) % modulus;
      if delta >= modulus / 2 {
        return Err("native clock ordering/wrap is ambiguous".into());
      }
      self.unwrapped += delta;
    } else {
      self.unwrapped = u128::from(counter);
    }
    self.last = Some(counter);
    let x = self.unwrapped as f64;
    self.pairs.push_back((x, kernel_at as f64, uncertainty_ns));
    if self.pairs.len() > 32 {
      self.pairs.pop_front();
    }
    let (base_x, base_y, _) = self.pairs[0];
    let nominal = 1e9 / self.tick_hz as f64;
    let mut slopes: Vec<f64> = self
      .pairs
      .iter()
      .filter_map(|(xx, yy, _)| ((*xx - base_x) > 0.0).then_some((*yy - base_y) / (*xx - base_x)))
      .filter(|s| s.is_finite() && *s > nominal * 0.8 && *s < nominal * 1.2)
      .collect();
    slopes.sort_by(f64::total_cmp);
    let slope = slopes.get(slopes.len() / 2).copied().unwrap_or(nominal);
    let mut offsets: Vec<f64> = self
      .pairs
      .iter()
      .map(|(xx, yy, _)| yy - (xx - base_x) * slope)
      .collect();
    offsets.sort_by(f64::total_cmp);
    let offset = offsets[offsets.len() / 2];
    let residual = self
      .pairs
      .iter()
      .map(|(xx, yy, u)| (yy - (offset + (xx - base_x) * slope)).abs() + *u as f64)
      .fold(0.0, f64::max);
    let estimate = ClockEstimate {
      common_time_ns: (offset + (x - base_x) * slope).max(0.0).round() as u64,
      uncertainty_ns: residual.ceil() as u64,
      epoch,
    };
    self.estimate = Some(estimate);
    self.slope_ns_tick = slope;
    Ok(estimate)
  }
  pub fn estimate_earlier(&self, counter: u64) -> Result<ClockEstimate, String> {
    let last = self
      .last
      .ok_or("clock has no synchronization observation")?;
    let modulus = 1_u128 << self.bits;
    let difference = (u128::from(last) + modulus - u128::from(counter)) % modulus;
    if difference >= modulus / 2 {
      return Err("sample counter age is ambiguous".into());
    }
    let estimate = self.estimate.ok_or("clock has no estimate")?;
    Ok(ClockEstimate {
      common_time_ns: (estimate.common_time_ns as f64 - difference as f64 * self.slope_ns_tick)
        .max(0.0)
        .round() as u64,
      ..estimate
    })
  }
}

/// Injected clock passed to device/control timeouts, including during blackouts.
#[derive(Debug, Clone, Copy)]
pub struct SimulationClock {
  pub now_ns: TimeNs,
}
impl SimulationClock {
  pub fn expired(self, deadline_ns: TimeNs) -> bool {
    self.now_ns >= deadline_ns
  }
}
