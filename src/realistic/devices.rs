//! FIFO/interrupt/bus driver boundary. Hidden clock truth never enters a mapper.
use super::{
  clocks::{ClockMapper, DeviceClock},
  profile::{ImuDevice, SensorFaultKind, TimestampSource},
  sensors::{self, ImuModel, ImuMotion, ImuSample, SampleTiming},
  time::TimeNs,
  transport::DelayTrace,
};
use rand::RngExt;
use rand_chacha::ChaCha8Rng;

pub(crate) struct DeviceState {
  pub config: ImuDevice,
  pub legacy: bool,
  model: ImuModel,
  clock: DeviceClock,
  mapper: ClockMapper,
  wakeup: DelayTrace,
  rng: ChaCha8Rng,
  watermarks: u64,
  reset_fault: Option<usize>,
  frozen: Option<ImuSample>,
  pub overflow_samples: u64,
  fifo: Vec<(u64, u64, ImuSample)>,
}
impl DeviceState {
  pub fn new(
    config: ImuDevice,
    legacy: bool,
    seed: u64,
    world: usize,
    robot: usize,
    device: u64,
  ) -> Self {
    let (tick, bits) = if config.timing.source == TimestampSource::SampleCount {
      (u64::from(config.physical.rate_hz), 64)
    } else {
      (
        config.timing.clock.tick_hz,
        config.timing.clock.counter_bits,
      )
    };
    Self {
      model: ImuModel::new(
        config.physical.clone(),
        sensors::rng(seed, world, robot, device),
      )
      .with_device(config.clone()),
      clock: DeviceClock::new(config.timing.clock.clone()),
      mapper: ClockMapper::new(tick, bits),
      wakeup: DelayTrace::new(config.timing.wakeup_trace_ns.clone()),
      rng: sensors::rng(seed, world, robot, device + 1000),
      config,
      legacy,
      watermarks: 0,
      reset_fault: None,
      frozen: None,
      overflow_samples: 0,
      fifo: Vec::new(),
    }
  }
  pub fn sample(
    &mut self,
    index: u64,
    at: TimeNs,
    physical_at: TimeNs,
    motion: ImuMotion,
    gravity: f64,
  ) -> Option<(TimeNs, Vec<ImuSample>)> {
    let fault = self
      .config
      .faults
      .iter()
      .enumerate()
      .find(|(_, f)| at >= f.start_ns && at < f.end_ns)
      .map(|(i, f)| (i, f.kind));
    if let Some((i, SensorFaultKind::ClockReset)) = fault
      && self.reset_fault != Some(i)
    {
      self.clock.reset(at);
      self.reset_fault = Some(i);
    }
    self.model.sample_motion(at, physical_at, motion, gravity);
    let mut sample = self.model.fifo.pop().unwrap();
    if fault.is_some_and(|(_, f)| f == SensorFaultKind::Missing) {
      return None;
    }
    if fault.is_some_and(|(_, f)| f == SensorFaultKind::Frozen) {
      if let Some(frozen) = &self.frozen {
        sample.angular_rad_s = frozen.angular_rad_s;
        sample.specific_force_m_s2 = frozen.specific_force_m_s2;
      }
    } else {
      self.frozen = Some(sample.clone());
    }
    if fault.is_some_and(|(_, f)| f == SensorFaultKind::Clipped) {
      sample.angular_rad_s = [self.config.physical.gyro_range_rad_s; 3];
      sample.specific_force_m_s2 = [self.config.physical.accel_range_m_s2; 3];
      sample.clipped = true;
    }
    let jitter = if self.legacy {
      0_i64
    } else {
      (self.rng.random::<f64>() * 2.0 * self.config.timing.kernel_capture_jitter_ns as f64
        - self.config.timing.kernel_capture_jitter_ns as f64)
        .round() as i64
    };
    let kernel = at
      .saturating_add(self.config.timing.interrupt_delay_ns)
      .saturating_add_signed(jitter);
    let counter = if self.config.timing.source == TimestampSource::SampleCount {
      index
    } else {
      self.clock.counter(at)
    };
    self.fifo.push((counter, kernel, sample));
    if self.fifo.len() > 128 {
      self.fifo.remove(0);
      self.overflow_samples += 1;
    }
    if self.fifo.len() < self.config.physical.fifo_watermark {
      return None;
    }
    self.watermarks += 1;
    if self
      .config
      .timing
      .missed_interrupt_every
      .is_some_and(|n| n > 0 && self.watermarks % n == 0)
    {
      return None;
    }
    if self.legacy {
      return Some((
        at + self.config.physical.delivery_delay_ns,
        self.fifo.drain(..).map(|(_, _, s)| s).collect(),
      ));
    }
    let wake = self.wakeup.sample()?;
    let available = kernel
      .max(at)
      .saturating_add(wake)
      .saturating_add(self.config.timing.bus_transaction_ns)
      .saturating_add(self.config.physical.delivery_delay_ns);
    let epoch = self.clock.epoch();
    let corrected_kernel = kernel.saturating_sub(self.config.timing.interrupt_delay_ns);
    if self.config.timing.source != TimestampSource::HostInterrupt
      && self
        .mapper
        .observe(
          counter,
          epoch,
          corrected_kernel,
          self.config.timing.timestamp_uncertainty_ns,
        )
        .is_err()
    {
      self.fifo.clear();
      return None;
    }
    let mut batch = Vec::new();
    for (native, kernel, mut sample) in self.fifo.drain(..) {
      let estimate = if self.config.timing.source == TimestampSource::HostInterrupt {
        super::clocks::ClockEstimate {
          common_time_ns: kernel.saturating_sub(self.config.timing.interrupt_delay_ns),
          uncertainty_ns: self.config.timing.timestamp_uncertainty_ns,
          epoch,
        }
      } else {
        self.mapper.estimate_earlier(native).ok()?
      };
      sample.data_ready_at = estimate.common_time_ns;
      sample.window_start_at = sample
        .data_ready_at
        .saturating_sub((1e9 / f64::from(self.config.physical.rate_hz)).round() as u64);
      sample.timing = Some(SampleTiming {
        source: self.config.timing.source,
        native_counter: (self.config.timing.source == TimestampSource::NativeCounter)
          .then_some(native),
        counter_epoch: epoch,
        kernel_capture_at: kernel,
        available_at: available,
        uncertainty_ns: estimate.uncertainty_ns,
      });
      batch.push(sample);
    }
    if fault.is_some_and(|(_, f)| f == SensorFaultKind::Reordered) {
      batch.reverse();
    }
    if fault.is_some_and(|(_, f)| f == SensorFaultKind::Duplicate)
      && let Some(sample) = batch.last().cloned()
    {
      batch.push(sample);
    }
    Some((available, batch))
  }
}
