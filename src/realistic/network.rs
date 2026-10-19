//! Bounded shared resources, explicit queue policy, and empirical delay models.
use super::time::TimeNs;
use rand::RngExt;
use rand_chacha::ChaCha8Rng;
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum DelayDistribution {
  Constant {
    delay_ns: TimeNs,
  },
  Trace {
    values_ns: Vec<i64>,
  },
  Quantiles {
    probability: Vec<f64>,
    delay_ns: Vec<TimeNs>,
    loss_probability: f64,
  },
  Burst {
    good_ns: TimeNs,
    bad_ns: TimeNs,
    enter_probability: f64,
    leave_probability: f64,
    bad_loss_probability: f64,
  },
  TimeVarying {
    segments: Vec<DelaySegment>,
  },
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DelaySegment {
  pub start_ns: TimeNs,
  pub end_ns: TimeNs,
  pub distribution: Box<DelayDistribution>,
}
impl Default for DelayDistribution {
  fn default() -> Self {
    Self::Constant { delay_ns: 0 }
  }
}
pub struct DelaySampler {
  profile: DelayDistribution,
  index: usize,
  bad: bool,
  children: Vec<DelaySampler>,
}
impl DelaySampler {
  pub fn new(profile: DelayDistribution) -> Self {
    let children = if let DelayDistribution::TimeVarying { segments } = &profile {
      segments
        .iter()
        .map(|s| Self::new(*s.distribution.clone()))
        .collect()
    } else {
      Vec::new()
    };
    Self {
      profile,
      index: 0,
      bad: false,
      children,
    }
  }
  pub fn sample(&mut self, at: TimeNs, rng: &mut ChaCha8Rng) -> Option<TimeNs> {
    match &self.profile {
      DelayDistribution::Constant { delay_ns } => Some(*delay_ns),
      DelayDistribution::Trace { values_ns } => {
        let value = values_ns[self.index % values_ns.len()];
        self.index = self.index.wrapping_add(1);
        u64::try_from(value).ok()
      }
      DelayDistribution::Quantiles {
        probability,
        delay_ns,
        loss_probability,
      } => {
        if rng.random::<f64>() < *loss_probability {
          return None;
        }
        let p = rng.random::<f64>();
        let upper = probability
          .partition_point(|q| *q < p)
          .min(probability.len() - 1);
        if upper == 0 {
          return Some(delay_ns[0]);
        }
        let lower = upper - 1;
        let f = (p - probability[lower]) / (probability[upper] - probability[lower]);
        Some(
          (delay_ns[lower] as f64 + f * (delay_ns[upper] - delay_ns[lower]) as f64).round() as u64,
        )
      }
      DelayDistribution::Burst {
        good_ns,
        bad_ns,
        enter_probability,
        leave_probability,
        bad_loss_probability,
      } => {
        if rng.random::<f64>()
          < if self.bad {
            *leave_probability
          } else {
            *enter_probability
          }
        {
          self.bad = !self.bad;
        }
        if self.bad && rng.random::<f64>() < *bad_loss_probability {
          None
        } else {
          Some(if self.bad { *bad_ns } else { *good_ns })
        }
      }
      DelayDistribution::TimeVarying { segments } => {
        let i = segments
          .iter()
          .position(|s| at >= s.start_ns && at < s.end_ns)?;
        self.children[i].sample(at, rng)
      }
    }
  }
}
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum QueuePolicy {
  #[default]
  Latest,
  Fifo,
  DropOldest,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct LinkConfig {
  pub bits_per_second: u64,
  pub overhead_bytes: u32,
  pub capacity_bytes: usize,
  pub latency: DelayDistribution,
  pub retry_probability: f64,
  pub retry_limit: u32,
  pub turnaround_ns: TimeNs,
  pub power_save_interval_ns: TimeNs,
  pub policy: QueuePolicy,
  pub command_priority: bool,
}
impl Default for LinkConfig {
  fn default() -> Self {
    Self {
      bits_per_second: 54_000_000,
      overhead_bytes: 80,
      capacity_bytes: 1_000_000,
      latency: DelayDistribution::Constant { delay_ns: 100_000 },
      retry_probability: 0.0,
      retry_limit: 3,
      turnaround_ns: 20_000,
      power_save_interval_ns: 0,
      policy: QueuePolicy::Latest,
      command_priority: true,
    }
  }
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct BulkTraffic {
  pub start_ns: TimeNs,
  pub end_ns: TimeNs,
  pub rate_hz: u32,
  pub bytes_per_burst: usize,
  pub flow: u64,
}
impl Default for BulkTraffic {
  fn default() -> Self {
    Self {
      start_ns: 0,
      end_ns: u64::MAX,
      rate_hz: 30,
      bytes_per_burst: 50_000,
      flow: 100,
    }
  }
}
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct NetworkConfig {
  pub empirical_commands: bool,
  pub wifi: Option<LinkConfig>,
  pub lan: Option<LinkConfig>,
  pub bulk: Vec<BulkTraffic>,
  pub wifi_copy_every_publications: u64,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrafficClass {
  Command,
  Telemetry,
  Bulk,
}
pub struct QueuedPacket<T> {
  pub flow: u64,
  pub bytes: usize,
  pub class: TrafficClass,
  pub payload: T,
}
pub struct Transmission<T> {
  pub finishes_at: TimeNs,
  pub available_at: TimeNs,
  pub payload: T,
  pub delivered: bool,
  pub started_at: TimeNs,
}
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct LinkDiagnostics {
  pub enqueued: u64,
  pub dropped: u64,
  pub coalesced: u64,
  pub completed: u64,
  pub airtime_ns: u64,
  pub queued_bytes: usize,
  pub max_queued_bytes: usize,
  pub retries: u64,
}
pub struct SharedLink<T> {
  config: LinkConfig,
  queue: VecDeque<QueuedPacket<T>>,
  active: bool,
  delay: DelaySampler,
  pub diagnostics: LinkDiagnostics,
}
impl<T> SharedLink<T> {
  pub fn new(config: LinkConfig) -> Self {
    Self {
      delay: DelaySampler::new(config.latency.clone()),
      config,
      queue: VecDeque::new(),
      active: false,
      diagnostics: LinkDiagnostics::default(),
    }
  }
  pub fn enqueue(&mut self, packet: QueuedPacket<T>) -> bool {
    self.diagnostics.enqueued += 1;
    if self.config.policy == QueuePolicy::Latest && packet.class != TrafficClass::Bulk {
      if let Some(index) = self
        .queue
        .iter()
        .position(|p| p.flow == packet.flow && p.class == packet.class)
      {
        let old = self.queue.remove(index).unwrap();
        self.diagnostics.queued_bytes -= old.bytes;
        self.diagnostics.coalesced += 1;
      }
    }
    while self.diagnostics.queued_bytes.saturating_add(packet.bytes) > self.config.capacity_bytes {
      if self.config.policy == QueuePolicy::DropOldest
        && let Some(old) = self.queue.pop_front()
      {
        self.diagnostics.queued_bytes -= old.bytes;
        self.diagnostics.dropped += 1;
      } else {
        self.diagnostics.dropped += 1;
        return false;
      }
    }
    self.diagnostics.queued_bytes += packet.bytes;
    self.diagnostics.max_queued_bytes = self
      .diagnostics
      .max_queued_bytes
      .max(self.diagnostics.queued_bytes);
    self.queue.push_back(packet);
    true
  }
  pub fn start(&mut self, at: TimeNs, rng: &mut ChaCha8Rng) -> Option<Transmission<T>> {
    if self.active {
      return None;
    }
    let index = if self.config.command_priority {
      self
        .queue
        .iter()
        .position(|p| p.class == TrafficClass::Command)
        .unwrap_or(0)
    } else {
      0
    };
    let packet = self.queue.remove(index)?;
    self.diagnostics.queued_bytes -= packet.bytes;
    self.active = true;
    let mut retries = 0;
    while retries < self.config.retry_limit && rng.random::<f64>() < self.config.retry_probability {
      retries += 1;
    }
    let loss =
      retries == self.config.retry_limit && rng.random::<f64>() < self.config.retry_probability;
    self.diagnostics.retries += u64::from(retries);
    let bits = (packet.bytes as u128 + u128::from(self.config.overhead_bytes)) * 8;
    let one = (bits * 1_000_000_000).div_ceil(u128::from(self.config.bits_per_second)) as u64
      + self.config.turnaround_ns;
    let airtime = one * (u64::from(retries) + 1);
    self.diagnostics.airtime_ns += airtime;
    let service_start = if self.config.power_save_interval_ns > 0 {
      at.div_ceil(self.config.power_save_interval_ns) * self.config.power_save_interval_ns
    } else {
      at
    };
    let delay = self.delay.sample(at, rng);
    Some(Transmission {
      finishes_at: service_start + airtime,
      available_at: service_start + airtime + delay.unwrap_or(0),
      payload: packet.payload,
      delivered: !loss && delay.is_some(),
      started_at: service_start,
    })
  }
  pub fn complete(&mut self) {
    self.active = false;
    self.diagnostics.completed += 1;
  }
}

impl DelayDistribution {
  pub fn validate(&self) -> Result<(), String> {
    let probability = |v: f64| v.is_finite() && (0.0..=1.0).contains(&v);
    match self {
      Self::Constant { .. } => {}
      Self::Trace { values_ns } => {
        if values_ns.is_empty() || values_ns.iter().any(|v| *v < -1) {
          return Err("invalid delay trace".into());
        }
      }
      Self::Quantiles {
        probability: p,
        delay_ns,
        loss_probability,
      } => {
        if p.len() < 2
          || p.len() != delay_ns.len()
          || p[0] != 0.0
          || p[p.len() - 1] != 1.0
          || !probability(*loss_probability)
          || p.iter().any(|v| !probability(*v))
          || p.windows(2).any(|w| w[0] >= w[1])
          || delay_ns.windows(2).any(|w| w[0] > w[1])
        {
          return Err("invalid latency quantile curve".into());
        }
      }
      Self::Burst {
        enter_probability,
        leave_probability,
        bad_loss_probability,
        ..
      } => {
        if ![
          *enter_probability,
          *leave_probability,
          *bad_loss_probability,
        ]
        .into_iter()
        .all(probability)
        {
          return Err("invalid burst-state probability".into());
        }
      }
      Self::TimeVarying { segments } => {
        let mut end = 0;
        for s in segments {
          if s.start_ns < end || s.end_ns <= s.start_ns {
            return Err("overlapping/empty delay segments".into());
          }
          s.distribution.validate()?;
          end = s.end_ns;
        }
      }
    }
    Ok(())
  }
}
impl LinkConfig {
  pub fn validate(&self) -> Result<(), String> {
    if self.bits_per_second == 0
      || self.capacity_bytes == 0
      || !self.retry_probability.is_finite()
      || !(0.0..=1.0).contains(&self.retry_probability)
      || self.retry_limit > 100
    {
      return Err("invalid shared link parameters".into());
    }
    self.latency.validate()
  }
}
