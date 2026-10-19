//! Nonpreemptive processing resources. A job owns its input snapshot while it
//! runs; queued work cannot change that input or publish before completion.
use super::{
  network::{DelayDistribution, DelaySampler},
  time::TimeNs,
};
use rand_chacha::ChaCha8Rng;
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Overload {
  #[default]
  Latest,
  Fifo,
  DropNewest,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct ProcessingConfig {
  pub queued_capacity: usize,
  pub overload: Overload,
  /// Additional service time, independent of the stage's base duration.
  pub extra_service: DelayDistribution,
}
impl Default for ProcessingConfig {
  fn default() -> Self {
    Self {
      queued_capacity: 1,
      overload: Overload::Latest,
      extra_service: DelayDistribution::Constant { delay_ns: 0 },
    }
  }
}
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ProcessingDiagnostics {
  pub started: u64,
  pub completed: u64,
  pub dropped: u64,
  pub coalesced: u64,
  pub max_wait_ns: TimeNs,
  pub max_depth: usize,
}
pub struct ProcessingResource<T> {
  config: ProcessingConfig,
  queue: VecDeque<(TimeNs, TimeNs, T)>,
  busy: bool,
  service: DelaySampler,
  pub diagnostics: ProcessingDiagnostics,
}
pub struct Started<T> {
  pub input: T,
  pub started_at: TimeNs,
  pub completes_at: TimeNs,
}
impl<T> ProcessingResource<T> {
  pub fn new(config: ProcessingConfig) -> Self {
    Self {
      service: DelaySampler::new(config.extra_service.clone()),
      config,
      queue: VecDeque::new(),
      busy: false,
      diagnostics: ProcessingDiagnostics::default(),
    }
  }
  pub fn submit(&mut self, at: TimeNs, base_duration: TimeNs, input: T) {
    // Permit one runnable job even when no waiting queue was configured.
    let capacity = self.config.queued_capacity + usize::from(!self.busy);
    if matches!(self.config.overload, Overload::Latest) && !self.queue.is_empty() {
      self.diagnostics.coalesced += self.queue.len() as u64;
      self.queue.clear();
    }
    if self.queue.len() >= capacity {
      self.diagnostics.dropped += 1;
      return;
    }
    self.queue.push_back((at, base_duration, input));
    self.diagnostics.max_depth = self.diagnostics.max_depth.max(self.queue.len());
  }
  pub fn start(&mut self, at: TimeNs, rng: &mut ChaCha8Rng) -> Option<Started<T>> {
    if self.busy {
      return None;
    }
    loop {
      let (submitted, duration, input) = self.queue.pop_front()?;
      let Some(extra) = self.service.sample(at, rng) else {
        self.diagnostics.dropped += 1;
        continue;
      };
      self.busy = true;
      self.diagnostics.started += 1;
      self.diagnostics.max_wait_ns = self
        .diagnostics
        .max_wait_ns
        .max(at.saturating_sub(submitted));
      return Some(Started {
        input,
        started_at: at,
        completes_at: at.saturating_add(duration).saturating_add(extra),
      });
    }
  }
  pub fn complete(&mut self) {
    assert!(self.busy);
    self.busy = false;
    self.diagnostics.completed += 1;
  }
}
