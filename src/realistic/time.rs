//! Integer time with stable event ordering and drift-free rational rates.
use std::{cmp::Ordering, collections::BinaryHeap};

pub type TimeNs = u64;
pub const SECOND: TimeNs = 1_000_000_000;

struct Entry<E> {
  at: TimeNs,
  order: u64,
  event: E,
}
impl<E> PartialEq for Entry<E> {
  fn eq(&self, other: &Self) -> bool {
    (self.at, self.order) == (other.at, other.order)
  }
}
impl<E> Eq for Entry<E> {}
impl<E> PartialOrd for Entry<E> {
  fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
    Some(self.cmp(other))
  }
}
impl<E> Ord for Entry<E> {
  fn cmp(&self, other: &Self) -> Ordering {
    (other.at, other.order).cmp(&(self.at, self.order))
  }
}

pub struct Timeline<E> {
  now: TimeNs,
  order: u64,
  events: BinaryHeap<Entry<E>>,
}
impl<E> Default for Timeline<E> {
  fn default() -> Self {
    Self {
      now: 0,
      order: 0,
      events: BinaryHeap::new(),
    }
  }
}
impl<E> Timeline<E> {
  pub fn now(&self) -> TimeNs {
    self.now
  }
  pub fn len(&self) -> usize {
    self.events.len()
  }
  pub fn is_empty(&self) -> bool {
    self.events.is_empty()
  }
  pub fn schedule(&mut self, at: TimeNs, event: E) {
    assert!(at >= self.now, "cannot schedule an event in the past");
    self.events.push(Entry {
      at,
      order: self.order,
      event,
    });
    self.order = self.order.checked_add(1).expect("event sequence exhausted");
  }
  pub fn pop_until(&mut self, end: TimeNs) -> Option<(TimeNs, E)> {
    assert!(end >= self.now, "time cannot move backwards");
    if self.events.peek().is_some_and(|e| e.at <= end) {
      let entry = self.events.pop().unwrap();
      self.now = entry.at;
      Some((entry.at, entry.event))
    } else {
      self.now = end;
      None
    }
  }
}

/// Uses an absolute sample index, rather than repeatedly rounding a period.
#[derive(Debug, Clone)]
pub struct Periodic {
  phase: TimeNs,
  rate_hz: u32,
  index: u64,
}
impl Periodic {
  pub fn new(rate_hz: u32, phase: TimeNs) -> Self {
    assert!(rate_hz > 0 && rate_hz <= SECOND as u32);
    Self {
      phase,
      rate_hz,
      index: 0,
    }
  }
  pub fn next_time(&mut self) -> TimeNs {
    let offset = u128::from(self.index) * u128::from(SECOND) / u128::from(self.rate_hz);
    self.index = self.index.checked_add(1).expect("sample index exhausted");
    self
      .phase
      .checked_add(u64::try_from(offset).expect("sample time exhausted"))
      .expect("sample time exhausted")
  }
}
