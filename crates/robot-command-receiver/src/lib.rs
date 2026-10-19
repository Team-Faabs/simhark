//! Simulator-independent receiver. Payloads and local clocks are supplied by the
//! production host/firmware adapter; this crate has no simulator dependency.
use serde::{Deserialize, Serialize};
pub trait ActionCommand: Clone {
  fn clear_action(&mut self);
  fn has_action(&self) -> bool;
}
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Safety {
  Run,
  Stop,
  Halt,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommandPacket<C> {
  pub session: u64,
  pub sequence: u64,
  pub decision_at: u64,
  pub published_at: u64,
  pub expires_at: u64,
  pub safety_epoch: u64,
  pub safety: Safety,
  pub action_id: Option<u64>,
  pub command: C,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Rejection {
  WrongSession,
  Expired,
  FutureTimestamp,
  DuplicateOrOlder,
  SafetyRegression,
  ClockUnsynchronized,
}
#[derive(Debug, Clone, Copy)]
pub struct SenderClockEstimate {
  pub now_ns: u64,
  pub uncertainty_ns: u64,
}
pub struct Receiver<C> {
  session: u64,
  sequence: Option<u64>,
  safety_epoch: u64,
  safety: Safety,
  last_action: Option<u64>,
  accepted_at: Option<u64>,
  packet: Option<CommandPacket<C>>,
  watchdog_ns: u64,
  remaining_at_accept_ns: u64,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReceiverCheckpoint {
  pub session: u64,
  pub sequence: Option<u64>,
  pub safety_epoch: u64,
  pub safety: Safety,
  pub last_action: Option<u64>,
}
fn newer(value: u64, previous: u64) -> bool {
  let distance = value.wrapping_sub(previous);
  distance != 0 && distance < (1_u64 << 63)
}
impl<C: ActionCommand> Receiver<C> {
  pub fn new(session: u64, watchdog_ns: u64) -> Self {
    Self {
      session,
      sequence: None,
      safety_epoch: 0,
      safety: Safety::Halt,
      last_action: None,
      accepted_at: None,
      packet: None,
      watchdog_ns,
      remaining_at_accept_ns: 0,
    }
  }
  pub fn begin_session(&mut self, session: u64) {
    *self = Self::new(session, self.watchdog_ns);
  }
  pub fn restart(&mut self) {
    self.packet = None;
    self.accepted_at = None;
    self.remaining_at_accept_ns = 0;
  }
  pub fn checkpoint(&self) -> ReceiverCheckpoint {
    ReceiverCheckpoint {
      session: self.session,
      sequence: self.sequence,
      safety_epoch: self.safety_epoch,
      safety: self.safety,
      last_action: self.last_action,
    }
  }
  pub fn restore(checkpoint: ReceiverCheckpoint, watchdog_ns: u64) -> Self {
    Self {
      session: checkpoint.session,
      sequence: checkpoint.sequence,
      safety_epoch: checkpoint.safety_epoch,
      safety: checkpoint.safety,
      last_action: checkpoint.last_action,
      accepted_at: None,
      packet: None,
      watchdog_ns,
      remaining_at_accept_ns: 0,
    }
  }
  pub fn accept(&mut self, packet: CommandPacket<C>, now: u64) -> Result<(), Rejection> {
    self.accept_with_clock(
      packet,
      now,
      Some(SenderClockEstimate {
        now_ns: now,
        uncertainty_ns: 0,
      }),
    )
  }
  /// Uncertainty bounds sender time; local acceptance time separately drives the
  /// watchdog. Unknown clock mappings fail closed instead of asserting that a
  /// previously unseen delayed sequence is fresh.
  pub fn accept_with_clock(
    &mut self,
    mut packet: CommandPacket<C>,
    local_now: u64,
    clock: Option<SenderClockEstimate>,
  ) -> Result<(), Rejection> {
    if packet.session != self.session {
      return Err(Rejection::WrongSession);
    }
    let clock = clock.ok_or(Rejection::ClockUnsynchronized)?;
    if packet.decision_at > packet.published_at
      || packet.published_at > clock.now_ns.saturating_add(clock.uncertainty_ns)
    {
      return Err(Rejection::FutureTimestamp);
    }
    if clock.now_ns.saturating_add(clock.uncertainty_ns) >= packet.expires_at
      || packet.expires_at <= packet.published_at
    {
      return Err(Rejection::Expired);
    }
    if packet.safety_epoch < self.safety_epoch
      || (packet.safety_epoch == self.safety_epoch && packet.safety != self.safety)
    {
      return Err(Rejection::SafetyRegression);
    }
    if packet.safety_epoch > self.safety_epoch {
      self.safety_epoch = packet.safety_epoch;
      self.safety = packet.safety;
    }
    if self
      .sequence
      .is_some_and(|last| !newer(packet.sequence, last))
    {
      return Err(Rejection::DuplicateOrOlder);
    }
    self.sequence = Some(packet.sequence);
    if let Some(id) = packet.action_id {
      if self.last_action.is_some_and(|last| !newer(id, last)) {
        packet.command.clear_action();
      }
    } else {
      packet.command.clear_action();
    }
    self.remaining_at_accept_ns =
      packet.expires_at - clock.now_ns.saturating_add(clock.uncertainty_ns);
    self.accepted_at = Some(local_now);
    self.packet = Some(packet);
    Ok(())
  }
  pub fn active(&self, local_now: u64) -> Option<&CommandPacket<C>> {
    // The expiry deadline is translated at acceptance. It cannot be refreshed
    // by duplicate/reordered copies, even if both paths keep retrying.
    let packet = self.packet.as_ref()?;
    let accepted = self.accepted_at?;
    (self.safety == Safety::Run
      && local_now.saturating_sub(accepted) < self.watchdog_ns
      && local_now.saturating_sub(accepted) < self.remaining_at_accept_ns)
      .then_some(packet)
  }
  pub fn take_action(&mut self, now: u64) -> Option<C> {
    self.active(now)?;
    let p = self.packet.as_mut()?;
    if !p.command.has_action() {
      return None;
    }
    let command = p.command.clone();
    self.last_action = p.action_id;
    p.command.clear_action();
    Some(command)
  }
  pub fn take_kick(&mut self, now: u64) -> Option<C> {
    self.take_action(now)
  }
  pub fn last_action_id(&self) -> Option<u64> {
    self.last_action
  }
  pub fn safety(&self) -> Safety {
    self.safety
  }
}
