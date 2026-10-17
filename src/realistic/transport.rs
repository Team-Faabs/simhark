//! Receiver contract shared by both radio paths. No physics or simulator truth access.
use super::{
  config::{SlotPurpose, TransportConfig},
  time::TimeNs,
};
use crate::{RobotCommand, WorldCommand};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Radio {
  Uwb,
  Wifi,
}
pub use robot_command_receiver::{ReceiverCheckpoint, Rejection, Safety, SenderClockEstimate};
pub type CommandPacket = robot_command_receiver::CommandPacket<RobotCommand>;
pub type Receiver = robot_command_receiver::Receiver<RobotCommand>;
impl robot_command_receiver::ActionCommand for RobotCommand {
  fn clear_action(&mut self) {
    self.kick_speed = 0.0;
  }
  fn has_action(&self) -> bool {
    self.kick_speed > 0.0
  }
}

/// Stateful trace consumption is separate from the simulator's random streams.
pub struct DelayTrace {
  values: Vec<i64>,
  index: usize,
}
impl DelayTrace {
  pub fn new(values: Vec<i64>) -> Self {
    assert!(!values.is_empty());
    Self { values, index: 0 }
  }
  pub fn sample(&mut self) -> Option<TimeNs> {
    let value = self.values[self.index];
    self.index = (self.index + 1) % self.values.len();
    u64::try_from(value).ok()
  }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AirtimeBudget {
  pub superframe_ns: u64,
  pub occupied_and_guard_ns: u64,
  pub utilization: f64,
  pub command_slots: usize,
  pub note: String,
}
impl TransportConfig {
  /// Configuration validation proves non-overlap as well as total utilization.
  pub fn airtime_budget(&self) -> AirtimeBudget {
    let occupied_and_guard_ns = self
      .slots
      .iter()
      .map(|s| (s.occupied_ns + self.guard_ns) as f64 / f64::from(s.every_superframes))
      .sum::<f64>()
      .ceil() as u64;
    AirtimeBudget { superframe_ns: self.superframe_ns, occupied_and_guard_ns,
      utilization: occupied_and_guard_ns as f64 / self.superframe_ns as f64,
      command_slots: self.slots.iter().filter(|s| matches!(s.purpose, SlotPurpose::BlueCommand | SlotPurpose::YellowCommand)).count(),
      note: "Average reserved occupancy including cadence and guards. Packet format and exchange capacity are validated separately; QM35825 firmware support and measured link margin remain hardware acceptance inputs.".into() }
  }
}

pub(crate) struct Publication {
  pub actions: Vec<super::runtime::ActionIdentity>,
  pub sequence: u64,
  pub decision_at: TimeNs,
  pub published_at: TimeNs,
  pub safety_epoch: u64,
  pub safety: Safety,
  pub commands: WorldCommand,
}
