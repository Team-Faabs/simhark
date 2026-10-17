//! Optional process boundary for unchanged native firmware/controller hosts.
//! The process exchanges one JSON line per request/response. No wall-time work
//! is presented as deterministic simulated service time.
use super::{controller::RobotController, sensors::Observations, time::TimeNs};
use crate::RobotCommand;
use serde::{Deserialize, Serialize};
use std::{
  io::{BufRead, BufReader, Read, Write},
  process::{Child, Command, Stdio},
  sync::mpsc::{self, Receiver, SyncSender},
};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ControllerRequest {
  pub sequence: u64,
  pub at: TimeNs,
  pub command: Option<RobotCommand>,
  pub observations: Observations,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ControllerResponse {
  pub sequence: u64,
  pub observed_at: TimeNs,
  pub voltage_v: [Option<f64>; 4],
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BoundaryRecord {
  pub received_at: TimeNs,
  pub response: ControllerResponse,
}
pub struct ProcessController {
  child: Child,
  tx: SyncSender<ControllerRequest>,
  rx: Receiver<ControllerResponse>,
  pending: std::collections::BTreeMap<u64, TimeNs>,
  sequence: u64,
  last: Option<ControllerResponse>,
  ttl: TimeNs,
  pub boundary: std::sync::Arc<std::sync::Mutex<Vec<BoundaryRecord>>>,
}
impl ProcessController {
  pub fn spawn(command: &mut Command, output_ttl_ns: TimeNs) -> std::io::Result<Self> {
    let mut child = command
      .stdin(Stdio::piped())
      .stdout(Stdio::piped())
      .stderr(Stdio::inherit())
      .spawn()?;
    let mut stdin = child.stdin.take().unwrap();
    let stdout = child.stdout.take().unwrap();
    let (tx, requests) = mpsc::sync_channel::<ControllerRequest>(1);
    let (responses, rx) = mpsc::sync_channel(1);
    std::thread::spawn(move || {
      while let Ok(request) = requests.recv() {
        if serde_json::to_writer(&mut stdin, &request).is_err()
          || stdin.write_all(b"\n").and_then(|_| stdin.flush()).is_err()
        {
          break;
        }
      }
    });
    std::thread::spawn(move || {
      let mut reader = BufReader::new(stdout);
      loop {
        let mut line = String::new();
        // Bound framing before allocation, including malformed/no-newline peers.
        let read = std::io::Read::by_ref(&mut reader)
          .take(4 * 1024 * 1024)
          .read_line(&mut line);
        if !matches!(read,Ok(n) if n>0) || !line.ends_with('\n') {
          break;
        }
        if let Ok(response) = serde_json::from_str::<ControllerResponse>(&line) {
          if responses.send(response).is_err() {
            break;
          }
        } else {
          break;
        }
      }
    });
    Ok(Self {
      child,
      tx,
      rx,
      pending: Default::default(),
      sequence: 0,
      last: None,
      ttl: output_ttl_ns,
      boundary: Default::default(),
    })
  }
}
impl RobotController for ProcessController {
  fn tick(
    &mut self,
    now: TimeNs,
    command: Option<&RobotCommand>,
    observations: &Observations,
  ) -> [Option<f64>; 4] {
    while let Ok(response) = self.rx.try_recv() {
      if self.pending.remove(&response.sequence) == Some(response.observed_at)
        && response.observed_at <= now
        && self
          .last
          .as_ref()
          .is_none_or(|last| response.sequence > last.sequence)
        && response.voltage_v.iter().flatten().all(|v| v.is_finite())
      {
        self.boundary.lock().unwrap().push(BoundaryRecord {
          received_at: now,
          response: response.clone(),
        });
        self.last = Some(response);
      }
    }
    let request = ControllerRequest {
      sequence: self.sequence,
      at: now,
      command: command.cloned(),
      observations: observations.clone(),
    };
    if self.tx.try_send(request).is_ok() {
      self.pending.insert(self.sequence, now);
    }
    self.sequence = self.sequence.wrapping_add(1);
    self
      .pending
      .retain(|_, at| now.saturating_sub(*at) < self.ttl);
    while self.pending.len() > 64 {
      self.pending.pop_first();
    }
    self
      .last
      .as_ref()
      .filter(|r| now.saturating_sub(r.observed_at) < self.ttl)
      .map_or([None; 4], |r| r.voltage_v)
  }
}
impl Drop for ProcessController {
  fn drop(&mut self) {
    let _ = self.child.kill();
    let _ = self.child.wait();
  }
}

/// Pacing is explicitly separate from deterministic advance_to(). Live runs
/// should record boundary outputs before replaying under simulated scheduling.
pub struct LivePacer {
  started: std::time::Instant,
  origin: TimeNs,
}
impl LivePacer {
  pub fn new(origin: TimeNs) -> Self {
    Self {
      started: std::time::Instant::now(),
      origin,
    }
  }
  pub fn pace(&self, now: TimeNs) {
    if let Some(wait) = std::time::Duration::from_nanos(now.saturating_sub(self.origin))
      .checked_sub(self.started.elapsed())
    {
      std::thread::sleep(wait);
    }
  }
}
