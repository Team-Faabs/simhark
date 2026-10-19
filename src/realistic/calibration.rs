//! Offline calibration evidence. Camera disagreement is not absolute accuracy.
use prost::Message;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{collections::BTreeMap, path::Path};

pub fn digest<T: Serialize>(value: &T) -> Result<String, String> {
  Ok(format!(
    "{:x}",
    Sha256::digest(serde_json::to_vec(value).map_err(|e| e.to_string())?)
  ))
}
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Distribution {
  pub count: usize,
  pub p50: f64,
  pub p95: f64,
  pub p99: f64,
  pub min: f64,
  pub max: f64,
  pub mean: f64,
}
impl Distribution {
  pub fn from_values(values: &[f64]) -> Self {
    let mut values: Vec<_> = values.iter().copied().filter(|v| v.is_finite()).collect();
    values.sort_by(f64::total_cmp);
    if values.is_empty() {
      return Self::default();
    }
    let q = |p: f64| values[((values.len() - 1) as f64 * p).round() as usize];
    Self {
      count: values.len(),
      p50: q(0.5),
      p95: q(0.95),
      p99: q(0.99),
      min: values[0],
      max: values[values.len() - 1],
      mean: values.iter().sum::<f64>() / values.len() as f64,
    }
  }
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CameraStatistics {
  pub camera_id: u32,
  pub frames: u64,
  pub duplicate_frames: u64,
  pub frame_gaps: u64,
  pub clock_resets: u64,
  pub missing_native_timestamp: u64,
  pub native_interval_ms: Distribution,
  pub image_receive_interval_ms: Distribution,
  pub receive_interval_ms: Distribution,
  pub image_receive_to_send_ms: Distribution,
  pub apparent_stationary_residual_m: Distribution,
  pub heading_flip_candidates: u64,
  pub ball_misses: u64,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LogAnalysis {
  pub path: String,
  pub canonical_record_sha256: String,
  pub records: u64,
  pub bounded_prefix: bool,
  pub modern_vision: u64,
  pub legacy_vision_excluded: u64,
  pub tracked_messages: u64,
  pub malformed: u64,
  pub camera: Vec<CameraStatistics>,
  pub overlap_disagreement_m: Distribution,
  pub timestamp_note: String,
  #[serde(skip)]
  pub intervals: BTreeMap<u32, Vec<f64>>,
  #[serde(skip)]
  pub camera_timing: BTreeMap<u32, Vec<CameraTimingSample>>,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CameraTimingSample {
  pub interval_ns: Option<u64>,
  pub image_receive_to_send_ns: u64,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CameraTimingPrior {
  pub camera_id: u32,
  pub samples: Vec<CameraTimingSample>,
  pub provenance: super::config::Provenance,
}
#[derive(Default)]
struct CameraAccumulator {
  timing: Vec<CameraTimingSample>,
  frames: u64,
  duplicates: u64,
  gaps: u64,
  resets: u64,
  missing: u64,
  flips: u64,
  misses: u64,
  last: Option<(u32, f64, Option<f64>, i64)>,
  native: Vec<f64>,
  image: Vec<f64>,
  receive: Vec<f64>,
  processing: Vec<f64>,
  stationary: Vec<f64>,
}
struct DetectionHistory {
  at: f64,
  position: [f64; 2],
  velocity: [f64; 2],
  yaw: Option<f32>,
  window: std::collections::VecDeque<(f64, [f64; 2])>,
}

pub fn analyze_log(path: &Path, max_records: Option<u64>) -> Result<LogAnalysis, String> {
  let mut reader = loguna::LogReader::open(path).map_err(|e| e.to_string())?;
  let mut hash = Sha256::new();
  let mut cameras: BTreeMap<u32, CameraAccumulator> = BTreeMap::new();
  let mut tracks: BTreeMap<(u32, u8, u32), DetectionHistory> = BTreeMap::new();
  let mut overlap = Vec::new();
  let (mut records, mut modern, mut legacy, mut tracked, mut malformed) = (0, 0, 0, 0, 0);
  let mut bounded = false;
  while let Some(message) = reader.next_message().map_err(|e| e.to_string())? {
    if max_records.is_some_and(|max| records >= max) {
      bounded = true;
      break;
    }
    records += 1;
    hash.update(message.timestamp_ns.to_be_bytes());
    hash.update(message.message_id.as_i32().to_be_bytes());
    hash.update((message.payload.len() as u64).to_be_bytes());
    hash.update(&message.payload);
    match message.message_id {
      loguna::MessageId::Vision2010 => {
        legacy += 1;
        continue;
      }
      loguna::MessageId::VisionTracker2020 => {
        tracked += 1;
        continue;
      }
      loguna::MessageId::Vision2014 => {}
      _ => continue,
    }
    let Ok(wrapper) = loguna::proto::SslWrapperPacket::decode(message.payload.as_slice()) else {
      malformed += 1;
      continue;
    };
    let Some(frame) = wrapper.detection else {
      continue;
    };
    if !frame.t_capture.is_finite()
      || !frame.t_sent.is_finite()
      || frame.t_capture_camera.is_some_and(|t| !t.is_finite())
    {
      malformed += 1;
      continue;
    }
    modern += 1;
    let stats = cameras.entry(frame.camera_id).or_default();
    let native_interval = stats
      .last
      .and_then(|(_, _, last, _)| last.zip(frame.t_capture_camera))
      .map(|(last, now)| now - last);
    if let Some((previous, host, native, received)) = stats.last {
      let gap = frame.frame_number.wrapping_sub(previous);
      if gap == 0 {
        stats.duplicates += 1;
        continue;
      }
      if gap < 1 << 31 {
        stats.gaps += u64::from(gap - 1);
      } else {
        stats.resets += 1;
      }
      let interval = frame.t_capture - host;
      if interval > 0.0 {
        stats.image.push(interval * 1000.0);
      } else {
        stats.resets += 1;
      }
      if let (Some(now), Some(last)) = (frame.t_capture_camera, native) {
        let dt = now - last;
        if dt > 0.0 {
          stats.native.push(dt * 1000.0);
        } else {
          stats.resets += 1;
        }
      }
      if message.timestamp_ns > received {
        stats
          .receive
          .push((message.timestamp_ns - received) as f64 * 1e-6);
      }
    }
    stats.last = Some((
      frame.frame_number,
      frame.t_capture,
      frame.t_capture_camera,
      message.timestamp_ns,
    ));
    if frame.t_sent >= frame.t_capture && stats.timing.len() < 10000 {
      stats.timing.push(CameraTimingSample {
        interval_ns: native_interval
          .filter(|v| *v > 0.0 && *v < 10.0)
          .map(|v| (v * 1e9).round() as u64),
        image_receive_to_send_ns: ((frame.t_sent - frame.t_capture) * 1e9).round() as u64,
      });
    }
    stats.frames += 1;
    if frame.t_capture_camera.is_none() {
      stats.missing += 1;
    }
    if frame.t_sent >= frame.t_capture {
      stats
        .processing
        .push((frame.t_sent - frame.t_capture) * 1000.0);
    } else {
      stats.resets += 1;
    }
    if frame.balls.is_empty() {
      stats.misses += 1;
    }
    for (team, robots) in [(0, &frame.robots_blue), (1, &frame.robots_yellow)] {
      for robot in robots {
        if robot.confidence < 0.8 {
          continue;
        }
        let Some(id) = robot.robot_id else {
          continue;
        };
        let position = [f64::from(robot.x) * 0.001, f64::from(robot.y) * 0.001];
        if position.iter().any(|v| !v.is_finite()) {
          continue;
        }
        let key = (frame.camera_id, team, id);
        let mut velocity = [0.0; 2];
        if let Some(previous) = tracks.get(&key) {
          let dt = frame.t_capture - previous.at;
          if dt > 0.0 && dt < 0.1 {
            velocity = [
              (position[0] - previous.position[0]) / dt,
              (position[1] - previous.position[1]) / dt,
            ];
            if let (Some(now), Some(last)) = (robot.orientation, previous.yaw)
              && super::vision::wrap(f64::from(now - last)).abs() > 2.0
            {
              stats.flips += 1;
            }
          }
        }
        for (&(camera, other_team, other_id), other) in &tracks {
          if camera == frame.camera_id || other_team != team || other_id != id {
            continue;
          }
          let dt = frame.t_capture - other.at;
          if dt.abs() > 0.03 {
            continue;
          }
          let discrepancy = (position[0] - (other.position[0] + other.velocity[0] * dt))
            .hypot(position[1] - (other.position[1] + other.velocity[1] * dt));
          overlap.push(discrepancy);
        }
        let previous = tracks.entry(key).or_insert_with(|| DetectionHistory {
          at: frame.t_capture,
          position,
          velocity,
          yaw: robot.orientation,
          window: Default::default(),
        });
        previous.window.push_back((frame.t_capture, position));
        while previous
          .window
          .front()
          .is_some_and(|(t, _)| frame.t_capture - *t > 0.5)
        {
          previous.window.pop_front();
        }
        if previous
          .window
          .front()
          .is_some_and(|(t, _)| frame.t_capture - *t >= 0.3)
        {
          let n = previous.window.len() as f64;
          let mean = [
            previous.window.iter().map(|(_, p)| p[0]).sum::<f64>() / n,
            previous.window.iter().map(|(_, p)| p[1]).sum::<f64>() / n,
          ];
          if previous
            .window
            .iter()
            .all(|(_, p)| (p[0] - mean[0]).hypot(p[1] - mean[1]) < 0.01)
          {
            stats
              .stationary
              .push((position[0] - mean[0]).hypot(position[1] - mean[1]));
          }
        }
        previous.at = frame.t_capture;
        previous.position = position;
        previous.velocity = velocity;
        previous.yaw = robot.orientation;
      }
    }
  }
  let intervals = cameras
    .iter()
    .map(|(id, c)| (*id, c.native.clone()))
    .collect();
  let camera_timing = cameras
    .iter()
    .map(|(id, c)| (*id, c.timing.clone()))
    .collect();
  let camera = cameras
    .into_iter()
    .map(|(id, c)| CameraStatistics {
      camera_id: id,
      frames: c.frames,
      duplicate_frames: c.duplicates,
      frame_gaps: c.gaps,
      clock_resets: c.resets,
      missing_native_timestamp: c.missing,
      native_interval_ms: Distribution::from_values(&c.native),
      image_receive_interval_ms: Distribution::from_values(&c.image),
      receive_interval_ms: Distribution::from_values(&c.receive),
      image_receive_to_send_ms: Distribution::from_values(&c.processing),
      apparent_stationary_residual_m: Distribution::from_values(&c.stationary),
      heading_flip_candidates: c.flips,
      ball_misses: c.misses,
    })
    .collect();
  Ok(LogAnalysis {path:path.display().to_string(),canonical_record_sha256:format!("{:x}",hash.finalize()),records,bounded_prefix:bounded,modern_vision:modern,
    legacy_vision_excluded:legacy,tracked_messages:tracked,malformed,camera,overlap_disagreement_m:Distribution::from_values(&overlap),
    timestamp_note:"Intervals use native camera time where present. Overlap motion compensation uses common Vision-PC image reception times; exposure/acquisition skew remains confounded. Receive deltas are jitter, not absolute one-way delay. Stationary windows are selected from measured motion, not referee labels. Tournament profiles are venue priors, not robot hardware measurements.".into(),intervals,camera_timing})
}
pub fn ks_distance(a: &[f64], b: &[f64]) -> Option<f64> {
  if a.is_empty() || b.is_empty() {
    return None;
  }
  let (mut a, mut b) = (a.to_vec(), b.to_vec());
  a.sort_by(f64::total_cmp);
  b.sort_by(f64::total_cmp);
  let (mut i, mut j, mut maximum) = (0, 0, 0.0_f64);
  while i < a.len() || j < b.len() {
    let x = if j == b.len() || i < a.len() && a[i] <= b[j] {
      a[i]
    } else {
      b[j]
    };
    while i < a.len() && a[i] <= x {
      i += 1;
    }
    while j < b.len() && b[j] <= x {
      j += 1;
    }
    maximum = maximum.max((i as f64 / a.len() as f64 - j as f64 / b.len() as f64).abs());
  }
  Some(maximum)
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MeasurementTrial {
  pub hardware_id: String,
  pub scenario: String,
  pub build_id: String,
  pub measured: Vec<f64>,
  pub simulated: Vec<f64>,
  pub tolerance: f64,
  pub units: String,
  pub boundary: String,
  pub held_out: bool,
  pub dataset_sha256: String,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrialComparison {
  pub rmse: f64,
  pub max_error: f64,
  pub passed: bool,
}
pub fn validate_trial(trial: &MeasurementTrial) -> Result<TrialComparison, String> {
  if trial.measured.is_empty()
    || trial.measured.len() != trial.simulated.len()
    || trial.hardware_id.trim().is_empty()
    || trial.scenario.trim().is_empty()
    || trial.build_id.trim().is_empty()
    || trial.units.trim().is_empty()
    || trial.boundary.trim().is_empty()
    || trial.dataset_sha256.len() != 64
    || !trial.dataset_sha256.bytes().all(|b| b.is_ascii_hexdigit())
    || !trial.held_out
    || !trial.tolerance.is_finite()
    || trial.tolerance < 0.0
  {
    return Err("held-out trials require paired samples, hardware/dataset identity, units, boundary, and a declared tolerance".into());
  }
  let errors: Vec<_> = trial
    .measured
    .iter()
    .zip(&trial.simulated)
    .map(|(a, b)| (a - b).abs())
    .collect();
  if errors.iter().any(|e| !e.is_finite()) {
    return Err("nonfinite trial samples".into());
  }
  let max_error = errors.iter().copied().fold(0.0, f64::max);
  let rmse = (errors.iter().map(|e| e * e).sum::<f64>() / errors.len() as f64).sqrt();
  Ok(TrialComparison {
    rmse,
    max_error,
    passed: max_error <= trial.tolerance,
  })
}
