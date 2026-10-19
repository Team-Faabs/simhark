//! Offline parameter fitting with explicit measurement boundaries and independent
//! recordings. Synthetic fixtures can test fitting but never become "measured".
use super::{
  RealisticConfig,
  calibration::{Distribution, digest, ks_distance},
  config::{JointDelay, MotorConfig, ParameterSource, Provenance},
};
use serde::{Deserialize, Serialize};
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum DatasetKind {
  Hardware,
  Synthetic,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Metadata {
  pub recording_id: String,
  pub kind: DatasetKind,
  pub hardware_id: String,
  pub firmware_id: String,
  pub source: String,
  pub clock_boundary: String,
  pub common_clock_verified: bool,
}
impl Metadata {
  fn validate(&self) -> Result<(), String> {
    if [
      self.recording_id.as_str(),
      self.hardware_id.as_str(),
      self.firmware_id.as_str(),
      self.source.as_str(),
      self.clock_boundary.as_str(),
    ]
    .iter()
    .any(|s| s.trim().is_empty())
      || !self.common_clock_verified
    {
      return Err("recordings require identity, source, firmware, boundary, and verified common-clock alignment".into());
    }
    Ok(())
  }
  fn provenance(&self) -> Provenance {
    Provenance {
      classification: if self.kind == DatasetKind::Hardware {
        ParameterSource::Measured
      } else {
        ParameterSource::Provisional
      },
      source: format!(
        "{}; recording {}; {}",
        self.source, self.recording_id, self.clock_boundary
      ),
      owner: self.hardware_id.clone(),
    }
  }
}
fn independent(a: &Metadata, b: &Metadata) -> Result<(), String> {
  a.validate()?;
  b.validate()?;
  if a.recording_id == b.recording_id {
    return Err("training and held-out recording IDs must differ".into());
  }
  if a.hardware_id != b.hardware_id
    || a.kind != b.kind
    || a.firmware_id != b.firmware_id
    || a.clock_boundary != b.clock_boundary
  {
    return Err(
      "held-out recording must identify the same hardware/firmware and dataset kind".into(),
    );
  }
  Ok(())
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MotorSample {
  pub at_ns: u64,
  pub voltage_v: f64,
  pub phase_current_a: f64,
  pub wheel_speed_rad_s: f64,
  pub enabled: bool,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MotorDataset {
  pub schema_version: u32,
  pub metadata: Metadata,
  pub samples: Vec<MotorSample>,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotorFit {
  pub training_sha256: String,
  pub held_out_sha256: String,
  pub resistance_ohm: f64,
  pub inductance_h: f64,
  pub back_emf_v_per_rad_s: f64,
  pub condition_number: f64,
  pub held_out_voltage_residual_v: Distribution,
  pub tolerance_v: f64,
  pub passed: bool,
  pub hardware_calibration: bool,
  pub profile_overlay_toml: String,
}
fn rows(data: &MotorDataset) -> Result<Vec<([f64; 3], f64)>, String> {
  data.metadata.validate()?;
  if data.schema_version != 1 || data.samples.len() < 12 {
    return Err("motor dataset needs schema 1 and at least 12 time-aligned samples".into());
  }
  if data.samples.iter().any(|s| {
    [s.voltage_v, s.phase_current_a, s.wheel_speed_rad_s]
      .iter()
      .any(|v| !v.is_finite())
  }) {
    return Err("nonfinite motor samples".into());
  }
  let mut rows = Vec::new();
  for pair in data.samples.windows(2) {
    let (a, b) = (&pair[0], &pair[1]);
    if b.at_ns <= a.at_ns {
      return Err("motor sample times must increase".into());
    }
    if !a.enabled || !b.enabled {
      continue;
    }
    let dt = (b.at_ns - a.at_ns) as f64 * 1e-9;
    // Trapezoidal integral form V = R I + L dI/dt + Ke omega.
    rows.push((
      [
        (a.phase_current_a + b.phase_current_a) * 0.5,
        (b.phase_current_a - a.phase_current_a) / dt,
        (a.wheel_speed_rad_s + b.wheel_speed_rad_s) * 0.5,
      ],
      (a.voltage_v + b.voltage_v) * 0.5,
    ));
  }
  if rows.len() < 10 {
    return Err("insufficient powered intervals".into());
  }
  Ok(rows)
}
pub fn fit_motor(
  train: &MotorDataset,
  held: &MotorDataset,
  tolerance_v: f64,
) -> Result<MotorFit, String> {
  independent(&train.metadata, &held.metadata)?;
  if !tolerance_v.is_finite() || tolerance_v <= 0.0 {
    return Err("declare a positive held-out voltage tolerance".into());
  }
  let training = rows(train)?;
  let validation = rows(held)?;
  let scale = std::array::from_fn::<_, 3, _>(|c| {
    (training.iter().map(|(x, _)| x[c] * x[c]).sum::<f64>() / training.len() as f64).sqrt()
  });
  if scale.iter().any(|v| *v < 1e-9) {
    return Err(
      "motor data does not excite current, current dynamics, and speed independently".into(),
    );
  }
  let a = nalgebra::DMatrix::from_fn(training.len(), 3, |r, c| training[r].0[c] / scale[c]);
  let b = nalgebra::DVector::from_iterator(training.len(), training.iter().map(|(_, v)| *v));
  let svd = a.svd(true, true);
  let singular = &svd.singular_values;
  let condition = singular.max() / singular.min();
  if !condition.is_finite() || condition > 1e6 {
    return Err("motor parameters are not identifiable from this recording".into());
  }
  let solved = svd.solve(&b, 1e-10).map_err(|e| e.to_string())?;
  let parameters = std::array::from_fn::<_, 3, _>(|i| solved[i] / scale[i]);
  if parameters.iter().any(|v| !v.is_finite() || *v <= 0.0) {
    return Err(
      "motor fit is nonphysical; check units, time alignment, commutation, and excitation".into(),
    );
  }
  let residuals: Vec<_> = validation
    .iter()
    .map(|(x, v)| (v - (0..3).map(|c| parameters[c] * x[c]).sum::<f64>()).abs())
    .collect();
  let statistics = Distribution::from_values(&residuals);
  let passed = statistics.p95 <= tolerance_v;
  let mut config = RealisticConfig::default();
  config.motor = MotorConfig {
    resistance_ohm: parameters[0],
    inductance_h: parameters[1],
    back_emf_v_per_rad_s: parameters[2],
    ..Default::default()
  };
  let p = train.metadata.provenance();
  let mut provenance = std::collections::BTreeMap::new();
  for key in [
    "motor.resistance_ohm",
    "motor.inductance_h",
    "motor.back_emf_v_per_rad_s",
  ] {
    provenance.insert(key, p.clone());
  }
  let overlay=toml::to_string_pretty(&serde_json::json!({"motor":{"resistance_ohm":parameters[0],"inductance_h":parameters[1],"back_emf_v_per_rad_s":parameters[2]},"parameter_provenance":provenance})).map_err(|e|e.to_string())?;
  config.validate()?;
  Ok(MotorFit {
    training_sha256: digest(train)?,
    held_out_sha256: digest(held)?,
    resistance_ohm: parameters[0],
    inductance_h: parameters[1],
    back_emf_v_per_rad_s: parameters[2],
    condition_number: condition,
    held_out_voltage_residual_v: statistics,
    tolerance_v,
    passed,
    hardware_calibration: passed && train.metadata.kind == DatasetKind::Hardware,
    profile_overlay_toml: overlay,
  })
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TimingSample {
  pub source_ns: u64,
  pub common_ready_ns: Option<u64>,
  pub wifi_available_ns: Option<u64>,
  pub uwb_air_end_ns: Option<u64>,
  pub uwb_available_ns: Option<u64>,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TimingDataset {
  pub schema_version: u32,
  pub metadata: Metadata,
  pub samples: Vec<TimingSample>,
}
fn joint(data: &TimingDataset) -> Result<Vec<JointDelay>, String> {
  data.metadata.validate()?;
  if data.schema_version != 1 || data.samples.is_empty() {
    return Err("empty/unsupported timing dataset".into());
  }
  let delta = |end: Option<u64>, start: u64| -> Result<i64, String> {
    match end {
      None => Ok(-1),
      Some(end) => {
        let value = end
          .checked_sub(start)
          .ok_or("timing boundary is before its cause")?;
        if value > 10_000_000_000 {
          return Err("timing interval exceeds profile limit".into());
        }
        Ok(value as i64)
      }
    }
  };
  if data
    .samples
    .windows(2)
    .any(|w| w[0].source_ns >= w[1].source_ns)
  {
    return Err("timing samples must be in increasing publication order".into());
  }
  let mut out = Vec::new();
  for sample in &data.samples {
    if sample.uwb_available_ns.is_some() && sample.uwb_air_end_ns.is_none() {
      return Err("UWB post-airtime fitting requires the actual air-end timestamp; do not mix slot wait/airtime into host delay".into());
    }
    if sample.common_ready_ns.is_none()
      && (sample.wifi_available_ns.is_some() || sample.uwb_available_ns.is_some())
    {
      return Err("radio cannot deliver without the common stage completing".into());
    }
    let split = sample.common_ready_ns.unwrap_or(sample.source_ns);
    if sample.uwb_air_end_ns.is_some_and(|at| at < split) {
      return Err("airtime precedes radio split".into());
    }
    out.push(JointDelay {
      common_delay_ns: delta(sample.common_ready_ns, sample.source_ns)?,
      wifi_post_split_ns: delta(sample.wifi_available_ns, split)?,
      uwb_post_airtime_ns: delta(
        sample.uwb_available_ns,
        sample.uwb_air_end_ns.unwrap_or(split),
      )?,
    });
  }
  Ok(out)
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimingFit {
  pub training_sha256: String,
  pub held_out_sha256: String,
  pub joint_trace: Vec<JointDelay>,
  pub held_out_ks: [Option<f64>; 3],
  pub held_out_loss_error: [f64; 3],
  pub max_ks: f64,
  pub max_loss_error: f64,
  pub passed: bool,
  pub hardware_calibration: bool,
  pub profile_overlay_toml: String,
}
pub fn fit_timing(
  train: &TimingDataset,
  held: &TimingDataset,
  max_ks: f64,
  max_loss_error: f64,
) -> Result<TimingFit, String> {
  independent(&train.metadata, &held.metadata)?;
  if [max_ks, max_loss_error]
    .iter()
    .any(|v| !v.is_finite() || !(0.0..=1.0).contains(v))
  {
    return Err("timing tolerances must be in 0..=1".into());
  }
  let a = joint(train)?;
  let b = joint(held)?;
  let channel = |samples: &[JointDelay], i| {
    samples
      .iter()
      .map(|s| match i {
        0 => s.common_delay_ns,
        1 => s.wifi_post_split_ns,
        _ => s.uwb_post_airtime_ns,
      })
      .collect::<Vec<_>>()
  };
  let mut ks = [None; 3];
  let mut loss = [0.0; 3];
  for i in 0..3 {
    let x = channel(&a, i);
    let y = channel(&b, i);
    let losses = |v: &[i64]| v.iter().filter(|x| **x < 0).count() as f64 / v.len() as f64;
    loss[i] = (losses(&x) - losses(&y)).abs();
    ks[i] = ks_distance(
      &x.iter()
        .filter(|v| **v >= 0)
        .map(|v| *v as f64)
        .collect::<Vec<_>>(),
      &y.iter()
        .filter(|v| **v >= 0)
        .map(|v| *v as f64)
        .collect::<Vec<_>>(),
    );
  }
  let passed =
    ks.iter().all(|v| v.is_none_or(|v| v <= max_ks)) && loss.iter().all(|v| *v <= max_loss_error);
  let provenance = std::collections::BTreeMap::from([(
    "transport.joint_delay_trace",
    train.metadata.provenance(),
  )]);
  let overlay=toml::to_string_pretty(&serde_json::json!({"transport":{"joint_delay_trace":a},"network":{"empirical_commands":true},"parameter_provenance":provenance})).map_err(|e|e.to_string())?;
  Ok(TimingFit {
    training_sha256: digest(train)?,
    held_out_sha256: digest(held)?,
    joint_trace: a,
    held_out_ks: ks,
    held_out_loss_error: loss,
    max_ks,
    max_loss_error,
    passed,
    hardware_calibration: passed && train.metadata.kind == DatasetKind::Hardware,
    profile_overlay_toml: overlay,
  })
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ResponseSample {
  pub at_ns: u64,
  pub reference: f64,
  pub sensor: f64,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ResponseDataset {
  pub schema_version: u32,
  pub metadata: Metadata,
  pub units: String,
  pub samples: Vec<ResponseSample>,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ResponseSearch {
  pub max_delay_ns: u64,
  pub max_time_constant_ns: u64,
  pub grid_step_ns: u64,
  pub held_out_rmse_tolerance: f64,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResponseFit {
  pub training_sha256: String,
  pub held_out_sha256: String,
  pub units: String,
  pub delay_ns: u64,
  pub time_constant_s: f64,
  pub scale: f64,
  pub bias: f64,
  pub training_rmse: f64,
  pub held_out_rmse: f64,
  pub passed: bool,
  pub hardware_calibration: bool,
  pub note: String,
}
fn response_model(
  data: &ResponseDataset,
  delay: u64,
  tau_ns: u64,
) -> Result<Vec<(f64, f64)>, String> {
  data.metadata.validate()?;
  if data.schema_version != 1
    || data.samples.len() < 20
    || data.units.trim().is_empty()
    || data
      .samples
      .iter()
      .any(|s| !s.reference.is_finite() || !s.sensor.is_finite())
    || data.samples.windows(2).any(|w| w[1].at_ns <= w[0].at_ns)
  {
    return Err(
      "response dataset requires increasing times, units, and at least 20 finite samples".into(),
    );
  }
  let samples = &data.samples;
  let mut rows = Vec::new();
  let mut filtered = samples[0].reference;
  let mut last = samples[0].at_ns;
  let mut index = 0;
  for sample in samples {
    let Some(input_at) = sample.at_ns.checked_sub(delay) else {
      continue;
    };
    if input_at < samples[0].at_ns {
      continue;
    }
    while index + 1 < samples.len() && samples[index + 1].at_ns < input_at {
      index += 1;
    }
    let a = &samples[index];
    let b = &samples[(index + 1).min(samples.len() - 1)];
    let reference = if b.at_ns == a.at_ns {
      a.reference
    } else {
      a.reference
        + (b.reference - a.reference) * (input_at - a.at_ns) as f64 / (b.at_ns - a.at_ns) as f64
    };
    let dt = sample.at_ns - last;
    last = sample.at_ns;
    filtered = if tau_ns == 0 {
      reference
    } else {
      filtered + (reference - filtered) * (1.0 - (-(dt as f64) / tau_ns as f64).exp())
    };
    // Skip initial condition dependence when comparing measured response.
    if sample.at_ns.saturating_sub(samples[0].at_ns)
      > delay.saturating_add(tau_ns.saturating_mul(5))
    {
      rows.push((filtered, sample.sensor));
    }
  }
  if rows.len() < 10 {
    return Err("response record too short for search horizon".into());
  }
  Ok(rows)
}
fn scale_bias(rows: &[(f64, f64)]) -> Result<(f64, f64), String> {
  let n = rows.len() as f64;
  let x = rows.iter().map(|(x, _)| x).sum::<f64>() / n;
  let y = rows.iter().map(|(_, y)| y).sum::<f64>() / n;
  let variance = rows.iter().map(|(a, _)| (a - x).powi(2)).sum::<f64>();
  if variance < 1e-12 {
    return Err("reference does not excite sensor response".into());
  }
  let scale = rows.iter().map(|(a, b)| (a - x) * (b - y)).sum::<f64>() / variance;
  Ok((scale, y - scale * x))
}
fn response_rmse(rows: &[(f64, f64)], scale: f64, bias: f64) -> f64 {
  (rows
    .iter()
    .map(|(x, y)| (y - scale * x - bias).powi(2))
    .sum::<f64>()
    / rows.len() as f64)
    .sqrt()
}
pub fn fit_response(
  train: &ResponseDataset,
  held: &ResponseDataset,
  search: &ResponseSearch,
) -> Result<ResponseFit, String> {
  independent(&train.metadata, &held.metadata)?;
  if train.units != held.units
    || search.grid_step_ns == 0
    || !search.held_out_rmse_tolerance.is_finite()
    || search.held_out_rmse_tolerance <= 0.0
  {
    return Err("response units/tolerance/search grid invalid".into());
  }
  let nd = search.max_delay_ns / search.grid_step_ns;
  let nt = search.max_time_constant_ns / search.grid_step_ns;
  if (nd as u128 + 1) * (nt as u128 + 1) > 100_000 {
    return Err("response search exceeds 100000 candidates".into());
  }
  let mut best = None;
  for d in 0..=nd {
    for t in 0..=nt {
      let delay = d * search.grid_step_ns;
      let tau = t * search.grid_step_ns;
      let rows = response_model(train, delay, tau)?;
      let (scale, bias) = scale_bias(&rows)?;
      let error = response_rmse(&rows, scale, bias);
      if best.is_none_or(|(old, _, _, _, _)| error < old) {
        best = Some((error, delay, tau, scale, bias));
      }
    }
  }
  let (training_rmse, delay, tau, scale, bias) = best.ok_or("empty response fit")?;
  let held_out_rmse = response_rmse(&response_model(held, delay, tau)?, scale, bias);
  let passed = held_out_rmse <= search.held_out_rmse_tolerance;
  Ok(ResponseFit{training_sha256:digest(train)?,held_out_sha256:digest(held)?,units:train.units.clone(),delay_ns:delay,time_constant_s:tau as f64*1e-9,scale,bias,training_rmse,held_out_rmse,passed,hardware_calibration:passed&&train.metadata.kind==DatasetKind::Hardware,
    note:"Equivalent first-order response with pure delay, scale, and bias; grid resolution is explicit. Delay and bandwidth may be confounded; inspect excitation and fit residuals before selecting physical versus estimator compensation parameters. No automatic change to timestamp meaning or vIMU calibration.".into()})
}
