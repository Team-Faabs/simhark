use simhark::realistic::calibration::{MeasurementTrial, analyze_log, ks_distance, validate_trial};
use std::{error::Error, path::PathBuf};
fn main() -> Result<(), Box<dyn Error>> {
  let mut args = std::env::args().skip(1);
  let mode=args.next().ok_or("use camera --train log.gz --held-out log.gz [--max-records N] [--output report.json], or trial file.json")?;
  if mode == "templates" {
    use simhark::realistic::calibration_fit::*;
    let directory = PathBuf::from(args.next().ok_or("templates OUTPUT_DIRECTORY")?);
    std::fs::create_dir_all(&directory)?;
    for id in ["train", "held-out"] {
      let metadata = Metadata {
        recording_id: format!("REPLACE-{id}-recording-id"),
        kind: DatasetKind::Hardware,
        hardware_id: "REPLACE-robot-motor-sensor-serials".into(),
        firmware_id: "REPLACE-versions-and-programmed-modes".into(),
        source: "REPLACE-recording-path-and-measurement-setup".into(),
        clock_boundary: "REPLACE-exact-measurement-boundaries".into(),
        common_clock_verified: false,
      };
      let write = |name: &str, value: &serde_json::Value| {
        std::fs::write(
          directory.join(format!("{name}-{id}.json")),
          serde_json::to_vec_pretty(value).unwrap(),
        )
      };
      write(
        "motor",
        &serde_json::to_value(MotorDataset {
          schema_version: 1,
          metadata: metadata.clone(),
          samples: Vec::new(),
        })?,
      )?;
      write(
        "timing",
        &serde_json::to_value(TimingDataset {
          schema_version: 1,
          metadata: metadata.clone(),
          samples: Vec::new(),
        })?,
      )?;
      write(
        "response",
        &serde_json::to_value(ResponseDataset {
          schema_version: 1,
          metadata,
          units: "REPLACE-SI-units".into(),
          samples: Vec::new(),
        })?,
      )?;
    }
    std::fs::write(
      directory.join("response-search.json"),
      serde_json::to_vec_pretty(&ResponseSearch {
        max_delay_ns: 5_000_000,
        max_time_constant_ns: 5_000_000,
        grid_step_ns: 100_000,
        held_out_rmse_tolerance: 0.01,
      })?,
    )?;
    std::fs::write(
      directory.join("trial.json"),
      serde_json::to_vec_pretty(&MeasurementTrial {
        hardware_id: "REPLACE-robot-and-profile-revision".into(),
        scenario: "REPLACE-braking-slip-trip-tilt-or-sensor-scenario".into(),
        build_id: "REPLACE-simulator-build".into(),
        measured: Vec::new(),
        simulated: Vec::new(),
        tolerance: 0.0,
        units: "REPLACE-SI-units".into(),
        boundary: "REPLACE-exact-measured-observable-and-alignment".into(),
        held_out: true,
        dataset_sha256: "REPLACE-canonical-recording-SHA256".into(),
      })?,
    )?;
    println!(
      "Wrote empty dataset templates. Fill actual samples and identities, verify clock alignment, and declare tolerances before calibration."
    );
    return Ok(());
  }
  if mode == "response" {
    use simhark::realistic::calibration_fit::*;
    let train: ResponseDataset = serde_json::from_slice(&std::fs::read(
      args
        .next()
        .ok_or("response TRAIN.json HELD_OUT.json SEARCH.json REPORT.json")?,
    )?)?;
    let held: ResponseDataset = serde_json::from_slice(&std::fs::read(
      args.next().ok_or("missing held-out response dataset")?,
    )?)?;
    let search: ResponseSearch = serde_json::from_slice(&std::fs::read(
      args
        .next()
        .ok_or("missing response search/tolerance specification")?,
    )?)?;
    let path = args.next().ok_or("missing response report path")?;
    let fit = fit_response(&train, &held, &search)?;
    std::fs::write(path, serde_json::to_vec_pretty(&fit)?)?;
    if !fit.passed {
      return Err("held-out sensor response tolerance exceeded".into());
    }
    return Ok(());
  }
  if mode == "motor" || mode == "timing" {
    use simhark::realistic::calibration_fit::*;
    let train = std::fs::read(
      args
        .next()
        .ok_or("motor|timing TRAIN.json HELD_OUT.json TOLERANCE REPORT.json PROFILE.toml")?,
    )?;
    let held = std::fs::read(args.next().ok_or("missing held-out dataset")?)?;
    let tolerance = args
      .next()
      .ok_or("missing declared validation tolerance")?
      .parse::<f64>()?;
    let report_path = args.next().ok_or("missing report output path")?;
    let profile_path = args.next().ok_or("missing profile overlay path")?;
    let (report, overlay, passed) = if mode == "motor" {
      let fit = fit_motor(
        &serde_json::from_slice(&train)?,
        &serde_json::from_slice(&held)?,
        tolerance,
      )?;
      (
        serde_json::to_value(&fit)?,
        fit.profile_overlay_toml,
        fit.passed,
      )
    } else {
      let fit = fit_timing(
        &serde_json::from_slice(&train)?,
        &serde_json::from_slice(&held)?,
        tolerance,
        tolerance,
      )?;
      (
        serde_json::to_value(&fit)?,
        fit.profile_overlay_toml,
        fit.passed,
      )
    };
    std::fs::write(report_path, serde_json::to_vec_pretty(&report)?)?;
    if !passed {
      return Err("held-out calibration failed; no profile overlay was written".into());
    }
    std::fs::write(profile_path, overlay)?;
    return Ok(());
  }
  if mode == "capture" {
    let address = args
      .next()
      .ok_or("capture IP:PORT SECONDS OUTPUT.json")?
      .parse()?;
    let seconds = args.next().ok_or("missing capture duration")?.parse()?;
    let path = args.next().ok_or("missing output path")?;
    let report = simhark::realistic::protocol::capture_report(address, seconds)?;
    std::fs::write(path, serde_json::to_vec_pretty(&report)?)?;
    if report["detection_frames"].as_u64().unwrap_or(0) == 0 {
      return Err("no actual SSL-Vision detections received".into());
    }
    return Ok(());
  }
  if mode == "trial" {
    let path = args.next().ok_or("missing trial JSON path")?;
    let trial: MeasurementTrial = serde_json::from_slice(&std::fs::read(path)?)?;
    let report = validate_trial(&trial)?;
    println!("{}", serde_json::to_string_pretty(&report)?);
    if !report.passed {
      return Err("held-out measurement agreement failed".into());
    }
    return Ok(());
  }
  if mode != "camera" {
    return Err("unknown calibration mode".into());
  }
  let (mut train, mut held, mut output, mut limit) = (
    Vec::<PathBuf>::new(),
    Vec::<PathBuf>::new(),
    None::<PathBuf>,
    None::<u64>,
  );
  let mut fitted_profile = None::<PathBuf>;
  while let Some(flag) = args.next() {
    match flag.as_str() {
      "--train" => train.push(args.next().ok_or("missing train path")?.into()),
      "--held-out" => held.push(args.next().ok_or("missing held-out path")?.into()),
      "--output" => output = Some(args.next().ok_or("missing output path")?.into()),
      "--fit-profile" => {
        fitted_profile = Some(args.next().ok_or("missing fitted profile path")?.into())
      }
      "--max-records" => limit = Some(args.next().ok_or("missing limit")?.parse()?),
      _ => return Err(format!("unknown argument {flag}").into()),
    }
  }
  if train.is_empty() || held.is_empty() {
    return Err("provide independent training and held-out logs".into());
  }
  let mut training = Vec::new();
  let mut validation = Vec::new();
  for path in train {
    training.push(analyze_log(&path, limit)?);
  }
  for path in held {
    validation.push(analyze_log(&path, limit)?);
  }
  for a in &training {
    for b in &validation {
      if a.canonical_record_sha256 == b.canonical_record_sha256 {
        return Err("training/validation data duplicate by content digest".into());
      }
    }
  }
  if let Some(path) = fitted_profile {
    use simhark::realistic::{
      RealisticConfig,
      calibration::CameraTimingPrior,
      config::{ParameterSource, Provenance},
      vision::CameraConfig,
    };
    if training.len() != 1 {
      return Err("fit one venue/recording at a time; camera IDs across recordings do not identify the same hardware".into());
    }
    let dataset = &training[0];
    let mut profile = RealisticConfig::full();
    if dataset.camera_timing.is_empty()
      || dataset
        .camera_timing
        .values()
        .any(|samples| !samples.iter().any(|s| s.interval_ns.is_some()))
    {
      return Err("camera timing fitting requires recorded native capture intervals; no nominal-rate substitution is allowed".into());
    }
    profile.cameras = dataset
      .camera_timing
      .keys()
      .map(|id| CameraConfig {
        id: *id,
        ..Default::default()
      })
      .collect();
    profile.provenance.source="Recorded venue timing prior; camera geometry, noise, acquisition delay, network and robot hardware remain provisional. Native camera-clock scale assumed nominal; processing trace spans Vision-PC image reception to send and already includes queueing.".into();
    profile.camera_timing_priors=dataset.camera_timing.iter().map(|(id,samples)|CameraTimingPrior{camera_id:*id,samples:samples.iter().take(2048).cloned().collect(),provenance:Provenance{classification:ParameterSource::Measured,source:format!("camera {}; {} SHA256 {}; recorded native intervals and image-reception-to-send durations only",id,dataset.path,dataset.canonical_record_sha256),owner:"venue timing dataset; not assembled-robot calibration".into()}}).collect();
    profile.validate()?;
    std::fs::write(path, profile.to_toml()?)?;
  }
  let mut comparisons = Vec::new();
  for a in &training {
    for b in &validation {
      for (camera, values) in &a.intervals {
        if let Some(other) = b.intervals.get(camera) {
          comparisons.push(serde_json::json!({
    "training":a.canonical_record_sha256,"validation":b.canonical_record_sha256,"camera_id":camera,"native_interval_ks":ks_distance(values,other),
    "same_hardware_or_venue_is_not_assumed":true}));
        }
      }
    }
  }
  let report = serde_json::json!({"schema_version":1,"training":training,"held_out":validation,"comparisons":comparisons,
    "physical_robot_calibration":false,"note":"Camera identities across matches may differ. Statistical comparisons report transfer discrepancy; no absolute accuracy or one-way network latency is inferred."});
  let bytes = serde_json::to_vec_pretty(&report)?;
  if let Some(path) = output {
    std::fs::write(path, bytes)?;
  } else {
    println!("{}", String::from_utf8(bytes)?);
  }
  Ok(())
}
