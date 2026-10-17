//! Profile overlays are resolved before constructing a world. Arrays replace
//! rather than append, so device/slot ownership stays explicit and reviewable.
use super::{RealisticConfig, calibration::digest};
use serde::{Deserialize, Serialize};
use sha2::Digest;
use std::path::PathBuf;
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProfileLayer {
  pub path: PathBuf,
  pub content_sha256: String,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ResolvedProfile {
  pub config: RealisticConfig,
  pub layers: Vec<ProfileLayer>,
  pub sha256: String,
}
pub fn resolve(paths: &[PathBuf]) -> Result<ResolvedProfile, String> {
  if paths.is_empty() {
    return Err("provide at least one profile layer".into());
  }
  let mut value: toml::Value =
    toml::from_str(&RealisticConfig::default().to_toml()?).map_err(|e| e.to_string())?;
  let mut layers = Vec::new();
  for path in paths {
    let content = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let overlay: toml::Value =
      toml::from_str(&content).map_err(|e| format!("{}: {e}", path.display()))?;
    merge(&mut value, overlay);
    layers.push(ProfileLayer {
      path: path.clone(),
      content_sha256: format!("{:x}", sha2::Sha256::digest(content.as_bytes())),
    });
  }
  let mut config: RealisticConfig = value
    .try_into()
    .map_err(|e: toml::de::Error| e.to_string())?;
  config.profile_layers = layers.clone();
  config.validate()?;
  let sha256 = digest(&config)?;
  Ok(ResolvedProfile {
    config,
    layers,
    sha256,
  })
}
fn merge(base: &mut toml::Value, overlay: toml::Value) {
  if let (toml::Value::Table(a), toml::Value::Table(b)) = (&mut *base, &overlay) {
    for (key, value) in b {
      if let Some(old) = a.get_mut(key) {
        merge(old, value.clone());
      } else {
        a.insert(key.clone(), value.clone());
      }
    }
  } else {
    *base = overlay;
  }
}

/// Controller/estimator knowledge is separate from a physical robot override.
/// None means nominal team geometry/nominal motor calibration, never the
/// per-robot physical mismatch installed in RobotOverride.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct KnownCalibration {
  pub robot: Option<crate::RobotConfig>,
  pub motor: Option<super::config::MotorConfig>,
  pub x16_mounting_rpy_rad: [f64; 3],
}
