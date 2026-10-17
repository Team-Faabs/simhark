//! Generates a separate SSL-Vision CaptureFromFile configuration. Calibration
//! values come from the configured camera estimate; the detector still has to
//! find blobs/patterns in real rendered pixels. The LUT is a synthetic lighting
//! fixture, not a measured venue color calibration.
use super::{RealisticConfig, vision::CameraConfig};
use std::path::Path;
fn var(name: &str, kind: &str, value: impl std::fmt::Display) -> String {
  format!("<Var name=\"{name}\" type=\"{kind}\">{value}</Var>")
}
fn list(name: &str, value: impl std::fmt::Display) -> String {
  var(name, "list", value)
}
fn xml(value: impl std::fmt::Display) -> String {
  format!("<?xml version=\"1.0\" encoding=\"utf-8\"?><VarXML>{value}</VarXML>")
}
fn escape(value: &str) -> String {
  value
    .replace('&', "&amp;")
    .replace('<', "&lt;")
    .replace('>', "&gt;")
    .replace('"', "&quot;")
}
pub fn write(
  source: &Path,
  frames: &Path,
  output: &Path,
  config: &RealisticConfig,
  camera_id: u32,
) -> Result<(), String> {
  config.validate()?;
  let c = config
    .cameras
    .iter()
    .find(|c| c.id == camera_id)
    .ok_or("camera ID absent")?;
  if !source.join("bin/vision").is_file() {
    return Err("build unchanged SSL-Vision first (bin/vision required)".into());
  }
  if !frames.is_dir() {
    return Err("rendered BMP directory missing".into());
  }
  let frames = frames.canonicalize().map_err(|e| e.to_string())?;
  std::fs::create_dir_all(output.join("bin")).map_err(|e| e.to_string())?;
  std::fs::copy(source.join("bin/vision"), output.join("bin/vision")).map_err(|e| e.to_string())?;
  std::fs::create_dir_all(output.join("patterns/teams")).map_err(|e| e.to_string())?;
  std::fs::copy(
    source.join("patterns/teams/standard2010_16.png"),
    output.join("patterns/teams/standard2010_16.png"),
  )
  .map_err(|e| e.to_string())?;
  std::fs::copy(
    source.join("robocup-ssl-teams.xml"),
    output.join("robocup-ssl-teams.xml"),
  )
  .map_err(|e| e.to_string())?;
  let write =
    |path: &str, value: String| std::fs::write(output.join(path), value).map_err(|e| e.to_string());
  write(
    "settings.xml",
    xml(list(
      "Vision System",
      "<Var name=\"RoboCup SSL Multi-Cam\" type=\"external\" filename=\"robocup-ssl.xml\"/>",
    )),
  )?;
  let capture = list(
    "Image Capture",
    format!(
      "{}{}",
      list(
        "Capture Control",
        var("Capture Module", "stringenum", "Read from files")
      ),
      list(
        "Read from files",
        format!(
          "{}{}",
          list(
            "Capture Settings",
            var("directory", "string", escape(&frames.display().to_string()))
          ),
          list(
            "Conversion Settings",
            var("convert to mode", "stringenum", "rgb")
          )
        )
      )
    ),
  );
  let global = list(
    "Global",
    list(
      "Network Output",
      format!(
        "{}{}",
        var("Multicast Address", "string", "224.5.23.2"),
        var("Multicast Port", "int", 10006)
      ),
    ),
  );
  let lut = "<Var name=\"LUT 3D\" type=\"external\" filename=\"robocup-ssl-cam-0-lut-yuv.xml\"/>";
  write(
    "robocup-ssl.xml",
    xml(format!(
      "{global}{}",
      list(
        "Thread 0",
        format!(
          "{}{capture}{}",
          list("Global", lut),
          list("Camera Calibrator", camera_parameters(c))
        )
      )
    )),
  )?;
  let mut table = vec![0_u8; 1 << 17];
  for y in 0..16 {
    for u in 0..64 {
      for v in 0..64 {
        let yy = y as f64 * 16.0 + 8.0;
        let uu = u as f64 * 4.0 + 2.0 - 128.0;
        let vv = v as f64 * 4.0 + 2.0 - 128.0;
        let r = (yy + 1.402 * vv).clamp(0.0, 255.0);
        let g = (yy - 0.344 * uu - 0.714 * vv).clamp(0.0, 255.0);
        let b = (yy + 1.772 * uu).clamp(0.0, 255.0);
        let class = if r > 150.0 && g < r * 0.72 && b < r * 0.4 && g > r * 0.15 {
          2
        } else if r > 100.0 && g > 100.0 && b < r.min(g) * 0.5 {
          3
        } else if b > 100.0 && b > r * 1.6 && b > g * 1.6 {
          4
        } else if r > 100.0 && b > 100.0 && g < r.min(b) * 0.55 {
          5
        } else if g > 130.0 && g > r * 1.5 && g > b * 1.5 {
          7
        } else if r > 160.0 && g > 160.0 && b > 160.0 {
          8
        } else if g > 50.0 && g > r * 1.4 && g > b * 1.3 {
          1
        } else {
          0
        };
        table[(y << 12) | (u << 6) | v] = class;
      }
    }
  }
  write(
    "robocup-ssl-cam-0-lut-yuv.xml",
    xml(var("LUT Data", "blob", base64(&table))),
  )?;
  write(
    "robocup-ssl-cam-0-mask.xml",
    xml(list("Convex Hull Points", "")),
  )?;
  Ok(())
}
fn camera_parameters(c: &CameraConfig) -> String {
  let r = c.camera_to_world_rpy_rad;
  let e = c.calibration_rpy_error_rad;
  let q =
    nalgebra::UnitQuaternion::from_euler_angles(r[0] + e[0], r[1] + e[1], r[2] + e[2]).inverse();
  let rv = q.scaled_axis();
  let p = nalgebra::Vector3::from(c.position_world_m)
    + nalgebra::Vector3::from(c.calibration_position_error_m);
  let t = -(q * p) * 1000.0;
  let mut intrinsic = String::new();
  for (name, value) in [
    (
      "focal_length_x",
      c.focal_px[0] * (1.0 + c.calibration_focal_fraction_error[0]),
    ),
    (
      "focal_length_y",
      c.focal_px[1] * (1.0 + c.calibration_focal_fraction_error[1]),
    ),
    ("principal_point_x", c.principal_px[0]),
    ("principal_point_y", c.principal_px[1]),
    ("dist_coeff_k1", c.distortion[0]),
    ("dist_coeff_k2", c.distortion[1]),
    ("dist_coeff_p1", c.distortion[2]),
    ("dist_coeff_p2", c.distortion[3]),
    ("dist_coeff_k3", c.distortion[4]),
  ] {
    intrinsic += &var(name, "double", value);
  }
  let mut extrinsic = String::new();
  for (name, value) in [
    ("rvec x", rv.x),
    ("rvec y", rv.y),
    ("rvec z", rv.z),
    ("tvec x", t.x),
    ("tvec y", t.y),
    ("tvec z", t.z),
  ] {
    extrinsic += &var(name, "double", value);
  }
  list(
    "Camera Parameters",
    format!(
      "{}{}{}",
      list("Intrinsic Parameters", intrinsic),
      list("Extrinsic Parameters", extrinsic),
      var("use openCV camera model", "bool", true)
    ),
  )
}
fn base64(bytes: &[u8]) -> String {
  const ALPHABET: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  let mut out = String::new();
  for chunk in bytes.chunks(3) {
    let n = (u32::from(chunk[0]) << 16)
      | (u32::from(*chunk.get(1).unwrap_or(&0)) << 8)
      | u32::from(*chunk.get(2).unwrap_or(&0));
    out.push(ALPHABET[((n >> 18) & 63) as usize] as char);
    out.push(ALPHABET[((n >> 12) & 63) as usize] as char);
    out.push(if chunk.len() > 1 {
      ALPHABET[((n >> 6) & 63) as usize] as char
    } else {
      '='
    });
    out.push(if chunk.len() > 2 {
      ALPHABET[(n & 63) as usize] as char
    } else {
      '='
    });
  }
  out
}
