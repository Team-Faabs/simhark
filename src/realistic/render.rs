//! CPU image backend for unchanged SSL-Vision's CaptureFromFile (24-bit BMP).
//! Rendering uses physical surfaces, never synthetic detection coordinates.
use super::{scene::Scene, vision::CameraConfig};
use crate::{TeamColor, World};
use nalgebra::{UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};
use std::{
  io::{self, Write},
  path::Path,
};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RenderConfig {
  pub exposure_samples: u32,
  pub brightness: f64,
  pub vignette: f64,
  pub pixel_noise_std: f64,
}
impl Default for RenderConfig {
  fn default() -> Self {
    Self {
      exposure_samples: 1,
      brightness: 1.0,
      vignette: 0.15,
      pixel_noise_std: 0.0,
    }
  }
}
pub struct Image {
  pub width: u32,
  pub height: u32,
  pub rgb: Vec<u8>,
}
impl Image {
  pub fn write_bmp(&self, path: &Path) -> io::Result<()> {
    let stride = (self.width * 3 + 3) & !3;
    let size = stride
      .checked_mul(self.height)
      .ok_or_else(|| io::Error::other("image too large"))?;
    let mut out = std::io::BufWriter::new(std::fs::File::create(path)?);
    out.write_all(b"BM")?;
    out.write_all(&(54 + size).to_le_bytes())?;
    out.write_all(&[0; 4])?;
    out.write_all(&54_u32.to_le_bytes())?;
    out.write_all(&40_u32.to_le_bytes())?;
    out.write_all(&self.width.to_le_bytes())?;
    out.write_all(&self.height.to_le_bytes())?;
    out.write_all(&1_u16.to_le_bytes())?;
    out.write_all(&24_u16.to_le_bytes())?;
    out.write_all(&[0; 24])?;
    for row in (0..self.height as usize).rev() {
      for col in 0..self.width as usize {
        let i = (row * self.width as usize + col) * 3;
        out.write_all(&[self.rgb[i + 2], self.rgb[i + 1], self.rgb[i]])?;
      }
      out.write_all(&vec![0; (stride - self.width * 3) as usize])?;
    }
    out.flush()
  }
}
// Green bits in SVG ellipse order, standard2010_16 from RoboCup-SSL/ssl-vision
// (GPL-3.0); front left, rear left, rear right, front right. IDs are image order.
const PATTERNS: [u8; 16] = [2, 3, 11, 10, 4, 5, 13, 12, 15, 0, 6, 9, 7, 1, 14, 8];
struct RobotSurface {
  p: Vector3<f64>,
  q: UnitQuaternion<f64>,
  v: Vector3<f64>,
  omega: Vector3<f64>,
  radius: f64,
  half_height: f64,
  team: TeamColor,
  id: usize,
}

pub(crate) fn render(
  world: &World,
  camera: &CameraConfig,
  environment: &super::scene::Environment,
  config: &RenderConfig,
  seed: u64,
) -> Result<Image, String> {
  if camera.width == 0
    || camera.height == 0
    || u64::from(camera.width) * u64::from(camera.height) > 32_000_000
    || config.exposure_samples == 0
    || config.exposure_samples > 64
    || [config.brightness, config.vignette, config.pixel_noise_std]
      .iter()
      .any(|x| !x.is_finite() || *x < 0.0)
  {
    return Err("invalid rendering dimensions, exposure, or photometric parameters".into());
  }
  let mut robots = Vec::new();
  for (team, handles, sims, geometry) in [
    (
      TeamColor::Blue,
      &world.physics.blue_robots,
      &world.blue_sims,
      &world.config.blue_robots,
    ),
    (
      TeamColor::Yellow,
      &world.physics.yellow_robots,
      &world.yellow_sims,
      &world.config.yellow_robots,
    ),
  ] {
    for (id, (handle, sim)) in handles.iter().zip(sims).enumerate() {
      if !sim.is_on {
        continue;
      }
      let body = &world.physics.rigid_body_set[handle.chassis_body];
      let q = body.rotation();
      let p = body.translation();
      let v = body.linvel();
      let w = body.angvel();
      robots.push(RobotSurface {
        p: Vector3::new(p.x as f64, p.y as f64, p.z as f64),
        q: UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
          q.w as f64, q.x as f64, q.y as f64, q.z as f64,
        )),
        v: Vector3::new(v.x as f64, v.y as f64, v.z as f64),
        omega: Vector3::new(w.x as f64, w.y as f64, w.z as f64),
        radius: geometry.radius,
        half_height: geometry.height / 2.0,
        team,
        id,
      });
    }
  }
  let mut environment_only = environment.clone();
  // Static venue objects share the same physical geometry as other sensors.
  environment_only.known_map.clear();
  let mut scene = Scene::from_world(world, &environment_only, None);
  scene.bodies.drain(0..robots.len());
  let ball_v = world.physics.rigid_body_set[world.physics.ball_body].linvel();
  let ball_v = Vector3::new(ball_v.x as f64, ball_v.y as f64, ball_v.z as f64);
  let origin = Vector3::from(camera.position_world_m);
  let r = camera.camera_to_world_rpy_rad;
  let rotation = UnitQuaternion::from_euler_angles(r[0], r[1], r[2]);
  let mut image = Image {
    width: camera.width,
    height: camera.height,
    rgb: vec![0; camera.width as usize * camera.height as usize * 3],
  };
  let mut rng = super::sensors::rng(seed, world.id, camera.id as usize, 1300);
  for y in 0..camera.height {
    for x in 0..camera.width {
      let xd = (f64::from(x) + 0.5 - camera.principal_px[0]) / camera.focal_px[0];
      let yd = (f64::from(y) + 0.5 - camera.principal_px[1]) / camera.focal_px[1];
      let (mut u, mut v) = (xd, yd);
      let [k1, k2, p1, p2, k3] = camera.distortion;
      for _ in 0..8 {
        let r = u * u + v * v;
        let radial = 1.0 + k1 * r + k2 * r * r + k3 * r * r * r;
        u = (xd - 2.0 * p1 * u * v - p2 * (r + 2.0 * u * u)) / radial;
        v = (yd - p1 * (r + 2.0 * v * v) - 2.0 * p2 * u * v) / radial;
      }
      let direction = (rotation * Vector3::new(u, v, 1.0)).normalize();
      let mut color = [0.0; 3];
      for sample in 0..config.exposure_samples {
        let row_time = if camera.rolling_shutter {
          camera.readout_ns as f64 * f64::from(y) / f64::from(camera.height)
        } else {
          0.0
        };
        // Captured state is at exposure start; this is a constant-twist shutter
        // approximation. It does not advance the physics world during rendering.
        let dt = (row_time
          + camera.exposure_ns as f64 * (f64::from(sample) + 0.5)
            / f64::from(config.exposure_samples))
          * 1e-9;
        let mut distance = if direction.z < 0.0 {
          -origin.z / direction.z
        } else {
          f64::INFINITY
        };
        let point = origin + direction * distance;
        let mut c = field_color(point, &world.config.field);
        for robot in &robots {
          let q = UnitQuaternion::from_scaled_axis(robot.omega * dt) * robot.q;
          let inv = q.inverse();
          let o = inv * (origin - robot.p - robot.v * dt);
          let d = inv * direction;
          if let Some((t, local, top)) = cylinder(o, d, robot.radius, robot.half_height) {
            if t < distance {
              distance = t;
              c = if top {
                marker(local, robot.team, robot.id, robot.radius)
              } else {
                [24.0, 24.0, 28.0]
              };
            }
          }
        }
        scene.ball_position += ball_v * dt;
        if let Some((t, _)) = scene.cast(origin, direction, distance) {
          let p = origin + direction * t;
          c = if (p - scene.ball_position).norm() < scene.ball_radius * 1.01 {
            [255.0, 105.0, 0.0]
          } else {
            [90.0, 90.0, 90.0]
          };
        }
        scene.ball_position -= ball_v * dt;
        for i in 0..3 {
          color[i] += c[i] / f64::from(config.exposure_samples);
        }
      }
      let light = config.brightness * (1.0 - config.vignette * (u * u + v * v)).max(0.0);
      let at = (y as usize * camera.width as usize + x as usize) * 3;
      for i in 0..3 {
        image.rgb[at + i] = (color[i] * light
          + super::sensors::normal(&mut rng) * config.pixel_noise_std)
          .round()
          .clamp(0.0, 255.0) as u8;
      }
    }
  }
  Ok(image)
}
fn cylinder(o: Vector3<f64>, d: Vector3<f64>, r: f64, h: f64) -> Option<(f64, Vector3<f64>, bool)> {
  let mut best = None;
  if d.z.abs() > 1e-12 {
    for z in [-h, h] {
      let t = (z - o.z) / d.z;
      let p = o + d * t;
      if t > 0.0 && p.x * p.x + p.y * p.y <= r * r && best.is_none_or(|(old, _, _)| t < old) {
        best = Some((t, p, z > 0.0));
      }
    }
  }
  let a = d.x * d.x + d.y * d.y;
  let b = o.x * d.x + o.y * d.y;
  let c = o.x * o.x + o.y * o.y - r * r;
  let disc = b * b - a * c;
  if a > 1e-12 && disc >= 0.0 {
    for t in [(-b - disc.sqrt()) / a, (-b + disc.sqrt()) / a] {
      let p = o + d * t;
      if t > 0.0 && p.z.abs() <= h && best.is_none_or(|(old, _, _)| t < old) {
        best = Some((t, p, false));
      }
    }
  }
  best
}
fn marker(p: Vector3<f64>, team: TeamColor, id: usize, radius: f64) -> [f64; 3] {
  if p.x.hypot(p.y) < radius * 0.292 {
    return if team == TeamColor::Blue {
      [0.0, 30.0, 255.0]
    } else {
      [255.0, 240.0, 0.0]
    };
  }
  for (i, (x, y)) in [
    (0.416, 0.646),
    (-0.646, 0.416),
    (-0.646, -0.416),
    (0.416, -0.646),
  ]
  .into_iter()
  .enumerate()
  {
    if (p.x - x * radius).hypot(p.y - y * radius) < radius * 0.233 {
      return if PATTERNS[id % 16] & (1 << i) != 0 {
        [0.0, 255.0, 0.0]
      } else {
        [255.0, 0.0, 255.0]
      };
    }
  }
  [12.0, 12.0, 14.0]
}
fn field_color(p: Vector3<f64>, f: &crate::config::FieldConfig) -> [f64; 3] {
  if !p.x.is_finite() || !p.y.is_finite() {
    return [30.0; 3];
  }
  let (x, y) = (p.x.abs(), p.y.abs());
  let (hx, hy) = (f.field_length / 2.0, f.field_width / 2.0);
  let w = f.field_line_width / 2.0;
  let white = ((x - hx).abs() < w && y < hy + w)
    || ((y - hy).abs() < w && x < hx + w)
    || (x < w && y < hy)
    || ((p.x.hypot(p.y) - f.field_center_radius).abs() < w)
    || ((x - (hx - f.penalty_depth)).abs() < w && y < f.penalty_width / 2.0)
    || ((y - f.penalty_width / 2.0).abs() < w && x > hx - f.penalty_depth && x < hx);
  if white {
    [240.0; 3]
  } else if x < hx + f.margin_goal_line && y < hy + f.margin_touch_line {
    [10.0, 110.0, 38.0]
  } else {
    [60.0; 3]
  }
}
