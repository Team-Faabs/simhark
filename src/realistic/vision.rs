//! Camera geometry and a tracker that consumes delivered detections only.
use super::{
  profile::ClockConfig,
  sensors::{self, RobotDetection, VisionFrame},
  time::TimeNs,
};
use crate::{BallState, RobotState, TeamColor, WorldConfig, WorldState};
use nalgebra::{UnitQuaternion, Vector3};
use rand::RngExt;
use rand_chacha::ChaCha8Rng;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct CameraConfig {
  pub id: u32,
  pub rate_hz: u32,
  pub phase_ns: TimeNs,
  pub position_world_m: [f64; 3],
  pub camera_to_world_rpy_rad: [f64; 3],
  pub width: u32,
  pub height: u32,
  pub focal_px: [f64; 2],
  pub principal_px: [f64; 2],
  pub distortion: [f64; 5],
  pub calibration_position_error_m: [f64; 3],
  pub calibration_rpy_error_rad: [f64; 3],
  pub calibration_focal_fraction_error: [f64; 2],
  pub pixel_noise_std: f64,
  pub persistent_position_bias_m: [f64; 2],
  pub spatial_error_m_per_m: [f64; 2],
  pub drift_m_per_s: [f64; 2],
  pub correlated_noise_tau_s: f64,
  pub dropout_probability: f64,
  pub id_swap_probability: f64,
  pub heading_flip_probability: f64,
  pub false_detection_probability: f64,
  pub exposure_ns: TimeNs,
  pub readout_ns: TimeNs,
  pub rolling_shutter: bool,
  pub acquisition_delay_ns: TimeNs,
  pub processing_ns: TimeNs,
  pub network_delay_ns: TimeNs,
  pub clock: ClockConfig,
  pub blue_enabled: bool,
  pub yellow_enabled: bool,
  pub officials_enabled: bool,
}
impl Default for CameraConfig {
  fn default() -> Self {
    Self {
      id: 0,
      rate_hz: 60,
      phase_ns: 0,
      position_world_m: [0.0, 0.0, 5.0],
      camera_to_world_rpy_rad: [std::f64::consts::PI, 0.0, 0.0],
      width: 1920,
      height: 1200,
      focal_px: [900.0, 900.0],
      principal_px: [960.0, 600.0],
      distortion: [0.0; 5],
      calibration_position_error_m: [0.0; 3],
      calibration_rpy_error_rad: [0.0; 3],
      calibration_focal_fraction_error: [0.0; 2],
      pixel_noise_std: 0.3,
      persistent_position_bias_m: [0.0; 2],
      spatial_error_m_per_m: [0.0; 2],
      drift_m_per_s: [0.0; 2],
      correlated_noise_tau_s: 0.02,
      dropout_probability: 0.0,
      id_swap_probability: 0.0,
      heading_flip_probability: 0.0,
      false_detection_probability: 0.0,
      exposure_ns: 1_000_000,
      readout_ns: 1_000_000,
      rolling_shutter: false,
      acquisition_delay_ns: 1_000_000,
      processing_ns: 8_000_000,
      network_delay_ns: 1_000_000,
      clock: ClockConfig::default(),
      blue_enabled: true,
      yellow_enabled: true,
      officials_enabled: true,
    }
  }
}
impl CameraConfig {
  pub fn validate(&self) -> Result<(), String> {
    if u64::from(self.width) * u64::from(self.height) > 32_000_000
      || self.width == 0
      || self.height == 0
      || self.focal_px.iter().any(|v| !v.is_finite() || *v <= 0.0)
      || self
        .calibration_focal_fraction_error
        .iter()
        .any(|v| !v.is_finite() || *v <= -1.0)
      || self
        .position_world_m
        .iter()
        .chain(&self.camera_to_world_rpy_rad)
        .chain(&self.principal_px)
        .chain(&self.distortion)
        .chain(&self.calibration_position_error_m)
        .chain(&self.calibration_rpy_error_rad)
        .chain(&self.persistent_position_bias_m)
        .chain(&self.spatial_error_m_per_m)
        .chain(&self.drift_m_per_s)
        .any(|v| !v.is_finite())
      || [self.pixel_noise_std, self.correlated_noise_tau_s]
        .iter()
        .any(|v| !v.is_finite() || *v < 0.0)
      || self.clock.tick_hz == 0
      || self.clock.counter_bits == 0
      || self.clock.counter_bits > 64
      || !self.clock.skew_ppm.is_finite()
      || !self.clock.drift_ppm_per_s.is_finite()
    {
      return Err("invalid camera geometry/response/clock".into());
    }
    Ok(())
  }
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct TrackerConfig {
  pub processing_ns: TimeNs,
  pub measurement_variance_m2: f64,
  pub acceleration_variance: f64,
  pub max_age_ns: TimeNs,
  pub gate_sigma: f64,
  pub initial_velocity_variance: f64,
}
impl Default for TrackerConfig {
  fn default() -> Self {
    Self {
      processing_ns: 1_000_000,
      measurement_variance_m2: 0.0001,
      acceleration_variance: 4.0,
      max_age_ns: 200_000_000,
      gate_sigma: 8.0,
      initial_velocity_variance: 100.0,
    }
  }
}

struct CaptureGeometry {
  scene: super::scene::Scene,
  markers: BTreeMap<(u8, usize), ([f64; 3], [f64; 3])>,
}
pub struct CameraModel {
  pub config: CameraConfig,
  rng: ChaCha8Rng,
  correlated: [f64; 2],
  last_at: TimeNs,
  pub sequence: u64,
}
impl CameraModel {
  pub fn new(config: CameraConfig, seed: u64, world: usize) -> Self {
    let id = config.id;
    Self {
      config,
      rng: sensors::rng(seed, world, id as usize, 500),
      correlated: [0.0; 2],
      last_at: 0,
      sequence: 0,
    }
  }
  fn orientation(&self, calibrated: bool) -> UnitQuaternion<f64> {
    let r = self.config.camera_to_world_rpy_rad;
    let e = if calibrated {
      self.config.calibration_rpy_error_rad
    } else {
      [0.0; 3]
    };
    UnitQuaternion::from_euler_angles(r[0] + e[0], r[1] + e[1], r[2] + e[2])
  }
  pub fn project(&self, point: [f64; 3]) -> Option<[f64; 2]> {
    let p = self.orientation(false).inverse()
      * (Vector3::from(point) - Vector3::from(self.config.position_world_m));
    if p.z <= 0.0 {
      return None;
    }
    let x = p.x / p.z;
    let y = p.y / p.z;
    let [k1, k2, p1, p2, k3] = self.config.distortion;
    let r2 = x * x + y * y;
    let radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
    let pixel = [
      self.config.focal_px[0] * (x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x))
        + self.config.principal_px[0],
      self.config.focal_px[1] * (y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y)
        + self.config.principal_px[1],
    ];
    (pixel[0] >= 0.0
      && pixel[0] < f64::from(self.config.width)
      && pixel[1] >= 0.0
      && pixel[1] < f64::from(self.config.height))
    .then_some(pixel)
  }
  pub fn unproject(&self, pixel: [f64; 2], height: f64) -> Option<[f64; 2]> {
    let xd = (pixel[0] - self.config.principal_px[0])
      / (self.config.focal_px[0] * (1.0 + self.config.calibration_focal_fraction_error[0]));
    let yd = (pixel[1] - self.config.principal_px[1])
      / (self.config.focal_px[1] * (1.0 + self.config.calibration_focal_fraction_error[1]));
    let (mut x, mut y) = (xd, yd);
    let [k1, k2, p1, p2, k3] = self.config.distortion;
    for _ in 0..8 {
      let r = x * x + y * y;
      let radial = 1.0 + k1 * r + k2 * r * r + k3 * r * r * r;
      x = (xd - 2.0 * p1 * x * y - p2 * (r + 2.0 * x * x)) / radial;
      y = (yd - p1 * (r + 2.0 * y * y) - 2.0 * p2 * x * y) / radial;
    }
    let dir = self.orientation(true) * Vector3::new(x, y, 1.0);
    let origin = Vector3::from(self.config.position_world_m)
      + Vector3::from(self.config.calibration_position_error_m);
    if dir.z.abs() < 1e-9 {
      return None;
    }
    let distance = (height - origin.z) / dir.z;
    (distance > 0.0).then_some([origin.x + distance * dir.x, origin.y + distance * dir.y])
  }
  pub fn capture(
    &mut self,
    at: TimeNs,
    physical_at: TimeNs,
    state: &WorldState,
    world: &WorldConfig,
  ) -> VisionFrame {
    self.capture_impl(at, physical_at, state, world, None)
  }
  pub(crate) fn capture_physical(
    &mut self,
    at: TimeNs,
    physical_at: TimeNs,
    world: &crate::World,
    environment: &super::scene::Environment,
  ) -> VisionFrame {
    let mut geometry = CaptureGeometry {
      scene: super::scene::Scene::from_world(world, environment, None),
      markers: BTreeMap::new(),
    };

    for (team, handles, config) in [
      (
        TeamColor::Blue,
        &world.physics.blue_robots,
        &world.config.blue_robots,
      ),
      (
        TeamColor::Yellow,
        &world.physics.yellow_robots,
        &world.config.yellow_robots,
      ),
    ] {
      for (id, handle) in handles.iter().enumerate() {
        let body = &world.physics.rigid_body_set[handle.chassis_body];
        let p = body
          .position()
          .transform_point(rapier3d::prelude::Vector::new(
            0.0,
            0.0,
            (config.height * 0.5) as f32,
          ));
        let h = body
          .position()
          .transform_point(rapier3d::prelude::Vector::new(
            0.07,
            0.0,
            (config.height * 0.5) as f32,
          ));
        let row = self
          .project([p.x as f64, p.y as f64, p.z as f64])
          .map_or(0.5, |px| {
            (px[1] / f64::from(self.config.height)).clamp(0.0, 1.0)
          });
        let exposure = (self.config.exposure_ns as f64 * 0.5
          + if self.config.rolling_shutter {
            self.config.readout_ns as f64 * row
          } else {
            0.0
          }) as f32
          * 1e-9;
        let p = p + body.velocity_at_point(p) * exposure;
        let h = h + body.velocity_at_point(h) * exposure;
        geometry.markers.insert(
          (team as u8, id),
          (
            [p.x as f64, p.y as f64, p.z as f64],
            [h.x as f64, h.y as f64, h.z as f64],
          ),
        );
      }
    }
    self.capture_impl(
      at,
      physical_at,
      &world.get_state(),
      &world.config,
      Some(&geometry),
    )
  }
  fn capture_impl(
    &mut self,
    at: TimeNs,
    physical_at: TimeNs,
    state: &WorldState,
    world: &WorldConfig,
    physical: Option<&CaptureGeometry>,
  ) -> VisionFrame {
    let dt = (at - self.last_at) as f64 * 1e-9;
    self.last_at = at;
    let rho = if self.config.correlated_noise_tau_s > 0.0 {
      (-dt / self.config.correlated_noise_tau_s).exp()
    } else {
      0.0
    };
    for value in &mut self.correlated {
      *value = rho * *value
        + (1.0 - rho * rho).sqrt() * sensors::normal(&mut self.rng) * self.config.pixel_noise_std;
    }
    let mut robots = Vec::new();
    for r in state.blue_robots.iter().chain(&state.yellow_robots) {
      if !r.is_on || self.rng.random::<f64>() < self.config.dropout_probability {
        continue;
      }
      let geometry = if r.team == TeamColor::Blue {
        &world.blue_robots
      } else {
        &world.yellow_robots
      };
      let height = geometry.start_z() + geometry.height * 0.5;
      let exposure = (self.config.exposure_ns as f64 * 0.5
        + self.config.readout_ns as f64 * 0.5 * f64::from(self.config.rolling_shutter))
        * 1e-9;
      let marker = physical.and_then(|p| p.markers.get(&(r.team as u8, r.id)));
      let point = marker.map_or(
        [
          r.x + r.vx * exposure,
          r.y + r.vy * exposure,
          r.z + geometry.height * 0.5,
        ],
        |m| m.0,
      );
      let hidden = if let Some(p) = physical {
        p.scene.occluded(
          Vector3::from(self.config.position_world_m),
          Vector3::from(point),
        )
      } else {
        occluded(
          self.config.position_world_m,
          point,
          state,
          world,
          Some((r.team, r.id)),
        )
      };
      if hidden {
        continue;
      }
      if let Some(pixel) = self.project(point) {
        let noisy = [
          pixel[0]
            + self.correlated[0]
            + sensors::normal(&mut self.rng) * self.config.pixel_noise_std,
          pixel[1]
            + self.correlated[1]
            + sensors::normal(&mut self.rng) * self.config.pixel_noise_std,
        ];
        if let Some(mut position) = self.unproject(noisy, height) {
          for axis in 0..2 {
            position[axis] += self.config.persistent_position_bias_m[axis]
              + self.config.spatial_error_m_per_m[axis] * position[axis]
              + self.config.drift_m_per_s[axis] * at as f64 * 1e-9;
          }
          let id = if self.rng.random::<f64>() < self.config.id_swap_probability {
            (r.id + 1) % world.robots_per_team.max(1)
          } else {
            r.id
          };
          let heading = marker
            .and_then(|m| self.project(m.1))
            .and_then(|px| self.unproject(px, height))
            .zip(self.unproject(pixel, height))
            .map_or(r.orientation, |(p, center)| {
              (p[1] - center[1]).atan2(p[0] - center[0])
            });
          let orientation = heading
            + if self.rng.random::<f64>() < self.config.heading_flip_probability {
              std::f64::consts::PI
            } else {
              0.0
            }
            + sensors::normal(&mut self.rng) * self.config.pixel_noise_std
              / self.config.focal_px[0];
          robots.push(RobotDetection {
            id,
            team: r.team,
            position_m: position,
            orientation_rad: orientation,
          });
        }
      }
    }
    if self.rng.random::<f64>() < self.config.false_detection_probability {
      robots.push(RobotDetection {
        id: 0,
        team: TeamColor::Blue,
        position_m: [
          self.rng.random::<f64>() * world.field.field_length - world.field.field_length / 2.0,
          self.rng.random::<f64>() * world.field.field_width - world.field.field_width / 2.0,
        ],
        orientation_rad: 0.0,
      });
    }
    let ball = [state.ball.x, state.ball.y, state.ball.z];
    let ball_hidden = if let Some(p) = physical {
      let origin = Vector3::from(self.config.position_world_m);
      let delta = Vector3::from(ball) - origin;
      let distance = delta.norm();
      distance <= 1e-9
        || p
          .scene
          .cast(
            origin,
            delta / distance,
            (distance - world.ball.radius * 1.01).max(0.0),
          )
          .is_some()
    } else {
      occluded(self.config.position_world_m, ball, state, world, None)
    };
    let ball_position_m =
      if self.rng.random::<f64>() >= self.config.dropout_probability && !ball_hidden {
        self
          .project(ball)
          .and_then(|p| {
            let noisy = [
              p[0] + sensors::normal(&mut self.rng) * self.config.pixel_noise_std,
              p[1] + sensors::normal(&mut self.rng) * self.config.pixel_noise_std,
            ];
            self.unproject(noisy, world.ball.radius)
          })
          .map(|p| [p[0], p[1], world.ball.radius])
      } else {
        None
      };
    let mut frame = VisionFrame {
      captured_at: at,
      physical_state_at: physical_at,
      robots,
      ball_position_m,
      camera_id: self.config.id,
      frame_number: self.sequence,
      camera_capture_ticks: None,
      image_received_at: at
        + self.config.exposure_ns
        + self.config.readout_ns
        + self.config.acquisition_delay_ns,
      sent_at: at
        + self.config.exposure_ns
        + self.config.readout_ns
        + self.config.acquisition_delay_ns
        + self.config.processing_ns,
    };
    let clock = super::clocks::DeviceClock::new(self.config.clock.clone());
    frame.camera_capture_ticks = Some(clock.counter(at));
    self.sequence += 1;
    frame
  }
}
fn occluded(
  camera: [f64; 3],
  target: [f64; 3],
  state: &WorldState,
  world: &WorldConfig,
  skip: Option<(TeamColor, usize)>,
) -> bool {
  let from = Vector3::from(camera);
  let segment = Vector3::from(target) - from;
  state
    .blue_robots
    .iter()
    .chain(&state.yellow_robots)
    .any(|r| {
      if !r.is_on || skip == Some((r.team, r.id)) {
        return false;
      }
      let geometry = if r.team == TeamColor::Blue {
        &world.blue_robots
      } else {
        &world.yellow_robots
      };
      let center = Vector3::new(r.x, r.y, r.z);
      let denom = segment.x * segment.x + segment.y * segment.y;
      let t = if denom > 1e-9 {
        ((center.x - from.x) * segment.x + (center.y - from.y) * segment.y) / denom
      } else {
        0.0
      };
      if t <= 0.0 || t >= 1.0 {
        return false;
      }
      let point = from + segment * t;
      (point.x - center.x).hypot(point.y - center.y) < geometry.radius
        && (point.z - center.z).abs() < geometry.height / 2.0
    })
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrackedObject {
  pub position_m: [f64; 2],
  pub velocity_m_s: [f64; 2],
  pub orientation_rad: Option<f64>,
  pub angular_rate_rad_s: Option<f64>,
  pub position_variance_m2: f64,
  pub last_observed_at: TimeNs,
  pub predicted_at: TimeNs,
}
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ObservedWorld {
  pub at: TimeNs,
  #[serde(with = "robot_map")]
  pub robots: BTreeMap<(u8, usize), TrackedObject>,
  pub ball: Option<TrackedObject>,
  pub ball_height_m: Option<f64>,
}
mod robot_map {
  use super::*;
  pub fn serialize<S: serde::Serializer>(
    map: &BTreeMap<(u8, usize), TrackedObject>,
    serializer: S,
  ) -> Result<S::Ok, S::Error> {
    map
      .iter()
      .map(|(k, v)| (*k, v))
      .collect::<Vec<_>>()
      .serialize(serializer)
  }
  pub fn deserialize<'de, D: serde::Deserializer<'de>>(
    deserializer: D,
  ) -> Result<BTreeMap<(u8, usize), TrackedObject>, D::Error> {
    Ok(
      Vec::<((u8, usize), TrackedObject)>::deserialize(deserializer)?
        .into_iter()
        .collect(),
    )
  }
}
#[derive(Clone)]
struct Filter {
  x: [f64; 4],
  p: [[f64; 4]; 4],
  at: TimeNs,
  seen: TimeNs,
  yaw: Option<f64>,
  omega: Option<f64>,
}
pub struct VisionTracker {
  camera_origins: BTreeMap<u32, Vector3<f64>>,
  ball_rays: BTreeMap<u32, (TimeNs, Vector3<f64>)>,
  ball_height: Option<(TimeNs, f64)>,
  config: TrackerConfig,
  robots: BTreeMap<(u8, usize), Filter>,
  ball: Option<Filter>,
  pub rejected: u64,
}
impl VisionTracker {
  pub fn new(config: TrackerConfig) -> Self {
    Self {
      camera_origins: BTreeMap::new(),
      ball_rays: BTreeMap::new(),
      ball_height: None,
      config,
      robots: BTreeMap::new(),
      ball: None,
      rejected: 0,
    }
  }
  pub fn with_cameras(mut self, cameras: &[CameraConfig]) -> Self {
    self.camera_origins = cameras
      .iter()
      .map(|c| {
        (
          c.id,
          Vector3::from(c.position_world_m) + Vector3::from(c.calibration_position_error_m),
        )
      })
      .collect();
    self
  }
  fn triangulate_ball(&mut self, frame: &VisionFrame, ground: [f64; 3]) -> Option<[f64; 3]> {
    let current_origin = *self.camera_origins.get(&frame.camera_id)?;
    let current_direction = (Vector3::from(ground) - current_origin).normalize();
    let velocity = self
      .ball
      .as_ref()
      .map_or(Vector3::zeros(), |b| Vector3::new(b.x[2], b.x[3], 0.0));
    let mut best = None;
    for (&camera, &(at, old_ground)) in &self.ball_rays {
      if camera == frame.camera_id || at > frame.captured_at || frame.captured_at - at > 30_000_000
      {
        continue;
      }
      let origin = *self.camera_origins.get(&camera)?;
      let direction = (old_ground - origin).normalize();
      if current_direction.cross(&direction).norm() < 0.02 {
        continue;
      }
      let corrected = origin + velocity * ((frame.captured_at - at) as f64 * 1e-9);
      let a = nalgebra::Matrix3::identity() - current_direction * current_direction.transpose();
      let b = nalgebra::Matrix3::identity() - direction * direction.transpose();
      if let Some(inverse) = (a + b).try_inverse() {
        let point = inverse * (a * current_origin + b * corrected);
        let residual = (a * (point - current_origin)).norm() + (b * (point - corrected)).norm();
        if point.z >= 0.0
          && point.z < 5.0
          && (point - current_origin).dot(&current_direction) > 0.0
          && (point - corrected).dot(&direction) > 0.0
          && residual < 0.1
          && best.is_none_or(|(old, _)| residual < old)
        {
          best = Some((residual, point));
        }
      }
    }
    self
      .ball_rays
      .insert(frame.camera_id, (frame.captured_at, Vector3::from(ground)));
    if let Some((_, p)) = best {
      self.ball_height = Some((frame.captured_at, p.z));
      Some([p.x, p.y, p.z])
    } else {
      None
    }
  }
  pub fn ingest(&mut self, frame: &VisionFrame) {
    for detection in &frame.robots {
      let key = (detection.team as u8, detection.id);
      let accepted = update(
        self
          .robots
          .entry(key)
          .or_insert_with(|| Filter::new(detection.position_m, frame.captured_at, &self.config)),
        detection.position_m,
        Some(detection.orientation_rad),
        frame.captured_at,
        &self.config,
      );
      if !accepted {
        self.rejected += 1;
      }
    }
    if let Some(raw_ball) = frame.ball_position_m {
      let ball = self.triangulate_ball(frame, raw_ball).unwrap_or(raw_ball);
      let filter = self
        .ball
        .get_or_insert_with(|| Filter::new([ball[0], ball[1]], frame.captured_at, &self.config));
      if !update(
        filter,
        [ball[0], ball[1]],
        None,
        frame.captured_at,
        &self.config,
      ) {
        self.rejected += 1;
      }
    }
  }
  pub fn snapshot(&self, at: TimeNs) -> ObservedWorld {
    let mut world = ObservedWorld {
      ball_height_m: self
        .ball_height
        .filter(|(seen, _)| at.saturating_sub(*seen) <= self.config.max_age_ns)
        .map(|(_, h)| h),
      at,
      ..ObservedWorld::default()
    };
    for (&id, f) in &self.robots {
      if at.saturating_sub(f.seen) <= self.config.max_age_ns {
        world.robots.insert(id, f.snapshot(at, &self.config));
      }
    }
    world.ball = self
      .ball
      .as_ref()
      .filter(|f| at.saturating_sub(f.seen) <= self.config.max_age_ns)
      .map(|f| f.snapshot(at, &self.config));
    world
  }
}
impl Filter {
  fn new(position: [f64; 2], at: TimeNs, config: &TrackerConfig) -> Self {
    let mut p = [[0.0; 4]; 4];
    for (i, row) in p.iter_mut().enumerate() {
      row[i] = if i < 2 {
        config.measurement_variance_m2
      } else {
        config.initial_velocity_variance
      };
    }
    Self {
      x: [position[0], position[1], 0.0, 0.0],
      p,
      at,
      seen: at,
      yaw: None,
      omega: None,
    }
  }
  fn predict(&mut self, at: TimeNs, config: &TrackerConfig) {
    let dt = at.saturating_sub(self.at) as f64 * 1e-9;
    let f = nalgebra::SMatrix::<f64, 4, 4>::new(
      1.0, 0.0, dt, 0.0, 0.0, 1.0, 0.0, dt, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    );
    let p = nalgebra::SMatrix::<f64, 4, 4>::from_fn(|r, c| self.p[r][c]);
    let g =
      nalgebra::SMatrix::<f64, 4, 2>::new(dt * dt / 2.0, 0.0, 0.0, dt * dt / 2.0, dt, 0.0, 0.0, dt);
    let predicted = f * p * f.transpose() + g * g.transpose() * config.acceleration_variance;
    self.p = std::array::from_fn(|r| std::array::from_fn(|c| predicted[(r, c)]));
    self.x[0] += self.x[2] * dt;
    self.x[1] += self.x[3] * dt;
    self.at = at;
  }
  fn snapshot(&self, at: TimeNs, config: &TrackerConfig) -> TrackedObject {
    let mut f = self.clone();
    f.predict(at, config);
    TrackedObject {
      position_m: [f.x[0], f.x[1]],
      velocity_m_s: [f.x[2], f.x[3]],
      orientation_rad: f
        .yaw
        .map(|y| wrap(y + f.omega.unwrap_or(0.0) * (at.saturating_sub(f.seen) as f64 * 1e-9))),
      angular_rate_rad_s: f.omega,
      position_variance_m2: f.p[0][0].max(f.p[1][1]),
      last_observed_at: f.seen,
      predicted_at: at,
    }
  }
}
fn update(
  f: &mut Filter,
  position: [f64; 2],
  yaw: Option<f64>,
  at: TimeNs,
  config: &TrackerConfig,
) -> bool {
  if at < f.at {
    return false;
  }
  let dt = (at - f.seen) as f64 * 1e-9;
  f.predict(at, config);
  let h = nalgebra::SMatrix::<f64, 2, 4>::new(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  let p = nalgebra::SMatrix::<f64, 4, 4>::from_fn(|r, c| f.p[r][c]);
  let r = nalgebra::Matrix2::identity() * config.measurement_variance_m2;
  let s = h * p * h.transpose() + r;
  let Some(inv) = s.try_inverse() else {
    return false;
  };
  let residual = nalgebra::Vector2::new(position[0] - f.x[0], position[1] - f.x[1]);
  if (residual.transpose() * inv * residual)[0] > config.gate_sigma.powi(2) {
    return false;
  }
  let k = p * h.transpose() * inv;
  let state = nalgebra::SVector::<f64, 4>::from(f.x) + k * residual;
  let eye = nalgebra::SMatrix::<f64, 4, 4>::identity() - k * h;
  let covariance = eye * p * eye.transpose() + k * r * k.transpose();
  f.x = state.into();
  f.p = std::array::from_fn(|r| std::array::from_fn(|c| covariance[(r, c)]));
  if let Some(yaw) = yaw {
    if let Some(previous) = f.yaw
      && dt > 0.0
    {
      f.omega = Some(wrap(yaw - previous) / dt);
    }
    f.yaw = Some(yaw);
  }
  f.seen = at;
  true
}
pub(crate) fn wrap(angle: f64) -> f64 {
  (angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

impl ObservedWorld {
  pub fn legacy_state(
    &self,
    world_id: usize,
    frame: u64,
    config: &WorldConfig,
  ) -> Option<WorldState> {
    let ball = self.ball.as_ref()?;
    let team = |color| {
      (0..config.robots_per_team)
        .map(|id| {
          let track = self.robots.get(&(color as u8, id));
          let position = track.map_or([0.0; 2], |t| t.position_m);
          let velocity = track.map_or([0.0; 2], |t| t.velocity_m_s);
          RobotState {
            id,
            team: color,
            x: position[0],
            y: position[1],
            z: 0.0,
            orientation: track.and_then(|t| t.orientation_rad).unwrap_or(0.0),
            vx: velocity[0],
            vy: velocity[1],
            vz: 0.0,
            v_angular: track.and_then(|t| t.angular_rate_rad_s).unwrap_or(0.0),
            infrared: false,
            dribbler_on: false,
            kick_status: crate::state::KickStatus::NoKick,
            is_on: track.is_some(),
            wheel_speeds: [0.0; 4],
          }
        })
        .collect()
    };
    Some(WorldState {
      world_id,
      frame,
      sim_time: self.at as f64 * 1e-9,
      ball: BallState {
        x: ball.position_m[0],
        y: ball.position_m[1],
        z: self.ball_height_m.unwrap_or(config.ball.radius),
        vx: ball.velocity_m_s[0],
        vy: ball.velocity_m_s[1],
        vz: 0.0,
      },
      blue_robots: team(TeamColor::Blue),
      yellow_robots: team(TeamColor::Yellow),
      goal_blue: self.ball_height_m.unwrap_or(config.ball.radius) < config.field.goal_height
        && ball.position_m[0] > config.field.field_length * 0.5
        && ball.position_m[1].abs() < config.field.goal_width * 0.5,
      goal_yellow: self.ball_height_m.unwrap_or(config.ball.radius) < config.field.goal_height
        && ball.position_m[0] < -config.field.field_length * 0.5
        && ball.position_m[1].abs() < config.field.goal_width * 0.5,
    })
  }
}
