//! Physical geometry shared by camera, LiDAR, and RF occlusion generators.
use crate::World;
use nalgebra::{UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LandmarkBox {
  pub min_m: [f64; 3],
  pub max_m: [f64; 3],
  pub reflectivity: f64,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Environment {
  pub legacy_field_colliders: bool,
  pub standard_goals: bool,
  pub physical: Vec<LandmarkBox>,
  pub known_map: Vec<LandmarkBox>,
}
impl Default for Environment {
  fn default() -> Self {
    Self {
      legacy_field_colliders: true,
      standard_goals: false,
      physical: Vec::new(),
      known_map: Vec::new(),
    }
  }
}
impl Environment {
  /// Standard goal structure is an explicit venue choice, not an implicit
  /// raised boundary around the pitch. Physical and surveyed maps stay separate.
  pub fn goal_boxes(field: &crate::config::FieldConfig) -> Vec<LandmarkBox> {
    let mut boxes = Vec::new();
    let half = field.field_length * 0.5;
    let t = field.goal_thickness;
    for side in [-1.0, 1.0] {
      let (front, back) = (side * half, side * (half + field.goal_depth));
      let (lo, hi) = (front.min(back), front.max(back));
      for y in [-field.goal_width * 0.5, field.goal_width * 0.5] {
        boxes.push(LandmarkBox {
          min_m: [lo, y - t * 0.5, 0.0],
          max_m: [hi, y + t * 0.5, field.goal_height],
          reflectivity: 0.6,
        });
      }
      boxes.push(LandmarkBox {
        min_m: [back - t * 0.5, -field.goal_width * 0.5, 0.0],
        max_m: [back + t * 0.5, field.goal_width * 0.5, field.goal_height],
        reflectivity: 0.25,
      });
      boxes.push(LandmarkBox {
        min_m: [
          front - t * 0.5,
          -field.goal_width * 0.5,
          field.goal_height - t,
        ],
        max_m: [front + t * 0.5, field.goal_width * 0.5, field.goal_height],
        reflectivity: 0.6,
      });
    }
    boxes
  }
  pub fn physical_boxes(&self, field: &crate::config::FieldConfig) -> Vec<LandmarkBox> {
    let mut boxes = self.physical.clone();
    if self.standard_goals {
      boxes.extend(Self::goal_boxes(field));
    }
    boxes
  }
  pub(crate) fn install(&self, world: &mut World) {
    let physics = &mut world.physics;
    if !self.legacy_field_colliders {
      for collider in physics.wall_colliders.drain(..) {
        physics.collider_set.remove(
          collider,
          &mut physics.island_manager,
          &mut physics.rigid_body_set,
          true,
        );
      }
    }
    let groups = physics.collider_set[physics.ground_collider].collision_groups();
    for b in self.physical_boxes(&world.config.field) {
      let lo = Vector3::from(b.min_m);
      let hi = Vector3::from(b.max_m);
      let c = (lo + hi) * 0.5;
      let h = (hi - lo) * 0.5;
      physics.collider_set.insert(
        rapier3d::prelude::ColliderBuilder::cuboid(h.x as f32, h.y as f32, h.z as f32)
          .translation(rapier3d::prelude::Vector::new(
            c.x as f32, c.y as f32, c.z as f32,
          ))
          .friction(0.4)
          .restitution(world.config.ball.bounce as f32)
          .collision_groups(groups)
          .build(),
      );
    }
  }
}
pub struct SceneBody {
  pub position: Vector3<f64>,
  pub orientation: UnitQuaternion<f64>,
  pub half_extent: Vector3<f64>,
  pub reflectivity: f64,
}
pub struct Scene {
  pub bodies: Vec<SceneBody>,
  pub ball_position: Vector3<f64>,
  pub ball_radius: f64,
}
impl Scene {
  pub fn from_world(
    world: &World,
    environment: &Environment,
    skip: Option<(crate::TeamColor, usize)>,
  ) -> Self {
    let mut bodies = Vec::new();
    for (team, handles, sims, geometry) in [
      (
        crate::TeamColor::Blue,
        &world.physics.blue_robots,
        &world.blue_sims,
        &world.config.blue_robots,
      ),
      (
        crate::TeamColor::Yellow,
        &world.physics.yellow_robots,
        &world.yellow_sims,
        &world.config.yellow_robots,
      ),
    ] {
      for (id, (handle, sim)) in handles.iter().zip(sims).enumerate() {
        if !sim.is_on || skip == Some((team, id)) {
          continue;
        }
        let body = &world.physics.rigid_body_set[handle.chassis_body];
        let q = body.rotation();
        let p = body.translation();
        bodies.push(SceneBody {
          position: Vector3::new(p.x as f64, p.y as f64, p.z as f64),
          orientation: UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            q.w as f64, q.x as f64, q.y as f64, q.z as f64,
          )),
          half_extent: Vector3::new(geometry.radius, geometry.radius, geometry.height / 2.0),
          reflectivity: 0.5,
        });
      }
    }
    for b in &environment.physical_boxes(&world.config.field) {
      let min = Vector3::from(b.min_m);
      let max = Vector3::from(b.max_m);
      bodies.push(SceneBody {
        position: (min + max) / 2.0,
        orientation: UnitQuaternion::identity(),
        half_extent: (max - min) / 2.0,
        reflectivity: b.reflectivity,
      });
    }
    let p = world.physics.get_body_position(world.physics.ball_body);
    Self {
      bodies,
      ball_position: Vector3::new(p.x as f64, p.y as f64, p.z as f64),
      ball_radius: world.config.ball.radius,
    }
  }
  pub fn cast(
    &self,
    origin: Vector3<f64>,
    direction: Vector3<f64>,
    max_range: f64,
  ) -> Option<(f64, f64)> {
    let mut hit = None;
    for body in &self.bodies {
      let inverse = body.orientation.inverse();
      let o = inverse * (origin - body.position);
      let d = inverse * direction;
      if let Some(distance) = ray_box(o, d, body.half_extent)
        && distance <= max_range
        && hit.is_none_or(|(old, _)| distance < old)
      {
        hit = Some((distance, body.reflectivity));
      }
    }
    let o = origin - self.ball_position;
    let b = o.dot(&direction);
    let c = o.norm_squared() - self.ball_radius.powi(2);
    let disc = b * b - c;
    if disc >= 0.0 {
      let distance = -b - disc.sqrt();
      if distance > 0.0 && distance <= max_range && hit.is_none_or(|(old, _)| distance < old) {
        hit = Some((distance, 0.6));
      }
    }
    hit
  }
  pub fn occluded(&self, from: Vector3<f64>, to: Vector3<f64>) -> bool {
    let delta = to - from;
    let length = delta.norm();
    length > 1e-9 && self.cast(from, delta / length, length - 1e-5).is_some()
  }
}
fn ray_box(origin: Vector3<f64>, direction: Vector3<f64>, half: Vector3<f64>) -> Option<f64> {
  let mut near = 0.0_f64;
  let mut far = f64::INFINITY;
  for i in 0..3 {
    if direction[i].abs() < 1e-12 {
      if origin[i].abs() > half[i] {
        return None;
      }
    } else {
      let a = (-half[i] - origin[i]) / direction[i];
      let b = (half[i] - origin[i]) / direction[i];
      near = near.max(a.min(b));
      far = far.min(a.max(b));
    }
  }
  (far >= near && far > 0.0).then_some(if near > 0.0 { near } else { far })
}
