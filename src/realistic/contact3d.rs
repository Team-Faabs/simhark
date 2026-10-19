//! Compliant point supports and directional omniwheel traction on a free chassis.
use super::{
  config::{ContactConfig, MotorConfig},
  hardware::{DriveHardware, WheelContact},
  profile::HardwareProfile,
};
use crate::{
  RobotConfig,
  physics::{PhysicsWorld, RobotHandles},
};
use rapier3d::prelude::{MassProperties, Vector};

pub(crate) fn install(
  physics: &mut PhysicsWorld,
  handle: &RobotHandles,
  robot: &RobotConfig,
  hardware: &HardwareProfile,
  com_height: f64,
) {
  // Geometry remains attached to the body; this profile supplies mass/inertia
  // separately so raising COM does not move visual/collision geometry.
  for collider in [
    handle.chassis_collider,
    handle.dummy_collider,
    handle.kicker_collider,
  ] {
    physics.collider_set[collider].set_mass(0.0);
  }
  let mass = hardware
    .mass_kg
    .or(robot.mass_kg)
    .unwrap_or(robot.body_mass + robot.kicker_mass);
  let com = hardware
    .center_of_mass_body_m
    .or(robot.center_of_mass_body_m)
    .unwrap_or([0.0, 0.0, com_height - robot.start_z()]);
  let transverse = mass * (3.0 * robot.radius.powi(2) + robot.height.powi(2)) / 12.0;
  let inertia = hardware.inertia_diagonal_kg_m2.unwrap_or([
    transverse,
    transverse,
    mass * robot.radius.powi(2) / 2.0,
  ]);
  let body = &mut physics.rigid_body_set[handle.chassis_body];
  if hardware.dynamics == super::profile::Dynamics::Free3d {
    body.set_enabled_rotations(true, true, true, true);
    body.set_enabled_translations(true, true, true, true);
  }
  body.set_additional_mass_properties(
    MassProperties::new(
      Vector::new(com[0] as f32, com[1] as f32, com[2] as f32),
      mass as f32,
      Vector::new(inertia[0] as f32, inertia[1] as f32, inertia[2] as f32),
    ),
    true,
  );
  body.recompute_mass_properties_from_colliders(&physics.collider_set);
}

pub(crate) struct ContactStep<'a> {
  pub dt_ns: u64,
  pub now: u64,
  pub robot: &'a RobotConfig,
  pub motor: &'a MotorConfig,
  pub profile: &'a HardwareProfile,
  pub contact: &'a ContactConfig,
}
pub(crate) fn advance(
  body: &mut rapier3d::prelude::RigidBody,
  drive: &mut DriveHardware,
  step: ContactStep<'_>,
) {
  let ContactStep {
    dt_ns,
    now,
    robot,
    motor,
    profile,
    contact,
  } = step;
  let response = crate::contact::ContactResponse::new(
    &profile.wheel,
    &contact.floor,
    contact.friction,
    profile.transverse_friction,
    contact.slip_stiffness_n_per_m_s,
    profile.rolling_resistance,
  );
  for i in 0..4 {
    let angle = robot.wheel_angles[i].to_radians();
    let local = Vector::new(
      (robot.radius * angle.cos()) as f32,
      (robot.radius * angle.sin()) as f32,
      -robot.start_z() as f32,
    );
    let world = body.position().transform_point(local);
    let velocity = body.velocity_at_point(world);
    let penetration = (-world.z).max(0.0) as f64;
    let normal = if world.z <= 0.0 {
      (profile.support_stiffness_n_m * penetration
        - profile.support_damping_n_s_m * f64::from(velocity.z))
      .max(0.0)
    } else {
      0.0
    };
    let mut direction =
      body
        .rotation()
        .mul_vec3(Vector::new(-angle.sin() as f32, angle.cos() as f32, 0.0));
    direction.z = 0.0;
    if direction.length_squared() > 1e-12 {
      direction = direction.normalize();
    }
    let sideways = Vector::new(-direction.y, direction.x, 0.0);
    let forward_speed = f64::from(velocity.dot(direction));
    let side_speed = f64::from(velocity.dot(sideways));
    let slip = drive.motors[i].speed_rad_s * robot.wheel_radius - forward_speed;
    let traction = response.longitudinal_force(slip, normal);
    let side_force = response.transverse_force(-side_speed, normal);
    let mut force = direction * traction as f32 + sideways * side_force as f32;
    force.z = normal as f32;
    body.add_force_at_point(force, world, true);
    let rolling = response.rolling_resistance
      * normal
      * robot.wheel_radius
      * drive.motors[i].speed_rad_s.signum();
    drive.motors[i].step(
      drive.voltage[i],
      traction * robot.wheel_radius + rolling,
      dt_ns,
      now,
      motor,
    );
    drive.contacts[i] = WheelContact {
      normal_load_n: normal,
      traction_n: traction,
      slip_m_s: slip,
      lifted: normal <= 1e-6,
    };
  }
}
