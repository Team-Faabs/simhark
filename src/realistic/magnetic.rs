use super::{
  hardware::MotorState,
  profile::MagnetometerConfig,
  sensors::{MagneticSample, normal},
  time::TimeNs,
};
use nalgebra::{UnitQuaternion, Vector3};
use rand_chacha::ChaCha8Rng;
pub fn sample(
  config: &MagnetometerConfig,
  at: TimeNs,
  orientation: UnitQuaternion<f64>,
  position_world_m: [f64; 3],
  motors: &[MotorState; 4],
  dribbler: f64,
  charging: bool,
  rng: &mut ChaCha8Rng,
) -> MagneticSample {
  let sensor_from_body = UnitQuaternion::from_euler_angles(
    config.mounting_rpy_rad[0],
    config.mounting_rpy_rad[1],
    config.mounting_rpy_rad[2],
  )
  .inverse();
  let sensor_position =
    Vector3::from(position_world_m) + orientation * Vector3::from(config.position_body_m);
  let mut ambient = Vector3::from(config.earth_field_world_ut);
  for axis in 0..3 {
    for c in 0..3 {
      ambient[axis] += config.ambient_gradient_ut_per_m[axis][c] * sensor_position[c];
    }
  }
  let mut body_field = orientation.inverse() * ambient;
  for (i, motor) in motors.iter().enumerate() {
    let phase = motor.angle_rad * f64::from(config.rotor_harmonic);
    for axis in 0..3 {
      body_field[axis] += config.current_field_ut_per_a[i][axis] * motor.current_a
        + config.rotor_field_ut[i][axis] * phase.cos();
    }
  }
  for source in &config.motor_dipoles {
    let motor = &motors[source.wheel];
    let rotation = UnitQuaternion::from_axis_angle(
      &nalgebra::Unit::new_normalize(Vector3::from(source.rotation_axis_body)),
      motor.angle_rad * f64::from(config.rotor_harmonic),
    );
    let moment = Vector3::from(source.current_moment_am2_per_a) * motor.current_a
      + rotation * Vector3::from(source.rotor_moment_am2);
    let delta = Vector3::from(config.position_body_m) - Vector3::from(source.position_body_m);
    let distance = delta.norm();
    let direction = delta / distance;
    body_field += (direction * (3.0 * moment.dot(&direction)) - moment) * (0.1 / distance.powi(3));
  }
  let transformed = sensor_from_body * body_field;
  let mut field = [0.0; 3];
  let mut clipped = false;
  for axis in 0..3 {
    field[axis] = config.hard_iron_ut[axis] + normal(rng) * config.noise_std_ut;
    for other in 0..3 {
      field[axis] += config.soft_iron[axis][other] * transformed[other];
    }
    field[axis] += config.dribbler_field_ut[axis] * dribbler
      + if charging {
        config.charging_field_ut[axis]
      } else {
        0.0
      };
    clipped |= field[axis].abs() > config.range_ut;
    field[axis] = field[axis].clamp(-config.range_ut, config.range_ut);
  }
  MagneticSample {
    sampled_at: at,
    field_ut: field,
    clipped,
  }
}
