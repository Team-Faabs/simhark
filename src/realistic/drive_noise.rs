//! Independent seeded driver and encoder errors. Physical shaft state remains truth.
use super::{config::MotorConfig, hardware::MotorState, sensors};
use rand::RngExt;
use rand_chacha::ChaCha8Rng;

pub(crate) struct DriveNoise {
  motor_rng: ChaCha8Rng,
  encoder_rng: ChaCha8Rng,
  voltage_error_v: [f64; 4],
  initialized: bool,
}

impl DriveNoise {
  pub fn new(seed: u64, world: usize, robot: usize) -> Self {
    Self {
      motor_rng: sensors::rng(seed, world, robot, 1300),
      encoder_rng: sensors::rng(seed, world, robot, 1301),
      voltage_error_v: [0.0; 4],
      initialized: false,
    }
  }

  pub fn voltages(
    &mut self,
    requested: [Option<f64>; 4],
    dt_s: f64,
    config: &MotorConfig,
  ) -> [Option<f64>; 4] {
    if config.voltage_noise_std_v == 0.0 {
      return requested;
    }
    // Exact Ornstein-Uhlenbeck transition with a stationary initial sample.
    let rho = if self.initialized && config.voltage_noise_time_constant_s > 0.0 {
      (-dt_s / config.voltage_noise_time_constant_s).exp()
    } else {
      0.0
    };
    let innovation = (1.0 - rho * rho).max(0.0).sqrt() * config.voltage_noise_std_v;
    self.initialized = true;
    std::array::from_fn(|i| {
      self.voltage_error_v[i] =
        rho * self.voltage_error_v[i] + innovation * sensors::normal(&mut self.motor_rng);
      // Open outputs remain open; noise cannot bypass safety/watchdog gating.
      requested[i].map(|v| {
        (v + self.voltage_error_v[i]).clamp(-config.supply_voltage_v, config.supply_voltage_v)
      })
    })
  }

  pub fn encoder_ticks(
    &mut self,
    motors: &[MotorState; 4],
    config: &MotorConfig,
  ) -> Option<[i64; 4]> {
    if config.encoder_dropout_probability > 0.0
      && self.encoder_rng.random::<f64>() < config.encoder_dropout_probability
    {
      return None;
    }
    Some(std::array::from_fn(|i| {
      if config.encoder_noise_std_ticks == 0.0 {
        motors[i].encoder_ticks(config)
      } else {
        let ticks = motors[i].angle_rad * f64::from(config.encoder_ticks_per_revolution)
          / std::f64::consts::TAU;
        (ticks + config.encoder_noise_std_ticks * sensors::normal(&mut self.encoder_rng)).floor()
          as i64
      }
    }))
  }
}
