//! Averaged wheel-shaft RL motors, latched drivers, and planar omniwheel contacts.
use super::{
  config::{ContactConfig, MotorConfig},
  time::TimeNs,
};
use crate::RobotConfig;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DriverState {
  Enabled,
  CurrentLimited,
  FaultLatched,
  Resetting { ready_at: TimeNs },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotorState {
  pub speed_rad_s: f64,
  pub angle_rad: f64,
  pub current_a: f64,
  pub peak_current_a: f64,
  pub filtered_phase_current_a: f64,
  pub supply_current_a: f64,
  pub average_phase_current_a: f64,
  pub applied_phase_voltage_v: f64,
  pub driver: DriverState,
  pub trips: u64,
  exposure_ns: f64,
}
impl Default for MotorState {
  fn default() -> Self {
    Self {
      speed_rad_s: 0.0,
      angle_rad: 0.0,
      current_a: 0.0,
      peak_current_a: 0.0,
      filtered_phase_current_a: 0.0,
      supply_current_a: 0.0,
      average_phase_current_a: 0.0,
      applied_phase_voltage_v: 0.0,
      driver: DriverState::Enabled,
      trips: 0,
      exposure_ns: 0.0,
    }
  }
}
impl MotorState {
  /// Reset is explicit and takes modeled time. A fault never auto-clears.
  pub fn request_reset(&mut self, now: TimeNs, config: &MotorConfig) {
    if self.driver == DriverState::FaultLatched {
      self.driver = DriverState::Resetting {
        ready_at: now
          .checked_add(config.reset_duration_ns)
          .expect("time overflow"),
      };
    }
  }
  pub fn encoder_ticks(&self, config: &MotorConfig) -> i64 {
    (self.angle_rad * f64::from(config.encoder_ticks_per_revolution) / std::f64::consts::TAU)
      .floor() as i64
  }
  /// Constant-voltage/back-EMF exact RL evolution within a mechanical step.
  /// The driver monitors absolute phase current plus an explicit peak margin.
  /// Disabled output is modeled as open circuit/coasting, not active braking.
  pub fn step(
    &mut self,
    voltage_v: Option<f64>,
    reaction_torque_nm: f64,
    dt_ns: u64,
    now: TimeNs,
    config: &MotorConfig,
  ) {
    if matches!(self.driver, DriverState::Resetting { ready_at } if now >= ready_at) {
      self.driver = DriverState::Enabled;
      self.exposure_ns = 0.0;
    }
    if self.driver == DriverState::CurrentLimited {
      self.driver = DriverState::Enabled;
    }
    let dt = dt_ns as f64 * 1e-9;
    let old_speed = self.speed_rad_s;
    let mut torque = 0.0;
    self.peak_current_a = 0.0;
    if self.driver == DriverState::Enabled
      && let Some(voltage) = voltage_v
    {
      let mut steady = (voltage.clamp(-config.supply_voltage_v, config.supply_voltage_v)
        - config.back_emf_v_per_rad_s * self.speed_rad_s)
        / config.resistance_ohm;
      if let Some(limit) = config.current_limit_a {
        if steady.abs() > limit {
          steady = steady.clamp(-limit, limit);
          self.driver = DriverState::CurrentLimited;
        }
      }
      let tau = config.inductance_h / config.resistance_ohm;
      let initial = self.current_a;
      let value_at = |t: f64| steady + (initial - steady) * (-t / tau).exp();
      let final_current = value_at(dt);
      let effective_voltage =
        steady * config.resistance_ohm + config.back_emf_v_per_rad_s * self.speed_rad_s;
      let ratio = effective_voltage.abs() / config.supply_voltage_v.max(1e-9);
      let old_filtered = self.filtered_phase_current_a;
      let filtered = old_filtered
        + (0.5 * (initial.abs() + final_current.abs()) - old_filtered)
          * (1.0 - (-dt / config.current_filter_time_constant_s).exp());
      let metric_at = |t: f64| match config.trip_current_type {
        super::config::TripCurrent::PhasePeak => value_at(t).abs() + config.peak_margin_a,
        super::config::TripCurrent::Supply => value_at(t).abs() * ratio + config.peak_margin_a,
        super::config::TripCurrent::PhaseFiltered => {
          old_filtered + (filtered - old_filtered) * t / dt
        }
      };
      let scale = if config.trip_current_type == super::config::TripCurrent::Supply {
        ratio.max(1e-12)
      } else {
        1.0
      };
      let limit = (config.trip_current_a - config.peak_margin_a).max(0.0) / scale;
      // Split at both threshold crossings: reversal can pass through a safe
      // interval even if both endpoints have a large absolute current.
      let mut cuts = vec![0.0, dt];
      if initial != steady {
        for threshold in [-limit, limit] {
          let ratio = (threshold - steady) / (initial - steady);
          if ratio > 0.0 {
            let t = -tau * ratio.ln();
            if t > 0.0 && t < dt {
              cuts.push(t);
            }
          }
        }
      }
      if config.trip_current_type == super::config::TripCurrent::PhaseFiltered
        && filtered != old_filtered
      {
        let crossing = dt * (config.trip_current_a - old_filtered) / (filtered - old_filtered);
        if crossing > 0.0 && crossing < dt {
          cuts.push(crossing);
        }
      }
      cuts.sort_by(f64::total_cmp);
      let mut powered_time = dt;
      for interval in cuts.windows(2) {
        if metric_at((interval[0] + interval[1]) * 0.5) >= config.trip_current_a {
          let needed = (config.trip_exposure_ns as f64 - self.exposure_ns).max(0.0) * 1e-9;
          let length = interval[1] - interval[0];
          if needed <= length {
            powered_time = interval[0] + needed;
            self.driver = DriverState::FaultLatched;
            self.trips += 1;
            break;
          }
          self.exposure_ns += length * 1e9;
        } else {
          self.exposure_ns = 0.0;
        }
      }
      let integral =
        steady * powered_time + (initial - steady) * tau * (1.0 - (-powered_time / tau).exp());
      torque = config.torque_constant_nm_per_a * integral / dt;
      self.peak_current_a = initial.abs().max(value_at(powered_time).abs()) + config.peak_margin_a;
      self.filtered_phase_current_a = filtered;
      self.average_phase_current_a = integral / dt;
      self.applied_phase_voltage_v = effective_voltage * powered_time / dt;
      self.supply_current_a = integral / dt * effective_voltage / config.supply_voltage_v.max(1e-9);
      self.current_a = if matches!(
        self.driver,
        DriverState::Enabled | DriverState::CurrentLimited
      ) {
        final_current
      } else {
        0.0
      };
    } else {
      self.current_a = 0.0;
      self.supply_current_a = 0.0;
      self.average_phase_current_a = 0.0;
      self.applied_phase_voltage_v = 0.0;
      self.filtered_phase_current_a *= (-dt / config.current_filter_time_constant_s).exp();
      self.exposure_ns = 0.0;
    }
    self.speed_rad_s +=
      (torque - reaction_torque_nm - config.viscous_drag_nm_per_rad_s * old_speed)
        / config.wheel_inertia_kg_m2
        * dt;
    self.angle_rad += (old_speed + self.speed_rad_s) * 0.5 * dt;
  }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WheelContact {
  pub normal_load_n: f64,
  pub traction_n: f64,
  pub slip_m_s: f64,
  pub lifted: bool,
}
impl Default for WheelContact {
  fn default() -> Self {
    Self {
      normal_load_n: 0.0,
      traction_n: 0.0,
      slip_m_s: 0.0,
      lifted: false,
    }
  }
}
pub(crate) struct DriveHardware {
  pub motors: [MotorState; 4],
  pub contacts: [WheelContact; 4],
  pub voltage: [Option<f64>; 4],
  pub load_acceleration: [f64; 2],
}
pub(crate) struct DriveStep {
  pub body_velocity: [f64; 3],
  pub acceleration: [f64; 2],
  pub dt_ns: u64,
  pub now: TimeNs,
  pub gravity: f64,
}
impl DriveHardware {
  pub fn new() -> Self {
    Self {
      motors: std::array::from_fn(|_| MotorState::default()),
      contacts: std::array::from_fn(|_| WheelContact::default()),
      voltage: [None; 4],
      load_acceleration: [0.0; 2],
    }
  }
  /// Local [Fx,Fy,Tz]. Chassis remains planar; this reports unloading and slip,
  /// and deliberately makes no claim to simulate complete tip-over.
  pub fn step(
    &mut self,
    step: DriveStep,
    robot: &RobotConfig,
    motor: &MotorConfig,
    contact: &ContactConfig,
    profile: &super::profile::HardwareProfile,
  ) -> [f64; 3] {
    let DriveStep {
      mut body_velocity,
      acceleration,
      dt_ns,
      now,
      gravity,
    } = step;
    let dt = dt_ns as f64 * 1e-9;
    let gain = 1.0 - (-dt / contact.load_transfer_time_constant_s).exp();
    for (load, acceleration) in self.load_acceleration.iter_mut().zip(acceleration) {
      *load += gain * (acceleration - *load);
    }
    let angles = robot.wheel_angles.map(f64::to_radians);
    // Minimum-norm equal-compliance load distribution satisfying support force
    // and pitch/roll moments before lift. Clamp/renormalize after lift: this is
    // a provisional planar approximation, not a rigid four-contact solution.
    let geometry = nalgebra::SMatrix::<f64, 3, 4>::from_fn(|row, col| match row {
      0 => 1.0,
      1 => robot.radius * angles[col].cos(),
      _ => robot.radius * angles[col].sin(),
    });
    let response = crate::contact::ContactResponse::new(
      &profile.wheel,
      &contact.floor,
      contact.friction,
      profile.transverse_friction,
      contact.slip_stiffness_n_per_m_s,
      profile.rolling_resistance,
    );
    let material_overrides = profile.wheel != crate::config::WheelProfile::default()
      || contact.floor != crate::config::FloorConfig::default();
    let mass = profile
      .mass_kg
      .or(robot.mass_kg)
      .unwrap_or(robot.body_mass + robot.kicker_mass);
    let weight = mass * gravity;
    let com = profile
      .center_of_mass_body_m
      .or(robot.center_of_mass_body_m)
      .unwrap_or([0.0, 0.0, contact.center_of_mass_height_m - robot.start_z()]);
    let com_height = com[2] + robot.start_z();
    // Wheel kinematics use chassis-origin velocity; Rapier reports COM velocity.
    body_velocity[0] += body_velocity[2] * com[1];
    body_velocity[1] -= body_velocity[2] * com[0];
    // Preserve legacy planar load transfer unless physical mass/COM is supplied.
    let transfer_mass = if profile.mass_kg.is_some()
      || profile.center_of_mass_body_m.is_some()
      || robot.mass_kg.is_some()
      || robot.center_of_mass_body_m.is_some()
    {
      mass
    } else {
      robot.body_mass
    };
    let target = nalgebra::Vector3::new(
      weight,
      weight * com[0] - transfer_mass * com_height * self.load_acceleration[0],
      weight * com[1] - transfer_mass * com_height * self.load_acceleration[1],
    );
    let loads = geometry.transpose()
      * (geometry * geometry.transpose())
        .try_inverse()
        .expect("validated wheel support geometry")
      * target;
    let sum: f64 = loads.iter().map(|v| v.max(0.0)).sum();
    let mut wrench = [0.0; 3];
    for i in 0..4 {
      let direction = [-angles[i].sin(), angles[i].cos()];
      let surface_speed = direction[0] * body_velocity[0]
        + direction[1] * body_velocity[1]
        + robot.radius * body_velocity[2];
      let normal = if sum > 0.0 {
        loads[i].max(0.0) * weight / sum
      } else {
        0.0
      };
      let slip = self.motors[i].speed_rad_s * robot.wheel_radius - surface_speed;
      let traction = response.longitudinal_force(slip, normal);
      let side_speed = -direction[1] * body_velocity[0] + direction[0] * body_velocity[1];
      let side_force = if material_overrides {
        response.transverse_force(-side_speed, normal)
      } else {
        0.0
      };
      self.contacts[i] = WheelContact {
        normal_load_n: normal,
        traction_n: traction,
        slip_m_s: slip,
        lifted: normal <= 1e-6,
      };
      self.motors[i].step(
        self.voltage[i],
        traction * robot.wheel_radius
          + if material_overrides {
            response.rolling_resistance
              * normal
              * robot.wheel_radius
              * self.motors[i].speed_rad_s.signum()
          } else {
            0.0
          },
        dt_ns,
        now,
        motor,
      );
      wrench[0] += traction * direction[0] - side_force * direction[1];
      wrench[1] += traction * direction[1] + side_force * direction[0];
      wrench[2] += traction * robot.radius;
    }
    // The returned force is applied at COM, so express its yaw moment there too.
    wrench[2] += com[1] * wrench[0] - com[0] * wrench[1];
    wrench
  }
}
