use super::{
  hardware::{DriverState, MotorState},
  profile::{AuxiliaryConfig, BatteryConfig},
  time::TimeNs,
};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PowerState {
  pub voltage_v: f64,
  pub supply_current_a: f64,
  pub charge_coulomb: f64,
  pub temperature_c: f64,
  pub motor_temperature_c: [f64; 4],
  pub brownout: bool,
  pub brownout_count: u64,
  pub restart_at: TimeNs,
  pub kicker_ready_at: TimeNs,
  pub dribbler_fraction: f64,
}
impl PowerState {
  pub fn new(config: &BatteryConfig) -> Self {
    Self {
      voltage_v: config.open_circuit_voltage_v,
      supply_current_a: 0.0,
      charge_coulomb: config.capacity_coulomb,
      temperature_c: config.ambient_temperature_c,
      motor_temperature_c: [config.ambient_temperature_c; 4],
      brownout: false,
      brownout_count: 0,
      restart_at: 0,
      kicker_ready_at: 0,
      dribbler_fraction: 0.0,
    }
  }
  pub fn try_kick(&mut self, at: TimeNs, aux: &AuxiliaryConfig) -> bool {
    if self.brownout || at < self.kicker_ready_at {
      return false;
    }
    self.kicker_ready_at = at.saturating_add(aux.kick_charge_time_ns);
    if self.voltage_v > 0.0 {
      self.charge_coulomb = (self.charge_coulomb - aux.kick_energy_j / self.voltage_v).max(0.0);
    }
    true
  }
  pub fn step(
    &mut self,
    at: TimeNs,
    dt_s: f64,
    config: &BatteryConfig,
    aux: &AuxiliaryConfig,
    motors: &mut [MotorState; 4],
    _applied_voltage: [Option<f64>; 4],
    resistance_ohm: f64,
    dribbler: bool,
  ) {
    let dribbler_target = if dribbler { 1.0 } else { 0.0 };
    let gain = 1.0 - (-dt_s / aux.dribbler_time_constant_s).exp();
    self.dribbler_fraction += (dribbler_target - self.dribbler_fraction) * gain;
    let mut power = config.electronics_power_w
      + aux.dribbler_power_w * self.dribbler_fraction
      + if at < self.kicker_ready_at {
        aux.charging_power_w
      } else {
        0.0
      };
    for i in 0..4 {
      let electrical = motors[i].supply_current_a * self.voltage_v;
      power += if config.regenerative {
        electrical
      } else {
        electrical.max(0.0)
      };
      let heating = motors[i].current_a.powi(2) * resistance_ohm;
      self.motor_temperature_c[i] += (heating
        - config.motor_cooling_w_k * (self.motor_temperature_c[i] - config.ambient_temperature_c))
        * dt_s
        / config.motor_thermal_capacity_j_k;
      if self.motor_temperature_c[i] >= config.motor_shutdown_temperature_c {
        motors[i].driver = DriverState::FaultLatched;
      }
    }
    if !config.enabled {
      return;
    }
    self.supply_current_a = power / self.voltage_v.max(1.0);
    self.charge_coulomb =
      (self.charge_coulomb - self.supply_current_a * dt_s).clamp(0.0, config.capacity_coulomb);
    let soc = self.charge_coulomb / config.capacity_coulomb;
    let open = config.open_circuit_voltage_v * (0.8 + 0.2 * soc);
    self.voltage_v = (open - self.supply_current_a * config.internal_resistance_ohm).max(0.0);
    self.temperature_c += (self.supply_current_a.powi(2) * config.internal_resistance_ohm
      - config.cooling_w_k * (self.temperature_c - config.ambient_temperature_c))
      * dt_s
      / config.thermal_capacity_j_k;
    if !self.brownout && self.voltage_v < config.brownout_voltage_v {
      self.brownout = true;
      self.brownout_count += 1;
      self.restart_at = at.saturating_add(config.restart_ns);
    }
    if self.brownout && at >= self.restart_at && self.voltage_v >= config.recovery_voltage_v {
      self.brownout = false;
    }
  }
}
