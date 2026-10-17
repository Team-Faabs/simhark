//! Shared wheel/floor material response for planar and free-3D contacts.
use crate::config::{FloorConfig, WheelProfile};

#[derive(Clone, Copy)]
pub(crate) struct ContactResponse {
  pub longitudinal_friction: f64,
  pub transverse_friction: f64,
  longitudinal_stiffness: f64,
  transverse_stiffness: f64,
  pub rolling_resistance: f64,
}

impl ContactResponse {
  pub fn new(
    wheel: &WheelProfile,
    floor: &FloorConfig,
    longitudinal_friction: f64,
    transverse_friction: f64,
    slip_stiffness: f64,
    rolling_resistance: f64,
  ) -> Self {
    // Wheel and carpet deform in series: their slip compliances add.
    let stiffness = |wheel: f64, floor: f64| wheel / (1.0 + wheel * floor);
    Self {
      longitudinal_friction: wheel.longitudinal_friction.unwrap_or(longitudinal_friction)
        * floor.longitudinal_grip_scale,
      transverse_friction: wheel.transverse_friction.unwrap_or(transverse_friction)
        * floor.transverse_grip_scale,
      longitudinal_stiffness: stiffness(
        wheel
          .longitudinal_slip_stiffness_n_per_m_s
          .unwrap_or(slip_stiffness),
        floor.longitudinal_slip_compliance_m_s_per_n,
      ),
      transverse_stiffness: stiffness(
        wheel
          .transverse_slip_stiffness_n_per_m_s
          .unwrap_or(slip_stiffness),
        floor.transverse_slip_compliance_m_s_per_n,
      ),
      rolling_resistance: wheel.rolling_resistance.unwrap_or(rolling_resistance)
        * floor.rolling_resistance_scale,
    }
  }

  pub fn longitudinal_force(&self, slip: f64, normal: f64) -> f64 {
    let limit = self.longitudinal_friction * normal;
    (self.longitudinal_stiffness * slip).clamp(-limit, limit)
  }

  pub fn transverse_force(&self, slip: f64, normal: f64) -> f64 {
    let limit = self.transverse_friction * normal;
    (self.transverse_stiffness * slip).clamp(-limit, limit)
  }
}
