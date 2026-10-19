//! Actual unchanged TIGERs fixed-point motor PI, with Rust sensor/actuator HAL.
//! This executes the controller used by motor2019, not a complete STM32 image.
use serde::{Deserialize, Serialize};
use simhark::{
  MoveCommand, RobotCommand, TeamColor, WorldCommand, WorldConfig,
  realistic::{
    RealisticConfig, RealisticWorld, controller::RobotController, sensors::Observations,
    transport::Safety,
  },
};
use std::{
  error::Error,
  sync::{
    Arc,
    atomic::{AtomicU64, Ordering},
  },
};
#[repr(C)]
#[derive(Clone, Copy, Default)]
struct NativePi {
  setpoint: i32,
  kp: i32,
  ki: i32,
  i_term: i32,
  output: i32,
  output_min: i32,
  output_max: i32,
  anti_jitter: i32,
  anti_jitter_reciprocal: i32,
  overload: u8,
  enabled: u8,
}
unsafe extern "C" {
  fn PICtrlS12Init(pi: *mut NativePi, kp: i32, ki: i32, output_min: i32, output_max: i32);
  fn PICtrlS12Enable(pi: *mut NativePi, enable: u8);
  fn PICtrlS12Update(pi: *mut NativePi, measured: i32);
  fn PICtrlS12Setpoint(pi: *mut NativePi, setpoint: i32);
}
#[derive(Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct Gains {
  speed_kp: i32,
  speed_ki: i32,
  current_kp: i32,
  current_ki: i32,
  current_limit_a: f64,
  encoder_ticks_per_revolution: u32,
}
impl Default for Gains {
  fn default() -> Self {
    Self {
      speed_kp: 12288,
      speed_ki: 819,
      current_kp: 512,
      current_ki: 512,
      current_limit_a: 1.5,
      encoder_ticks_per_revolution: 2048,
    }
  }
}
struct NativeMotorController {
  speed: [NativePi; 4],
  current: [NativePi; 4],
  gains: Gains,
  geometry: simhark::RobotConfig,
  last_encoder: Option<u64>,
  last_ticks: [i64; 4],
  calls: Arc<AtomicU64>,
}
impl NativeMotorController {
  fn new(geometry: simhark::RobotConfig, gains: Gains, calls: Arc<AtomicU64>) -> Self {
    assert_eq!(std::mem::size_of::<NativePi>(), 40);
    let limit = (gains.current_limit_a * 4096.0 / 10.8)
      .round()
      .clamp(1.0, 3420.0) as i32;
    let mut speed = [NativePi::default(); 4];
    let mut current = [NativePi::default(); 4];
    for i in 0..4 {
      unsafe {
        PICtrlS12Init(&mut speed[i], gains.speed_kp, gains.speed_ki, -limit, limit);
        PICtrlS12Init(
          &mut current[i],
          gains.current_kp,
          gains.current_ki,
          -3000,
          3000,
        );
      }
    }
    Self {
      speed,
      current,
      gains,
      geometry,
      last_encoder: None,
      last_ticks: [0; 4],
      calls,
    }
  }
}
impl RobotController for NativeMotorController {
  fn tick(
    &mut self,
    now: u64,
    command: Option<&RobotCommand>,
    observations: &Observations,
  ) -> [Option<f64>; 4] {
    let Some(command) = command else {
      for i in 0..4 {
        unsafe {
          PICtrlS12Enable(&mut self.speed[i], 0);
          PICtrlS12Enable(&mut self.current[i], 0);
        }
      }
      return [None; 4];
    };
    let (Some(encoder), Some(adc)) = (&observations.encoder, &observations.motor_adc) else {
      return [None; 4];
    };
    if now.saturating_sub(adc.sampled_at) > 200_000
      || now.saturating_sub(encoder.sampled_at) > 3_000_000
    {
      return [None; 4];
    }
    let wheels = match command.move_command {
      Some(MoveCommand::WheelVelocity(w)) => w,
      Some(MoveCommand::LocalVelocity {
        forward,
        left,
        angular,
      }) => self.geometry.wheel_angles.map(|a| {
        let a = a.to_radians();
        (-forward * a.sin() + left * a.cos() + self.geometry.radius * angular)
          / self.geometry.wheel_radius
      }),
      _ => return [None; 4],
    };
    if self
      .last_encoder
      .is_none_or(|last| encoder.sampled_at > last)
    {
      // These exact units match motor2019/drive_ctrl.c: encoder delta per ms
      // multiplied by two; PI output 4096 represents 10.8 A q current.
      let dt = self
        .last_encoder
        .map_or(0.001, |last| (encoder.sampled_at - last) as f64 * 1e-9);
      for i in 0..4 {
        let desired = (wheels[i] * f64::from(self.gains.encoder_ticks_per_revolution)
          / std::f64::consts::TAU
          * 0.001
          * 2.0)
          .round()
          .clamp(-4096.0, 4095.0) as i32;
        let measured = ((encoder.ticks[i] - self.last_ticks[i]) as f64 / dt * 0.001 * 2.0)
          .round()
          .clamp(-4096.0, 4095.0) as i32;
        unsafe {
          PICtrlS12Enable(&mut self.speed[i], 1);
          PICtrlS12Setpoint(&mut self.speed[i], desired);
          PICtrlS12Update(&mut self.speed[i], measured);
        }
      }
      self.last_ticks = encoder.ticks;
      self.last_encoder = Some(encoder.sampled_at);
    }
    std::array::from_fn(|i| {
      let measured = (adc.phase_current_a[i] * 4096.0 / 10.8)
        .round()
        .clamp(-4096.0, 4095.0) as i32;
      unsafe {
        PICtrlS12Enable(&mut self.current[i], 1);
        PICtrlS12Setpoint(&mut self.current[i], self.speed[i].output);
        PICtrlS12Update(&mut self.current[i], measured);
      }
      self.calls.fetch_add(1, Ordering::Relaxed);
      // motor2019 maps q output to four millivolts per integer unit.
      Some((f64::from(self.current[i].output) * 0.004).clamp(-adc.bus_voltage_v, adc.bus_voltage_v))
    })
  }
}
fn main() -> Result<(), Box<dyn Error>> {
  let gains: Gains = std::env::args()
    .nth(1)
    .map(|path| {
      std::fs::read(path).and_then(|b| serde_json::from_slice(&b).map_err(std::io::Error::other))
    })
    .transpose()?
    .unwrap_or_default();
  if [gains.speed_kp, gains.current_kp]
    .iter()
    .any(|v| !(0..=131071).contains(v))
    || [gains.speed_ki, gains.current_ki]
      .iter()
      .any(|v| !(0..=65535).contains(v))
    || !gains.current_limit_a.is_finite()
    || !(0.001..=9.0).contains(&gains.current_limit_a)
    || gains.encoder_ticks_per_revolution == 0
  {
    return Err("unsupported native PI gains, current limit or encoder resolution".into());
  }
  let mut base = WorldConfig::division_b();
  base.robots_per_team = 1;
  let mut profile = RealisticConfig::full();
  profile.physics_step_ns = 25_000;
  profile.control_rate_hz = 20_000;
  profile.motor_adc.enabled = true;
  profile.imu.delivery_delay_ns = 0;
  let calls = Arc::new(AtomicU64::new(0));
  let controller = NativeMotorController::new(base.blue_robots.clone(), gains, calls.clone());
  let mut sim = RealisticWorld::new(0, base, profile)?;
  sim.set_controller(TeamColor::Blue, 0, Box::new(controller))?;
  let start = sim.truth().blue_robots[0].clone();
  for tick in 0..2000 {
    let now = sim.now();
    sim.publish(
      WorldCommand {
        blue: vec![RobotCommand {
          move_command: Some(MoveCommand::LocalVelocity {
            forward: if tick < 1000 { 0.5 } else { -0.5 },
            left: 0.0,
            angular: 0.0,
          }),
          ..RobotCommand::noop(0)
        }],
        ..Default::default()
      },
      now,
      1,
      Safety::Run,
    )?;
    sim.advance_to(now + 1_000_000)?;
  }
  let end = sim.truth().blue_robots[0].clone();
  let n = calls.load(Ordering::Relaxed);
  println!(
    "{}",
    serde_json::to_string_pretty(
      &serde_json::json!({"firmware_revision":env!("SIMHARK_NATIVE_FIRMWARE_REV"),"controller":"unchanged motor2019 PICtrlS12 speed/current code","peripheral_model":"quantized encoder and ADC; averaged scalar q-axis RL plant; no STM32 instruction/peripheral emulation","profile":"provisional gains/plant","native_current_controller_calls":n,"start":[start.x,start.y],"end":[end.x,end.y],"diagnostics":sim.diagnostics(),"hardware":sim.robot_diagnostics()})
    )?
  );
  if n < 100_000 || !end.x.is_finite() || !end.y.is_finite() {
    return Err("native controller integration failed".into());
  }
  Ok(())
}
