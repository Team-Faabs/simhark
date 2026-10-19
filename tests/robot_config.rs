use simhark::{MoveCommand, RobotCommand, World, WorldCommand, WorldConfig};

fn loaded(text: &str) -> WorldConfig {
  let mut config = WorldConfig::division_b();
  config.apply_simulation_toml(text).unwrap();
  config.robots_per_team = 1;
  config
}

#[test]
fn simulation_loader_preserves_defaults_and_applies_team_and_floor_settings() {
  let config = loaded(
    r#"
[server]
robot_socket_port = 8192
[simulation.blue_robots]
mass_kg = 3.4
center_of_mass_body_m = [0.01, -0.02, -0.05]
[simulation.blue_robots.wheel]
longitudinal_friction = 0.6
transverse_slip_stiffness_n_per_m_s = 3.0
[simulation.physics.floor]
longitudinal_grip_scale = 0.7
"#,
  );
  assert_eq!(config.field.field_length, 9.0);
  assert_eq!(config.blue_robots.mass_kg, Some(3.4));
  assert_eq!(config.yellow_robots.mass_kg, None);
  assert_eq!(config.blue_robots.radius, 0.09);
  assert_eq!(
    config.blue_robots.wheel.unwrap().longitudinal_friction,
    Some(0.6)
  );
  assert_eq!(config.physics.floor.longitudinal_grip_scale, 0.7);
  assert_eq!(config.physics.floor.transverse_grip_scale, 1.0);
}

#[test]
fn simulation_loader_rejects_invalid_values_and_typos_without_partial_updates() {
  let mut config = WorldConfig::division_b();
  for text in [
    "[simulation.blue_robots]\nmass_kg = -1.0",
    "[simulation.blue_robots]\nmass_kg = nan",
    "[simulation.blue_robots.wheel]\nlongitudinal_friction = inf",
    "[simulation.blue_robots]\ncenter_of_mass_body_m = [0.0, 0.0]",
    "[simulation.blue_robots.wheel]\nlongitudinal_friction = -0.1",
    "[simulation.blue_robots.wheel]\nlongitudnal_friction = 0.5",
    "[simulation.physics.floor]\nlongitudinal_slip_compliance_m_s_per_n = -0.1",
    "[simulation.blue_robots]\nbody_mas = 3.0",
  ] {
    assert!(config.apply_simulation_toml(text).is_err(), "{text}");
    assert_eq!(config.blue_robots.mass_kg, None);
    assert_eq!(config.blue_robots.body_mass, 2.0);
  }
}

#[test]
fn standard_physics_installs_loaded_total_mass_and_com() {
  let config =
    loaded("[simulation.blue_robots]\nmass_kg = 3.4\ncenter_of_mass_body_m = [0.01, -0.02, -0.05]");
  let mut world = World::new(0, config);
  world.step(&WorldCommand::default());
  let body = &world.physics.rigid_body_set[world.physics.blue_robots[0].chassis_body];
  assert!((body.mass() - 3.4).abs() < 1e-5);
  let com = body.mass_properties().local_mprops.local_com;
  assert!((com.x - 0.01).abs() < 1e-6);
  assert!((com.y + 0.02).abs() < 1e-6);
  assert!((com.z + 0.05).abs() < 1e-6);
}

fn driven_speed(wheel_grip: f64, floor_grip: f64, compliance: f64) -> f64 {
  let config = loaded(&format!(
    r#"
[simulation.blue_robots.wheel]
longitudinal_friction = {wheel_grip}
rolling_resistance = 0.0
[simulation.physics.floor]
longitudinal_grip_scale = {floor_grip}
longitudinal_slip_compliance_m_s_per_n = {compliance}
"#
  ));
  let wheel_speeds = config
    .blue_robots
    .wheel_angles
    .map(|a| -a.to_radians().sin() / config.blue_robots.wheel_radius);
  let mut world = World::new(0, config);
  let command = WorldCommand {
    blue: vec![RobotCommand {
      move_command: Some(MoveCommand::WheelVelocity(wheel_speeds)),
      ..RobotCommand::noop(0)
    }],
    ..Default::default()
  };
  let mut state = world.get_state();
  for _ in 0..15 {
    state = world.step(&command);
  }
  state.blue_robots[0].vx
}

#[test]
fn standard_wheel_and_floor_settings_independently_change_traction() {
  let normal = driven_speed(0.8, 1.0, 0.0);
  let slippery_floor = driven_speed(0.8, 0.05, 0.0);
  let slippery_wheel = driven_speed(0.04, 1.0, 0.0);
  let soft_floor = driven_speed(0.8, 1.0, 2.0);
  assert!(normal > 0.2, "{normal}");
  assert!(
    slippery_floor < normal * 0.5,
    "normal {normal}, floor {slippery_floor}"
  );
  assert!((slippery_floor - slippery_wheel).abs() < 1e-5);
  assert!(
    soft_floor < normal * 0.5,
    "normal {normal}, soft {soft_floor}"
  );
  assert!(driven_speed(0.8, 0.0, 0.0).abs() < 1e-6);
}

#[test]
fn repository_config_loads_with_legacy_default_motion() {
  let mut config = WorldConfig::division_b();
  let before = serde_json::to_value(&config).unwrap();
  config.load_default_simulation_config().unwrap();
  assert_eq!(before, serde_json::to_value(&config).unwrap());
}

#[cfg(feature = "realistic")]
#[test]
fn realistic_loader_applies_mass_com_and_separate_material_settings() {
  use simhark::realistic::{RealisticConfig, RealisticWorld};
  RealisticConfig::from_toml(include_str!("../profiles/realistic/full.toml")).unwrap();
  let config = RealisticConfig::from_toml(
    r#"
[hardware]
dynamics = "free3d"
mass_kg = 3.4
center_of_mass_body_m = [0.015, 0.0, -0.0505]
[hardware.wheel]
longitudinal_friction = 0.6
transverse_friction = 0.02
longitudinal_slip_stiffness_n_per_m_s = 20.0
transverse_slip_stiffness_n_per_m_s = 4.0
rolling_resistance = 0.015
[contact.floor]
longitudinal_grip_scale = 0.7
transverse_grip_scale = 0.8
longitudinal_slip_compliance_m_s_per_n = 0.01
transverse_slip_compliance_m_s_per_n = 0.03
rolling_resistance_scale = 1.2
"#,
  )
  .unwrap();
  let roundtrip = RealisticConfig::from_toml(&config.to_toml().unwrap()).unwrap();
  assert_eq!(roundtrip.hardware.mass_kg, Some(3.4));
  assert_eq!(roundtrip.hardware.wheel.transverse_friction, Some(0.02));
  assert_eq!(roundtrip.contact.floor.transverse_grip_scale, 0.8);
  let base = loaded("");
  let angles = base.blue_robots.wheel_angles.map(f64::to_radians);
  let radius = base.blue_robots.radius;
  let mut sim = RealisticWorld::new(0, base, config).unwrap();
  sim.advance_to(400_000_000).unwrap();
  let diagnostics = sim.robot_diagnostics();
  let contacts = &diagnostics[0].contacts;
  let load: f64 = contacts.iter().map(|c| c.normal_load_n).sum();
  assert!((load - 3.4 * 9.81).abs() < 0.5, "support load {load}");
  let moment: f64 = contacts
    .iter()
    .zip(angles)
    .map(|(c, a)| c.normal_load_n * radius * a.cos())
    .sum();
  assert!(
    (moment / load - 0.015).abs() < 0.003,
    "CoM projection {}",
    moment / load
  );
}

#[cfg(feature = "realistic")]
#[test]
fn realistic_planar_and_free3d_use_wheel_and_floor_grip() {
  use simhark::{
    TeamColor,
    realistic::{
      RealisticConfig, RealisticWorld, controller::RobotController, sensors::Observations,
    },
  };
  struct Voltage;
  impl RobotController for Voltage {
    fn tick(&mut self, _: u64, _: Option<&RobotCommand>, _: &Observations) -> [Option<f64>; 4] {
      [Some(-3.0), Some(-3.0), Some(3.0), Some(3.0)]
    }
  }
  for mode in ["planar", "free3d"] {
    let run = |wheel_grip: f64, floor_grip: f64| {
      let config = RealisticConfig::from_toml(&format!(
        r#"
[hardware]
dynamics = "{mode}"
[hardware.wheel]
longitudinal_friction = {wheel_grip}
rolling_resistance = 0.0
[contact.floor]
longitudinal_grip_scale = {floor_grip}
"#
      ))
      .unwrap();
      let mut sim = RealisticWorld::new(0, loaded(""), config).unwrap();
      sim
        .set_controller(TeamColor::Blue, 0, Box::new(Voltage))
        .unwrap();
      let x = sim.truth().blue_robots[0].x;
      for _ in 0..20 {
        let now = sim.now();
        sim
          .publish(
            WorldCommand {
              blue: vec![RobotCommand {
                move_command: Some(MoveCommand::LocalVelocity {
                  forward: 1.0,
                  left: 0.0,
                  angular: 0.0,
                }),
                ..RobotCommand::noop(0)
              }],
              ..Default::default()
            },
            now,
            1,
            simhark::realistic::transport::Safety::Run,
          )
          .unwrap();
        sim.advance_to(now + 10_000_000).unwrap();
      }
      (sim.truth().blue_robots[0].x - x).abs()
    };
    let normal = run(0.8, 1.0);
    assert!(normal > 0.005, "{mode}: {normal}");
    assert!(run(0.8, 0.0) < normal * 0.05, "{mode}: floor has no effect");
    assert!(run(0.0, 1.0) < normal * 0.05, "{mode}: wheel has no effect");
  }
}

#[cfg(feature = "realistic")]
#[test]
fn realistic_loader_rejects_invalid_mass_com_and_material_values() {
  use simhark::realistic::RealisticConfig;
  for text in [
    "[hardware]\nmass_kg = 0.0",
    "[hardware]\nmass_kg = nan",
    "[hardware]\ncenter_of_mass_body_m = [inf, 0.0, 0.0]",
    "[hardware.wheel]\ntransverse_slip_stiffness_n_per_m_s = -1.0",
    "[hardware.wheel]\nlongitudinal_friction = nan",
    "[contact.floor]\nlongitudinal_grip_scale = -0.5",
    "[contact.floor]\ntransverse_slip_compliance_m_s_per_n = inf",
  ] {
    assert!(RealisticConfig::from_toml(text).is_err(), "{text}");
  }
}
