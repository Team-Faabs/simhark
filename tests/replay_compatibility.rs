use simhark::{ReplayLog, ReplayRecorder, WorldConfig};

#[test]
fn reads_binary_replay_written_by_pre_realistic_revision() {
  // This fixture was generated using the unmodified config/state/command/replay
  // sources from 1b1f84b (PR #2's base), not the compatibility decoder's DTOs.
  let path = std::env::temp_dir().join(format!(
    "simhark-legacy-fixture-{}.shreplay",
    std::process::id()
  ));
  std::fs::write(&path, include_bytes!("fixtures/replay-v1-base.shreplay")).unwrap();
  let replay = ReplayLog::read_zstd(&path).unwrap();
  std::fs::remove_file(path).unwrap();
  let cfg = &replay.metadata.world_config;
  assert_eq!(cfg.seed, 9283);
  assert_eq!(cfg.robots_per_team, 1);
  assert_eq!(cfg.blue_robots.radius, 0.087);
  assert_eq!(cfg.yellow_robots.body_mass, 2.7);
  assert_eq!(cfg.physics.delta_time, 0.005);
  assert_eq!(cfg.physics.gravity, 9.7);
  assert!(cfg.blue_robots.mass_kg.is_none());
  assert!(cfg.yellow_robots.center_of_mass_body_m.is_none());
  assert!(cfg.blue_robots.wheel.is_none());
  assert_eq!(cfg.physics.floor, Default::default());
  assert_eq!(replay.frames.len(), 1);
  let frame = &replay.frames[0];
  assert_eq!(frame.frame, 42);
  assert_eq!(frame.states[0].ball.vx, 6.0);
  assert_eq!(frame.commands[0].blue[0].kick_speed, 3.5);
  assert_eq!(frame.debug[0].strategy.as_deref(), Some("legacy debug"));
  assert_eq!(frame.events.len(), 1);
  assert_eq!(replay.events.len(), 1);
}

#[test]
fn v2_round_trip_and_pr2_files_with_unversioned_expanded_config_are_readable() {
  let path =
    std::env::temp_dir().join(format!("simhark-replay-v2-{}.shreplay", std::process::id()));
  let mut config = WorldConfig::division_b();
  config.blue_robots.mass_kg = Some(3.2);
  config.yellow_robots.center_of_mass_body_m = Some([0.01, 0.02, 0.03]);
  config.physics.floor.longitudinal_grip_scale = 0.7;
  let log = ReplayRecorder::new(1, config, 60.0, "version test".into()).finish();
  log.write_zstd(&path).unwrap();
  let mut bytes = std::fs::read(&path).unwrap();
  assert_eq!(&bytes[..8], b"SHREPL02");
  let verify = |decoded: ReplayLog| {
    assert_eq!(decoded.metadata.world_config.blue_robots.mass_kg, Some(3.2));
    assert_eq!(
      decoded
        .metadata
        .world_config
        .yellow_robots
        .center_of_mass_body_m,
      Some([0.01, 0.02, 0.03])
    );
    assert_eq!(
      decoded
        .metadata
        .world_config
        .physics
        .floor
        .longitudinal_grip_scale,
      0.7
    );
  };
  verify(ReplayLog::read_zstd(&path).unwrap());
  // PR #2 wrote this exact expanded bincode layout under the old magic.
  bytes[..8].copy_from_slice(b"SHREPL01");
  std::fs::write(&path, bytes).unwrap();
  verify(ReplayLog::read_zstd(&path).unwrap());
  std::fs::remove_file(path).unwrap();
}
