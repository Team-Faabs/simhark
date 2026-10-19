use simhark::*;
fn main() {
  let mut world = World::new(0, WorldConfig::division_a());
  let command = WorldCommand {
    blue: vec![RobotCommand {
      move_command: Some(MoveCommand::LocalVelocity {
        forward: 1.0,
        left: 0.1,
        angular: 0.2,
      }),
      ..RobotCommand::noop(0)
    }],
    ..WorldCommand::default()
  };
  let mut hash = 0xcbf29ce484222325_u64;
  let start = std::time::Instant::now();
  for _ in 0..600 {
    for byte in bincode::serialize(&world.step(&command)).unwrap() {
      hash = (hash ^ u64::from(byte)).wrapping_mul(0x100000001b3);
    }
  }
  println!(
    "World={} PhysicsWorld={} RobotSim={} hash={:016x} seconds={:.3}",
    std::mem::size_of::<World>(),
    std::mem::size_of::<simhark::physics::PhysicsWorld>(),
    std::mem::size_of::<simhark::robot::RobotSim>(),
    hash,
    start.elapsed().as_secs_f64()
  );
}
