use core_dump::types::DummyAi;
use referris::RefereeState;
use simhark::{SimulationEngine, TeamColor, WorldCommand, WorldConfig};
use simhark_faabs::Faabs;
use std::thread;

fn main() {
  #[cfg(any(
    feature = "viewer",
    feature = "interface",
    feature = "ssl_game_controller"
  ))]
  tokio::runtime::Builder::new_multi_thread()
    .enable_all()
    .build()
    .unwrap()
    .block_on(async {
      tokio::task::spawn_blocking(run).await.unwrap();
    });

  #[cfg(not(any(
    feature = "viewer",
    feature = "interface",
    feature = "ssl_game_controller"
  )))]
  run()
}

fn run() {
  let config = WorldConfig::division_b();

  let mut engine = SimulationEngine::new(1, config.clone());

  let mut referris = referris_simhark::ReferrisDriver::new(&config);

  let mut faabs = Faabs::<DummyAi>::with_interface(6, TeamColor::Yellow);

  let command = WorldCommand::default();

  let mut state = engine.step_with_commands(&[command]).remove(0);

  loop {
    let _ = referris.step(&state);
    let referee = referris
      .autoref_for(state.world_id)
      .referee_state()
      .map(RefereeState::to_referee);

    let mut command = WorldCommand::default();

    faabs.step(&state, &mut command, referee);

    // dbg!(&command);

    state = engine.step_with_commands(&[command]).remove(0);

    thread::sleep(std::time::Duration::from_millis(2));
  }
}
