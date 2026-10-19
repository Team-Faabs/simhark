//! JSONL process adapter example. This is the explicit surrogate, not firmware.
use simhark::{
  TeamColor, WorldConfig,
  realistic::{
    config::MotorConfig,
    controller::{EncoderVelocityController, RobotController},
    live::{ControllerRequest, ControllerResponse},
  },
};
use std::io::{BufRead, Write};
fn main() -> Result<(), Box<dyn std::error::Error>> {
  let team = if std::env::var("SIMHARK_TEAM").as_deref() == Ok("yellow") {
    TeamColor::Yellow
  } else {
    TeamColor::Blue
  };
  let id = std::env::var("SIMHARK_ROBOT_ID")
    .unwrap_or_else(|_| "0".into())
    .parse()?;
  let mut controller = EncoderVelocityController::new(
    WorldConfig::division_b().blue_robots,
    MotorConfig::default(),
    team,
    id,
  );
  let mut output = std::io::BufWriter::new(std::io::stdout().lock());
  for line in std::io::stdin().lock().lines() {
    let request: ControllerRequest = serde_json::from_str(&line?)?;
    let response = ControllerResponse {
      sequence: request.sequence,
      observed_at: request.at,
      voltage_v: controller.tick(request.at, request.command.as_ref(), &request.observations),
    };
    serde_json::to_writer(&mut output, &response)?;
    output.write_all(b"\n")?;
    output.flush()?;
  }
  Ok(())
}
