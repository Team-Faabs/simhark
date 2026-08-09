use std::io::Result;

fn main() -> Result<()> {
  build_protos()?;
  Ok(())
}

fn build_protos() -> Result<()> {
  let proto_files = [
    "proto/grSim_Commands.proto",
    "proto/grSim_Packet.proto",
    "proto/grSim_Replacement.proto",
    "proto/grSim_Robotstatus.proto",
    "proto/ssl_gc_common.proto",
    "proto/ssl_simulation_config.proto",
    "proto/ssl_simulation_control.proto",
    "proto/ssl_simulation_error.proto",
    "proto/ssl_simulation_robot_control.proto",
    "proto/ssl_simulation_robot_feedback.proto",
    "proto/ssl_vision_detection.proto",
    "proto/ssl_vision_geometry.proto",
    "proto/ssl_vision_wrapper.proto",
  ];

  let mut config = prost_build::Config::new();
  config.extern_path(".google.protobuf.Any", "::prost_types::Any");
  config.compile_protos(&proto_files, &["proto"])?;

  println!("cargo:rerun-if-changed=proto");
  Ok(())
}
