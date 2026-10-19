use std::{error::Error, path::PathBuf};
fn main() -> Result<(), Box<dyn Error>> {
  let mut args = std::env::args().skip(1);
  let source=PathBuf::from(args.next().ok_or("usage: simhark-vision-fixture SSL_VISION_REPO BMP_DIRECTORY OUTPUT_DIRECTORY [profile.toml] [camera_id]")?);
  let frames = PathBuf::from(args.next().ok_or("missing BMP directory")?);
  let output = PathBuf::from(args.next().ok_or("missing output directory")?);
  let config = if let Some(path) = args.next() {
    simhark::realistic::RealisticConfig::from_toml(&std::fs::read_to_string(path)?)?
  } else {
    simhark::realistic::RealisticConfig::full()
  };
  let camera = args
    .next()
    .map(|s| s.parse::<u32>())
    .transpose()?
    .unwrap_or(0);
  simhark::realistic::vision_fixture::write(&source, &frames, &output, &config, camera)?;
  println!(
    "Run ./bin/vision -c 1 -s from {}. Detections go to 224.5.23.2:10006. This is a synthetic color/calibration fixture.",
    output.display()
  );
  Ok(())
}
