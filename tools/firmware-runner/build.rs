use sha2::{Digest, Sha256};
use std::{path::PathBuf, process::Command};
fn main() {
  println!("cargo:rerun-if-env-changed=SIMHARK_FIRMWARE_ROOT");
  let root = std::env::var_os("SIMHARK_FIRMWARE_ROOT")
    .map(PathBuf::from)
    .unwrap_or_else(|| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../Firmware"));
  let directory = root.join("src/shared/math");
  for (name, expected) in [
    (
      "pi_ctrl_s12.h",
      "d3cb0d6ecad74b4f189232c3d705aa0d1d71e587db09fbc0ecb7060e955efe2d",
    ),
    (
      "pi_ctrl_s12.c",
      "90b0735062fd84e95874e8282e738a3acacdfeb3a5925f8da636d8c2904b403b",
    ),
  ] {
    let path = directory.join(name);
    println!("cargo:rerun-if-changed={}", path.display());
    let data = std::fs::read(&path).unwrap_or_else(|e| {
      panic!(
        "unchanged TIGERs Firmware source {} required: {e}",
        path.display()
      )
    });
    assert_eq!(
      format!("{:x}", Sha256::digest(data)),
      expected,
      "firmware ABI/source changed; review the Rust native interface before changing this pin"
    );
  }
  let out = PathBuf::from(std::env::var_os("OUT_DIR").unwrap());
  let object = out.join("pi_ctrl_s12.o");
  assert!(
    Command::new("cc")
      .args(["-c", "-std=c99", "-O2", "-fwrapv", "-fPIC"])
      .arg(directory.join("pi_ctrl_s12.c"))
      .arg("-o")
      .arg(&object)
      .status()
      .unwrap()
      .success()
  );
  assert!(
    Command::new("ar")
      .arg("crs")
      .arg(out.join("libtigers_pi.a"))
      .arg(object)
      .status()
      .unwrap()
      .success()
  );
  println!("cargo:rustc-link-search=native={}", out.display());
  println!("cargo:rustc-link-lib=static=tigers_pi");
  let revision = Command::new("git")
    .arg("-C")
    .arg(root)
    .args(["rev-parse", "HEAD"])
    .output()
    .unwrap();
  println!(
    "cargo:rustc-env=SIMHARK_NATIVE_FIRMWARE_REV={}",
    String::from_utf8(revision.stdout).unwrap().trim()
  );
}
