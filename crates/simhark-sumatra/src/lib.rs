use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};

use anyhow::{Context, Result, bail};
use serde::{Deserialize, Serialize};
use tempfile::TempDir;

mod proto {
  include!(concat!(env!("OUT_DIR"), "/edu.tigers.sumatra.sim.net.rs"));
}
mod sim_net;

pub use sim_net::{SumatraSimNetConfig, SumatraSimNetServer};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SumatraLaunchConfig {
  pub repo_root: PathBuf,
  pub moduli: String,
  pub headless: bool,
  pub ai_blue: bool,
  pub ai_yellow: bool,
  pub auto_ref: bool,
  pub max_speed: bool,
  pub host: Option<String>,
  pub sim_net_port: Option<u16>,
  pub remote_client: bool,
}

impl Default for SumatraLaunchConfig {
  fn default() -> Self {
    Self {
      repo_root: default_sumatra_repo(),
      moduli: "simulation_protocol".to_string(),
      headless: true,
      ai_blue: true,
      ai_yellow: true,
      auto_ref: false,
      max_speed: true,
      host: None,
      sim_net_port: None,
      remote_client: false,
    }
  }
}

pub struct SumatraInstance {
  child: Child,
  #[allow(dead_code)]
  temp_dir: Option<TempDir>,
}

impl SumatraInstance {
  pub fn spawn(config: &SumatraLaunchConfig) -> Result<Self> {
    ensure_sumatra_repo(&config.repo_root)?;
    let launcher = sumatra_launcher(&config.repo_root);
    if !launcher.exists() {
      bail!("missing built Sumatra launcher at {}", launcher.display());
    }

    let mut temp_dir = None;
    let working_dir = if let Some(port) = config.sim_net_port {
      let dir = tempfile::Builder::new()
        .prefix("simhark-sumatra-")
        .tempdir()
        .context("failed to create isolated Sumatra config dir")?;
      // An isolated client still needs geometry, bot limits and AI settings.
      // Copy rather than symlink: Sumatra may persist changes at shutdown.
      copy_config_tree(&config.repo_root.join("config"), &dir.path().join("config"))?;
      write_user_config(dir.path(), port)?;
      temp_dir = Some(dir);
      temp_dir
        .as_ref()
        .expect("temp dir just created")
        .path()
        .to_path_buf()
    } else {
      config.repo_root.clone()
    };

    let mut sumatra_args = Vec::new();
    let moduli = if config.remote_client {
      repo_moduli_path(&config.repo_root, "sim_client")
    } else {
      repo_moduli_path(&config.repo_root, &config.moduli)
    };
    sumatra_args.push(format!("--moduli={moduli}"));
    if config.headless {
      sumatra_args.push("--headless".to_string());
    }
    if config.ai_blue {
      sumatra_args.push("--aiBlue".to_string());
    }
    if config.ai_yellow {
      sumatra_args.push("--aiYellow".to_string());
    }
    if config.auto_ref {
      sumatra_args.push("--autoRef".to_string());
    }
    // sim_client has no local simulator, so Sumatra's max-speed setup crashes there.
    if config.max_speed && !config.remote_client {
      sumatra_args.push("--maxSpeed".to_string());
    }
    if let Some(host) = &config.host {
      sumatra_args.push(format!("--host={host}"));
    }

    let child = Command::new(&launcher)
      .args(sumatra_args)
      .current_dir(&working_dir)
      .stdin(Stdio::null())
      .stdout(Stdio::inherit())
      .stderr(Stdio::inherit())
      .spawn()
      .with_context(|| {
        format!(
          "failed to start Sumatra from {}",
          config.repo_root.display()
        )
      })?;
    Ok(Self { child, temp_dir })
  }

  pub fn try_wait(&mut self) -> Result<Option<std::process::ExitStatus>> {
    Ok(self.child.try_wait()?)
  }

  pub fn kill(&mut self) -> Result<()> {
    if self.child.try_wait()?.is_none() {
      self
        .child
        .kill()
        .context("failed to kill Sumatra process")?;
    }
    Ok(())
  }
}

impl Drop for SumatraInstance {
  fn drop(&mut self) {
    let _ = self.kill();
  }
}

pub fn default_sumatra_repo() -> PathBuf {
  env::var_os("SIMHARK_SUMATRA_REPO_ROOT")
    .map(PathBuf::from)
    .unwrap_or_else(|| {
      Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../../Sumatra")
        .canonicalize()
        .unwrap_or_else(|_| Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../Sumatra"))
    })
}

fn ensure_sumatra_repo(path: &Path) -> Result<()> {
  if !path.exists() {
    bail!(
      "Sumatra checkout not found at {}. Please clone it manually there.",
      path.display()
    );
  }
  if !path.join("gradlew").exists() {
    bail!("{} does not look like a Sumatra checkout", path.display());
  }
  Ok(())
}

fn sumatra_launcher(repo_root: &Path) -> PathBuf {
  let name = if cfg!(windows) {
    "sumatra.bat"
  } else {
    "sumatra"
  };
  repo_root.join("build/install/sumatra/bin").join(name)
}

fn repo_moduli_path(repo_root: &Path, moduli: &str) -> String {
  let file_name = if moduli.ends_with(".xml") {
    moduli.to_string()
  } else {
    format!("{moduli}.xml")
  };
  repo_root
    .join("config/moduli")
    .join(file_name)
    .to_string_lossy()
    .into_owned()
}

fn write_user_config(dir: &Path, sim_net_port: u16) -> Result<()> {
  let config_dir = dir.join("config");
  fs::create_dir_all(&config_dir)
    .with_context(|| format!("failed to create {}", config_dir.display()))?;
  let content = format!(
    "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<configuration base=\"edu.tigers.sumatra.sim.net.SimNetClient\">\n    <port>{sim_net_port}</port>\n</configuration>\n"
  );
  fs::write(config_dir.join("user.xml"), content)
    .with_context(|| format!("failed to write {}", config_dir.join("user.xml").display()))
}

fn copy_config_tree(source: &Path, destination: &Path) -> Result<()> {
  fs::create_dir_all(destination)?;
  for entry in fs::read_dir(source)? {
    let entry = entry?;
    let target = destination.join(entry.file_name());
    if entry.file_type()?.is_dir() {
      copy_config_tree(&entry.path(), &target)?;
    } else if entry.file_type()?.is_file() {
      fs::copy(entry.path(), target)?;
    }
  }
  Ok(())
}
