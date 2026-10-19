use super::time::Timeline;
use super::{
  config::{RealisticConfig, SlotPurpose},
  controller::{EncoderVelocityController, RobotController},
  hardware::{DriveHardware, DriveStep, MotorState, WheelContact},
  sensors::{self, EncoderSample, ImuSample, Observations, RobotDetection, VisionFrame},
  time::{Periodic, TimeNs},
  transport::{CommandPacket, DelayTrace, Publication, Radio, Receiver, Rejection, Safety},
};
use crate::{RobotCommand, RobotConfig, TeamColor, World, WorldCommand, WorldConfig, WorldState};
use rand::RngExt;
use rand_chacha::ChaCha8Rng;
use rapier3d::prelude::Vector;
use serde::{Deserialize, Serialize};
mod auxiliaries;
mod peripherals;
mod ranging;

enum Event {
  MotorAdc {
    robot: usize,
    index: u64,
  },
  MotorAdcDelivery {
    robot: usize,
    sample: sensors::MotorAdcSample,
  },
  PublishLatest {
    index: u64,
  },
  DecisionComplete(Decision),
  CameraAcquired {
    camera: usize,
    frame: VisionFrame,
  },
  CameraComplete {
    camera: usize,
    frame: VisionFrame,
  },
  EmpiricalCameraComplete {
    camera: usize,
    frame: VisionFrame,
  },
  RadioAux {
    slot: usize,
    index: u64,
  },
  SynchronizationDelivery {
    ticket: u64,
  },
  UwbStart {
    team: TeamColor,
    packets: Vec<CommandPacket>,
    occupied_ns: u64,
  },
  UwbResponse(super::radio::RangeJob),
  UwbFinal(super::radio::RangeJob),
  UwbRangingDelivery(super::radio::RangeJob),
  TdoaDelivery {
    ticket: u64,
    observations: Vec<(usize, super::localization::UwbObservation)>,
  },
  UwbTelemetryDelivery {
    robot: usize,
    ticket: u64,
    telemetry: Option<sensors::RobotTelemetry>,
    infrared: Option<sensors::InfraredSample>,
    action: Option<u64>,
  },
  LidarBeam {
    robot: usize,
    index: u64,
  },
  LidarDelivery {
    robot: usize,
    scan: super::localization::LidarScan,
  },
  OnboardCamera {
    robot: usize,
    index: u64,
  },
  OnboardDelivery {
    robot: usize,
    ball: Option<super::localization::BallObservation>,
  },
  VideoEncoded {
    robot: usize,
    captured_at: TimeNs,
  },
  VideoDecoded {
    robot: usize,
    frame: super::localization::StreamedFrame,
  },
  WifiRelease,
  LanRelease,
  LanEnqueue(VisionFrame),
  Bulk {
    flow: usize,
    index: u64,
  },
  Physics,
  Frame {
    index: u64,
  },
  Control,
  Encoder,
  Imu {
    robot: usize,
    device: usize,
    index: u64,
  },
  ImuDelivery {
    robot: usize,
    samples: Vec<ImuSample>,
  },
  EncoderDelivery {
    robot: usize,
    sample: EncoderSample,
  },
  Vision,
  Camera {
    camera: usize,
    index: u64,
  },
  TrackerComplete {
    audience: usize,
    world: super::vision::ObservedWorld,
  },
  VisionDelivery(VisionFrame),
  Magnetometer {
    robot: usize,
    index: u64,
  },
  MagneticDelivery {
    robot: usize,
    sample: sensors::MagneticSample,
  },
  TelemetryDelivery {
    robot: usize,
    telemetry: sensors::RobotTelemetry,
    infrared: sensors::InfraredSample,
  },
  Ready(Publication),
  Transmit {
    team: TeamColor,
    occupied_ns: u64,
  },
  Delivery {
    ticket: Option<u64>,
    team: TeamColor,
    packets: Vec<CommandPacket>,
    radio: Radio,
  },
}

enum WifiPayload {
  Command {
    team: TeamColor,
    packets: Vec<CommandPacket>,
  },
  Video {
    robot: usize,
    captured_at: TimeNs,
    encoded_at: TimeNs,
  },
  Bulk,
}
struct PendingKick {
  fires_at: TimeNs,
  expires_at: TimeNs,
  safety_epoch: u64,
  command: RobotCommand,
}
struct Robot {
  pending_kick: Option<PendingKick>,
  pose_estimator: super::localization::PoseEstimator,
  lidar_beams: Vec<super::localization::LidarBeam>,
  localization_rng: ChaCha8Rng,
  last_radio_sync: Option<TimeNs>,
  team: TeamColor,
  id: usize,
  config: RobotConfig,
  hardware: DriveHardware,
  imu_devices: Vec<super::devices::DeviceState>,
  receiver: Receiver,
  controller: Box<dyn RobotController>,
  observations: Observations,
  previous_ticks: [i64; 4],
  previous_encoder_at: Option<TimeNs>,
  previous_velocity: [f64; 3],
  acceleration: [f64; 2],
  requested_voltage: [Option<f64>; 4],
  requested_at: TimeNs,
  requested_command_times: Option<(TimeNs, TimeNs)>,
  motor_config: super::config::MotorConfig,
  hardware_profile: super::profile::HardwareProfile,
  power: super::power::PowerState,
  magnetic_profile: super::profile::MagnetometerConfig,
  magnetic_rng: ChaCha8Rng,
  infrared_candidate: bool,
  infrared_changed_at: TimeNs,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Diagnostics {
  pub radio: super::radio::RadioDiagnostics,
  pub wifi_link: super::network::LinkDiagnostics,
  pub lan_link: super::network::LinkDiagnostics,
  pub actions_consumed: u64,
  pub accepted: u64,
  pub rejected: u64,
  pub coalesced: u64,
  pub dropped: u64,
  pub physics_steps: u64,
  pub control_ticks: u64,
  pub max_decision_age_at_actuation_ns: u64,
  pub max_publication_age_at_actuation_ns: u64,
  pub max_actuation_quantization_ns: u64,
  pub max_sensor_quantization_ns: u64,
  pub watchdog_or_expired_ticks: u64,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotDiagnostics {
  pub team: TeamColor,
  pub id: usize,
  pub motors: [MotorState; 4],
  pub contacts: [WheelContact; 4],
  pub power: super::power::PowerState,
  pub attitude_xyzw: [f64; 4],
  pub angular_velocity_rad_s: [f64; 3],
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum TraceEvent {
  PacketCopies {
    at: TimeNs,
    team: TeamColor,
    radio: Radio,
    packets: Vec<CommandPacket>,
  },
  CameraInput {
    at: TimeNs,
    frame: VisionFrame,
  },
  RadioInput {
    at: TimeNs,
    team: TeamColor,
    id: usize,
    observation: super::localization::UwbObservation,
  },
  Sensors {
    at: TimeNs,
    team: TeamColor,
    id: usize,
    delivered: Box<Observations>,
  },
  Publication {
    at: TimeNs,
    sequence: u64,
    decision_at: TimeNs,
  },
  Acceptance {
    at: TimeNs,
    team: TeamColor,
    id: usize,
    sequence: u64,
    radio: Radio,
    rejection: Option<Rejection>,
  },
  Control {
    at: TimeNs,
    team: TeamColor,
    id: usize,
    sequence: Option<u64>,
    decision_age_ns: Option<u64>,
    voltage_v: [Option<f64>; 4],
  },
  Actuation {
    at: TimeNs,
    team: TeamColor,
    id: usize,
    quantization_ns: u64,
    decision_age_ns: Option<u64>,
    publication_age_ns: Option<u64>,
  },
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Recording {
  pub schema_version: u32,
  pub build_identity: String,
  pub resolved_profile_sha256: String,
  pub world_id: usize,
  pub config: RealisticConfig,
  pub world_config: WorldConfig,
  pub diagnostics: Diagnostics,
  pub events: Vec<TraceEvent>,
  pub inputs: Vec<RecordedInput>,
  pub uses_custom_controller: bool,
  pub final_truth: WorldState,
  pub final_hardware: Vec<RobotDiagnostics>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ActionIdentity {
  pub team: TeamColor,
  pub robot_id: usize,
  pub action_id: u64,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum RecordedInput {
  LocalizationReset {
    team: TeamColor,
    id: usize,
    initial: Option<[f64; 3]>,
  },
  Decision {
    commands: WorldCommand,
    decision_at: TimeNs,
    safety_epoch: u64,
    safety: Safety,
  },
  PublishActions {
    commands: WorldCommand,
    decision_at: TimeNs,
    safety_epoch: u64,
    safety: Safety,
    actions: Vec<ActionIdentity>,
  },
  VisionIngress {
    frame: VisionFrame,
  },
  Attitude {
    team: TeamColor,
    id: usize,
    rpy_rad: [f64; 3],
    height_m: Option<f64>,
  },
  Publish {
    commands: WorldCommand,
    decision_at: TimeNs,
    safety_epoch: u64,
    safety: Safety,
  },
  Advance {
    end: TimeNs,
  },
  Teleport {
    commands: WorldCommand,
  },
  DriverReset {
    team: TeamColor,
    id: usize,
  },
}

impl Recording {
  /// Replay only under the same caller-supplied build identity. Custom
  /// controllers must provide their own replay factory and are rejected here.
  pub fn replay(&self, build_identity: &str) -> Result<RealisticWorld, String> {
    self.replay_inner(build_identity, None)
  }
  pub fn replay_with_controllers(
    &self,
    build_identity: &str,
    mut factory: impl FnMut(TeamColor, usize) -> Box<dyn RobotController>,
  ) -> Result<RealisticWorld, String> {
    self.replay_inner(build_identity, Some(&mut factory))
  }
  /// Replay the recorded nondeterministic/native controller output boundary.
  /// Sensor/receiver/physics paths are still rerun and must reproduce their trace.
  pub fn replay_recorded_controls(&self, build_identity: &str) -> Result<RealisticWorld, String> {
    self.replay_with_controllers(build_identity, |team, id| {
      Box::new(super::controller::RecordedController::new(
        self
          .events
          .iter()
          .filter_map(|event| match event {
            TraceEvent::Control {
              at,
              team: t,
              id: i,
              voltage_v,
              ..
            } if *t == team && *i == id => Some((*at, *voltage_v)),
            _ => None,
          })
          .collect(),
      ))
    })
  }
  fn replay_inner(
    &self,
    build_identity: &str,
    mut factory: Option<&mut dyn FnMut(TeamColor, usize) -> Box<dyn RobotController>>,
  ) -> Result<RealisticWorld, String> {
    if self.schema_version != 1 {
      return Err("unsupported recording schema".into());
    }
    if self.resolved_profile_sha256
      != super::calibration::digest(&(&self.config, &self.world_config))?
    {
      return Err("recording profile digest mismatch".into());
    }
    if self.build_identity != build_identity {
      return Err("recording build identity does not match".into());
    }
    if self.uses_custom_controller && factory.is_none() {
      return Err("recording needs a custom controller replay adapter".into());
    }
    let mut sim = RealisticWorld::new(
      self.world_id,
      self.world_config.clone(),
      self.config.clone(),
    )?;
    sim.enable_recording();
    if let Some(factory) = &mut factory {
      for team in [TeamColor::Blue, TeamColor::Yellow] {
        for id in 0..self.world_config.robots_per_team {
          sim.set_controller(team, id, factory(team, id))?;
        }
      }
    }
    for input in &self.inputs {
      match input {
        RecordedInput::PublishActions {
          commands,
          decision_at,
          safety_epoch,
          safety,
          actions,
        } => {
          sim.publish_with_actions(
            commands.clone(),
            *decision_at,
            *safety_epoch,
            *safety,
            actions.clone(),
          )?;
        }
        RecordedInput::Decision {
          commands,
          decision_at,
          safety_epoch,
          safety,
        } => sim.submit_decision(commands.clone(), *decision_at, *safety_epoch, *safety)?,
        RecordedInput::LocalizationReset { team, id, initial } => {
          sim.reset_localization(*team, *id, *initial)?
        }
        RecordedInput::VisionIngress { frame } => sim.inject_vision(frame.clone())?,
        RecordedInput::Attitude {
          team,
          id,
          rpy_rad,
          height_m,
        } => sim.set_attitude(*team, *id, *rpy_rad, *height_m)?,
        RecordedInput::Publish {
          commands,
          decision_at,
          safety_epoch,
          safety,
        } => {
          sim.publish(commands.clone(), *decision_at, *safety_epoch, *safety)?;
        }
        RecordedInput::Advance { end } => sim.advance_to(*end)?,
        RecordedInput::Teleport { commands } => sim.teleport(commands),
        RecordedInput::DriverReset { team, id } => sim.request_driver_reset(*team, *id)?,
      }
    }
    Ok(sim)
  }
}

#[derive(Clone)]
struct Decision {
  commands: WorldCommand,
  decision_at: TimeNs,
  safety_epoch: u64,
  safety: Safety,
  actions: Vec<ActionIdentity>,
}
/// Team adapters get only explicitly delivered observations. `truth()` and
/// diagnostics are separate evaluation APIs, never arguments to controller ticks.
pub struct RealisticWorld {
  camera_jobs: Vec<super::processing::ProcessingResource<VisionFrame>>,
  tracker_jobs: [super::processing::ProcessingResource<VisionFrame>; 3],
  decision_jobs: super::processing::ProcessingResource<Decision>,
  latest_decision: Option<Decision>,
  internal_publication: bool,
  mac: super::radio::MacArbiter,
  host_feedback: Vec<
    Option<(
      sensors::RobotTelemetry,
      Option<sensors::InfraredSample>,
      Option<u64>,
    )>,
  >,
  host_video: Vec<Option<super::localization::StreamedFrame>>,
  tdoa_reference: Vec<Option<(u32, TimeNs, u64, u64)>>,
  wifi_link: Option<super::network::SharedLink<WifiPayload>>,
  lan_link: Option<super::network::SharedLink<VisionFrame>>,
  network_rng: ChaCha8Rng,
  cameras: Vec<super::vision::CameraModel>,
  trackers: [super::vision::VisionTracker; 3],
  tracked: [super::vision::ObservedWorld; 3],
  visible: [super::vision::ObservedWorld; 3],
  world: World,
  config: RealisticConfig,
  timeline: Timeline<Event>,
  robots: Vec<Robot>,
  sequence: u64,
  action_sequence: u64,
  session: u64,
  pending: [Option<Publication>; 2],
  wifi: DelayTrace,
  uwb: DelayTrace,
  common: DelayTrace,
  control_clock: Periodic,
  encoder_clock: Periodic,
  vision_clock: Periodic,
  vision_rng: ChaCha8Rng,
  physical_time: TimeNs,
  pending_auxiliaries: std::collections::VecDeque<(TimeNs, WorldCommand)>,
  diagnostics: Diagnostics,
  trace: Option<Vec<TraceEvent>>,
  delivered_frames: std::collections::BTreeMap<u32, VisionFrame>,
  inputs: Vec<RecordedInput>,
  uses_custom_controller: bool,
  setup_performed: bool,
}
impl RealisticWorld {
  pub fn new(
    id: usize,
    world_config: WorldConfig,
    mut config: RealisticConfig,
  ) -> Result<Self, String> {
    if config.contact.floor == crate::config::FloorConfig::default() {
      config.contact.floor = world_config.physics.floor.clone();
    }
    config.validate()?;
    Self::validate_world(&world_config)?;
    if config
      .robot_overrides
      .iter()
      .any(|r| r.id >= world_config.robots_per_team)
    {
      return Err("robot override addresses a robot outside this world".into());
    }
    if let Some(known) = &config.known_calibration.robot {
      let mut k = world_config.clone();
      k.blue_robots = known.clone();
      k.yellow_robots = known.clone();
      Self::validate_world(&k)?;
    }

    let command_airtime = config
      .transport
      .command_format
      .airtime_ns(world_config.robots_per_team)?;
    if config.transport.slots.iter().any(|s| {
      matches!(
        s.purpose,
        SlotPurpose::BlueCommand | SlotPurpose::YellowCommand
      ) && s.occupied_ns < command_airtime
    }) {
      return Err(format!(
        "{} robots need {command_airtime} ns command airtime; a reserved command slot is too short",
        world_config.robots_per_team
      ));
    }
    let mut robots = Vec::new();
    for (team, robot_config) in [
      (TeamColor::Blue, &world_config.blue_robots),
      (TeamColor::Yellow, &world_config.yellow_robots),
    ] {
      for robot_id in 0..world_config.robots_per_team {
        let ordinal = robots.len();
        let override_profile = config
          .robot_overrides
          .iter()
          .find(|r| r.team == team && r.id == robot_id);
        let motor_config = override_profile
          .and_then(|r| r.motor.clone())
          .unwrap_or_else(|| config.motor.clone());
        let mut hardware_profile = override_profile
          .and_then(|r| r.hardware.clone())
          .unwrap_or_else(|| config.hardware.clone());
        if hardware_profile.wheel == crate::config::WheelProfile::default() {
          hardware_profile.wheel = robot_config.wheel.clone().unwrap_or_default();
        }
        let profiles = override_profile
          .and_then(|r| r.devices.clone())
          .unwrap_or_else(|| config.devices.clone());
        let legacy = profiles.imus.is_empty();
        let magnetic_profile = profiles.magnetometer.clone();
        let imus = if legacy {
          super::profile::DeviceProfiles::dual_imu(&config.imu)
            .imus
            .into_iter()
            .take(1)
            .collect()
        } else {
          profiles.imus
        };
        let imu_devices = imus
          .into_iter()
          .enumerate()
          .map(|(d, mut profile)| {
            if legacy {
              profile.physical = config.imu.clone();
            }
            super::devices::DeviceState::new(
              profile,
              legacy,
              world_config.seed,
              id,
              ordinal,
              d as u64 + 1,
            )
          })
          .collect();
        robots.push(Robot {
          pending_kick: None,
          pose_estimator: super::localization::PoseEstimator::new(
            config
              .known_calibration
              .robot
              .as_ref()
              .unwrap_or(robot_config),
            config.environment.known_map.clone(),
            config.lidar.initial_pose,
          ),
          lidar_beams: Vec::new(),
          localization_rng: sensors::rng(world_config.seed, id, ordinal, 1200),
          last_radio_sync: None,
          team,
          id: robot_id,
          config: robot_config.clone(),
          hardware: DriveHardware::new(),
          imu_devices,
          receiver: Receiver::new(1, config.watchdog_ns),
          controller: Box::new(EncoderVelocityController::new(
            config
              .known_calibration
              .robot
              .clone()
              .unwrap_or_else(|| robot_config.clone()),
            config
              .known_calibration
              .motor
              .clone()
              .unwrap_or_else(|| config.motor.clone()),
            team,
            robot_id,
          )),
          observations: Observations::default(),
          previous_ticks: [0; 4],
          previous_encoder_at: None,
          previous_velocity: [0.0; 3],
          acceleration: [0.0; 2],
          requested_voltage: [None; 4],
          requested_at: 0,
          requested_command_times: None,
          power: super::power::PowerState::new(&hardware_profile.battery),
          motor_config,
          hardware_profile,
          magnetic_profile,
          magnetic_rng: sensors::rng(world_config.seed, id, ordinal, 900),
          infrared_candidate: false,
          infrared_changed_at: 0,
        });
      }
    }
    for r in &mut robots {
      r.pose_estimator.known_x16_mounting_rpy_rad = config.known_calibration.x16_mounting_rpy_rad;
    }
    let mut world = World::new(id, world_config.clone());
    config.environment.install(&mut world);
    world.physics.integration_parameters.dt = config.physics_step_ns as f32 * 1e-9;
    // Baseline chassis damping is an ideal-drive stabilizer. Hardware drag and
    // tire contact supply the resistance in this backend.
    for handle in world
      .physics
      .blue_robots
      .iter()
      .chain(&world.physics.yellow_robots)
    {
      let body = &mut world.physics.rigid_body_set[handle.chassis_body];
      body.set_linear_damping(0.0);
      body.set_angular_damping(0.0);
    }
    for robot in &robots {
      if robot.hardware_profile.dynamics == super::profile::Dynamics::Free3d
        || robot.hardware_profile.mass_kg.is_some()
        || robot.hardware_profile.center_of_mass_body_m.is_some()
        || robot.hardware_profile.inertia_diagonal_kg_m2.is_some()
      {
        let handle = if robot.team == TeamColor::Blue {
          world.physics.blue_robots[robot.id].clone()
        } else {
          world.physics.yellow_robots[robot.id].clone()
        };
        super::contact3d::install(
          &mut world.physics,
          &handle,
          &robot.config,
          &robot.hardware_profile,
          config.contact.center_of_mass_height_m,
        );
      }
    }
    let mut timeline = Timeline::default();
    for (flow, traffic) in config.network.bulk.iter().enumerate() {
      timeline.schedule(traffic.start_ns, Event::Bulk { flow, index: 0 });
    }
    timeline.schedule(config.physics_step_ns, Event::Physics);
    timeline.schedule(0, Event::PublishLatest { index: 0 });
    timeline.schedule(
      (world_config.physics.delta_time * 1e9).round() as u64,
      Event::Frame { index: 1 },
    );
    timeline.schedule(0, Event::Encoder);
    for (r, robot) in robots.iter().enumerate() {
      if config.motor_adc.enabled {
        timeline.schedule(0, Event::MotorAdc { robot: r, index: 0 });
      }
      if config.lidar.enabled {
        timeline.schedule(0, Event::LidarBeam { robot: r, index: 0 });
      }
      if config.onboard_camera.enabled {
        timeline.schedule(0, Event::OnboardCamera { robot: r, index: 0 });
      }
      if robot.magnetic_profile.enabled {
        timeline.schedule(0, Event::Magnetometer { robot: r, index: 0 });
      }
      for (d, device) in robot.imu_devices.iter().enumerate() {
        timeline.schedule(
          device.config.physical.phase_ns,
          Event::Imu {
            robot: r,
            device: d,
            index: 0,
          },
        );
      }
    }
    if config.external_vision {
      // A live/recorded boundary supplies frames through inject_vision().
    } else if config.cameras.is_empty() {
      timeline.schedule(0, Event::Vision);
    } else {
      for (i, camera) in config.cameras.iter().enumerate() {
        timeline.schedule(
          camera.phase_ns,
          Event::Camera {
            camera: i,
            index: 0,
          },
        );
      }
    }
    timeline.schedule(0, Event::Control);
    for (slot_index, slot) in config.transport.slots.iter().enumerate() {
      let team = match slot.purpose {
        SlotPurpose::BlueCommand => TeamColor::Blue,
        SlotPurpose::YellowCommand => TeamColor::Yellow,
        _ => {
          timeline.schedule(
            slot.offset_ns + u64::from(slot.phase_superframe) * config.transport.superframe_ns,
            Event::RadioAux {
              slot: slot_index,
              index: 0,
            },
          );
          continue;
        }
      };
      timeline.schedule(
        slot.offset_ns + u64::from(slot.phase_superframe) * config.transport.superframe_ns,
        Event::Transmit {
          team,
          occupied_ns: slot.occupied_ns,
        },
      );
    }
    let mut control_clock = Periodic::new(config.control_rate_hz, 0);
    control_clock.next_time();
    let mut encoder_clock = Periodic::new(config.control_rate_hz, 0);
    encoder_clock.next_time();
    let mut vision_clock = Periodic::new(config.vision.rate_hz, 0);
    vision_clock.next_time();
    Ok(Self {
      mac: super::radio::MacArbiter::default(),
      host_feedback: vec![None; robots.len()],
      host_video: vec![None; robots.len()],
      tdoa_reference: vec![None; robots.len()],
      wifi_link: config
        .network
        .wifi
        .clone()
        .map(super::network::SharedLink::new),
      lan_link: config
        .network
        .lan
        .clone()
        .map(super::network::SharedLink::new),
      network_rng: sensors::rng(world_config.seed, id, 0, 1100),
      cameras: config
        .cameras
        .iter()
        .cloned()
        .map(|c| super::vision::CameraModel::new(c, world_config.seed, id))
        .collect(),
      decision_jobs: super::processing::ProcessingResource::new(config.ai_processing.clone()),
      latest_decision: None,
      internal_publication: false,
      camera_jobs: config
        .cameras
        .iter()
        .map(|_| super::processing::ProcessingResource::new(config.camera_processing.clone()))
        .collect(),
      tracker_jobs: std::array::from_fn(|_| {
        super::processing::ProcessingResource::new(config.tracker_processing.clone())
      }),
      trackers: std::array::from_fn(|_| {
        super::vision::VisionTracker::new(config.tracker.clone()).with_cameras(&config.cameras)
      }),
      tracked: std::array::from_fn(|_| super::vision::ObservedWorld::default()),
      visible: std::array::from_fn(|_| super::vision::ObservedWorld::default()),
      world,
      robots,
      sequence: 0,
      action_sequence: 0,
      session: 1,
      pending: [None, None],
      wifi: DelayTrace::new(config.transport.wifi_delay_trace_ns.clone()),
      uwb: DelayTrace::new(config.transport.uwb_host_delay_trace_ns.clone()),
      common: DelayTrace::new(config.transport.common_delay_trace_ns.clone()),
      control_clock,
      encoder_clock,
      vision_clock,
      vision_rng: sensors::rng(world_config.seed, id, 0, 2),
      physical_time: 0,
      pending_auxiliaries: std::collections::VecDeque::new(),
      diagnostics: Diagnostics::default(),
      trace: None,
      delivered_frames: Default::default(),
      inputs: Vec::new(),
      uses_custom_controller: false,
      setup_performed: false,
      config,
      timeline,
    })
  }
  fn validate_world(config: &WorldConfig) -> Result<(), String> {
    config.validate_robot_physics()?;
    if config.robots_per_team > crate::config::MAX_ROBOTS_PER_TEAM {
      return Err("too many robots".into());
    }
    if !config.physics.delta_time.is_finite()
      || config.physics.delta_time < 1e-6
      || !config.physics.gravity.is_finite()
      || config.physics.gravity <= 0.0
    {
      return Err(
        "world frame interval must be at least 1 us; gravity must be finite and positive".into(),
      );
    }
    for robot in [&config.blue_robots, &config.yellow_robots] {
      for value in [
        robot.radius,
        robot.height,
        robot.wheel_radius,
        robot.body_mass,
        robot.kicker_mass,
        robot.vel_absolute_max,
        robot.vel_angular_max,
      ] {
        if !value.is_finite() || value <= 0.0 {
          return Err("invalid robot geometry/mass/limits".into());
        }
      }
      if robot.wheel_angles.iter().any(|v| !v.is_finite()) {
        return Err("invalid wheel angles".into());
      }
      let matrix = nalgebra::SMatrix::<f64, 4, 3>::from_fn(|row, col| {
        let angle = robot.wheel_angles[row].to_radians();
        match col {
          0 => -angle.sin(),
          1 => angle.cos(),
          _ => robot.radius,
        }
      });
      if (matrix.transpose() * matrix).determinant().abs() < 1e-10 {
        return Err("wheel geometry is singular".into());
      }
    }
    Ok(())
  }
  pub fn now(&self) -> TimeNs {
    self.timeline.now()
  }
  pub fn config(&self) -> &RealisticConfig {
    &self.config
  }
  pub fn diagnostics(&self) -> &Diagnostics {
    &self.diagnostics
  }
  pub fn robot_diagnostics(&self) -> Vec<RobotDiagnostics> {
    self
      .robots
      .iter()
      .map(|r| {
        let handle = if r.team == TeamColor::Blue {
          &self.world.physics.blue_robots[r.id]
        } else {
          &self.world.physics.yellow_robots[r.id]
        };
        let body = &self.world.physics.rigid_body_set[handle.chassis_body];
        let q = body.rotation();
        let w = body.angvel();
        RobotDiagnostics {
          team: r.team,
          id: r.id,
          motors: r.hardware.motors.clone(),
          contacts: r.hardware.contacts.clone(),
          power: r.power.clone(),
          attitude_xyzw: [q.x as f64, q.y as f64, q.z as f64, q.w as f64],
          angular_velocity_rad_s: [w.x as f64, w.y as f64, w.z as f64],
        }
      })
      .collect()
  }
  pub fn truth(&self) -> WorldState {
    self.world.get_state()
  }
  /// Evaluation/image-backend boundary, never passed to onboard controllers.
  pub fn render_camera(
    &self,
    camera_id: u32,
    settings: &super::render::RenderConfig,
  ) -> Result<super::render::Image, String> {
    let camera = self
      .cameras
      .iter()
      .find(|c| c.config.id == camera_id)
      .ok_or("unknown camera ID")?;
    super::render::render(
      &self.world,
      &camera.config,
      &self.config.environment,
      settings,
      self.world.config.seed,
    )
  }
  pub fn observations(&self, team: TeamColor, id: usize) -> Option<&Observations> {
    self
      .robots
      .iter()
      .find(|r| r.team == team && r.id == id)
      .map(|r| &r.observations)
  }
  pub fn team_observations(&self, team: TeamColor) -> &super::vision::ObservedWorld {
    &self.visible[team as usize]
  }
  pub fn reset_localization(
    &mut self,
    team: TeamColor,
    id: usize,
    initial: Option<[f64; 3]>,
  ) -> Result<(), String> {
    let now = self.now();
    let robot = self
      .robots
      .iter_mut()
      .find(|r| r.team == team && r.id == id)
      .ok_or("unknown robot")?;
    robot.pose_estimator.reset(now, initial)?;
    robot.observations.radio.clear();
    robot.observations.lidar = None;
    robot.observations.pose = Some(robot.pose_estimator.estimate.clone());
    self.record_input(RecordedInput::LocalizationReset { team, id, initial });
    Ok(())
  }
  pub fn host_feedback(
    &self,
    team: TeamColor,
    id: usize,
  ) -> Option<(
    &sensors::RobotTelemetry,
    Option<&sensors::InfraredSample>,
    Option<u64>,
  )> {
    let index = self
      .robots
      .iter()
      .position(|r| r.team == team && r.id == id)?;
    self.host_feedback[index]
      .as_ref()
      .map(|(t, i, a)| (t, i.as_ref(), *a))
  }
  /// Decoded stream timing metadata; pixels/codecs are supplied by a live image backend.
  pub fn host_video(
    &self,
    team: TeamColor,
    id: usize,
  ) -> Option<&super::localization::StreamedFrame> {
    let index = self
      .robots
      .iter()
      .position(|r| r.team == team && r.id == id)?;
    self.host_video[index].as_ref()
  }
  pub fn delivered_camera_frames(&self) -> Vec<VisionFrame> {
    self.delivered_frames.values().cloned().collect()
  }
  pub fn inject_vision(&mut self, frame: VisionFrame) -> Result<(), String> {
    if !self.config.external_vision {
      return Err("external vision must be explicitly enabled".into());
    }
    if frame.sent_at > self.now()
      || frame.captured_at > frame.sent_at
      || frame.robots.iter().any(|r| {
        r.position_m
          .iter()
          .chain(std::iter::once(&r.orientation_rad))
          .any(|v| !v.is_finite())
      })
      || frame
        .ball_position_m
        .is_some_and(|p| p.iter().any(|v| !v.is_finite()))
    {
      return Err("invalid external vision coordinates/times".into());
    }
    self.record_input(RecordedInput::VisionIngress {
      frame: frame.clone(),
    });
    self
      .timeline
      .schedule(self.now(), Event::VisionDelivery(frame));
    Ok(())
  }
  pub fn official_observations(&self) -> &super::vision::ObservedWorld {
    &self.visible[2]
  }
  pub fn set_controller(
    &mut self,
    team: TeamColor,
    id: usize,
    controller: Box<dyn RobotController>,
  ) -> Result<(), String> {
    let robot = self
      .robots
      .iter_mut()
      .find(|r| r.team == team && r.id == id)
      .ok_or("unknown robot")?;
    robot.controller = controller;
    self.uses_custom_controller = true;
    Ok(())
  }
  /// Setup/repositioning is explicitly separate from radio command publication.
  /// Wheel angle/current and receiver state are retained, as for physical repositioning.
  pub fn teleport(&mut self, commands: &WorldCommand) {
    self.setup_performed = true;
    if self.trace.is_some() {
      self.record_input(RecordedInput::Teleport {
        commands: commands.clone(),
      });
    }
    if let Some(ball) = &commands.teleport_ball {
      self.world.teleport_ball(ball);
    }
    for robot in &commands.teleport_robots {
      self.world.teleport_robot(robot);
    }
  }
  pub fn request_driver_reset(&mut self, team: TeamColor, id: usize) -> Result<(), String> {
    let now = self.now();
    let robot = self
      .robots
      .iter_mut()
      .find(|r| r.team == team && r.id == id)
      .ok_or("unknown robot")?;
    for motor in &mut robot.hardware.motors {
      motor.request_reset(now, &robot.motor_config);
    }
    self.record_input(RecordedInput::DriverReset { team, id });
    Ok(())
  }
  pub fn set_attitude(
    &mut self,
    team: TeamColor,
    id: usize,
    rpy_rad: [f64; 3],
    height_m: Option<f64>,
  ) -> Result<(), String> {
    if id >= self.world.config.robots_per_team
      || rpy_rad.iter().any(|v| !v.is_finite())
      || height_m.is_some_and(|v| !v.is_finite() || v < 0.0)
    {
      return Err("invalid attitude setup".into());
    }
    self.setup_performed = true;
    self.record_input(RecordedInput::Attitude {
      team,
      id,
      rpy_rad,
      height_m,
    });
    let handle = if team == TeamColor::Blue {
      &self.world.physics.blue_robots[id]
    } else {
      &self.world.physics.yellow_robots[id]
    };
    let body = &mut self.world.physics.rigid_body_set[handle.chassis_body];
    let q = nalgebra::UnitQuaternion::from_euler_angles(rpy_rad[0], rpy_rad[1], rpy_rad[2]);
    body.set_rotation(
      rapier3d::prelude::Rotation::from_xyzw(q.i as f32, q.j as f32, q.k as f32, q.w as f32),
      true,
    );
    if let Some(height) = height_m {
      let mut position = body.translation();
      position.z = height as f32;
      body.set_translation(position, true);
    }
    Ok(())
  }
  pub fn enable_recording(&mut self) {
    assert!(
      self.now() == 0
        && self.sequence == 0
        && self.diagnostics.control_ticks == 0
        && !self.setup_performed,
      "recording must start before advancing or publishing"
    );
    self.trace.get_or_insert_with(Vec::new);
  }
  pub fn recording(&self, build_identity: String) -> Recording {
    Recording {
      schema_version: 1,
      build_identity,
      resolved_profile_sha256: super::calibration::digest(&(&self.config, &self.world.config))
        .expect("validated finite profile"),
      world_id: self.world.id,
      config: self.config.clone(),
      world_config: self.world.config.clone(),
      diagnostics: self.diagnostics.clone(),
      events: self.trace.clone().unwrap_or_default(),
      inputs: self.inputs.clone(),
      uses_custom_controller: self.uses_custom_controller,
      final_truth: self.truth(),
      final_hardware: self.robot_diagnostics(),
    }
  }
  fn trace(&mut self, event: TraceEvent) {
    if let Some(trace) = &mut self.trace {
      trace.push(event);
    }
  }
  fn record_input(&mut self, input: RecordedInput) {
    if self.trace.is_some() {
      self.inputs.push(input);
    }
  }
  /// Each ordinary publication creates a fresh logical action. Repeated kicks
  /// should use publish_with_actions with a stable action identity until ACK.
  pub fn publish(
    &mut self,
    commands: WorldCommand,
    decision_at: TimeNs,
    safety_epoch: u64,
    safety: Safety,
  ) -> Result<u64, String> {
    self.publish_with_actions(commands, decision_at, safety_epoch, safety, Vec::new())
  }
  pub fn publish_with_actions(
    &mut self,
    commands: WorldCommand,
    decision_at: TimeNs,
    safety_epoch: u64,
    safety: Safety,
    mut actions: Vec<ActionIdentity>,
  ) -> Result<u64, String> {
    let mut seen = std::collections::HashSet::new();
    for a in &actions {
      if !seen.insert((a.team as u8, a.robot_id))
        || !(if a.team == TeamColor::Blue {
          &commands.blue
        } else {
          &commands.yellow
        })
        .iter()
        .any(|c| c.id == a.robot_id && c.kick_speed > 0.0)
      {
        return Err("action IDs must uniquely identify an included kick".into());
      }
    }
    self.validate_publication(&commands, decision_at)?;
    let now = self.now();
    if actions.is_empty() {
      actions = self.fresh_actions(&commands)?;
    }
    for a in &actions {
      self.action_sequence = self.action_sequence.max(a.action_id);
    }
    if self.trace.is_some() && !self.internal_publication {
      self.record_input(RecordedInput::PublishActions {
        actions: actions.clone(),
        commands: commands.clone(),
        decision_at,
        safety_epoch,
        safety,
      });
    }
    self.sequence = self
      .sequence
      .checked_add(1)
      .ok_or("sender sequence exhausted; provision a new session")?;
    self.trace(TraceEvent::Publication {
      at: now,
      sequence: self.sequence,
      decision_at,
    });
    let common_delay = self.joint_delay(self.sequence).map_or_else(
      || self.common.sample(),
      |d| u64::try_from(d.common_delay_ns).ok(),
    );
    if let Some(delay) = common_delay {
      self.timeline.schedule(
        now.checked_add(delay).ok_or("publication time overflow")?,
        Event::Ready(Publication {
          sequence: self.sequence,
          decision_at,
          published_at: now,
          safety_epoch,
          safety,
          commands,
          actions,
        }),
      );
    } else {
      self.diagnostics.dropped += 1;
    }
    Ok(self.sequence)
  }
  fn validate_publication(
    &self,
    commands: &WorldCommand,
    decision_at: TimeNs,
  ) -> Result<(), String> {
    let now = self.now();
    if decision_at > now {
      return Err("decision timestamp is in the future".into());
    }
    if commands.teleport_ball.is_some() || !commands.teleport_robots.is_empty() {
      return Err("teleports must use the setup API".into());
    }
    for team in [&commands.blue, &commands.yellow] {
      let mut ids = std::collections::HashSet::new();
      for command in team {
        if command.id >= self.world.config.robots_per_team || !ids.insert(command.id) {
          return Err("unknown or duplicate robot ID".into());
        }
        let mut values = vec![command.kick_speed, command.kick_angle];
        if let Some(movement) = &command.move_command {
          match movement {
            crate::MoveCommand::LocalVelocity {
              forward,
              left,
              angular,
            } => values.extend([forward, left, angular]),
            crate::MoveCommand::GlobalVelocity { vx, vy, angular } => {
              values.extend([vx, vy, angular])
            }
            crate::MoveCommand::WheelVelocity(speeds) => values.extend(speeds),
            crate::MoveCommand::Onboard(bytes) => {
              if bytes.len() > 65536 {
                return Err("onboard instruction exceeds 64 KiB".into());
              }
            }
          }
        }
        if values.iter().any(|v| !v.is_finite()) {
          return Err("commands must be finite".into());
        }
      }
    }
    if self.timeline.len() >= self.config.max_pending_events {
      return Err("event queue capacity reached".into());
    }
    Ok(())
  }
  /// The caller computes from the observation snapshot available at decision_at.
  /// Its owned output is published only after the modeled AI job completes.
  pub fn submit_decision(
    &mut self,
    commands: WorldCommand,
    decision_at: TimeNs,
    safety_epoch: u64,
    safety: Safety,
  ) -> Result<(), String> {
    self.validate_publication(&commands, decision_at)?;
    self.record_input(RecordedInput::Decision {
      commands: commands.clone(),
      decision_at,
      safety_epoch,
      safety,
    });
    let actions = self.fresh_actions(&commands)?;
    self.decision_jobs.submit(
      self.now(),
      self.config.ai_processing_ns,
      Decision {
        commands,
        decision_at,
        safety_epoch,
        safety,
        actions,
      },
    );
    self.serve_decision(self.now());
    Ok(())
  }
  fn fresh_actions(&mut self, commands: &WorldCommand) -> Result<Vec<ActionIdentity>, String> {
    let mut actions = Vec::new();
    for (team, commands) in [
      (TeamColor::Blue, &commands.blue),
      (TeamColor::Yellow, &commands.yellow),
    ] {
      for c in commands {
        if c.kick_speed > 0.0 {
          self.action_sequence = self
            .action_sequence
            .checked_add(1)
            .ok_or("action counter exhausted")?;
          actions.push(ActionIdentity {
            team,
            robot_id: c.id,
            action_id: self.action_sequence,
          });
        }
      }
    }
    Ok(actions)
  }
  fn serve_decision(&mut self, at: TimeNs) {
    if let Some(job) = self.decision_jobs.start(at, &mut self.network_rng) {
      self
        .timeline
        .schedule(job.completes_at, Event::DecisionComplete(job.input));
    }
  }
  /// Logical time continues during total sensor/radio loss; no wall-clock sleep.
  pub fn advance_to(&mut self, end: TimeNs) -> Result<(), String> {
    if end < self.now() {
      return Err("cannot rewind simulation time".into());
    }
    self.record_input(RecordedInput::Advance { end });
    while let Some((at, event)) = self.timeline.pop_until(end) {
      self.handle(at, event);
      self.diagnostics.radio = self.mac.diagnostics.clone();
      if self.timeline.len() > self.config.max_pending_events {
        return Err("event queue capacity exceeded".into());
      }
    }
    self.visible =
      std::array::from_fn(|i| self.tracked[i].at_time(self.now(), self.config.tracker.max_age_ns));
    self.world.sim_time = self.physical_time as f64 * 1e-9;
    Ok(())
  }
  /// Frame adapter, with the subframe timeline retained across calls.
  pub fn step(&mut self, commands: WorldCommand) -> Result<WorldState, String> {
    self.publish(commands, self.now(), 1, Safety::Run)?;
    let end =
      ((self.world.frame + 1) as f64 * self.world.config.physics.delta_time * 1e9).round() as u64;
    self.advance_to(end)?;
    Ok(self.truth())
  }
  fn handle(&mut self, at: TimeNs, event: Event) {
    if self.trace.is_some() && self.config.record_raw_packets {
      if let Event::Delivery {
        team,
        radio,
        packets,
        ..
      } = &event
      {
        self.trace(TraceEvent::PacketCopies {
          at,
          team: *team,
          radio: *radio,
          packets: packets.clone(),
        });
      }
    }
    if self.trace.is_some() && self.config.record_raw_sensors {
      let delivery = match &event {
        Event::MotorAdcDelivery { robot, sample } => Some((
          *robot,
          Observations {
            motor_adc: Some(sample.clone()),
            ..Default::default()
          },
        )),
        Event::ImuDelivery { robot, samples } => Some((
          *robot,
          Observations {
            imu: samples.clone(),
            ..Default::default()
          },
        )),
        Event::EncoderDelivery { robot, sample } => Some((
          *robot,
          Observations {
            encoder: Some(sample.clone()),
            ..Default::default()
          },
        )),
        Event::MagneticDelivery { robot, sample } => Some((
          *robot,
          Observations {
            magnetic: Some(sample.clone()),
            ..Default::default()
          },
        )),
        Event::LidarDelivery { robot, scan } => Some((
          *robot,
          Observations {
            lidar: Some(scan.clone()),
            ..Default::default()
          },
        )),
        Event::OnboardDelivery { robot, ball } => Some((
          *robot,
          Observations {
            onboard_ball: ball.clone(),
            ..Default::default()
          },
        )),
        Event::TelemetryDelivery {
          robot,
          telemetry,
          infrared,
        } => Some((
          *robot,
          Observations {
            telemetry: Some(telemetry.clone()),
            infrared: Some(infrared.clone()),
            ..Default::default()
          },
        )),
        _ => None,
      };
      if let Some((i, mut delivered)) = delivery {
        delivered.delivered_at = at;
        let r = &self.robots[i];
        self.trace(TraceEvent::Sensors {
          at,
          team: r.team,
          id: r.id,
          delivered: Box::new(delivered),
        });
      }
      if let Event::VisionDelivery(frame) = &event {
        self.trace(TraceEvent::CameraInput {
          at,
          frame: frame.clone(),
        });
      }
    }
    match event {
      Event::MotorAdc { robot, index } => {
        let cfg = &self.config.motor_adc;
        let r = &mut self.robots[robot];
        let mut clipped = false;
        let currents = std::array::from_fn(|i| {
          let value = r.hardware.motors[i].current_a
            + sensors::normal(&mut r.localization_rng) * cfg.current_noise_std_a;
          clipped |= value.abs() > cfg.current_range_a;
          let value = value.clamp(-cfg.current_range_a, cfg.current_range_a);
          if cfg.current_resolution_a > 0.0 {
            (value / cfg.current_resolution_a).round() * cfg.current_resolution_a
          } else {
            value
          }
        });
        let voltage = if cfg.voltage_resolution_v > 0.0 {
          (r.power.voltage_v / cfg.voltage_resolution_v).round() * cfg.voltage_resolution_v
        } else {
          r.power.voltage_v
        };
        self.timeline.schedule(
          at + cfg.delivery_ns,
          Event::MotorAdcDelivery {
            robot,
            sample: sensors::MotorAdcSample {
              sampled_at: at,
              phase_current_a: currents,
              bus_voltage_v: voltage,
              clipped,
            },
          },
        );
        self.timeline.schedule(
          ((index + 1) as u128 * 1_000_000_000 / u128::from(cfg.rate_hz)) as u64,
          Event::MotorAdc {
            robot,
            index: index + 1,
          },
        );
        self.diagnostics.max_sensor_quantization_ns = self
          .diagnostics
          .max_sensor_quantization_ns
          .max(at - self.physical_time);
      }
      Event::MotorAdcDelivery { robot, sample } => {
        self.robots[robot].observations.motor_adc = Some(sample);
        self.robots[robot].observations.delivered_at = at;
      }
      Event::DecisionComplete(decision) => {
        self.decision_jobs.complete();
        self.latest_decision = Some(decision);
        self.serve_decision(at);
      }
      Event::PublishLatest { index } => {
        if let Some(decision) = self.latest_decision.clone() {
          self.internal_publication = true;
          if self
            .publish_with_actions(
              decision.commands,
              decision.decision_at,
              decision.safety_epoch,
              decision.safety,
              decision.actions,
            )
            .is_err()
          {
            self.diagnostics.dropped += 1;
          }
          self.internal_publication = false;
        }
        self.timeline.schedule(
          ((index + 1) as u128 * 1_000_000_000 / u128::from(self.config.publication_rate_hz))
            as u64,
          Event::PublishLatest { index: index + 1 },
        );
      }
      Event::RadioAux { slot, index } => self.radio_aux(at, slot, index),
      Event::SynchronizationDelivery { ticket } => {
        if self.mac.finish(ticket) {
          for r in &mut self.robots {
            r.last_radio_sync = Some(at);
          }
        } else {
          self.mac.diagnostics.missed_sync += 1;
        }
      }
      Event::UwbStart {
        team,
        packets,
        occupied_ns,
      } => {
        let ticket = self.mac.reserve(at, occupied_ns);
        self.mac.frame(
          self
            .config
            .transport
            .command_format
            .airtime_ns(packets.len())
            .expect("validated command format"),
        );
        if self.network_rng.random::<f64>() < self.config.uwb.loss_probability {
          self.mac.finish(ticket);
          self.diagnostics.dropped += 1;
          return;
        }
        let joint = packets.first().and_then(|p| self.joint_delay(p.sequence));
        let delay = joint.map_or_else(
          || self.uwb.sample(),
          |d| u64::try_from(d.uwb_post_airtime_ns).ok(),
        );
        if let Some(delay) = delay {
          self.timeline.schedule(
            at + occupied_ns + delay,
            Event::Delivery {
              ticket: Some(ticket),
              team,
              packets,
              radio: Radio::Uwb,
            },
          );
        } else {
          self.mac.finish(ticket);
          self.diagnostics.dropped += 1;
        }
      }
      Event::UwbResponse(job) => self.uwb_response(at, job),
      Event::UwbFinal(job) => self.uwb_final(at, job),
      Event::UwbRangingDelivery(job) => {
        if self.mac.finish(job.ticket) {
          self.radio_observation(at, job.robot, job.observation);
          self.mac.diagnostics.ranging_delivered += 1;
        } else {
          self.mac.diagnostics.responses_missed += 1;
        }
        self.diagnostics.radio = self.mac.diagnostics.clone();
      }
      Event::TdoaDelivery {
        ticket,
        observations,
      } => {
        if self.mac.finish(ticket) {
          for (robot, observation) in observations {
            self.radio_observation(at, robot, observation);
            self.mac.diagnostics.ranging_delivered += 1;
          }
        }
        self.diagnostics.radio = self.mac.diagnostics.clone();
      }
      Event::UwbTelemetryDelivery {
        robot,
        ticket,
        telemetry,
        infrared,
        action,
      } => {
        if self.mac.finish(ticket)
          && let Some(telemetry) = telemetry
        {
          self.host_feedback[robot] = Some((telemetry, infrared, action));
          self.mac.diagnostics.telemetry_delivered += 1;
        } else {
          self.mac.diagnostics.responses_missed += 1;
        }
        self.diagnostics.radio = self.mac.diagnostics.clone();
      }
      Event::LidarBeam { robot, index } => self.lidar_beam(at, robot, index),
      Event::LidarDelivery { robot, scan } => {
        self.robots[robot]
          .pose_estimator
          .lidar(&scan, &self.config.lidar);
        let r = &mut self.robots[robot];
        r.observations.lidar = Some(scan);
        r.observations.pose = Some(r.pose_estimator.estimate.clone());
        r.observations.delivered_at = at;
      }
      Event::OnboardCamera { robot, index } => self.onboard_camera(at, robot, index),
      Event::OnboardDelivery { robot, ball } => {
        self.robots[robot].observations.onboard_ball = ball;
        self.robots[robot].observations.delivered_at = at;
      }
      Event::VideoDecoded { robot, frame } => {
        self.host_video[robot] = Some(frame);
      }
      Event::VideoEncoded { robot, captured_at } => {
        if let Some(link) = &mut self.wifi_link {
          link.enqueue(super::network::QueuedPacket {
            flow: robot as u64 + 1000,
            bytes: self.config.onboard_camera.stream_bytes_per_frame,
            class: super::network::TrafficClass::Bulk,
            payload: WifiPayload::Video {
              robot,
              captured_at,
              encoded_at: at,
            },
          });
        }
        self.serve_wifi(at);
      }
      Event::WifiRelease => {
        if let Some(link) = &mut self.wifi_link {
          link.complete();
          self.diagnostics.wifi_link = link.diagnostics.clone();
        }
        self.serve_wifi(at);
      }
      Event::LanRelease => {
        if let Some(link) = &mut self.lan_link {
          link.complete();
          self.diagnostics.lan_link = link.diagnostics.clone();
        }
        self.serve_lan(at);
      }
      Event::LanEnqueue(frame) => {
        if let Some(link) = &mut self.lan_link {
          link.enqueue(super::network::QueuedPacket {
            flow: u64::from(frame.camera_id),
            bytes: 64 + frame.robots.len() * 32 + usize::from(frame.ball_position_m.is_some()) * 32,
            class: super::network::TrafficClass::Telemetry,
            payload: frame,
          });
        }
        self.serve_lan(at);
      }
      Event::Bulk { flow, index } => {
        let traffic = &self.config.network.bulk[flow];
        if at < traffic.end_ns {
          if let Some(link) = &mut self.wifi_link {
            link.enqueue(super::network::QueuedPacket {
              flow: traffic.flow,
              bytes: traffic.bytes_per_burst,
              class: super::network::TrafficClass::Bulk,
              payload: WifiPayload::Bulk,
            });
          }
          let next = traffic.start_ns
            + ((index + 1) as u128 * 1_000_000_000 / u128::from(traffic.rate_hz)) as u64;
          self.timeline.schedule(
            next,
            Event::Bulk {
              flow,
              index: index + 1,
            },
          );
          self.serve_wifi(at);
        }
      }
      Event::Physics => {
        self.physics(at);
        self
          .timeline
          .schedule(at + self.config.physics_step_ns, Event::Physics);
      }
      Event::Frame { index } => {
        self.world.frame = index;
        self.world.realistic_frame_end();
        let next = ((index + 1) as f64 * self.world.config.physics.delta_time * 1e9).round() as u64;
        self
          .timeline
          .schedule(next, Event::Frame { index: index + 1 });
      }
      Event::Control => {
        self.control(at);
        self
          .timeline
          .schedule(self.control_clock.next_time(), Event::Control);
      }
      Event::Encoder => {
        self.encoders(at);
        self
          .timeline
          .schedule(self.encoder_clock.next_time(), Event::Encoder);
      }
      Event::Imu {
        robot,
        device,
        index,
      } => {
        self.sample_imu(at, robot, device, index);
        self.timeline.schedule(
          sensors::imu_time(
            &self.robots[robot].imu_devices[device].config.physical,
            index + 1,
          ),
          Event::Imu {
            robot,
            device,
            index: index + 1,
          },
        );
      }
      Event::ImuDelivery { robot, samples } => {
        self.robots[robot].pose_estimator.imu(&samples);
        let o = &mut self.robots[robot].observations;
        o.delivered_at = at;
        o.imu.extend(samples);
        if o.imu.len() > 128 {
          o.imu.drain(..o.imu.len() - 128);
        }
      }
      Event::EncoderDelivery { robot, sample } => {
        self.robots[robot].pose_estimator.encoder(&sample);
        self.robots[robot].observations.pose =
          Some(self.robots[robot].pose_estimator.estimate.clone());
        let o = &mut self.robots[robot].observations;
        o.delivered_at = at;
        o.encoder = Some(sample);
      }
      Event::Vision => {
        self.vision(at);
        self
          .timeline
          .schedule(self.vision_clock.next_time(), Event::Vision);
      }
      Event::Camera { camera, index } => {
        let frame = self.cameras[camera].capture_physical(
          at,
          self.physical_time,
          &self.world,
          &self.config.environment,
        );
        let c = &self.cameras[camera].config;
        self.timeline.schedule(
          frame.image_received_at,
          Event::CameraAcquired { camera, frame },
        );
        let next = if let Some(prior) = self
          .config
          .camera_timing_priors
          .iter()
          .find(|p| p.camera_id == c.id)
        {
          at + prior.samples[(index as usize + 1) % prior.samples.len()]
            .interval_ns
            .unwrap_or(1_000_000_000 / u64::from(c.rate_hz))
        } else {
          c.phase_ns + ((index + 1) as u128 * 1_000_000_000 / u128::from(c.rate_hz)) as u64
        };
        self.timeline.schedule(
          next,
          Event::Camera {
            camera,
            index: index + 1,
          },
        );
      }
      Event::CameraAcquired { camera, frame } => {
        if let Some(prior) = self
          .config
          .camera_timing_priors
          .iter()
          .find(|p| p.camera_id == frame.camera_id)
        {
          let delay = prior.samples[frame.frame_number as usize % prior.samples.len()]
            .image_receive_to_send_ns;
          self
            .timeline
            .schedule(at + delay, Event::EmpiricalCameraComplete { camera, frame });
          return;
        }
        self.camera_jobs[camera].submit(at, self.cameras[camera].config.processing_ns, frame);
        self.serve_camera(at, camera);
      }
      Event::EmpiricalCameraComplete { camera, mut frame } => {
        frame.sent_at = at;
        if self.lan_link.is_some() {
          self.handle(at, Event::LanEnqueue(frame));
        } else {
          self.timeline.schedule(
            at + self.cameras[camera].config.network_delay_ns,
            Event::VisionDelivery(frame),
          );
        }
      }
      Event::CameraComplete { camera, mut frame } => {
        self.camera_jobs[camera].complete();
        frame.sent_at = at;
        if self.lan_link.is_some() {
          self.handle(at, Event::LanEnqueue(frame));
        } else {
          self.timeline.schedule(
            at + self.cameras[camera].config.network_delay_ns,
            Event::VisionDelivery(frame),
          );
        }
        self.serve_camera(at, camera);
      }
      Event::TrackerComplete { audience, world } => {
        self.tracker_jobs[audience].complete();
        if world.at >= self.tracked[audience].at {
          self.tracked[audience] = world;
        }
        self.serve_tracker(at, audience);
      }
      Event::VisionDelivery(frame) => {
        self.delivered_frames.insert(frame.camera_id, frame.clone());
        let config = self
          .cameras
          .iter()
          .find(|c| c.config.id == frame.camera_id)
          .map(|c| &c.config);
        let allowed = config.map_or([true; 3], |c| {
          [c.blue_enabled, c.yellow_enabled, c.officials_enabled]
        });
        for (audience, allowed) in allowed.into_iter().enumerate() {
          if allowed {
            self.tracker_jobs[audience].submit(
              at,
              self.config.tracker.processing_ns,
              frame.clone(),
            );
            self.serve_tracker(at, audience);
          }
        }
        for robot in &mut self.robots {
          if !allowed[robot.team as usize] {
            continue;
          }
          robot.observations.delivered_at = at;
          robot.observations.vision = Some(frame.clone());
        }
      }
      Event::Magnetometer { robot, index } => {
        let r = &mut self.robots[robot];
        let handle = if r.team == TeamColor::Blue {
          &self.world.physics.blue_robots[r.id]
        } else {
          &self.world.physics.yellow_robots[r.id]
        };
        let body = &self.world.physics.rigid_body_set[handle.chassis_body];
        let q = body.rotation();
        let orientation = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
          q.w as f64, q.x as f64, q.y as f64, q.z as f64,
        ));
        let sample = super::magnetic::sample(
          &r.magnetic_profile,
          at,
          orientation,
          {
            let p = body.translation();
            [p.x as f64, p.y as f64, p.z as f64]
          },
          &r.hardware.motors,
          r.power.dribbler_fraction,
          at < r.power.kicker_ready_at,
          &mut r.magnetic_rng,
        );
        self.timeline.schedule(
          at + r.magnetic_profile.delay_ns,
          Event::MagneticDelivery { robot, sample },
        );
        let next =
          ((index + 1) as u128 * 1_000_000_000 / u128::from(r.magnetic_profile.rate_hz)) as u64;
        self.timeline.schedule(
          next,
          Event::Magnetometer {
            robot,
            index: index + 1,
          },
        );
      }
      Event::MagneticDelivery { robot, sample } => {
        self.robots[robot].observations.magnetic = Some(sample);
      }
      Event::TelemetryDelivery {
        robot,
        telemetry,
        infrared,
      } => {
        let o = &mut self.robots[robot].observations;
        o.delivered_at = at;
        o.telemetry = Some(telemetry);
        o.infrared = Some(infrared);
      }
      Event::Ready(publication) => self.ready(at, publication),
      Event::Transmit { team, occupied_ns } => {
        if let Some(publication) = self.pending[team as usize].take() {
          if self.config.uwb.enabled {
            let jitter = (self.network_rng.random::<f64>() * self.config.uwb.slot_jitter_ns as f64)
              .round() as u64;
            self.timeline.schedule(
              at + self.config.uwb.tx_firmware_delay_ns + jitter,
              Event::UwbStart {
                team,
                packets: self.packets(&publication, team),
                occupied_ns,
              },
            );
          } else {
            let joint = self.joint_delay(publication.sequence);
            let delay = joint.map_or_else(
              || self.uwb.sample(),
              |d| u64::try_from(d.uwb_post_airtime_ns).ok(),
            );
            if let Some(delay) = delay {
              let packets = self.packets(&publication, team);
              self.timeline.schedule(
                at + occupied_ns + delay,
                Event::Delivery {
                  ticket: None,
                  team,
                  packets,
                  radio: Radio::Uwb,
                },
              );
            } else {
              self.diagnostics.dropped += 1;
            }
          }
        } else if self.config.uwb.enabled {
          self.timeline.schedule(
            at + self.config.uwb.tx_firmware_delay_ns,
            Event::UwbStart {
              team,
              packets: Vec::new(),
              occupied_ns,
            },
          );
        }
        let purpose = if team == TeamColor::Blue {
          SlotPurpose::BlueCommand
        } else {
          SlotPurpose::YellowCommand
        };
        let period = self
          .config
          .transport
          .slots
          .iter()
          .find(|s| s.purpose == purpose)
          .map_or(1, |s| s.every_superframes);
        self.timeline.schedule(
          at + self.config.transport.superframe_ns * u64::from(period),
          Event::Transmit { team, occupied_ns },
        );
      }
      Event::Delivery {
        ticket,
        team,
        packets,
        radio,
      } => {
        if let Some(ticket) = ticket {
          if !self.mac.finish(ticket) {
            self.diagnostics.dropped += 1;
            return;
          }
        }
        if radio == Radio::Uwb {
          let frame = at / self.config.transport.superframe_ns;
          if !self
            .config
            .uwb
            .synchronization_loss_every
            .is_some_and(|n| n > 0 && frame % n == 0)
          {
            for r in self.robots.iter_mut().filter(|r| r.team == team) {
              r.last_radio_sync = Some(at);
            }
          }
        }
        for packet in packets {
          let id = packet.command.id;
          let sequence = packet.sequence;
          let index = team as usize * self.world.config.robots_per_team + id;
          let synchronized = self.robots[index]
            .last_radio_sync
            .is_some_and(|t| at.saturating_sub(t) < self.config.uwb.synchronization_timeout_ns);
          let clock = (!self.config.uwb.require_sync_for_commands || synchronized).then_some(
            super::transport::SenderClockEstimate {
              now_ns: at,
              uncertainty_ns: self.config.uwb.command_clock_uncertainty_ns,
            },
          );
          let rejection = self.robots[index]
            .receiver
            .accept_with_clock(packet, at, clock)
            .err();
          if rejection.is_some() {
            self.diagnostics.rejected += 1;
          } else {
            self.diagnostics.accepted += 1;
          }
          self.trace(TraceEvent::Acceptance {
            at,
            team,
            id,
            sequence,
            radio,
            rejection,
          });
        }
      }
    }
  }
  fn serve_camera(&mut self, at: TimeNs, camera: usize) {
    if let Some(job) = self.camera_jobs[camera].start(at, &mut self.network_rng) {
      self.timeline.schedule(
        job.completes_at,
        Event::CameraComplete {
          camera,
          frame: job.input,
        },
      );
    }
  }
  fn serve_tracker(&mut self, at: TimeNs, audience: usize) {
    if let Some(job) = self.tracker_jobs[audience].start(at, &mut self.network_rng) {
      self.trackers[audience].ingest(&job.input);
      let world = self.trackers[audience].snapshot(job.started_at);
      self
        .timeline
        .schedule(job.completes_at, Event::TrackerComplete { audience, world });
    }
  }
  pub fn processing_diagnostics(
    &self,
  ) -> (
    Vec<super::processing::ProcessingDiagnostics>,
    Vec<super::processing::ProcessingDiagnostics>,
  ) {
    (
      self
        .camera_jobs
        .iter()
        .map(|p| p.diagnostics.clone())
        .collect(),
      self
        .tracker_jobs
        .iter()
        .map(|p| p.diagnostics.clone())
        .collect(),
    )
  }
  fn joint_delay(&self, sequence: u64) -> Option<super::config::JointDelay> {
    let trace = &self.config.transport.joint_delay_trace;
    (!trace.is_empty()).then(|| trace[sequence.wrapping_sub(1) as usize % trace.len()].clone())
  }
  fn packets(&self, publication: &Publication, team: TeamColor) -> Vec<CommandPacket> {
    let commands = if team == TeamColor::Blue {
      &publication.commands.blue
    } else {
      &publication.commands.yellow
    };
    commands
      .iter()
      .map(|command| CommandPacket {
        session: self.session,
        sequence: publication.sequence,
        decision_at: publication.decision_at,
        published_at: publication.published_at,
        expires_at: publication
          .published_at
          .saturating_add(self.config.command_ttl_ns),
        safety_epoch: publication.safety_epoch,
        safety: publication.safety,
        action_id: (command.kick_speed > 0.0).then(|| {
          publication
            .actions
            .iter()
            .find(|a| a.team == team && a.robot_id == command.id)
            .map_or(publication.sequence, |a| a.action_id)
        }),
        command: command.clone(),
      })
      .collect()
  }
  fn ready(&mut self, at: TimeNs, publication: Publication) {
    for team in [TeamColor::Blue, TeamColor::Yellow] {
      let commands = if team == TeamColor::Blue {
        &publication.commands.blue
      } else {
        &publication.commands.yellow
      };
      if commands.is_empty() {
        continue;
      }
      if publication.sequence % self.config.network.wifi_copy_every_publications.max(1) != 0 {
        // Explicit lower-rate copy experiment; UWB still receives the publication.
      } else if self.wifi_link.is_some() {
        let packets = self.packets(&publication, team);
        let link = self.wifi_link.as_mut().unwrap();
        for packet in packets {
          link.enqueue(super::network::QueuedPacket {
            flow: team as u64 * 16 + packet.command.id as u64,
            bytes: self.config.transport.command_format.header_bytes as usize
              + self.config.transport.command_format.bytes_per_robot as usize,
            class: super::network::TrafficClass::Command,
            payload: WifiPayload::Command {
              team,
              packets: vec![packet],
            },
          });
        }
      } else if let Some(delay) = self.joint_delay(publication.sequence).map_or_else(
        || self.wifi.sample(),
        |d| u64::try_from(d.wifi_post_split_ns).ok(),
      ) {
        self.timeline.schedule(
          at + delay,
          Event::Delivery {
            ticket: None,
            team,
            packets: self.packets(&publication, team),
            radio: Radio::Wifi,
          },
        );
      } else {
        self.diagnostics.dropped += 1;
      }
      let pending = &mut self.pending[team as usize];
      if pending
        .as_ref()
        .is_none_or(|p| publication.sequence > p.sequence)
      {
        if pending.is_some() {
          self.diagnostics.coalesced += 1;
        }
        // Shared team broadcast owns one slot regardless of robot count.
        *pending = Some(Publication {
          sequence: publication.sequence,
          decision_at: publication.decision_at,
          published_at: publication.published_at,
          safety_epoch: publication.safety_epoch,
          safety: publication.safety,
          commands: publication.commands.clone(),
          actions: publication.actions.clone(),
        });
      } else {
        self.diagnostics.coalesced += 1;
      }
    }
    self.serve_wifi(at);
  }
  fn serve_wifi(&mut self, at: TimeNs) {
    if let Some(link) = &mut self.wifi_link {
      if let Some(tx) = link.start(at, &mut self.network_rng) {
        self.timeline.schedule(tx.finishes_at, Event::WifiRelease);
        if tx.delivered {
          match tx.payload {
            WifiPayload::Command { team, packets } => self.timeline.schedule(
              tx.available_at,
              Event::Delivery {
                ticket: None,
                team,
                packets,
                radio: Radio::Wifi,
              },
            ),
            WifiPayload::Video {
              robot,
              captured_at,
              encoded_at,
            } => {
              let decoded_at = tx
                .available_at
                .saturating_add(self.config.onboard_camera.decode_delay_ns);
              self.timeline.schedule(
                decoded_at,
                Event::VideoDecoded {
                  robot,
                  frame: super::localization::StreamedFrame {
                    captured_at,
                    encoded_at,
                    received_at: tx.available_at,
                    decoded_at,
                    bytes: self.config.onboard_camera.stream_bytes_per_frame,
                  },
                },
              );
            }
            WifiPayload::Bulk => {}
          }
        } else {
          self.diagnostics.dropped += 1;
        }
      }
      self.diagnostics.wifi_link = link.diagnostics.clone();
    }
  }
  fn serve_lan(&mut self, at: TimeNs) {
    if let Some(link) = &mut self.lan_link {
      if let Some(tx) = link.start(at, &mut self.network_rng) {
        self.timeline.schedule(tx.finishes_at, Event::LanRelease);
        if tx.delivered {
          self
            .timeline
            .schedule(tx.available_at, Event::VisionDelivery(tx.payload));
        } else {
          self.diagnostics.dropped += 1;
        }
      }
      self.diagnostics.lan_link = link.diagnostics.clone();
    }
  }
  fn control(&mut self, at: TimeNs) {
    let mut auxiliary = WorldCommand::default();
    let mut traces = Vec::new();
    for robot in &mut self.robots {
      let active = robot.receiver.active(at);
      let (sequence, age) = if let Some(packet) = active {
        let decision_age = at - packet.decision_at;
        self.diagnostics.max_decision_age_at_actuation_ns = self
          .diagnostics
          .max_decision_age_at_actuation_ns
          .max(decision_age);
        self.diagnostics.max_publication_age_at_actuation_ns = self
          .diagnostics
          .max_publication_age_at_actuation_ns
          .max(at - packet.published_at);
        (Some(packet.sequence), Some(decision_age))
      } else {
        self.diagnostics.watchdog_or_expired_ticks += 1;
        (None, None)
      };
      robot.requested_command_times =
        active.map(|packet| (packet.decision_at, packet.published_at));
      let onboard = active.and_then(|p| match p.command.move_command.as_ref() {
        Some(crate::MoveCommand::Onboard(payload)) => robot.controller.onboard_command(
          at,
          payload,
          &robot.observations,
          !robot.power.brownout && at >= robot.power.kicker_ready_at,
        ),
        _ => None,
      });
      let is_onboard = active
        .is_some_and(|p| matches!(p.command.move_command, Some(crate::MoveCommand::Onboard(_))));
      let local_command = if is_onboard {
        onboard.as_ref()
      } else {
        active.map(|p| &p.command)
      };
      robot.requested_voltage = robot
        .controller
        .tick(at, local_command, &robot.observations);
      // Receiver safety/watchdog gates even a custom controller's output.
      if active.is_none() || robot.power.brownout {
        robot.requested_voltage = [None; 4];
      }
      for voltage in robot.requested_voltage.iter_mut().flatten() {
        *voltage = if voltage.is_finite() {
          voltage.clamp(
            -robot.motor_config.supply_voltage_v,
            robot.motor_config.supply_voltage_v,
          )
        } else {
          0.0
        };
      }
      let dribbler = local_command.is_some_and(|c| c.dribbler_on);
      let validity = active.map(|p| (p.expires_at, p.safety_epoch));
      // Onboard one-shot logic runs after transport; low-level host actions
      // retain receiver deduplication across both radio paths.
      let action = if is_onboard {
        onboard
          .filter(|c| c.kick_speed.is_finite() && c.kick_speed > 0.0 && c.kick_angle.is_finite())
      } else {
        robot.receiver.take_kick(at)
      };
      if action.is_some() {
        self.diagnostics.actions_consumed += 1;
      }
      let mut command = action.unwrap_or_else(|| RobotCommand::noop(robot.id));
      command.move_command = None;
      if command.kick_speed > 0.0
        && !robot
          .power
          .try_kick(at, &robot.hardware_profile.auxiliaries)
      {
        command.kick_speed = 0.0;
      }
      if command.kick_speed > 0.0 {
        if let Some((expires_at, safety_epoch)) = validity {
          robot.pending_kick = Some(PendingKick {
            fires_at: at.saturating_add(robot.hardware_profile.auxiliaries.kick_delay_ns),
            expires_at,
            safety_epoch,
            command: command.clone(),
          });
        }
        command.kick_speed = 0.0;
      }
      if validity.is_none() {
        robot.pending_kick = None;
      }
      command.dribbler_on = dribbler;
      if robot.team == TeamColor::Blue {
        auxiliary.blue.push(command);
      } else {
        auxiliary.yellow.push(command);
      }
      robot.requested_at = at;
      if at == self.physical_time {
        robot.hardware.voltage = robot.requested_voltage;
        note_actuation(&mut self.diagnostics, at, robot.requested_command_times);
        if self.trace.is_some() {
          traces.push(actuation_event(robot, at, 0));
        }
      }
      if self.trace.is_some() {
        traces.push(TraceEvent::Control {
          at,
          team: robot.team,
          id: robot.id,
          sequence,
          decision_age_ns: age,
          voltage_v: robot.requested_voltage,
        });
      }
      robot.observations.imu.clear();
    }
    if let Some(trace) = &mut self.trace {
      trace.extend(traces);
    }
    if at == self.physical_time {
      self.auxiliaries(at, &auxiliary);
    } else {
      self.pending_auxiliaries.push_back((at, auxiliary));
    }
    self.diagnostics.control_ticks += 1;
  }
  fn physics(&mut self, at: TimeNs) {
    let dt_ns = self.config.physics_step_ns;
    let dt = dt_ns as f64 * 1e-9;
    self.world.physics.clear_user_forces();
    self.world.physics.apply_ball_friction();
    for robot in &mut self.robots {
      let handle = if robot.team == TeamColor::Blue {
        &self.world.physics.blue_robots[robot.id]
      } else {
        &self.world.physics.yellow_robots[robot.id]
      };
      let body = &mut self.world.physics.rigid_body_set[handle.chassis_body];
      let facing = body.rotation().mul_vec3(Vector::X);
      let yaw = f64::from(facing.y.atan2(facing.x));
      let v = body.linvel();
      let omega = body.angvel().z;
      let velocity = [f64::from(v.x), f64::from(v.y), f64::from(omega)];
      robot.acceleration = [
        (velocity[0] - robot.previous_velocity[0]) / dt,
        (velocity[1] - robot.previous_velocity[1]) / dt,
      ];
      let local_acceleration = [
        robot.acceleration[0] * yaw.cos() + robot.acceleration[1] * yaw.sin(),
        -robot.acceleration[0] * yaw.sin() + robot.acceleration[1] * yaw.cos(),
      ];
      let local = [
        velocity[0] * yaw.cos() + velocity[1] * yaw.sin(),
        -velocity[0] * yaw.sin() + velocity[1] * yaw.cos(),
        velocity[2],
      ];
      let is_on = if robot.team == TeamColor::Blue {
        self.world.blue_sims[robot.id].is_on
      } else {
        self.world.yellow_sims[robot.id].is_on
      };
      if !is_on {
        robot.hardware.voltage = [None; 4];
      }
      if robot.power.brownout {
        robot.hardware.voltage = [None; 4];
      }
      let mut motor_config = robot.motor_config.clone();
      motor_config.supply_voltage_v = motor_config
        .supply_voltage_v
        .min(robot.power.voltage_v.max(0.0));
      if robot.hardware_profile.dynamics == super::profile::Dynamics::Free3d {
        super::contact3d::advance(
          body,
          &mut robot.hardware,
          super::contact3d::ContactStep {
            dt_ns,
            now: at - dt_ns,
            robot: &robot.config,
            motor: &motor_config,
            profile: &robot.hardware_profile,
            contact: &self.config.contact,
          },
        );
      } else {
        let wrench = robot.hardware.step(
          DriveStep {
            body_velocity: local,
            acceleration: local_acceleration,
            dt_ns,
            now: at - dt_ns,
            gravity: self.world.config.physics.gravity,
          },
          &robot.config,
          &motor_config,
          &self.config.contact,
          &robot.hardware_profile,
        );
        body.add_force(
          Vector::new(
            (wrench[0] * yaw.cos() - wrench[1] * yaw.sin()) as f32,
            (wrench[0] * yaw.sin() + wrench[1] * yaw.cos()) as f32,
            0.0,
          ),
          true,
        );
        body.add_torque(Vector::new(0.0, 0.0, wrench[2] as f32), true);
      }
      if motor_config.fault_scope == super::config::FaultScope::AllWheels
        && robot
          .hardware
          .motors
          .iter()
          .any(|m| m.driver == super::hardware::DriverState::FaultLatched)
      {
        for motor in &mut robot.hardware.motors {
          motor.driver = super::hardware::DriverState::FaultLatched;
        }
      }
      let dribbler = if robot.team == TeamColor::Blue {
        self.world.blue_sims[robot.id].dribbler_on
      } else {
        self.world.yellow_sims[robot.id].dribbler_on
      };
      let was_brownout = robot.power.brownout;
      robot.power.step(
        at,
        dt,
        &robot.hardware_profile.battery,
        &robot.hardware_profile.auxiliaries,
        &mut robot.hardware.motors,
        robot.hardware.voltage,
        motor_config.resistance_ohm,
        dribbler,
      );
      if !was_brownout && robot.power.brownout {
        robot.receiver.restart();
        robot.requested_voltage = [None; 4];
        robot.pending_kick = None;
      }
      robot.previous_velocity = velocity;
      let sim = if robot.team == TeamColor::Blue {
        &mut self.world.blue_sims[robot.id]
      } else {
        &mut self.world.yellow_sims[robot.id]
      };
      sim.wheel_speeds = std::array::from_fn(|i| robot.hardware.motors[i].speed_rad_s);
    }
    self.auxiliary_forces(at - dt_ns);
    let p = &mut self.world.physics;
    p.physics_pipeline.step(
      Vector::new(p.gravity.x, p.gravity.y, p.gravity.z),
      &p.integration_parameters,
      &mut p.island_manager,
      &mut p.broad_phase,
      &mut p.narrow_phase,
      &mut p.rigid_body_set,
      &mut p.collider_set,
      &mut p.impulse_joint_set,
      &mut p.multibody_joint_set,
      &mut p.ccd_solver,
      &(),
      &(),
    );
    // Deliberately bypass baseline velocity/position clamps: collision response
    // is physical, not limited to the motor command's maximum speed.
    self.physical_time = at;
    self.world.sim_time = at as f64 * 1e-9;
    self.diagnostics.physics_steps += 1;
    for robot in &mut self.robots {
      if robot.requested_at > at - dt_ns {
        self.diagnostics.max_actuation_quantization_ns = self
          .diagnostics
          .max_actuation_quantization_ns
          .max(at - robot.requested_at);
        note_actuation(&mut self.diagnostics, at, robot.requested_command_times);
        if let Some(trace) = &mut self.trace {
          trace.push(actuation_event(robot, at, at - robot.requested_at));
        }
      }
      robot.hardware.voltage = robot.requested_voltage;
    }
    while self
      .pending_auxiliaries
      .front()
      .is_some_and(|(time, _)| *time <= at)
    {
      let (_, auxiliary) = self.pending_auxiliaries.pop_front().unwrap();
      self.auxiliaries(at, &auxiliary);
    }
  }
  fn encoders(&mut self, at: TimeNs) {
    for (index, robot) in self.robots.iter_mut().enumerate() {
      let ticks =
        std::array::from_fn(|i| robot.hardware.motors[i].encoder_ticks(&robot.motor_config));
      let dt = robot
        .previous_encoder_at
        .map(|last| (at - last) as f64 * 1e-9);
      let speed_rad_s = std::array::from_fn(|i| {
        dt.map_or(0.0, |dt| {
          (ticks[i] - robot.previous_ticks[i]) as f64 * std::f64::consts::TAU
            / f64::from(robot.motor_config.encoder_ticks_per_revolution)
            / dt
        })
      });
      robot.previous_ticks = ticks;
      robot.previous_encoder_at = Some(at);
      self.timeline.schedule(
        at + self.config.imu.delivery_delay_ns,
        Event::EncoderDelivery {
          robot: index,
          sample: EncoderSample {
            sampled_at: at,
            ticks,
            speed_rad_s,
          },
        },
      );
      let handle = if robot.team == TeamColor::Blue {
        &self.world.physics.blue_robots[robot.id]
      } else {
        &self.world.physics.yellow_robots[robot.id]
      };
      let body = &self.world.physics.rigid_body_set[handle.chassis_body];
      let ball = self.world.physics.rigid_body_set[self.world.physics.ball_body].translation();
      let local = body.position().inverse_transform_point(ball);
      let front = robot.config.center_from_kicker;
      let kicker_z = -robot.config.height * 0.5 + robot.config.wheel_radius
        - robot.config.bottom_height
        + robot.config.kicker_z;
      let detected = (f64::from(local.x) - front).abs() <= self.world.config.ball.radius + 0.02
        && f64::from(local.y).abs() <= robot.config.kicker_width * 0.5
        && (f64::from(local.z) - kicker_z).abs()
          <= self.world.config.ball.radius + robot.config.kicker_height * 0.5;
      if detected != robot.infrared_candidate {
        robot.infrared_candidate = detected;
        robot.infrared_changed_at = at;
      }
      let detected = detected
        && at - robot.infrared_changed_at >= robot.hardware_profile.auxiliaries.ir_debounce_ns;
      let telemetry = sensors::RobotTelemetry {
        sampled_at: at,
        voltage_v: robot.power.voltage_v,
        phase_current_a: std::array::from_fn(|i| robot.hardware.motors[i].current_a),
        supply_current_a: robot.power.supply_current_a,
        motor_temperature_c: robot.power.motor_temperature_c,
        driver_fault: std::array::from_fn(|i| {
          matches!(
            robot.hardware.motors[i].driver,
            super::hardware::DriverState::FaultLatched
              | super::hardware::DriverState::Resetting { .. }
          )
        }),
        brownout: robot.power.brownout,
      };
      self.timeline.schedule(
        at + robot.hardware_profile.auxiliaries.ir_delay_ns,
        Event::TelemetryDelivery {
          robot: index,
          telemetry,
          infrared: sensors::InfraredSample {
            sampled_at: at,
            detected,
          },
        },
      );
    }
  }
  fn sample_imu(&mut self, at: TimeNs, robot_index: usize, device_index: usize, index: u64) {
    self.diagnostics.max_sensor_quantization_ns = self
      .diagnostics
      .max_sensor_quantization_ns
      .max(at - self.physical_time);
    {
      let robot = &mut self.robots[robot_index];
      let handle = if robot.team == TeamColor::Blue {
        &self.world.physics.blue_robots[robot.id]
      } else {
        &self.world.physics.yellow_robots[robot.id]
      };
      let body = &self.world.physics.rigid_body_set[handle.chassis_body];
      let v = body.velocity_at_point(body.translation());
      let omega = body.angvel();
      let q = body.rotation();
      let motion = sensors::ImuMotion {
        body_to_world: nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
          q.w as f64, q.x as f64, q.y as f64, q.z as f64,
        )),
        velocity_world: [v.x as f64, v.y as f64, v.z as f64],
        angular_world: [omega.x as f64, omega.y as f64, omega.z as f64],
        temperature_c: robot.power.temperature_c,
      };
      if let Some((available, samples)) = robot.imu_devices[device_index].sample(
        index,
        at,
        self.physical_time,
        motion,
        self.world.config.physics.gravity,
      ) {
        self.timeline.schedule(
          available,
          Event::ImuDelivery {
            robot: robot_index,
            samples,
          },
        );
      }
    }
  }
  fn vision(&mut self, at: TimeNs) {
    let state = self.world.get_state();
    let config = &self.config.vision;
    let mut robots = Vec::new();
    for r in state.blue_robots.iter().chain(&state.yellow_robots) {
      if !r.is_on || self.vision_rng.random::<f64>() < config.dropout_probability {
        continue;
      }
      robots.push(RobotDetection {
        id: r.id,
        team: r.team,
        position_m: [
          r.x + sensors::normal(&mut self.vision_rng) * config.position_noise_std_m,
          r.y + sensors::normal(&mut self.vision_rng) * config.position_noise_std_m,
        ],
        orientation_rad: r.orientation
          + sensors::normal(&mut self.vision_rng) * config.orientation_noise_std_rad,
      });
    }
    let ball_position_m =
      (self.vision_rng.random::<f64>() >= config.dropout_probability).then(|| {
        [
          state.ball.x + sensors::normal(&mut self.vision_rng) * config.position_noise_std_m,
          state.ball.y + sensors::normal(&mut self.vision_rng) * config.position_noise_std_m,
          state.ball.z,
        ]
      });
    self.timeline.schedule(
      at + config.delay_ns,
      Event::VisionDelivery(VisionFrame {
        camera_id: 0,
        frame_number: self.world.frame,
        camera_capture_ticks: None,
        image_received_at: at,
        sent_at: at,
        captured_at: at,
        physical_state_at: self.physical_time,
        robots,
        ball_position_m,
      }),
    );
  }
}

fn note_actuation(
  diagnostics: &mut Diagnostics,
  at: TimeNs,
  command_times: Option<(TimeNs, TimeNs)>,
) {
  if let Some((decision, publication)) = command_times {
    diagnostics.max_decision_age_at_actuation_ns = diagnostics
      .max_decision_age_at_actuation_ns
      .max(at - decision);
    diagnostics.max_publication_age_at_actuation_ns = diagnostics
      .max_publication_age_at_actuation_ns
      .max(at - publication);
  }
}
fn actuation_event(robot: &Robot, at: TimeNs, quantization_ns: u64) -> TraceEvent {
  TraceEvent::Actuation {
    at,
    team: robot.team,
    id: robot.id,
    quantization_ns,
    decision_age_ns: robot
      .requested_command_times
      .map(|(decision, _)| at - decision),
    publication_age_ns: robot
      .requested_command_times
      .map(|(_, publication)| at - publication),
  }
}
