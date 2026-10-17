use super::super::{
  localization::{RangingProtocol, UwbObservation},
  radio::{RangeJob, stamp},
  scene::Scene,
};
use super::*;
impl RealisticWorld {
  pub(super) fn radio_aux(&mut self, at: TimeNs, slot: usize, index: u64) {
    let profile = self.config.transport.slots[slot].clone();
    self.timeline.schedule(
      at + self.config.transport.superframe_ns * u64::from(profile.every_superframes),
      Event::RadioAux {
        slot,
        index: index + 1,
      },
    );
    if !self.config.uwb.enabled || self.robots.is_empty() {
      return;
    }
    match profile.purpose {
      SlotPurpose::Telemetry => {
        let robot = index as usize % self.robots.len();
        let r = &self.robots[robot];
        if r
          .last_radio_sync
          .is_none_or(|last| at.saturating_sub(last) > self.config.uwb.synchronization_timeout_ns)
        {
          self.mac.diagnostics.missed_sync += 1;
          return;
        }
        let ticket = self.mac.reserve(at, profile.occupied_ns);
        let format = &self.config.transport.command_format;
        let airtime = format.preamble_and_phy_header_ns
          + ((u128::from(self.config.uwb.telemetry_payload_bytes)
            + u128::from(format.header_bytes))
            * 8
            * 1_000_000_000)
            .div_ceil(u128::from(format.bitrate_bits_per_second)) as u64;
        self.mac.frame(airtime);
        if self.network_rng.random::<f64>() < self.config.uwb.loss_probability {
          self.mac.finish(ticket);
          self.diagnostics.dropped += 1;
          return;
        }
        self.timeline.schedule(
          at + profile.occupied_ns + self.config.uwb.host_delay_ns,
          Event::UwbTelemetryDelivery {
            robot,
            ticket,
            telemetry: r.observations.telemetry.clone(),
            infrared: r.observations.infrared.clone(),
            action: r.receiver.last_action_id(),
          },
        );
      }
      SlotPurpose::Synchronization => {
        let ticket = self.mac.reserve(at, profile.occupied_ns);
        self.mac.frame(self.config.uwb.frame_airtime_ns);
        if self.network_rng.random::<f64>() < self.config.uwb.loss_probability {
          self.mac.finish(ticket);
          self.mac.diagnostics.missed_sync += 1;
          return;
        }
        self.timeline.schedule(
          at + profile.occupied_ns + self.config.uwb.host_delay_ns,
          Event::SynchronizationDelivery { ticket },
        );
      }
      SlotPurpose::Ranging => {
        if self.config.uwb.protocol == RangingProtocol::Tdoa {
          self.tdoa(at, profile.occupied_ns, index);
          return;
        }
        let robot = index as usize % self.robots.len();
        let anchor = (index as usize / self.robots.len()) % self.config.uwb.anchors.len();
        let r = &self.robots[robot];
        if r
          .last_radio_sync
          .is_none_or(|last| at.saturating_sub(last) > self.config.uwb.synchronization_timeout_ns)
        {
          self.mac.diagnostics.missed_sync += 1;
          return;
        }
        let ticket = self.mac.reserve(at, profile.occupied_ns);
        self.mac.frame(self.config.uwb.frame_airtime_ns);
        let (position, _) = self.sensor_pose(robot);
        let target = nalgebra::Vector3::from(self.config.uwb.anchors[anchor].position_m);
        let scene = Scene::from_world(&self.world, &self.config.environment, Some((r.team, r.id)));
        let blocked = scene.occluded(position, target);
        let bias = if blocked {
          self.config.uwb.nlos_bias_m
        } else {
          0.0
        } + sensors::normal(&mut self.robots[robot].localization_rng)
          * self.config.uwb.range_noise_std_m;
        let tof = ((target - position).norm() + bias).max(0.0) / 299_792_458.0;
        let mut observation = super::super::radio::observation(
          &self.config.uwb,
          at,
          self.config.uwb.anchors[anchor].id,
        );
        observation.quality = if blocked { 0.2 } else { 1.0 };
        observation.timestamps[0] = stamp(&self.config.uwb.clock, at, 0.0);
        observation.timestamps[1] = stamp(
          &self.config.uwb.anchors[anchor].clock,
          at,
          tof
            + (self.config.uwb.antenna_delay_ns + self.config.uwb.anchors[anchor].antenna_delay_ns)
              * 1e-9,
        );
        let response_at = at
          + self.config.uwb.frame_airtime_ns
          + self.config.uwb.turnaround_ns
          + (tof * 1e9).ceil() as u64;
        let job = RangeJob {
          robot,
          anchor,
          ticket,
          slot_end: at + profile.occupied_ns,
          bias_m: bias,
          observation,
        };
        self.timeline.schedule(response_at, Event::UwbResponse(job));
      }
      _ => {}
    }
    self.diagnostics.radio = self.mac.diagnostics.clone();
  }
  pub(super) fn uwb_response(&mut self, at: TimeNs, mut job: RangeJob) {
    if !self.mac.valid(job.ticket)
      || self.network_rng.random::<f64>() < self.config.uwb.loss_probability
    {
      self.mac.finish(job.ticket);
      self.mac.diagnostics.responses_missed += 1;
      return;
    }
    self.mac.frame(self.config.uwb.frame_airtime_ns);
    let (position, orientation) = self.sensor_pose(job.robot);
    let anchor = &self.config.uwb.anchors[job.anchor];
    let distance = (nalgebra::Vector3::from(anchor.position_m) - position).norm();
    let tof = (distance + job.bias_m).max(0.0) / 299_792_458.0;
    job.observation.timestamps[2] = stamp(&anchor.clock, at, 0.0);
    job.observation.timestamps[3] = stamp(
      &self.config.uwb.clock,
      at,
      tof + (self.config.uwb.antenna_delay_ns + anchor.antenna_delay_ns) * 1e-9,
    );
    let bearing = super::super::radio::bearing(&self.config.uwb, position, orientation, anchor)
      + sensors::normal(&mut self.robots[job.robot].localization_rng)
        * self.config.uwb.bearing_noise_std_rad;
    job.observation.bearing_rad = Some(if self.config.uwb.bearing_ambiguity_pi {
      bearing.rem_euclid(std::f64::consts::PI)
    } else {
      super::super::vision::wrap(bearing)
    });
    if self.config.uwb.protocol == RangingProtocol::DoubleSided {
      self.timeline.schedule(
        at + self.config.uwb.frame_airtime_ns
          + self.config.uwb.turnaround_ns
          + (tof * 1e9).ceil() as u64,
        Event::UwbFinal(job),
      );
    } else {
      if at + self.config.uwb.frame_airtime_ns + (tof * 1e9).ceil() as u64 > job.slot_end
        || self.network_rng.random::<f64>() < self.config.uwb.loss_probability
      {
        self.mac.finish(job.ticket);
        self.mac.diagnostics.responses_missed += 1;
        return;
      }
      self.timeline.schedule(
        at + self.config.uwb.frame_airtime_ns
          + self.config.uwb.host_delay_ns
          + (tof * 1e9).ceil() as u64,
        Event::UwbRangingDelivery(job),
      );
    }
  }
  pub(super) fn uwb_final(&mut self, at: TimeNs, mut job: RangeJob) {
    if !self.mac.valid(job.ticket)
      || self.network_rng.random::<f64>() < self.config.uwb.loss_probability
    {
      self.mac.finish(job.ticket);
      self.mac.diagnostics.responses_missed += 1;
      return;
    }
    self.mac.frame(self.config.uwb.frame_airtime_ns);
    let (position, _) = self.sensor_pose(job.robot);
    let anchor = &self.config.uwb.anchors[job.anchor];
    let tof = ((nalgebra::Vector3::from(anchor.position_m) - position).norm() + job.bias_m)
      .max(0.0)
      / 299_792_458.0;
    job.observation.timestamps[4] = stamp(&self.config.uwb.clock, at, 0.0);
    job.observation.timestamps[5] = stamp(
      &anchor.clock,
      at,
      tof + (self.config.uwb.antenna_delay_ns + anchor.antenna_delay_ns) * 1e-9,
    );
    let end = at + self.config.uwb.frame_airtime_ns + (tof * 1e9).ceil() as u64;
    if end > job.slot_end || self.network_rng.random::<f64>() < self.config.uwb.loss_probability {
      self.mac.finish(job.ticket);
      self.mac.diagnostics.responses_missed += 1;
      return;
    }
    self.timeline.schedule(
      end + self.config.uwb.host_delay_ns,
      Event::UwbRangingDelivery(job),
    );
  }
  pub(super) fn radio_observation(
    &mut self,
    at: TimeNs,
    robot: usize,
    observation: UwbObservation,
  ) {
    if self.trace.is_some() && self.config.record_raw_sensors {
      let r = &self.robots[robot];
      self.trace(TraceEvent::RadioInput {
        at,
        team: r.team,
        id: r.id,
        observation: observation.clone(),
      });
    }
    let r = &mut self.robots[robot];
    if let Some(previous) = r
      .observations
      .radio
      .iter_mut()
      .find(|o| o.anchor_id == observation.anchor_id)
    {
      *previous = observation;
    } else {
      r.observations.radio.push(observation);
    }
    r.pose_estimator
      .ranges(&r.observations.radio, &self.config.uwb.anchors, at);
    r.observations.pose = Some(r.pose_estimator.estimate.clone());
    r.observations.delivered_at = at;
  }
  fn tdoa(&mut self, at: TimeNs, occupied: u64, index: u64) {
    let anchor = &self.config.uwb.anchors[index as usize % self.config.uwb.anchors.len()];
    let target = nalgebra::Vector3::from(anchor.position_m);
    let ticket = self.mac.reserve(at, occupied);
    self.mac.frame(self.config.uwb.frame_airtime_ns);
    if self.network_rng.random::<f64>() < self.config.uwb.loss_probability {
      self.mac.finish(ticket);
      self.mac.diagnostics.responses_missed += 1;
      return;
    }
    let tx = stamp(&anchor.clock, at, 0.0);
    let mut observations = Vec::new();
    for robot in 0..self.robots.len() {
      if self.robots[robot]
        .last_radio_sync
        .is_none_or(|last| at.saturating_sub(last) > self.config.uwb.synchronization_timeout_ns)
      {
        self.mac.diagnostics.missed_sync += 1;
        continue;
      }
      let (position, orientation) = self.sensor_pose(robot);
      let distance = (target - position).norm()
        + sensors::normal(&mut self.robots[robot].localization_rng)
          * self.config.uwb.range_noise_std_m;
      let rx = stamp(
        &self.config.uwb.clock,
        at,
        distance.max(0.0) / 299_792_458.0,
      );
      let mut observation = super::super::radio::observation(&self.config.uwb, at, anchor.id);
      observation.timestamps[0] = tx;
      observation.timestamps[1] = rx;
      observation.bearing_rad = Some(
        super::super::radio::bearing(&self.config.uwb, position, orientation, anchor)
          + sensors::normal(&mut self.robots[robot].localization_rng)
            * self.config.uwb.bearing_noise_std_rad,
      );
      if let Some((reference, _, old_rx, old_tx)) = self.tdoa_reference[robot] {
        let modulus = 1_u128 << self.config.uwb.clock.counter_bits;
        let delta = |a, b| {
          ((u128::from(a) + modulus - u128::from(b)) % modulus) as f64
            / self.config.uwb.clock.tick_hz as f64
        };
        observation.timestamps[2] = old_tx;
        observation.timestamps[3] = old_rx;
        observation.tdoa_reference = Some((
          reference,
          (delta(rx, old_rx) - delta(tx, old_tx)) * 299_792_458.0,
        ));
      }
      if self.tdoa_reference[robot].is_none() || anchor.id == self.config.uwb.anchors[0].id {
        self.tdoa_reference[robot] = Some((anchor.id, at, rx, tx));
      }
      observations.push((robot, observation));
    }
    self.timeline.schedule(
      at + occupied + self.config.uwb.host_delay_ns,
      Event::TdoaDelivery {
        ticket,
        observations,
      },
    );
  }
}
