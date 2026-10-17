use super::*;
impl RealisticWorld {
  pub(super) fn lidar_beam(&mut self, at: TimeNs, robot: usize, index: u64) {
    let r = &self.robots[robot];
    let scene = super::super::scene::Scene::from_world(
      &self.world,
      &self.config.environment,
      Some((r.team, r.id)),
    );
    let (position, orientation) = self.sensor_pose(robot);
    let r = &mut self.robots[robot];
    let beam = super::super::localization::lidar_beam(
      &self.config.lidar,
      at,
      index,
      position,
      orientation,
      &scene,
      &mut r.localization_rng,
    );
    r.lidar_beams.push(beam);
    if r.lidar_beams.len() == self.config.lidar.beams as usize {
      let scan = super::super::localization::LidarScan {
        started_at: r.lidar_beams[0].sampled_at,
        completed_at: at,
        beams: std::mem::take(&mut r.lidar_beams),
      };
      self.timeline.schedule(
        at + self.config.lidar.delivery_ns,
        Event::LidarDelivery { robot, scan },
      );
    }
    let rate = u128::from(self.config.lidar.scan_rate_hz) * u128::from(self.config.lidar.beams);
    self.timeline.schedule(
      ((index + 1) as u128 * 1_000_000_000 / rate) as u64,
      Event::LidarBeam {
        robot,
        index: index + 1,
      },
    );
  }
  pub(super) fn onboard_camera(&mut self, at: TimeNs, robot: usize, index: u64) {
    let r = &self.robots[robot];
    let scene = super::super::scene::Scene::from_world(
      &self.world,
      &self.config.environment,
      Some((r.team, r.id)),
    );
    let (position, orientation) = self.sensor_pose(robot);
    let ball = super::super::localization::onboard_ball(
      &self.config.onboard_camera,
      at,
      position,
      orientation,
      &scene,
      &mut self.robots[robot].localization_rng,
    );
    self.timeline.schedule(
      at + self.config.onboard_camera.detection_delay_ns,
      Event::OnboardDelivery { robot, ball },
    );
    if self.config.onboard_camera.streaming {
      self.timeline.schedule(
        at + self.config.onboard_camera.encode_delay_ns,
        Event::VideoEncoded {
          robot,
          captured_at: at,
        },
      );
    }
    self.timeline.schedule(
      ((index + 1) as u128 * 1_000_000_000 / u128::from(self.config.onboard_camera.rate_hz)) as u64,
      Event::OnboardCamera {
        robot,
        index: index + 1,
      },
    );
  }
  pub(super) fn sensor_pose(
    &self,
    robot: usize,
  ) -> (nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>) {
    let r = &self.robots[robot];
    let handle = if r.team == TeamColor::Blue {
      &self.world.physics.blue_robots[r.id]
    } else {
      &self.world.physics.yellow_robots[r.id]
    };
    let body = &self.world.physics.rigid_body_set[handle.chassis_body];
    let p = body.translation();
    let q = body.rotation();
    (
      nalgebra::Vector3::new(p.x as f64, p.y as f64, p.z as f64),
      nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        q.w as f64, q.x as f64, q.y as f64, q.z as f64,
      )),
    )
  }
}
