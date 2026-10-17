use super::*;
impl RealisticWorld {
  /// Apply auxiliary state at a physics boundary. Kicks use the full chassis
  /// orientation and an impulse with chassis recoil; no holder/teleport path.
  pub(super) fn auxiliaries(&mut self, _at: TimeNs, commands: &WorldCommand) {
    for (team, commands) in [
      (TeamColor::Blue, &commands.blue),
      (TeamColor::Yellow, &commands.yellow),
    ] {
      for command in commands {
        let Some(robot) = self
          .robots
          .iter_mut()
          .find(|r| r.team == team && r.id == command.id)
        else {
          continue;
        };
        let sim = if team == TeamColor::Blue {
          &mut self.world.blue_sims[robot.id]
        } else {
          &mut self.world.yellow_sims[robot.id]
        };
        sim.dribbler_on = command.dribbler_on && !robot.power.brownout;
      }
    }
  }
  pub(super) fn auxiliary_forces(&mut self, at: TimeNs) {
    let ball_handle = self.world.physics.ball_body;
    for robot in &mut self.robots {
      let handle = if robot.team == TeamColor::Blue {
        &self.world.physics.blue_robots[robot.id]
      } else {
        &self.world.physics.yellow_robots[robot.id]
      };
      let body = &self.world.physics.rigid_body_set[handle.chassis_body];
      let ball = &self.world.physics.rigid_body_set[ball_handle];
      let local = body.position().inverse_transform_point(ball.translation());
      let kicker_z = -robot.config.height * 0.5 + robot.config.wheel_radius
        - robot.config.bottom_height
        + robot.config.kicker_z;
      let radius = self.world.config.ball.radius;
      let touching = (local.x as f64 - robot.config.center_from_kicker).abs() <= radius + 0.012
        && (local.y as f64).abs() <= robot.config.kicker_width / 2.0 + radius * 0.25
        && (local.z as f64 - kicker_z).abs() <= robot.config.kicker_height / 2.0 + radius;
      let point = ball.translation();
      let orientation = *body.rotation();
      let relative = orientation
        .inverse()
        .mul_vec3(ball.linvel() - body.velocity_at_point(point));
      let is_on = if robot.team == TeamColor::Blue {
        self.world.blue_sims[robot.id].is_on
      } else {
        self.world.yellow_sims[robot.id].is_on
      };
      if !is_on || robot.power.brownout {
        robot.pending_kick = None;
        continue;
      }
      if robot.pending_kick.as_ref().is_some_and(|k| {
        at >= k.expires_at
          || robot.receiver.safety() != Safety::Run
          || robot
            .receiver
            .active(at)
            .is_none_or(|p| p.safety_epoch != k.safety_epoch)
      }) {
        robot.pending_kick = None;
      }
      if robot
        .pending_kick
        .as_ref()
        .is_some_and(|kick| kick.fires_at <= at)
      {
        let kick = robot.pending_kick.take().unwrap().command;
        if touching {
          let limit = if kick.kick_angle > 0.0 {
            robot.config.max_chip_kick_speed
          } else {
            robot.config.max_linear_kick_speed
          };
          let angle = kick.kick_angle.clamp(0.0, 90.0).to_radians();
          let speed = kick.kick_speed.min(limit).max(0.0);
          // Energy cap remains physical even with an excessive command.
          let speed = speed.min(
            (2.0 * robot.hardware_profile.auxiliaries.kick_energy_j / self.world.config.ball.mass)
              .sqrt(),
          );
          let desired = Vector::new(
            (speed * angle.cos()) as f32,
            relative.y.clamp(-0.1, 0.1),
            (speed * angle.sin()) as f32,
          );
          let impulse =
            orientation.mul_vec3(desired - relative) * self.world.config.ball.mass as f32;
          self.world.physics.rigid_body_set[ball_handle].apply_impulse(impulse, true);
          self.world.physics.rigid_body_set[handle.chassis_body]
            .apply_impulse_at_point(-impulse, point, true);
        }
      }
      if touching && robot.power.dribbler_fraction > 0.001 {
        // Finite roller/contact traction. Acceleration and collisions can pull
        // the ball away; the ball is never kinematically attached to a robot.
        let aux = &robot.hardware_profile.auxiliaries;
        let pull = ((robot.config.center_from_kicker + radius - local.x as f64)
          * aux.dribbler_stiffness_n_m
          - relative.x as f64 * aux.dribbler_damping_n_s_m
          - aux.dribbler_pull_n)
          * robot.power.dribbler_fraction;
        let lateral = (-(local.y as f64) * aux.dribbler_stiffness_n_m
          - relative.y as f64 * aux.dribbler_damping_n_s_m)
          * robot.power.dribbler_fraction;
        let max = aux.dribbler_max_force_n * robot.power.dribbler_fraction;
        let force = orientation.mul_vec3(Vector::new(
          pull.clamp(-max, max) as f32,
          lateral.clamp(-max, max) as f32,
          0.0,
        ));
        self.world.physics.rigid_body_set[ball_handle].add_force(force, true);
        self.world.physics.rigid_body_set[handle.chassis_body]
          .add_force_at_point(-force, point, true);
        let torque = orientation.mul_vec3(Vector::new(
          0.0,
          -(robot.power.dribbler_fraction * aux.dribbler_spin_torque_nm) as f32,
          0.0,
        ));
        self.world.physics.rigid_body_set[ball_handle].add_torque(torque, true);
        self.world.physics.rigid_body_set[handle.chassis_body].add_torque(-torque, true);
      }
    }
  }
}
