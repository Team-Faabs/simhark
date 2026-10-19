use loguna::proto::SslWrapperPacket;
use loguna::{LogReader, MessageId};
use prost::Message;

fn main() -> Result<(), Box<dyn std::error::Error>> {
  let mut args = std::env::args().skip(1);
  let Some(path) = args.next() else {
    eprintln!(
      "usage: inspect-pickup <log-path> [--team blue|yellow] [--from FRAME] [--to FRAME] [--loose-close] [--fast-pickup]"
    );
    std::process::exit(2);
  };
  let mut team = "yellow".to_string();
  let mut from_frame = 0_u64;
  let mut to_frame = u64::MAX;
  let mut loose_close_only = false;
  let mut fast_pickup_only = false;
  while let Some(arg) = args.next() {
    match arg.as_str() {
      "--team" => team = args.next().unwrap_or_else(|| "yellow".into()),
      "--from" => {
        from_frame = args
          .next()
          .and_then(|value| value.parse().ok())
          .unwrap_or(0)
      }
      "--to" => {
        to_frame = args
          .next()
          .and_then(|value| value.parse().ok())
          .unwrap_or(u64::MAX)
      }
      "--loose-close" => loose_close_only = true,
      "--fast-pickup" => fast_pickup_only = true,
      other => {
        eprintln!("unknown argument: {other}");
        std::process::exit(2);
      }
    }
  }

  let want_blue = team.eq_ignore_ascii_case("blue");
  let mut reader = LogReader::open(&path)?;
  let mut prev_ball: Option<(f64, f64)> = None;
  let mut frame = 0_u64;

  println!(
    "{:>5} {:>8} {:>8} {:>8} {:>4} {:>8} {:>8} {:>8} {:>7} {:>9} {:>8} {:>8} {:>6}",
    "frame",
    "ball_x",
    "ball_y",
    "v_mps",
    "id",
    "dist_m",
    "robot_x",
    "robot_y",
    "opp_id",
    "opp_dist",
    "target_x",
    "target_y",
    "lead"
  );

  while let Some(msg) = reader.next_message()? {
    if msg.message_id != MessageId::Vision2014 {
      continue;
    }
    let Ok(wrapper) = SslWrapperPacket::decode(msg.payload.as_slice()) else {
      continue;
    };
    let Some(det) = wrapper.detection else {
      continue;
    };
    let Some(ball) = det.balls.first() else {
      continue;
    };

    frame += 1;
    if frame < from_frame || frame > to_frame {
      prev_ball = Some((ball.x as f64, ball.y as f64));
      continue;
    }

    let ball_pos = (ball.x as f64, ball.y as f64);
    let ball_vel_mps = prev_ball
      .map(|prev| {
        (
          (ball_pos.0 - prev.0) * 60.0 / 1000.0,
          (ball_pos.1 - prev.1) * 60.0 / 1000.0,
        )
      })
      .unwrap_or((0.0, 0.0));
    let ball_speed_mps = hypot(ball_vel_mps.0, ball_vel_mps.1);
    prev_ball = Some(ball_pos);

    let (robots, opponents) = if want_blue {
      (&det.robots_blue, &det.robots_yellow)
    } else {
      (&det.robots_yellow, &det.robots_blue)
    };
    let Some((id, dist, rx, ry)) = robots
      .iter()
      .map(|robot| {
        let rx = robot.x as f64;
        let ry = robot.y as f64;
        (
          robot.robot_id.unwrap_or_default(),
          hypot(rx - ball_pos.0, ry - ball_pos.1),
          rx,
          ry,
        )
      })
      .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
    else {
      continue;
    };
    let Some((opp_id, opp_dist)) = closest_robot(opponents, ball_pos) else {
      continue;
    };

    let fast_candidate =
      fast_pickup_candidate(robots, opponents, ball_pos, ball_vel_mps, ball_speed_mps);

    if loose_close_only {
      if ball_speed_mps > 1.0 || dist > 180.0 || opp_dist < dist + 120.0 {
        continue;
      }
    } else if fast_pickup_only {
      if fast_candidate.is_none() {
        continue;
      }
    }

    let (id, dist, rx, ry, opp_id, opp_dist, target_x, target_y, lead) =
      if let Some(candidate) = fast_candidate {
        (
          candidate.robot_id,
          candidate.dist_m * 1000.0,
          candidate.robot_x_m * 1000.0,
          candidate.robot_y_m * 1000.0,
          candidate.opp_id,
          candidate.opp_dist_m * 1000.0,
          candidate.target_x_m,
          candidate.target_y_m,
          candidate.lead_s,
        )
      } else {
        (
          id,
          dist,
          rx,
          ry,
          opp_id,
          opp_dist,
          f64::NAN,
          f64::NAN,
          f64::NAN,
        )
      };

    println!(
      "{frame:>5} {:>8.2} {:>8.2} {:>8.2} {id:>4} {:>8.3} {:>8.2} {:>8.2} {opp_id:>7} {:>9.3} {:>8.2} {:>8.2} {:>6.2}",
      ball_pos.0 / 1000.0,
      ball_pos.1 / 1000.0,
      ball_speed_mps,
      dist / 1000.0,
      rx / 1000.0,
      ry / 1000.0,
      opp_dist / 1000.0,
      target_x,
      target_y,
      lead,
    );
  }

  Ok(())
}

struct FastCandidate {
  robot_id: u32,
  robot_x_m: f64,
  robot_y_m: f64,
  opp_id: u32,
  dist_m: f64,
  opp_dist_m: f64,
  target_x_m: f64,
  target_y_m: f64,
  lead_s: f64,
}

fn fast_pickup_candidate(
  robots: &[loguna::proto::SslDetectionRobot],
  opponents: &[loguna::proto::SslDetectionRobot],
  ball_pos_mm: (f64, f64),
  ball_vel_mps: (f64, f64),
  ball_speed_mps: f64,
) -> Option<FastCandidate> {
  if ball_speed_mps <= 1.0 {
    return None;
  }

  let ball_x_m = ball_pos_mm.0 / 1000.0;
  let ball_y_m = ball_pos_mm.1 / 1000.0;
  let ball_x_norm = ball_x_m / 9.0;
  let ball_y_norm = ball_y_m / 6.0;
  let ball_vx_norm = ball_vel_mps.0 / 9.0;
  let ball_vy_norm = ball_vel_mps.1 / 6.0;

  let max_lead = (0.90 / ball_speed_mps).clamp(0.30, 0.85);
  let mut lead = 0.15;
  while lead <= max_lead {
    let target = (
      (ball_x_norm + ball_vx_norm * lead).clamp(-0.5, 0.5),
      (ball_y_norm + ball_vy_norm * lead).clamp(-0.5, 0.5),
    );
    let Some((robot_id, dist_m, robot_x_m, robot_y_m)) =
      closest_robot_to_norm_point(robots, target)
    else {
      return None;
    };
    let Some((opp_id, opp_dist_m, _, _)) = closest_robot_to_norm_point(opponents, target) else {
      return None;
    };

    let reachable_dist = (0.16 + 0.55 * lead).min(0.42);
    if dist_m <= reachable_dist && opp_dist_m >= dist_m + 0.15 {
      return Some(FastCandidate {
        robot_id,
        robot_x_m,
        robot_y_m,
        opp_id,
        dist_m,
        opp_dist_m,
        target_x_m: target.0 * 9.0,
        target_y_m: target.1 * 6.0,
        lead_s: lead,
      });
    }

    lead += 1.0 / 30.0;
  }

  None
}

fn hypot(x: f64, y: f64) -> f64 {
  (x * x + y * y).sqrt()
}

fn closest_robot_to_norm_point(
  robots: &[loguna::proto::SslDetectionRobot],
  target_norm: (f64, f64),
) -> Option<(u32, f64, f64, f64)> {
  robots
    .iter()
    .map(|robot| {
      let rx_m = robot.x as f64 / 1000.0;
      let ry_m = robot.y as f64 / 1000.0;
      let dx_m = rx_m - target_norm.0 * 9.0;
      let dy_m = ry_m - target_norm.1 * 6.0;
      (
        robot.robot_id.unwrap_or_default(),
        hypot(dx_m, dy_m),
        rx_m,
        ry_m,
      )
    })
    .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
}

fn closest_robot(
  robots: &[loguna::proto::SslDetectionRobot],
  ball_pos: (f64, f64),
) -> Option<(u32, f64)> {
  robots
    .iter()
    .map(|robot| {
      let rx = robot.x as f64;
      let ry = robot.y as f64;
      (
        robot.robot_id.unwrap_or_default(),
        hypot(rx - ball_pos.0, ry - ball_pos.1),
      )
    })
    .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
}
