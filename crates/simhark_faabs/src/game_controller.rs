use crashpilot::communication::EventShare;
use crashpilot::{Communication, Config, Events, SslGameController};
use prost::Message;
use socket2::{Domain, Protocol, SockAddr, Socket, Type};
use std::io;
use std::net::{Ipv4Addr, SocketAddrV4};
use std::sync::Arc;
use tokio::net::UdpSocket;
use tokio::sync::RwLock;

#[derive(Clone, Debug)]
pub struct GameControllerCommunication {
  controller: SslGameController,
}

impl GameControllerCommunication {
  pub fn spawn(config: &Config, events: EventShare) -> Self {
    tokio::runtime::Handle::try_current()
      .expect("simhark_faabs/ssl_game_controller requires an active Tokio runtime");

    spawn_referee_multicast(config, events.clone());

    Self {
      controller: SslGameController::spawn(config, events),
    }
  }

  pub fn controller(&self) -> &SslGameController {
    &self.controller
  }
}

impl Communication for GameControllerCommunication {
  fn request_desired_keeper(&self, goalie: u8) {
    let controller = self.controller.clone();

    tokio::spawn(async move {
      if let Err(err) = controller.desired_keeper(goalie as i32).await {
        eprintln!("Failed to request new goalie {goalie}: {err:#}");
      }
    });
  }
}

pub fn event_share() -> EventShare {
  Arc::new(RwLock::new(Events::new()))
}

fn spawn_referee_multicast(config: &Config, events: EventShare) {
  let socket = create_multicast_socket(
    config.ssl.ssl_gc_ip,
    config.ssl.ssl_gc_port,
    config.ssl.ssl_interface,
  )
  .unwrap_or_else(|err| {
    panic!(
      "Failed to create multicast socket for referee {}:{} on interface {}: {}",
      config.ssl.ssl_gc_ip, config.ssl.ssl_gc_port, config.ssl.ssl_interface, err
    )
  });

  tokio::spawn(async move {
    let mut buf = [0_u8; 65_536];

    loop {
      match socket.recv_from(&mut buf).await {
        Ok((size, _)) => {
          let Ok(mut latest_referee) = core_dump::proto::Referee::decode(&buf[..size]) else {
            continue;
          };

          loop {
            match socket.try_recv_from(&mut buf) {
              Ok((size, _)) => {
                if let Ok(referee) = core_dump::proto::Referee::decode(&buf[..size]) {
                  latest_referee = referee;
                }
              }
              Err(err) if err.kind() == io::ErrorKind::WouldBlock => break,
              Err(err) => {
                eprintln!("SSL GameController multicast receive error: {err}");
                break;
              }
            }
          }

          let mut lock = events.write().await;
          lock.gc = Some(latest_referee);
        }
        Err(err) => {
          eprintln!("SSL GameController multicast receive error: {err}");
        }
      }
    }
  });
}

fn create_multicast_socket(
  multicast: Ipv4Addr,
  port: u16,
  interface: Ipv4Addr,
) -> io::Result<UdpSocket> {
  let socket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;

  socket.set_reuse_address(true)?;
  #[cfg(unix)]
  socket.set_reuse_port(true)?;

  let addr = SockAddr::from(SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, port));
  socket.bind(&addr)?;
  socket.join_multicast_v4(&multicast, &interface)?;
  socket.set_nonblocking(true)?;

  UdpSocket::from_std(socket.into())
}
