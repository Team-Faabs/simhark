use robot_command_receiver::*;
#[derive(Clone, Debug)]
struct Command(bool);
impl ActionCommand for Command {
  fn clear_action(&mut self) {
    self.0 = false;
  }
  fn has_action(&self) -> bool {
    self.0
  }
}
fn packet(sequence: u64, action: Option<u64>) -> CommandPacket<Command> {
  CommandPacket {
    session: 7,
    sequence,
    decision_at: 0,
    published_at: 0,
    expires_at: 100,
    safety_epoch: 1,
    safety: Safety::Run,
    action_id: action,
    command: Command(action.is_some()),
  }
}
#[test]
fn copies_and_republication_keep_a_pending_action_until_consumed() {
  let mut r = Receiver::new(7, 50);
  r.accept(packet(1, Some(10)), 1).unwrap();
  r.accept(packet(2, Some(10)), 2).unwrap();
  assert!(r.take_action(3).is_some());
  r.accept(packet(3, Some(10)), 4).unwrap();
  assert!(r.take_action(4).is_none());
  assert_eq!(r.last_action_id(), Some(10));
  assert_eq!(
    r.accept(packet(2, Some(10)), 40),
    Err(Rejection::DuplicateOrOlder)
  );
  assert!(r.active(54).is_none());
}
#[test]
fn sequence_and_action_serial_numbers_wrap_with_half_range_ordering() {
  let mut r = Receiver::new(7, 50);
  r.accept(packet(u64::MAX, Some(u64::MAX)), 1).unwrap();
  r.take_action(1).unwrap();
  r.accept(packet(0, Some(0)), 2).unwrap();
  assert!(r.take_action(2).is_some());
  assert_eq!(
    r.accept(packet(u64::MAX, None), 3),
    Err(Rejection::DuplicateOrOlder)
  );
  assert_eq!(
    r.accept(packet(1_u64 << 63, None), 3),
    Err(Rejection::DuplicateOrOlder)
  );
}
#[test]
fn unsynchronized_and_uncertain_expiry_fail_closed_but_watchdog_is_local() {
  let mut r = Receiver::new(7, 50);
  assert_eq!(
    r.accept_with_clock(packet(1, None), 1, None),
    Err(Rejection::ClockUnsynchronized)
  );
  assert_eq!(
    r.accept_with_clock(
      packet(1, None),
      1000,
      Some(SenderClockEstimate {
        now_ns: 90,
        uncertainty_ns: 10
      })
    ),
    Err(Rejection::Expired)
  );
  r.accept_with_clock(
    packet(1, None),
    1000,
    Some(SenderClockEstimate {
      now_ns: 50,
      uncertainty_ns: 10,
    }),
  )
  .unwrap();
  assert!(r.active(1039).is_some());
  assert!(r.active(1040).is_none());
}
#[test]
fn checkpoint_reboot_preserves_action_deduplication_without_liveness() {
  let mut r = Receiver::new(7, 50);
  r.accept(packet(1, Some(3)), 1).unwrap();
  r.take_action(1).unwrap();
  let mut restored = Receiver::restore(r.checkpoint(), 50);
  assert!(restored.active(2).is_none());
  restored.accept(packet(2, Some(3)), 2).unwrap();
  assert!(restored.take_action(2).is_none());
  restored.begin_session(8);
  assert_eq!(
    restored.accept(packet(3, None), 3),
    Err(Rejection::WrongSession)
  );
}
#[test]
fn delayed_running_commands_cannot_undo_a_new_safety_epoch() {
  let mut r = Receiver::new(7, 50);
  r.accept(packet(1, None), 1).unwrap();
  let mut halt = packet(2, None);
  halt.safety = Safety::Halt;
  halt.safety_epoch = 2;
  r.accept(halt, 2).unwrap();
  assert_eq!(
    r.accept(packet(3, None), 3),
    Err(Rejection::SafetyRegression)
  );
  assert!(r.active(3).is_none());
}
