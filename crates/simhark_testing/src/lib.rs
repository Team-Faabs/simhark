//! Batch test runner for simhark simulations.

use std::borrow::Cow;
use std::collections::HashMap;
use std::env;
use std::fmt;
use std::thread;
use std::time::Duration;

use serde::Serialize;
use simhark::{
  BallState, RobotCommand, SimulationEngine, TeamColor, TeleportBall, TeleportRobot, WorldCommand,
  WorldConfig, WorldState,
};

const MAX_ROBOTS_PER_TEAM: usize = 16;

#[derive(Debug, Clone, Copy)]
pub struct RobotInit {
  pub id: usize,
  pub x: f64,
  pub y: f64,
  pub orientation: f64,
  pub vx: f64,
  pub vy: f64,
  pub v_angular: f64,
}

impl RobotInit {
  pub fn at(id: usize, x: f64, y: f64, orientation: f64) -> Self {
    Self {
      id,
      x,
      y,
      orientation,
      vx: 0.0,
      vy: 0.0,
      v_angular: 0.0,
    }
  }
}

#[derive(Debug, Clone, Copy)]
pub struct BallInit {
  pub x: f64,
  pub y: f64,
  pub z: f64,
  pub vx: f64,
  pub vy: f64,
  pub vz: f64,
}

impl BallInit {
  pub fn at(x: f64, y: f64) -> Self {
    Self {
      x,
      y,
      z: 0.0,
      vx: 0.0,
      vy: 0.0,
      vz: 0.0,
    }
  }
}

impl From<BallState> for BallInit {
  fn from(ball: BallState) -> Self {
    Self {
      x: ball.x,
      y: ball.y,
      z: ball.z,
      vx: ball.vx,
      vy: ball.vy,
      vz: ball.vz,
    }
  }
}

#[derive(Debug, Clone)]
pub struct InitialWorld {
  pub blue: Vec<RobotInit>,
  pub yellow: Vec<RobotInit>,
  pub ball: BallInit,
}

impl InitialWorld {
  pub fn new(
    blue: impl IntoIterator<Item = RobotInit>,
    yellow: impl IntoIterator<Item = RobotInit>,
    ball: BallInit,
  ) -> Self {
    Self {
      blue: blue.into_iter().collect(),
      yellow: yellow.into_iter().collect(),
      ball,
    }
  }
}

#[derive(Debug, Clone, Default)]
pub struct TeamCommands {
  pub blue: Vec<RobotCommand>,
  pub yellow: Vec<RobotCommand>,
}

impl TeamCommands {
  pub fn empty() -> Self {
    Self::default()
  }
}

impl From<TeamCommands> for WorldCommand {
  fn from(commands: TeamCommands) -> Self {
    WorldCommand {
      blue: commands.blue,
      yellow: commands.yellow,
      ..WorldCommand::default()
    }
  }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TestOutcome {
  Running,
  Passed,
  Failed(String),
}

pub trait SimTest {
  fn name(&self) -> Cow<'_, str> {
    Cow::Borrowed("simhark test")
  }

  fn initial_state(&self) -> InitialWorld;

  fn drive(&mut self, state: &WorldState) -> TeamCommands;

  fn validate(&mut self, state: &WorldState) -> TestOutcome;
}

pub struct TestCase<T> {
  pub path: Vec<String>,
  pub test: T,
}

impl<T> TestCase<T> {
  pub fn new(name: impl Into<String>, test: T) -> Self {
    Self {
      path: vec![name.into()],
      test,
    }
  }

  pub fn with_path(path: impl IntoIterator<Item = impl Into<String>>, test: T) -> Self {
    Self {
      path: path.into_iter().map(Into::into).collect(),
      test,
    }
  }

  fn display_name(&self) -> String {
    self.path.join(" / ")
  }
}

impl<T> From<T> for TestCase<T>
where
  T: SimTest,
{
  fn from(test: T) -> Self {
    let name = test.name().into_owned();
    Self::new(name, test)
  }
}

pub struct TestSuite<T> {
  name: String,
  cases: Vec<TestCase<T>>,
}

impl<T> TestSuite<T> {
  pub fn new(name: impl Into<String>) -> Self {
    Self {
      name: name.into(),
      cases: Vec::new(),
    }
  }

  pub fn test(mut self, name: impl Into<String>, test: T) -> Self {
    self.cases.push(TestCase::new(name, test));
    self
  }

  pub fn subtests(
    mut self,
    name: impl Into<String>,
    cases: impl IntoIterator<Item = TestCase<T>>,
  ) -> Self {
    let name = name.into();
    for mut case in cases {
      case.path.insert(0, name.clone());
      self.cases.push(case);
    }
    self
  }

  pub fn into_cases(self) -> Vec<TestCase<T>> {
    self
      .cases
      .into_iter()
      .map(|mut case| {
        case.path.insert(0, self.name.clone());
        case
      })
      .collect()
  }
}

pub struct TestPlan<T> {
  groups: Vec<TestSuite<T>>,
}

impl<T> TestPlan<T> {
  pub fn new() -> Self {
    Self { groups: Vec::new() }
  }

  pub fn suite(mut self, suite: TestSuite<T>) -> Self {
    self.groups.push(suite);
    self
  }

  fn into_groups(self) -> Vec<Vec<TestCase<T>>> {
    self.groups.into_iter().map(TestSuite::into_cases).collect()
  }

  pub fn into_cases(self) -> Vec<TestCase<T>> {
    self.into_groups().into_iter().flatten().collect()
  }
}

impl<T> Default for TestPlan<T> {
  fn default() -> Self {
    Self::new()
  }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TestReportMode {
  Summary,
  All,
  Failures,
  FailureDetails,
}

#[derive(Debug, Clone)]
pub struct TestCli {
  pub report: TestReportMode,
  pub trace_patterns: Vec<String>,
  pub trace_failing: bool,
  pub trace_every: u64,
  pub help: bool,
}

impl Default for TestCli {
  fn default() -> Self {
    Self {
      report: TestReportMode::Summary,
      trace_patterns: Vec::new(),
      trace_failing: false,
      trace_every: 10,
      help: false,
    }
  }
}

impl TestCli {
  pub fn from_env() -> Result<Self, String> {
    Self::parse(env::args().skip(1))
  }

  pub fn parse(args: impl IntoIterator<Item = String>) -> Result<Self, String> {
    let mut cli = Self::default();
    let mut args = args.into_iter();

    while let Some(arg) = args.next() {
      match arg.as_str() {
        "-h" | "--help" => cli.help = true,
        "--summary" => cli.report = TestReportMode::Summary,
        "--list" | "--all" => cli.report = TestReportMode::All,
        "--failures" => cli.report = TestReportMode::Failures,
        "--failure-details" => cli.report = TestReportMode::FailureDetails,
        "--trace" => {
          let Some(pattern) = args.next() else {
            return Err("--trace requires a test name pattern".to_owned());
          };
          cli.trace_patterns.push(pattern);
        }
        "--trace-failing" => cli.trace_failing = true,
        "--trace-every" => {
          let Some(value) = args.next() else {
            return Err("--trace-every requires a positive integer".to_owned());
          };
          cli.trace_every = value
            .parse::<u64>()
            .map_err(|_| format!("invalid --trace-every value: {value}"))?
            .max(1);
        }
        "--trace-full" => cli.trace_every = 1,
        _ => {
          return Err(format!(
            "unknown argument: {arg}\nrun with --help for available test diagnostics"
          ));
        }
      }
    }

    Ok(cli)
  }
}

pub fn print_test_cli_help() {
  println!(
    "\
test diagnostics:
  --summary           print only pass/fail totals (default)
  --list, --all       print every test with status, frame, and message
  --failures          print only failed or timed-out tests
  --failure-details   print failures plus final ball and robot state
  --trace <pattern>   rerun matching test names and print world snapshots
  --trace-failing     rerun every failed or timed-out test and print snapshots
  --trace-every <n>   print every nth trace frame (default: 10)
  --trace-full        print every trace frame
  -h, --help          show this help"
  );
}

#[derive(Debug, Clone)]
pub struct TestRunnerConfig {
  pub world_config: WorldConfig,
  pub max_ticks: u64,
  pub validate_initial_state: bool,
  pub stop_finished_worlds: bool,
  pub concurrent_root_groups: usize,
}

impl Default for TestRunnerConfig {
  fn default() -> Self {
    Self {
      world_config: WorldConfig::division_b(),
      max_ticks: 600,
      validate_initial_state: true,
      stop_finished_worlds: true,
      concurrent_root_groups: 1,
    }
  }
}

#[derive(Clone)]
pub struct TestRunner {
  config: TestRunnerConfig,
}

impl TestRunner {
  pub fn new() -> Self {
    Self {
      config: TestRunnerConfig::default(),
    }
  }

  pub fn with_config(world_config: WorldConfig) -> Self {
    Self {
      config: TestRunnerConfig {
        world_config,
        ..TestRunnerConfig::default()
      },
    }
  }

  pub fn max_ticks(mut self, max_ticks: u64) -> Self {
    self.config.max_ticks = max_ticks;
    self
  }

  pub fn validate_initial_state(mut self, validate_initial_state: bool) -> Self {
    self.config.validate_initial_state = validate_initial_state;
    self
  }

  pub fn stop_finished_worlds(mut self, stop_finished_worlds: bool) -> Self {
    self.config.stop_finished_worlds = stop_finished_worlds;
    self
  }

  pub fn concurrent_root_groups(mut self, concurrent_root_groups: usize) -> Self {
    self.config.concurrent_root_groups = concurrent_root_groups.max(1);
    self
  }

  pub fn run<T>(self, tests: impl IntoIterator<Item = T>) -> TestReport
  where
    T: SimTest,
  {
    let cases = tests.into_iter().map(TestCase::from).collect();
    let mut batch = TestBatch::new(self.config, cases);
    batch.run_to_completion();
    batch.into_report()
  }

  pub fn run_cases<T>(self, tests: impl IntoIterator<Item = TestCase<T>>) -> TestReport
  where
    T: SimTest,
  {
    let mut batch = TestBatch::new(self.config, tests.into_iter().collect());
    batch.run_to_completion();
    batch.into_report()
  }

  pub fn run_suite<T>(self, suite: TestSuite<T>) -> TestReport
  where
    T: SimTest,
  {
    let mut batch = TestBatch::new(self.config, suite.into_cases());
    batch.run_to_completion();
    batch.into_report()
  }

  pub fn run_plan<T>(self, plan: TestPlan<T>) -> TestReport
  where
    T: SimTest,
  {
    let mut report = TestReport::default();
    for cases in group_chunks(plan.into_groups(), self.config.concurrent_root_groups) {
      let mut batch = TestBatch::new(self.config.clone(), cases);
      batch.run_to_completion();
      report.extend(batch.into_report());
    }
    report
  }

  pub fn run_plan_cli<T, F>(self, cli: &TestCli, mut make_plan: F) -> TestReport
  where
    T: SimTest,
    F: FnMut() -> TestPlan<T>,
  {
    if cli.help {
      print_test_cli_help();
      return TestReport::default();
    }

    let report = self.clone().run_plan(make_plan());
    print_report(&report, cli.report);

    if cli.trace_failing || !cli.trace_patterns.is_empty() {
      let failed_names = report
        .statuses
        .iter()
        .filter(|status| status.is_failure())
        .map(|status| status.name.as_str())
        .collect::<Vec<_>>();
      let traces = trace_plan(
        self.config.clone(),
        make_plan(),
        &cli.trace_patterns,
        if cli.trace_failing {
          Some(failed_names.as_slice())
        } else {
          None
        },
        cli.trace_every,
      );
      print_traces(&traces);
    }

    report
  }

  #[cfg(feature = "viewer")]
  pub fn run_with_viewer<T, I, F>(
    self,
    mut make_tests: F,
    viewer_config: simhark::viewer::ViewerConfig,
    tick_sleep: Duration,
  ) -> std::io::Result<TestReport>
  where
    T: SimTest,
    I: IntoIterator<Item = TestCase<T>>,
    F: FnMut() -> I,
  {
    let mut batch = TestBatch::new(self.config.clone(), make_tests().into_iter().collect());
    let viewer = simhark::viewer::ViewerServer::bind(
      viewer_config,
      batch.world_count(),
      &batch.engine.world(0).config,
    )?;
    viewer.enable_web_control();
    viewer.set_test_suite(batch.viewer_snapshot());
    publish_batch_viewer_frame(&viewer, &batch);
    println!("viewer: {} (testing mode)", viewer_config.http_url());

    loop {
      if viewer.take_stop_request() {
        break;
      }

      if viewer.take_restart_request() {
        let selected_worlds = viewer.selected_worlds();
        batch.rerun_worlds(
          make_tests().into_iter().collect(),
          selected_worlds.as_slice(),
        );
        viewer.reset_goals();
        viewer.set_test_suite(batch.viewer_snapshot());
        publish_batch_viewer_frame(&viewer, &batch);
      }

      if viewer.apply_robot_move_requests(&mut batch.engine) > 0 {
        batch.states = batch.engine.get_all_states();
      }

      if viewer.is_running() && !batch.is_finished() {
        batch.step_once();
        viewer.set_test_suite(batch.viewer_snapshot());
      }

      publish_batch_viewer_frame(&viewer, &batch);

      if batch.is_finished() && !viewer.is_running() {
        break;
      }

      thread::sleep(viewer.scaled_sleep(tick_sleep));
    }

    Ok(batch.into_report())
  }

  #[cfg(feature = "viewer")]
  pub fn run_plan_with_viewer<T, F>(
    self,
    mut make_plan: F,
    viewer_config: simhark::viewer::ViewerConfig,
    tick_sleep: Duration,
  ) -> std::io::Result<TestReport>
  where
    T: SimTest,
    F: FnMut() -> TestPlan<T>,
  {
    let mut session = PlanSession::new(self.config.clone(), make_plan());
    let viewer = simhark::viewer::ViewerServer::bind(
      viewer_config,
      session.world_count(),
      &session.batch.engine.world(0).config,
    )?;
    viewer.enable_web_control();
    viewer.set_test_suite(session.viewer_snapshot());
    publish_session_viewer_frame(&viewer, &session);
    println!("viewer: {} (testing mode)", viewer_config.http_url());

    loop {
      if viewer.take_stop_request() {
        break;
      }

      if viewer.take_restart_request() {
        let selected_worlds = viewer.selected_worlds();
        session.rerun_worlds(make_plan(), selected_worlds.as_slice());
        viewer.reset_goals();
        viewer.set_test_suite(session.viewer_snapshot());
        publish_session_viewer_frame(&viewer, &session);
      }

      if viewer.apply_robot_move_requests(&mut session.batch.engine) > 0 {
        session.batch.states = session.batch.engine.get_all_states();
      }

      if viewer.is_running() && !session.is_finished() {
        session.step_once();
        viewer.set_test_suite(session.viewer_snapshot());
      }

      publish_session_viewer_frame(&viewer, &session);

      if session.is_finished() && !viewer.is_running() {
        break;
      }

      thread::sleep(viewer.scaled_sleep(tick_sleep));
    }

    Ok(session.into_report())
  }
}

impl Default for TestRunner {
  fn default() -> Self {
    Self::new()
  }
}

#[cfg(feature = "viewer")]
fn publish_batch_viewer_frame<T>(viewer: &simhark::viewer::ViewerServer, batch: &TestBatch<T>)
where
  T: SimTest,
{
  if viewer.selected_worlds().len() > 1 {
    viewer.publish_states(batch.states());
  } else {
    viewer.publish(batch.selected_state(viewer.selected_world()));
  }
}

#[cfg(feature = "viewer")]
fn publish_session_viewer_frame<T>(viewer: &simhark::viewer::ViewerServer, session: &PlanSession<T>)
where
  T: SimTest,
{
  if viewer.selected_worlds().len() > 1 {
    viewer.publish_states(session.states());
  } else {
    viewer.publish(session.selected_state(viewer.selected_world()));
  }
}

struct PlanSession<T> {
  config: TestRunnerConfig,
  pending_groups: Vec<Vec<TestCase<T>>>,
  batch: TestBatch<T>,
  completed: TestReport,
  max_world_count: usize,
}

impl<T> PlanSession<T>
where
  T: SimTest,
{
  fn new(config: TestRunnerConfig, plan: TestPlan<T>) -> Self {
    let pending_groups = plan.into_groups();
    assert!(
      !pending_groups.is_empty(),
      "at least one root test group is required"
    );
    let max_world_count = max_chunk_world_count(&pending_groups, config.concurrent_root_groups);
    let concurrent = config.concurrent_root_groups;
    let mut iter = pending_groups.into_iter();
    let mut first_chunk = Vec::new();
    for _ in 0..concurrent {
      let Some(mut group) = iter.next() else {
        break;
      };
      first_chunk.append(&mut group);
    }
    assert!(!first_chunk.is_empty(), "at least one test is required");
    let remaining_groups = iter.collect::<Vec<_>>();
    Self {
      config: config.clone(),
      pending_groups: remaining_groups,
      batch: TestBatch::new(config, first_chunk),
      completed: TestReport::default(),
      max_world_count,
    }
  }

  fn world_count(&self) -> usize {
    self.max_world_count
  }

  fn selected_state(&self, selected_world: usize) -> &WorldState {
    self.batch.selected_state(selected_world)
  }

  fn states(&self) -> &[WorldState] {
    self.batch.states()
  }

  fn rerun_worlds(&mut self, fresh_plan: TestPlan<T>, worlds: &[usize]) {
    self.batch.rerun_worlds(fresh_plan.into_cases(), worlds);
  }

  fn is_finished(&self) -> bool {
    self.batch.is_finished() && self.pending_groups.is_empty()
  }

  fn step_once(&mut self) {
    if self.batch.is_finished() {
      if !self.pending_groups.is_empty() {
        let next_chunk = self.take_next_chunk();
        let finished = std::mem::replace(
          &mut self.batch,
          TestBatch::new(self.config.clone(), next_chunk),
        );
        self.completed.extend(finished.into_report());
      }
      return;
    }

    self.batch.step_once();

    if self.batch.is_finished() && self.pending_groups.is_empty() {
      // Leave the final batch visible in the viewer until the caller stops
      // or restarts the session.
    }
  }

  fn viewer_snapshot(&self) -> TestSuiteSnapshot {
    let mut snapshot = self.completed.viewer_snapshot();
    snapshot.extend(self.batch.viewer_snapshot());
    snapshot
  }

  fn into_report(mut self) -> TestReport {
    self.completed.extend(self.batch.into_report());
    self.completed
  }

  fn take_next_chunk(&mut self) -> Vec<TestCase<T>> {
    let end = self
      .config
      .concurrent_root_groups
      .min(self.pending_groups.len());
    let mut cases = Vec::new();
    for mut group in self.pending_groups.drain(0..end) {
      cases.append(&mut group);
    }
    cases
  }
}

struct TestBatch<T> {
  config: TestRunnerConfig,
  engine: SimulationEngine,
  tests: Vec<TestCase<T>>,
  statuses: Vec<TestStatus>,
  states: Vec<WorldState>,
}

impl<T> TestBatch<T>
where
  T: SimTest,
{
  fn new(mut config: TestRunnerConfig, tests: Vec<TestCase<T>>) -> Self {
    assert!(!tests.is_empty(), "at least one test is required");

    let initials = tests
      .iter()
      .map(|case| case.test.initial_state())
      .collect::<Vec<_>>();
    validate_initials(&initials);
    let max_robots = initials
      .iter()
      .map(|initial| initial.blue.len().max(initial.yellow.len()))
      .max()
      .unwrap_or(0);
    config.world_config.robots_per_team = max_robots;

    let mut engine = SimulationEngine::new(initials.len(), config.world_config.clone());
    let setup_commands = initials
      .iter()
      .map(|initial| setup_command(initial, max_robots))
      .collect::<Vec<_>>();
    let states = engine.step_with_commands(&setup_commands);

    let names = tests.iter().map(TestCase::display_name).collect::<Vec<_>>();
    let statuses = names
      .iter()
      .enumerate()
      .map(|(index, name)| {
        TestStatus::running(
          index,
          tests[index].path.clone(),
          name.clone(),
          states[index].frame,
        )
      })
      .collect::<Vec<_>>();

    let mut batch = Self {
      config,
      engine,
      tests,
      statuses,
      states,
    };

    if batch.config.validate_initial_state {
      batch.validate_current_states();
    }

    batch
  }

  fn world_count(&self) -> usize {
    self.tests.len()
  }

  fn is_finished(&self) -> bool {
    self
      .statuses
      .iter()
      .all(|status| status.outcome != TestStatusKind::Running)
  }

  fn selected_state(&self, selected_world: usize) -> &WorldState {
    let index = selected_world.min(self.states.len().saturating_sub(1));
    &self.states[index]
  }

  fn states(&self) -> &[WorldState] {
    &self.states
  }

  fn rerun_worlds(&mut self, fresh_tests: Vec<TestCase<T>>, worlds: &[usize]) {
    let mut fresh_by_name = fresh_tests
      .into_iter()
      .map(|case| (case.display_name(), case))
      .collect::<HashMap<_, _>>();
    let mut indices = worlds
      .iter()
      .copied()
      .filter(|index| *index < self.tests.len())
      .collect::<Vec<_>>();
    indices.sort_unstable();
    indices.dedup();

    let mut setup_indices = Vec::new();
    let mut setup_commands = Vec::new();

    for index in indices {
      let name = self.tests[index].display_name();
      let Some(fresh_test) = fresh_by_name.remove(&name) else {
        continue;
      };
      let initial = fresh_test.test.initial_state();
      validate_initials(std::slice::from_ref(&initial));
      self.tests[index] = fresh_test;
      self.statuses[index] = TestStatus::running(index, self.tests[index].path.clone(), name, 0);
      setup_indices.push(index);
      setup_commands.push(setup_command(
        &initial,
        self.config.world_config.robots_per_team,
      ));
    }

    if setup_indices.is_empty() {
      return;
    }

    self.engine.reset_worlds(&setup_indices);
    let reset_states = self.engine.step_subset(&setup_indices, &setup_commands);
    for (index, state) in setup_indices.into_iter().zip(reset_states.into_iter()) {
      self.statuses[index].frame = state.frame;
      self.states[index] = state;
    }
    self.validate_current_states();
  }

  fn run_to_completion(&mut self) {
    while !self.is_finished() {
      self.step_once();
    }
  }

  fn step_once(&mut self) {
    let commands = self
      .tests
      .iter_mut()
      .zip(self.states.iter())
      .zip(self.statuses.iter())
      .map(|((test, state), status)| {
        if status.outcome == TestStatusKind::Running {
          test.test.drive(state).into()
        } else if self.config.stop_finished_worlds {
          WorldCommand::default()
        } else {
          test.test.drive(state).into()
        }
      })
      .collect::<Vec<_>>();

    let next_states = self.engine.step_with_commands(&commands);
    if self.config.stop_finished_worlds {
      for ((state, next_state), status) in self
        .states
        .iter_mut()
        .zip(next_states.into_iter())
        .zip(self.statuses.iter())
      {
        if status.outcome == TestStatusKind::Running {
          *state = next_state;
        }
      }
    } else {
      self.states = next_states;
    }
    self.validate_current_states();
    for (status, state) in self.statuses.iter_mut().zip(self.states.iter()) {
      if status.outcome == TestStatusKind::Running && state.frame >= self.config.max_ticks {
        status.outcome = TestStatusKind::TimedOut;
        status.frame = state.frame;
        status.message = Some(format!("timed out after {} ticks", self.config.max_ticks));
      }
    }
  }

  fn validate_current_states(&mut self) {
    for ((test, state), status) in self
      .tests
      .iter_mut()
      .zip(self.states.iter())
      .zip(self.statuses.iter_mut())
    {
      if status.outcome != TestStatusKind::Running {
        continue;
      }

      match test.test.validate(state) {
        TestOutcome::Running => {
          status.frame = state.frame;
        }
        TestOutcome::Passed => {
          status.outcome = TestStatusKind::Passed;
          status.frame = state.frame;
          status.message = None;
        }
        TestOutcome::Failed(reason) => {
          status.outcome = TestStatusKind::Failed;
          status.frame = state.frame;
          status.message = Some(reason);
        }
      }
    }
  }

  fn viewer_snapshot(&self) -> TestSuiteSnapshot {
    TestSuiteSnapshot {
      passed: self
        .statuses
        .iter()
        .filter(|status| status.outcome == TestStatusKind::Passed)
        .count(),
      failed: self
        .statuses
        .iter()
        .filter(|status| status.outcome == TestStatusKind::Failed)
        .count(),
      timed_out: self
        .statuses
        .iter()
        .filter(|status| status.outcome == TestStatusKind::TimedOut)
        .count(),
      running: self
        .statuses
        .iter()
        .filter(|status| status.outcome == TestStatusKind::Running)
        .count(),
      tests: self.statuses.clone(),
    }
  }

  fn into_report(self) -> TestReport {
    TestReport {
      statuses: self.statuses,
      final_states: self.states,
    }
  }
}

#[derive(Debug, Clone, Serialize)]
pub struct TestSuiteSnapshot {
  pub passed: usize,
  pub failed: usize,
  pub timed_out: usize,
  pub running: usize,
  pub tests: Vec<TestStatus>,
}

impl TestSuiteSnapshot {
  fn extend(&mut self, other: TestSuiteSnapshot) {
    self.passed += other.passed;
    self.failed += other.failed;
    self.timed_out += other.timed_out;
    self.running += other.running;
    self.tests.extend(other.tests);
  }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum TestStatusKind {
  Running,
  Passed,
  Failed,
  TimedOut,
}

impl fmt::Display for TestStatusKind {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      Self::Running => f.write_str("running"),
      Self::Passed => f.write_str("passed"),
      Self::Failed => f.write_str("failed"),
      Self::TimedOut => f.write_str("timed_out"),
    }
  }
}

#[derive(Debug, Clone, Serialize)]
pub struct TestStatus {
  pub world_id: usize,
  pub path: Vec<String>,
  pub name: String,
  pub outcome: TestStatusKind,
  pub frame: u64,
  pub message: Option<String>,
}

impl TestStatus {
  fn running(world_id: usize, path: Vec<String>, name: String, frame: u64) -> Self {
    Self {
      world_id,
      path,
      name,
      outcome: TestStatusKind::Running,
      frame,
      message: None,
    }
  }

  pub fn is_failure(&self) -> bool {
    matches!(
      self.outcome,
      TestStatusKind::Failed | TestStatusKind::TimedOut
    )
  }
}

#[derive(Debug, Clone, Default)]
pub struct TestReport {
  pub statuses: Vec<TestStatus>,
  pub final_states: Vec<WorldState>,
}

impl TestReport {
  fn extend(&mut self, other: TestReport) {
    self.statuses.extend(other.statuses);
    self.final_states.extend(other.final_states);
  }

  fn viewer_snapshot(&self) -> TestSuiteSnapshot {
    TestSuiteSnapshot {
      passed: self
        .statuses
        .iter()
        .filter(|status| status.outcome == TestStatusKind::Passed)
        .count(),
      failed: self
        .statuses
        .iter()
        .filter(|status| status.outcome == TestStatusKind::Failed)
        .count(),
      timed_out: self
        .statuses
        .iter()
        .filter(|status| status.outcome == TestStatusKind::TimedOut)
        .count(),
      running: self
        .statuses
        .iter()
        .filter(|status| status.outcome == TestStatusKind::Running)
        .count(),
      tests: self.statuses.clone(),
    }
  }

  pub fn passed(&self) -> usize {
    self
      .statuses
      .iter()
      .filter(|status| status.outcome == TestStatusKind::Passed)
      .count()
  }

  pub fn failed(&self) -> usize {
    self
      .statuses
      .iter()
      .filter(|status| {
        matches!(
          status.outcome,
          TestStatusKind::Failed | TestStatusKind::TimedOut
        )
      })
      .count()
  }

  pub fn is_success(&self) -> bool {
    self.failed() == 0
  }
}

pub fn print_report(report: &TestReport, mode: TestReportMode) {
  println!(
    "testing finished: {} passed, {} failed",
    report.passed(),
    report.failed()
  );

  match mode {
    TestReportMode::Summary => {}
    TestReportMode::All => {
      for status in &report.statuses {
        print_status(status);
      }
    }
    TestReportMode::Failures => {
      for status in report.statuses.iter().filter(|status| status.is_failure()) {
        print_status(status);
      }
    }
    TestReportMode::FailureDetails => {
      for (status, state) in report
        .statuses
        .iter()
        .zip(report.final_states.iter())
        .filter(|(status, _)| status.is_failure())
      {
        print_status(status);
        print_state("  final", state);
      }
    }
  }
}

fn print_status(status: &TestStatus) {
  match &status.message {
    Some(message) => println!(
      "[{}] frame={} world={} {}: {}",
      status.outcome, status.frame, status.world_id, status.name, message
    ),
    None => println!(
      "[{}] frame={} world={} {}",
      status.outcome, status.frame, status.world_id, status.name
    ),
  }
}

#[derive(Debug, Clone)]
pub struct TestTrace {
  pub name: String,
  pub status: TestStatus,
  pub states: Vec<WorldState>,
}

pub fn trace_plan<T>(
  config: TestRunnerConfig,
  plan: TestPlan<T>,
  patterns: &[String],
  failed_names: Option<&[&str]>,
  trace_every: u64,
) -> Vec<TestTrace>
where
  T: SimTest,
{
  plan
    .into_cases()
    .into_iter()
    .filter(|case| should_trace(&case.display_name(), patterns, failed_names))
    .map(|case| trace_case(config.clone(), case, trace_every.max(1)))
    .collect()
}

fn should_trace(name: &str, patterns: &[String], failed_names: Option<&[&str]>) -> bool {
  let pattern_match = patterns.is_empty()
    || patterns
      .iter()
      .any(|pattern| name.contains(pattern.as_str()));
  let failed_match = failed_names.is_none_or(|names| names.contains(&name));
  pattern_match && failed_match
}

fn trace_case<T>(config: TestRunnerConfig, case: TestCase<T>, trace_every: u64) -> TestTrace
where
  T: SimTest,
{
  let name = case.display_name();
  let mut batch = TestBatch::new(config, vec![case]);
  let mut states = vec![batch.states[0].clone()];
  let mut last_outcome = batch.statuses[0].outcome;

  while !batch.is_finished() {
    batch.step_once();
    let status = &batch.statuses[0];
    if batch.states[0].frame % trace_every == 0 || status.outcome != last_outcome {
      states.push(batch.states[0].clone());
    }
    last_outcome = status.outcome;
  }

  TestTrace {
    name,
    status: batch.statuses.remove(0),
    states,
  }
}

pub fn print_traces(traces: &[TestTrace]) {
  if traces.is_empty() {
    println!("trace: no matching tests");
    return;
  }

  for trace in traces {
    println!();
    println!("trace: {}", trace.name);
    print_status(&trace.status);
    for state in &trace.states {
      print_state("  frame", state);
    }
  }
}

fn print_state(prefix: &str, state: &WorldState) {
  println!(
    "{prefix}={} t={:.3} ball=({:.3},{:.3},{:.3}) ball_vel=({:.3},{:.3},{:.3}) goal_blue={} goal_yellow={} blue={} yellow={}",
    state.frame,
    state.sim_time,
    state.ball.x,
    state.ball.y,
    state.ball.z,
    state.ball.vx,
    state.ball.vy,
    state.ball.vz,
    state.goal_blue,
    state.goal_yellow,
    format_robots(&state.blue_robots),
    format_robots(&state.yellow_robots),
  );
}

fn format_robots(robots: &[simhark::RobotState]) -> String {
  let active = robots
    .iter()
    .filter(|robot| robot.is_on)
    .map(|robot| {
      format!(
        "#{}@({:.3},{:.3},{:.2}deg) v=({:.3},{:.3}) ir={}",
        robot.id,
        robot.x,
        robot.y,
        robot.orientation.to_degrees(),
        robot.vx,
        robot.vy,
        robot.infrared
      )
    })
    .collect::<Vec<_>>();

  if active.is_empty() {
    "[]".to_owned()
  } else {
    format!("[{}]", active.join(", "))
  }
}

fn group_chunks<T>(groups: Vec<Vec<TestCase<T>>>, chunk_size: usize) -> Vec<Vec<TestCase<T>>> {
  let mut chunks = Vec::new();
  let mut current = Vec::new();
  let mut current_roots = 0usize;
  for mut group in groups {
    if current_roots == chunk_size {
      chunks.push(current);
      current = Vec::new();
      current_roots = 0;
    }
    current.append(&mut group);
    current_roots += 1;
  }
  if !current.is_empty() {
    chunks.push(current);
  }
  chunks
}

fn max_chunk_world_count<T>(groups: &[Vec<TestCase<T>>], chunk_size: usize) -> usize {
  groups
    .chunks(chunk_size.max(1))
    .map(|chunk| chunk.iter().map(Vec::len).sum())
    .max()
    .unwrap_or(0)
}

fn validate_initials(initials: &[InitialWorld]) {
  for (world_index, initial) in initials.iter().enumerate() {
    assert!(
      initial.blue.len() <= MAX_ROBOTS_PER_TEAM,
      "world {world_index} has more than {MAX_ROBOTS_PER_TEAM} blue robots"
    );
    assert!(
      initial.yellow.len() <= MAX_ROBOTS_PER_TEAM,
      "world {world_index} has more than {MAX_ROBOTS_PER_TEAM} yellow robots"
    );
    validate_unique_ids(world_index, TeamColor::Blue, &initial.blue);
    validate_unique_ids(world_index, TeamColor::Yellow, &initial.yellow);
  }
}

fn validate_unique_ids(world_index: usize, team: TeamColor, robots: &[RobotInit]) {
  let mut seen = [false; MAX_ROBOTS_PER_TEAM];
  for robot in robots {
    assert!(
      robot.id < MAX_ROBOTS_PER_TEAM,
      "world {world_index} {team:?} robot id {} exceeds max id {}",
      robot.id,
      MAX_ROBOTS_PER_TEAM - 1
    );
    assert!(
      !seen[robot.id],
      "world {world_index} {team:?} robot id {} is duplicated",
      robot.id
    );
    seen[robot.id] = true;
  }
}

fn setup_command(initial: &InitialWorld, robots_per_team: usize) -> WorldCommand {
  let mut teleport_robots = Vec::with_capacity(robots_per_team * 2);
  push_team_setup(
    &mut teleport_robots,
    TeamColor::Blue,
    &initial.blue,
    robots_per_team,
  );
  push_team_setup(
    &mut teleport_robots,
    TeamColor::Yellow,
    &initial.yellow,
    robots_per_team,
  );

  WorldCommand {
    teleport_ball: Some(TeleportBall {
      x: Some(initial.ball.x),
      y: Some(initial.ball.y),
      z: Some(initial.ball.z),
      vx: Some(initial.ball.vx),
      vy: Some(initial.ball.vy),
      vz: Some(initial.ball.vz),
    }),
    teleport_robots,
    ..WorldCommand::default()
  }
}

fn push_team_setup(
  out: &mut Vec<TeleportRobot>,
  team: TeamColor,
  robots: &[RobotInit],
  robots_per_team: usize,
) {
  for id in 0..robots_per_team {
    if let Some(robot) = robots.iter().find(|robot| robot.id == id) {
      out.push(TeleportRobot {
        id,
        team,
        x: Some(robot.x),
        y: Some(robot.y),
        orientation: Some(robot.orientation),
        vx: Some(robot.vx),
        vy: Some(robot.vy),
        v_angular: Some(robot.v_angular),
        present: Some(true),
      });
    } else {
      out.push(TeleportRobot {
        id,
        team,
        x: None,
        y: None,
        orientation: None,
        vx: None,
        vy: None,
        v_angular: None,
        present: Some(false),
      });
    }
  }
}
