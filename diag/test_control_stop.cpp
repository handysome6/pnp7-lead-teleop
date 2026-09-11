// Production finish/deadline guards, exercised without opening any hardware.
#define main pnp7_bridge_main
#include "../src/pnp7_teleop.cpp"
#undef main
#include <future>

void testSnapshot() {
  std::mutex mutex;
  LeadSnapshot latest{}, cached{};
  latest.seq = 2; latest.t_ns = 200; latest.ticks.fill(2);
  cached.seq = 1; cached.t_ns = 100; cached.ticks.fill(1);
  std::promise<void> locked, release;
  auto ready = locked.get_future(); auto released = release.get_future();
  std::thread writer([&] {
    std::lock_guard<std::mutex> lock(mutex);
    locked.set_value(); released.wait();
  });
  ready.wait();
  const bool copied = tryCopyLeadSnapshot(mutex, latest, cached);
  release.set_value(); writer.join();
  requireTest(!copied && cached.seq == 1 && cached.t_ns == 100 && cached.ticks[6] == 1,
              "busy publisher never blocks FCI and preserves sample timestamp");
  requireTest(tryCopyLeadSnapshot(mutex, latest, cached) && cached.seq == 2 &&
              cached.t_ns == 200 && cached.ticks[6] == 2, "complete sample copied after release");
}

void testFinish() {
  MotionFinishGuard guard;
  franka::RobotState state{};
  std::array<double, 7> command{};
  state.dq_d[3] = 0.05;
  state.dq[3] = 0.02;
  for (int i = 0; i < 200; ++i)
    requireTest(!guard.update(true, true, command, state, .001),
                "local generator at rest is not robot at rest");
  state.dq_d.fill(0.0); state.dq.fill(0.0);
  state.ddq_d[3] = 0.5;
  requireTest(!guard.update(true, true, command, state, .001), "accepted acceleration must settle");
  state.ddq_d.fill(0.0); state.q_d[3] = .001;
  requireTest(!guard.update(true, true, command, state, .001), "filter position must settle");
  state.q_d.fill(0.0);
  for (int i = 0; i < 99; ++i)
    requireTest(!guard.update(true, true, command, state, .001), "100 ms stable dwell");
  requireTest(guard.update(true, true, command, state, .001), "settled normal completion");
  requireTest(!guard.update(false, true, command, state, .001), "active session never finishes");
  std::array<bool, 41> errors{}; errors[0] = true;
  state.current_errors = franka::Errors(errors);
  for (int i = 0; i < 200; ++i)
    requireTest(!guard.update(true, true, command, state, .001), "reflex is never normal completion");
  state.current_errors = franka::Errors();
  for (int i = 0; i < 80; ++i) guard.update(true, true, command, state, .001);
  requireTest(!guard.update(true, true, command, state, .005), "gap resets finish dwell");
  for (int i = 0; i < 99; ++i)
    requireTest(!guard.update(true, true, command, state, .001), "full dwell after gap");
  requireTest(guard.update(true, true, command, state, .001), "settles after gap");
}

void testDeadline() {
  ControlDeadlineGuard guard;
  guard.enter(1000000, 0.0, 0);
  guard.commandReady(1050000);
  guard.enter(2000000, .001, 1);
  guard.commandReady(2050000);
  bool refused = false;
  try { guard.enter(8700000, .001, 2); } catch (const std::runtime_error& e) {
    refused = std::string(e.what()).find("host_gap_ms=6.7") != std::string::npos;
  }
  requireTest(refused, "stale callback with nominal 1 ms robot period is cancelled");
  ControlDeadlineGuard robot_gap;
  robot_gap.enter(1000000, 0, 0);
  refused = false;
  try { robot_gap.enter(2000000, .005, 5); } catch (const std::runtime_error&) { refused = true; }
  requireTest(refused, "robot period gap is cancelled");
  ControlDeadlineGuard work;
  work.enter(1000000, 0, 0);
  refused = false;
  try { work.commandReady(2100000); } catch (const std::runtime_error&) { refused = true; }
  requireTest(refused, "callback work overrun is cancelled before command return");
}

std::vector<std::string> split(const std::string& line) {
  std::istringstream input(line); std::string cell; std::vector<std::string> cells;
  while (std::getline(input, cell, ',')) cells.push_back(cell);
  return cells;
}
void replay(const std::string& path) {
  std::ifstream input(path); requireTest(input.good(), "replay trace exists");
  std::string line; std::getline(input, line); const auto header = split(line);
  auto index = [&](const std::string& key) {
    auto found = std::find(header.begin(), header.end(), key);
    requireTest(found != header.end(), "trace column " + key);
    return std::distance(header.begin(), found);
  };
  const auto t_col = index("t_ns"), dt_col = index("dt_s");
  ControlDeadlineGuard guard;
  size_t row = 0;
  while (std::getline(input, line)) {
    const auto values = split(line); ++row;
    try { guard.enter(std::stoll(values.at(t_col)), std::stod(values.at(dt_col)), 0); }
    catch (const std::runtime_error& e) {
      requireTest(std::stod(values.at(dt_col)) == .001,
                  "replay catches stale 1 ms callback before robot time jumps");
      std::cout << "REPLAY_REJECTED " << path << " row=" << row << " " << e.what() << '\n';
      return;
    }
  }
  throw std::runtime_error("expected the recorded deadline violation");
}
int main(int argc, char** argv) {
  try {
    testSnapshot(); testFinish(); testDeadline();
    for (int i = 1; i < argc; ++i) replay(argv[i]);
    std::cout << "CONTROL_STOP_TEST_OK\n";
    return 0;
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
