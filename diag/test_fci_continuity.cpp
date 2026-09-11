// Offline integration regression: production SafetyChain plus the signal
// processing used by libfranka 0.21.3's JointPositions control loop.
// No Robot, GELLO, pedal or camera is opened. The synthetic receiver extrapolates
// constant acceleration during missing callbacks; this is not a hardware model
// or proof of the cause of any particular real-world fault.
#define main pnp7_bridge_main
#include "../src/pnp7_teleop.cpp"
#undef main
#include <franka/lowpass_filter.h>
#include <franka/rate_limiting.h>

double simulate(bool limit_rate, bool gaps) {
  Config config;
  config.enabled.fill(true);
  config.sign.fill(1.0);
  config.scale.fill(1.0);
  config.max_joint_velocity.fill(0.45);
  config.max_joint_acceleration.fill(3.0);
  config.lowpass_hz = 6.0;
  const std::array<double, 7> origin{0.0, -0.4, 0.0, -2.2, 0.0, 1.9, 0.8};
  SafetyChain chain(config);
  chain.seed(origin);
  auto q = origin;
  std::array<double, 7> dq{}, ddq{}, delta{}, upper{}, lower{};
  upper.fill(1.0);
  lower.fill(-1.0);
  double worst_acceleration = 0.0;
  for (int k = 0; k < 6000; ++k) {
    const int period_ms = gaps && (k == 190 || k == 420 || k == 700) ? 5 : 1;
    // Advance the receiver for missing cycles before this callback.
    for (int lost = 1; lost < period_ms; ++lost) {
      for (int j = 0; j < 7; ++j) {
        dq[j] += ddq[j] * 0.001;
        q[j] += dq[j] * 0.001;
      }
    }
    delta.fill(k < 1500 ? 0.3 : -0.1);
    const auto raw = k < 3000 ? chain.step(origin, delta, period_ms * 0.001)
                              : chain.hold(period_ms * 0.001);
    std::array<double, 7> command{};
    for (int j = 0; j < 7; ++j)
      command[j] = franka::lowpassFilter(0.001, raw[j], q[j], kFciCutoffHz);
    if (limit_rate)
      command = franka::limitRate(upper, lower, franka::kMaxJointAcceleration,
                                  franka::kMaxJointJerk, command, q, dq, ddq);
    for (int j = 0; j < 7; ++j) {
      const double next_dq = (command[j] - q[j]) / 0.001;
      const double next_ddq = (next_dq - dq[j]) / 0.001;
      worst_acceleration = std::max(worst_acceleration, std::abs(next_ddq));
      if (limit_rate) {
        requireTest(std::abs(next_dq) <= upper[j] + 1e-7, "FCI velocity bound");
        requireTest(std::abs(next_ddq) <= franka::kMaxJointAcceleration[j] + 1e-6,
                    "FCI acceleration bound after a timing gap");
        requireTest(std::abs(next_ddq - ddq[j]) / 0.001 <=
                        franka::kMaxJointJerk[j] + 0.002,
                    "FCI jerk bound after a timing gap");
      }
      q[j] = command[j];
      dq[j] = next_dq;
      ddq[j] = next_ddq;
    }
  }
  for (int j = 0; j < 7; ++j)
    requireTest(std::abs(dq[j]) < 1e-6, "foot-brake release settles to rest");
  return worst_acceleration;
}

int main() {
  try {
    // Real scheduler checks, without opening hardware. A fresh login must have
    // rtprio permission, and I/O workers must drop inherited realtime priority.
    requireTest(configureBackgroundThread(), "start scheduler test at SCHED_OTHER");
    bool refused = false;
    try { requireRealtimeScheduling(); }
    catch (const std::runtime_error&) { refused = true; }
    requireTest(refused, "FCI startup refuses ordinary scheduling");
    sched_param priority{};
    priority.sched_priority = 99;
    requireTest(sched_setscheduler(0, SCHED_FIFO, &priority) == 0,
                "rtprio 99 permission (run from a fresh login)");
    requireRealtimeScheduling();
    bool background_ok = false;
    std::thread background([&] {
      background_ok = configureBackgroundThread() && sched_getscheduler(0) == SCHED_OTHER;
    });
    background.join();
    requireTest(background_ok && sched_getscheduler(0) == SCHED_FIFO,
                "background uses SCHED_OTHER while FCI retains FIFO");
    requireTest(configureBackgroundThread(), "restore normal scheduler after test");
    const double regular = simulate(false, false);
    const double old_gap = simulate(false, true);
    const double new_gap = simulate(kFciLimitRate, true);
    std::cout << "simulation peaks: regular=" << regular << " old_gap=" << old_gap
              << " limited_gap=" << new_gap << '\n';
    requireTest(regular < 3.0 + 1e-6, "smooth source at regular 1 ms periods");
    requireTest(old_gap > 10.0, "regression exercises an unconditioned discontinuity");
    requireTest(new_gap < 10.0, "production FCI conditioning limits discontinuity");

    // Diagnostic precision must preserve sub-microradian changes. Also ensure
    // the CSV exposes acceleration and error flags absent from the SDK helper.
    franka::Record record{};
    record.state.q[3] = -2.405940123456789;
    record.state.ddq_d[3] = 1.2345678901234567;
    record.state.robot_mode = franka::RobotMode::kReflex;
    std::array<bool, 41> errors{};
    errors.fill(true);
    record.state.current_errors = franka::Errors(errors);
    std::ostringstream csv;
    writeFrankaDiagnostic(csv, {record});
    requireTest(csv.str().find("state.ddq_d[3]") != std::string::npos,
                "diagnostic contains accepted acceleration");
    requireTest(csv.str().find("-2.405940123456789") != std::string::npos,
                "diagnostic retains double precision");
    requireTest(csv.str().find("\"\"joint_motion_generator_acceleration_discontinuity\"\"")
                    != std::string::npos, "diagnostic escapes CSV error strings");
    std::cout << "FCI_CONTINUITY_OK regular=" << regular << " old_gap=" << old_gap
              << " limited_gap=" << new_gap << " rad/s^2; diagnostics=ok scheduling=ok\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
