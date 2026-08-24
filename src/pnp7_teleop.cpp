// PNP-7 lead arm -> Franka relative joint teleoperation (roadmap V1).
//
// Structure follows the existing spacemouse_teleop controller in
// ~/workspace/andyls: hard compiled ceilings, a validated key=value config, a
// hold-to-enable dead-man with prior-release required, a staleness watchdog,
// freeze-on-release, and selftest / dry / robot modes so every layer can be
// exercised without the robot.
//
// Threading, per roadmap section 12: the lead arm is sampled on its own thread
// and published through an atomic snapshot. The FCI callback only reads that
// snapshot, applies the safety chain, and returns. It performs no USB traffic,
// no allocation, no logging I/O.

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/robot.h>
#include <franka/rate_limiting.h>

#include "dynamixel_sdk.h"

#include <fcntl.h>
#include <linux/input.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr int kNumJoints = 7;
constexpr int kNumServos = 8;
constexpr int kGripperIndex = 7;

constexpr double kPi = 3.14159265358979323846;

// Dynamixel X-series control table.
constexpr int kAddrPresentVelocity = 128;
constexpr int kLenVelPos = 8;   // 128..135 = velocity(4) + position(4)
constexpr int kAddrTorqueEnable = 64;

constexpr int kTicksPerRev = 4096;
constexpr int kHalfRev = kTicksPerRev / 2;
constexpr double kTicksToRad = 2.0 * kPi / kTicksPerRev;

// A human backdriving the lead arm cannot exceed this; anything faster is a
// glitched frame and the sample is dropped rather than propagated.
constexpr double kMaxTicksPerSecond = 40000.0;

// Compiled ceilings. Config values are validated against these and can only be
// more conservative, never less.
constexpr double kCeilJointVelocity = 0.60;      // rad/s
constexpr double kCeilJointAcceleration = 3.00;  // rad/s^2
constexpr double kCeilScale = 1.00;
constexpr double kCeilSessionDelta = 1.20;       // rad
constexpr int kCeilWatchdogMs = 500;
constexpr int kFloorWatchdogMs = 20;

// Franka Panda joint limits, tightened by a margin so the safety chain stops
// short of the controller's own limit reflex.
constexpr std::array<double, kNumJoints> kQMin = {
    -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
constexpr std::array<double, kNumJoints> kQMax = {
    2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};
constexpr double kJointLimitMargin = 0.10;  // rad

std::atomic<bool> g_interrupted{false};

int64_t monotonicNs() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

std::string trim(const std::string& input) {
  const auto begin = input.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) return "";
  const auto end = input.find_last_not_of(" \t\r\n");
  return input.substr(begin, end - begin + 1);
}

int32_t wrapDelta(int32_t curr, int32_t prev) {
  int32_t d = (curr - prev + kHalfRev) % kTicksPerRev;
  if (d < 0) d += kTicksPerRev;
  return d - kHalfRev;
}

// ---------------------------------------------------------------- config ---

struct Config {
  std::string lead_port{"/dev/pnp7_lead"};
  int lead_baud{1000000};
  std::string robot_ip{"172.16.0.2"};
  std::string deadman_device;
  // Which key on that device is the dead-man. The SpaceMouse reported BTN_0;
  // the button that replaced it is a nameless HID keyboard that emits an
  // ordinary keyboard code (KEY_F3), so this can no longer be hard-coded.
  //
  // The default stays on the old hardware deliberately. The new button does
  // not claim BTN_0, so a config written before the swap fails loudly at
  // startup instead of silently assuming which button is attached.
  int deadman_key{BTN_0};
  // Claim the device exclusively (EVIOCGRAB) for as long as teleop runs.
  // Without it the same press still reaches the desktop, which has its own
  // binding for that key -- a window popping up over the robot UI mid-session.
  bool deadman_grab{true};

  std::array<int, kNumJoints> servo_id{{1, 2, 3, 4, 5, 6, 7}};
  std::array<double, kNumJoints> sign{{1, 1, 1, 1, 1, 1, 1}};
  std::array<double, kNumJoints> scale{{0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25}};
  std::array<bool, kNumJoints> enabled{{false, false, false, false, false, false, true}};

  bool gripper_enabled{false};
  int gripper_ticks_closed{649};
  int gripper_ticks_open{1355};
  double gripper_speed{0.10};
  double gripper_min_change{0.002};
  double gripper_preempt{0.004};
  bool gripper_binary{false};
  double gripper_binary_threshold{0.5};
  double gripper_force{20.0};
  // 0 means "use the hand's full range". Reducing it cuts travel time
  // proportionally, which is the dominant latency when reversing quickly.
  double gripper_open_width{0.0};

  double lowpass_hz{6.0};
  // Per joint. The Franka's own dq limits differ across the arm (2.175 rad/s
  // on J1-J4, 2.61 on J5-J7), and a human rotates a wrist far faster than a
  // shoulder, so one global cap throttles the wrist while the big joints idle.
  std::array<double, kNumJoints> max_joint_velocity{
      {0.30, 0.30, 0.30, 0.30, 0.30, 0.30, 0.30}};
  std::array<double, kNumJoints> max_joint_acceleration{
      {1.50, 1.50, 1.50, 1.50, 1.50, 1.50, 1.50}};
  double max_session_delta{0.50};
  int watchdog_ms{100};
  // Encoder counts of hysteresis applied to the lead arm before scaling.
  // 0 disables it.
  double lead_deadband{2.0};
  std::string status_path;   // empty disables publishing
};

// Accepts a decimal/hex code or a kernel key name. The names come straight
// from <linux/input-event-codes.h> via the macro, so they cannot drift out of
// step with the header the way a hand-written number table would.
int parseKeyCode(const std::string& value) {
  static const std::map<std::string, int> kNames = {
#define PNP7_KEY(name) {#name, name},
      PNP7_KEY(BTN_0) PNP7_KEY(BTN_1) PNP7_KEY(BTN_2) PNP7_KEY(BTN_3)
      PNP7_KEY(BTN_4) PNP7_KEY(BTN_5) PNP7_KEY(BTN_6) PNP7_KEY(BTN_7)
      PNP7_KEY(BTN_8) PNP7_KEY(BTN_9)
      PNP7_KEY(BTN_LEFT) PNP7_KEY(BTN_RIGHT) PNP7_KEY(BTN_MIDDLE)
      PNP7_KEY(BTN_SIDE) PNP7_KEY(BTN_EXTRA) PNP7_KEY(BTN_FORWARD)
      PNP7_KEY(BTN_BACK) PNP7_KEY(BTN_TASK)
      PNP7_KEY(KEY_A) PNP7_KEY(KEY_B) PNP7_KEY(KEY_C) PNP7_KEY(KEY_D)
      PNP7_KEY(KEY_E) PNP7_KEY(KEY_F) PNP7_KEY(KEY_G) PNP7_KEY(KEY_H)
      PNP7_KEY(KEY_I) PNP7_KEY(KEY_J) PNP7_KEY(KEY_K) PNP7_KEY(KEY_L)
      PNP7_KEY(KEY_M) PNP7_KEY(KEY_N) PNP7_KEY(KEY_O) PNP7_KEY(KEY_P)
      PNP7_KEY(KEY_Q) PNP7_KEY(KEY_R) PNP7_KEY(KEY_S) PNP7_KEY(KEY_T)
      PNP7_KEY(KEY_U) PNP7_KEY(KEY_V) PNP7_KEY(KEY_W) PNP7_KEY(KEY_X)
      PNP7_KEY(KEY_Y) PNP7_KEY(KEY_Z)
      PNP7_KEY(KEY_0) PNP7_KEY(KEY_1) PNP7_KEY(KEY_2) PNP7_KEY(KEY_3)
      PNP7_KEY(KEY_4) PNP7_KEY(KEY_5) PNP7_KEY(KEY_6) PNP7_KEY(KEY_7)
      PNP7_KEY(KEY_8) PNP7_KEY(KEY_9)
      PNP7_KEY(KEY_F1) PNP7_KEY(KEY_F2) PNP7_KEY(KEY_F3) PNP7_KEY(KEY_F4)
      PNP7_KEY(KEY_F5) PNP7_KEY(KEY_F6) PNP7_KEY(KEY_F7) PNP7_KEY(KEY_F8)
      PNP7_KEY(KEY_F9) PNP7_KEY(KEY_F10) PNP7_KEY(KEY_F11) PNP7_KEY(KEY_F12)
      PNP7_KEY(KEY_SPACE) PNP7_KEY(KEY_ENTER) PNP7_KEY(KEY_TAB)
      PNP7_KEY(KEY_ESC) PNP7_KEY(KEY_LEFTCTRL) PNP7_KEY(KEY_LEFTSHIFT)
      PNP7_KEY(KEY_LEFTALT) PNP7_KEY(KEY_LEFTMETA)
      PNP7_KEY(KEY_INSERT) PNP7_KEY(KEY_DELETE) PNP7_KEY(KEY_HOME)
      PNP7_KEY(KEY_END) PNP7_KEY(KEY_PAGEUP) PNP7_KEY(KEY_PAGEDOWN)
      PNP7_KEY(KEY_UP) PNP7_KEY(KEY_DOWN) PNP7_KEY(KEY_LEFT)
      PNP7_KEY(KEY_RIGHT)
#undef PNP7_KEY
  };
  const auto it = kNames.find(value);
  if (it != kNames.end()) return it->second;

  size_t consumed = 0;
  int code = 0;
  try {
    code = std::stoi(value, &consumed, 0);
  } catch (const std::exception&) {
    throw std::invalid_argument("deadman_key: unknown key name '" + value + "'");
  }
  if (consumed != value.size())
    throw std::invalid_argument("deadman_key: trailing junk in '" + value + "'");
  if (code < 0 || code > KEY_MAX)
    throw std::invalid_argument("deadman_key out of range");
  return code;
}

std::vector<std::string> splitWords(const std::string& text) {
  std::istringstream stream(text);
  std::vector<std::string> out;
  std::string word;
  while (stream >> word) out.push_back(word);
  return out;
}

// A limit may be given as one number for the whole arm or as seven.
void fillPerJoint(std::array<double, kNumJoints>& out, const std::string& value,
                  const char* key) {
  const auto words = splitWords(value);
  if (words.size() == 1) {
    out.fill(std::stod(words[0]));
  } else if (words.size() == kNumJoints) {
    for (int i = 0; i < kNumJoints; ++i) out[i] = std::stod(words[i]);
  } else {
    throw std::invalid_argument(std::string(key) + " needs 1 or 7 values");
  }
}

void validateConfig(const Config& c) {
  if (c.lead_baud < 9600 || c.lead_baud > 4000000)
    throw std::invalid_argument("lead_baud out of range");
  if (c.deadman_device.empty())
    throw std::invalid_argument("deadman_device must be set");
  if (c.lowpass_hz < 0.5 || c.lowpass_hz > 50.0)
    throw std::invalid_argument("lowpass_hz must be within 0.5..50");
  for (int i = 0; i < kNumJoints; ++i) {
    if (c.max_joint_velocity[i] <= 0.0 ||
        c.max_joint_velocity[i] > kCeilJointVelocity)
      throw std::invalid_argument("max_joint_velocity exceeds compiled ceiling");
    if (c.max_joint_acceleration[i] <= 0.0 ||
        c.max_joint_acceleration[i] > kCeilJointAcceleration)
      throw std::invalid_argument(
          "max_joint_acceleration exceeds compiled ceiling");
  }
  if (c.max_session_delta <= 0.0 || c.max_session_delta > kCeilSessionDelta)
    throw std::invalid_argument("max_session_delta exceeds compiled ceiling");
  if (c.watchdog_ms < kFloorWatchdogMs || c.watchdog_ms > kCeilWatchdogMs)
    throw std::invalid_argument("watchdog_ms out of range");
  if (c.lead_deadband < 0.0 || c.lead_deadband > 20.0)
    throw std::invalid_argument("lead_deadband must be within 0..20 counts");

  std::array<bool, kNumServos + 1> seen{};
  for (int i = 0; i < kNumJoints; ++i) {
    if (std::fabs(c.sign[i]) != 1.0)
      throw std::invalid_argument("sign entries must be +1 or -1");
    if (c.scale[i] < 0.0 || c.scale[i] > kCeilScale)
      throw std::invalid_argument("scale exceeds compiled ceiling");
    if (c.servo_id[i] < 1 || c.servo_id[i] > kNumServos)
      throw std::invalid_argument("servo_id out of range");
    if (seen[c.servo_id[i]])
      throw std::invalid_argument("duplicate servo_id in mapping");
    seen[c.servo_id[i]] = true;
  }
  if (std::none_of(c.enabled.begin(), c.enabled.end(), [](bool b) { return b; }))
    throw std::invalid_argument("no joints enabled");
  if (c.gripper_enabled) {
    if (c.gripper_ticks_closed == c.gripper_ticks_open)
      throw std::invalid_argument("gripper closed and open ticks are equal");
    if (c.gripper_speed <= 0.0 || c.gripper_speed > 0.20)
      throw std::invalid_argument("gripper_speed must be within 0..0.20 m/s");
    if (c.gripper_min_change < 0.0005 || c.gripper_min_change > 0.02)
      throw std::invalid_argument("gripper_min_change must be within 0.5..20 mm");
    if (c.gripper_preempt < c.gripper_min_change || c.gripper_preempt > 0.04)
      throw std::invalid_argument(
          "gripper_preempt must be >= gripper_min_change and <= 40 mm");
    if (c.gripper_binary_threshold < 0.15 || c.gripper_binary_threshold > 0.85)
      throw std::invalid_argument(
          "gripper_binary_threshold must be within 0.15..0.85");
    if (c.gripper_force <= 0.0 || c.gripper_force > 70.0)
      throw std::invalid_argument("gripper_force must be within 0..70 N");
    if (c.gripper_open_width < 0.0 || c.gripper_open_width > 0.09)
      throw std::invalid_argument("gripper_open_width must be within 0..0.09 m");
  }
}

Config loadConfig(const std::string& path) {
  std::ifstream file(path);
  if (!file) throw std::runtime_error("cannot open config: " + path);

  Config c;
  std::string line;
  while (std::getline(file, line)) {
    const auto hash = line.find('#');
    if (hash != std::string::npos) line = line.substr(0, hash);
    line = trim(line);
    if (line.empty()) continue;
    const auto eq = line.find('=');
    if (eq == std::string::npos) continue;
    const std::string key = trim(line.substr(0, eq));
    const std::string value = trim(line.substr(eq + 1));

    if (key == "lead_port") c.lead_port = value;
    else if (key == "lead_baud") c.lead_baud = std::stoi(value);
    else if (key == "robot_ip") c.robot_ip = value;
    else if (key == "deadman_device") c.deadman_device = value;
    else if (key == "deadman_key") c.deadman_key = parseKeyCode(value);
    else if (key == "deadman_grab") c.deadman_grab = std::stoi(value) != 0;
    else if (key == "status_path") c.status_path = value;
    else if (key == "lead_deadband") c.lead_deadband = std::stod(value);
    else if (key == "lowpass_hz") c.lowpass_hz = std::stod(value);
    else if (key == "max_joint_velocity")
      fillPerJoint(c.max_joint_velocity, value, "max_joint_velocity");
    else if (key == "max_joint_acceleration")
      fillPerJoint(c.max_joint_acceleration, value, "max_joint_acceleration");
    else if (key == "max_session_delta") c.max_session_delta = std::stod(value);
    else if (key == "watchdog_ms") c.watchdog_ms = std::stoi(value);
    else if (key == "gripper_enabled") c.gripper_enabled = std::stoi(value) != 0;
    else if (key == "gripper_ticks_closed")
      c.gripper_ticks_closed = std::stoi(value);
    else if (key == "gripper_ticks_open") c.gripper_ticks_open = std::stoi(value);
    else if (key == "gripper_speed") c.gripper_speed = std::stod(value);
    else if (key == "gripper_min_change")
      c.gripper_min_change = std::stod(value);
    else if (key == "gripper_preempt") c.gripper_preempt = std::stod(value);
    else if (key == "gripper_binary") c.gripper_binary = std::stoi(value) != 0;
    else if (key == "gripper_binary_threshold")
      c.gripper_binary_threshold = std::stod(value);
    else if (key == "gripper_force") c.gripper_force = std::stod(value);
    else if (key == "gripper_open_width")
      c.gripper_open_width = std::stod(value);
    else if (key == "lead_servo_id") {
      const auto words = splitWords(value);
      if (words.size() != kNumJoints)
        throw std::invalid_argument("lead_servo_id needs 7 entries");
      for (int i = 0; i < kNumJoints; ++i) c.servo_id[i] = std::stoi(words[i]);
    } else if (key == "sign") {
      const auto words = splitWords(value);
      if (words.size() != kNumJoints)
        throw std::invalid_argument("sign needs 7 entries");
      for (int i = 0; i < kNumJoints; ++i) c.sign[i] = std::stod(words[i]);
    } else if (key == "scale") {
      const auto words = splitWords(value);
      if (words.size() != kNumJoints)
        throw std::invalid_argument("scale needs 7 entries");
      for (int i = 0; i < kNumJoints; ++i) c.scale[i] = std::stod(words[i]);
    } else if (key == "enabled_joints") {
      if (value.size() != kNumJoints)
        throw std::invalid_argument("enabled_joints needs 7 characters");
      for (int i = 0; i < kNumJoints; ++i) c.enabled[i] = value[i] == '1';
    }
  }
  validateConfig(c);
  return c;
}

// ------------------------------------------------------------- lead arm ---

struct LeadSnapshot {
  int64_t t_ns{0};
  int64_t seq{0};
  std::array<int32_t, kNumServos> ticks{};
};

// Sampled on its own thread; the FCI callback only reads `snapshot()`.
class LeadArmReader {
 public:
  explicit LeadArmReader(const Config& config) : config_(config) {}

  void open() {
    port_ = dynamixel::PortHandler::getPortHandler(config_.lead_port.c_str());
    packet_ = dynamixel::PacketHandler::getPacketHandler(2.0);
    if (!port_->openPort())
      throw std::runtime_error("cannot open " + config_.lead_port);
    if (!port_->setBaudRate(config_.lead_baud))
      throw std::runtime_error("cannot set lead arm baud rate");
    sync_ = new dynamixel::GroupSyncRead(port_, packet_, kAddrPresentVelocity,
                                         kLenVelPos);
    for (int id = 1; id <= kNumServos; ++id) {
      if (!sync_->addParam(static_cast<uint8_t>(id)))
        throw std::runtime_error("syncread addParam failed");
    }
  }

  // The lead arm is an input device. If any servo has torque on it can fight
  // the operator, so refuse to run.
  void assertTorqueDisabled() {
    for (int id = 1; id <= kNumServos; ++id) {
      uint8_t value = 0;
      uint8_t error = 0;
      const int comm = packet_->read1ByteTxRx(port_, static_cast<uint8_t>(id),
                                              kAddrTorqueEnable, &value, &error);
      if (comm != COMM_SUCCESS || error != 0)
        throw std::runtime_error("cannot read torque state of servo " +
                                 std::to_string(id));
      if (value != 0)
        throw std::runtime_error("torque is ENABLED on servo " +
                                 std::to_string(id) + "; lead arm must be passive");
    }
  }

  bool readOnce() {
    if (sync_->txRxPacket() != COMM_SUCCESS) {
      ++read_failures_;
      return false;
    }
    const int64_t t_ns = monotonicNs();
    std::array<int32_t, kNumServos> raw{};
    for (int i = 0; i < kNumServos; ++i) {
      const uint8_t id = static_cast<uint8_t>(i + 1);
      if (!sync_->isAvailable(id, kAddrPresentVelocity, kLenVelPos)) {
        ++read_failures_;
        return false;
      }
      raw[i] = static_cast<int32_t>(
          sync_->getData(id, kAddrPresentVelocity + 4, 4));
    }

    if (!have_last_) {
      cont_ = raw;
      have_last_ = true;
    } else {
      const double dt = std::max((t_ns - last_t_ns_) / 1e9, 1e-4);
      std::array<int32_t, kNumServos> deltas{};
      for (int i = 0; i < kNumServos; ++i) {
        deltas[i] = wrapDelta(raw[i], last_raw_[i]);
        if (std::fabs(deltas[i]) / dt > kMaxTicksPerSecond) {
          ++rejected_jumps_;
          last_raw_ = raw;
          last_t_ns_ = t_ns;
          return false;
        }
      }
      for (int i = 0; i < kNumServos; ++i) cont_[i] += deltas[i];
    }

    last_raw_ = raw;
    last_t_ns_ = t_ns;

    LeadSnapshot snap;
    snap.t_ns = t_ns;
    snap.seq = ++seq_;
    snap.ticks = cont_;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_ = snap;
    }
    last_good_ns_.store(t_ns);
    return true;
  }

  void start() {
    running_.store(true);
    thread_ = std::thread([this] {
      while (running_.load()) readOnce();
    });
  }

  void stop() {
    running_.store(false);
    if (thread_.joinable()) thread_.join();
  }

  LeadSnapshot snapshot() {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_;
  }

  int64_t lastGoodNs() const { return last_good_ns_.load(); }
  int64_t readFailures() const { return read_failures_; }
  int64_t rejectedJumps() const { return rejected_jumps_; }

  ~LeadArmReader() {
    stop();
    delete sync_;
    if (port_) port_->closePort();
  }

 private:
  Config config_;
  dynamixel::PortHandler* port_{nullptr};
  dynamixel::PacketHandler* packet_{nullptr};
  dynamixel::GroupSyncRead* sync_{nullptr};

  std::array<int32_t, kNumServos> last_raw_{};
  std::array<int32_t, kNumServos> cont_{};
  bool have_last_{false};
  int64_t last_t_ns_{0};
  int64_t seq_{0};

  std::atomic<int64_t> last_good_ns_{0};
  int64_t read_failures_{0};
  int64_t rejected_jumps_{0};

  std::mutex mutex_;
  LeadSnapshot latest_;
  std::atomic<bool> running_{false};
  std::thread thread_;
};

// -------------------------------------------------------------- deadman ---

bool testBit(const unsigned long* bits, int bit) {
  return (bits[bit / (8 * sizeof(unsigned long))] >>
          (bit % (8 * sizeof(unsigned long)))) & 1UL;
}

// Hold-to-enable on one key of a dedicated input device. A press is only
// honoured after a release has been seen, so a button already held at startup
// cannot enable motion.
//
// The key is configurable because the device changed: the SpaceMouse reported
// BTN_0, while the button that replaced it enumerates as a nameless HID
// keyboard (0483:5750) and emits an ordinary keyboard code.
class DeadmanReader {
 public:
  DeadmanReader(std::string device, int key_code, bool grab)
      : device_(std::move(device)), key_code_(key_code), grab_(grab) {}

  void open() {
    fd_ = ::open(device_.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd_ < 0)
      throw std::runtime_error("cannot open deadman device " + device_);

    unsigned long bits[(KEY_MAX + 8 * sizeof(unsigned long)) /
                       (8 * sizeof(unsigned long))]{};
    // Fail loudly on a device that cannot produce the configured key at all.
    // Getting this wrong is otherwise silent: the dead-man simply never fires
    // and the arm never moves, which reads like a lead-arm fault.
    if (ioctl(fd_, EVIOCGBIT(EV_KEY, sizeof(bits)), bits) < 0 ||
        !testBit(bits, key_code_)) {
      ::close(fd_);
      fd_ = -1;
      throw std::runtime_error(device_ + " does not report key code " +
                               std::to_string(key_code_) +
                               " -- wrong device or wrong deadman_key");
    }

    if (grab_) {
      // EVIOCGRAB routes the device to this process alone. Without it the
      // desktop still sees the press and acts on its own binding for that key.
      // The kernel drops the grab when the fd closes, including on a crash, so
      // there is no way to leave the button captured.
      if (ioctl(fd_, EVIOCGRAB, 1) < 0) {
        const int err = errno;
        ::close(fd_);
        fd_ = -1;
        throw std::runtime_error("cannot grab deadman device " + device_ +
                                 ": " + std::strerror(err) +
                                 " -- another process already holds it");
      }
      grabbed_ = true;
    }

    std::memset(bits, 0, sizeof(bits));
    if (ioctl(fd_, EVIOCGKEY(sizeof(bits)), bits) >= 0 &&
        testBit(bits, key_code_)) {
      // Held at startup: require a full release first.
      awaiting_release_.store(true);
    }
  }

  void poll() {
    input_event event{};
    for (;;) {
      const ssize_t n = ::read(fd_, &event, sizeof(event));
      if (n == static_cast<ssize_t>(sizeof(event))) {
        if (event.type != EV_KEY || event.code != key_code_) continue;
        // value 2 is auto-repeat, which a held keyboard key emits and the
        // SpaceMouse never did. It must not clear awaiting_release_, so only
        // 1 and 0 are acted on.
        if (event.value == 1) {
          if (!awaiting_release_.load()) pressed_.store(true);
        } else if (event.value == 0) {
          pressed_.store(false);
          awaiting_release_.store(false);
        }
        continue;
      }
      if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR))
        return;                    // nothing more queued this cycle
      // Anything else means the fd is dead -- ENODEV when the button is
      // unplugged mid-session. The kernel does release held keys on removal,
      // but if that release were ever missed the last value read would be a
      // press that nothing can ever clear, and the arm would stay enabled by a
      // button that is no longer attached. Latch it shut instead.
      lost_.store(true);
      pressed_.store(false);
      return;
    }
  }

  void start() {
    running_.store(true);
    thread_ = std::thread([this] {
      while (running_.load()) {
        poll();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    });
  }

  void stop() {
    running_.store(false);
    if (thread_.joinable()) thread_.join();
    pressed_.store(false);
  }

  bool pressed() const { return pressed_.load() && !lost_.load(); }

  // True once the device stopped being readable -- unplugged, or the hub it
  // hangs off went away. Latched: it never clears without a restart.
  bool lost() const { return lost_.load(); }

  ~DeadmanReader() {
    stop();
    if (fd_ >= 0) {
      if (grabbed_) ioctl(fd_, EVIOCGRAB, 0);
      ::close(fd_);
    }
  }

 private:
  std::string device_;
  int key_code_{BTN_0};
  bool grab_{true};
  bool grabbed_{false};
  int fd_{-1};
  std::atomic<bool> pressed_{false};
  std::atomic<bool> awaiting_release_{false};
  std::atomic<bool> lost_{false};
  std::atomic<bool> running_{false};
  std::thread thread_;
};

// ------------------------------------------------------------ safety ------

enum class State { kReady, kTeleop, kPaused };

const char* stateName(State s) {
  switch (s) {
    case State::kReady: return "READY";
    case State::kTeleop: return "TELEOP";
    case State::kPaused: return "PAUSED";
  }
  return "?";
}

// Applies, in order: relative mapping, session clamp, low-pass, velocity limit,
// acceleration limit, joint-limit clamp. Each stage is intentionally separate
// so a single one can be tested in isolation.
class SafetyChain {
 public:
  explicit SafetyChain(const Config& config) : c_(config) {}

  void seed(const std::array<double, kNumJoints>& q) {
    filtered_ = q;
    target_ = q;
    prev_target_ = q;
    prev_velocity_.fill(0.0);
  }

  std::array<double, kNumJoints> step(
      const std::array<double, kNumJoints>& q_origin,
      const std::array<double, kNumJoints>& delta_lead, double dt) {
    std::array<double, kNumJoints> desired = target_;

    for (int i = 0; i < kNumJoints; ++i) {
      if (!c_.enabled[i]) {
        desired[i] = q_origin[i];
        continue;
      }
      double d = c_.sign[i] * c_.scale[i] * delta_lead[i];
      d = std::clamp(d, -c_.max_session_delta, c_.max_session_delta);
      desired[i] = q_origin[i] + d;
    }

    // Joint limits are enforced HERE, on the desired position, before any rate
    // limiting. Clamping the output instead would let the clamp emit a step of
    // arbitrary size that bypassed the velocity and acceleration limits -- and
    // a position discontinuity is exactly what makes the controller fault.
    // Clamping the goal means an out-of-envelope request is approached at a
    // bounded rate rather than jumped to.
    for (int i = 0; i < kNumJoints; ++i) {
      desired[i] = std::clamp(desired[i], kQMin[i] + kJointLimitMargin,
                              kQMax[i] - kJointLimitMargin);
    }

    const double alpha =
        1.0 - std::exp(-2.0 * kPi * c_.lowpass_hz * std::max(dt, 1e-6));
    for (int i = 0; i < kNumJoints; ++i) {
      filtered_[i] += alpha * (desired[i] - filtered_[i]);
      // Keep the filter state inside the envelope too, so it cannot wind up.
      filtered_[i] = std::clamp(filtered_[i], kQMin[i] + kJointLimitMargin,
                                kQMax[i] - kJointLimitMargin);
    }

    std::array<double, kNumJoints> out{};
    for (int i = 0; i < kNumJoints; ++i) {
      const double remaining = filtered_[i] - prev_target_[i];

      // Cap speed by the distance still available to brake in. Without this the
      // joint saturates at max_joint_velocity and then cannot decelerate inside
      // the acceleration limit, so it sails past the target and rings around it.
      //
      // The continuous form sqrt(2*a*d) is only marginally feasible: at exactly
      // that speed the cap shrinks at rate a, the same rate the limiter can
      // decelerate, so discretisation pushes it slightly past. Subtracting one
      // step of braking gives the discrete-exact bound.
      const double a_dt = c_.max_joint_acceleration[i] * dt;
      const double v_brake =
          std::sqrt(2.0 * c_.max_joint_acceleration[i] * std::fabs(remaining) +
                    a_dt * a_dt) -
          a_dt;
      const double v_cap = std::min(c_.max_joint_velocity[i], v_brake);

      double v = std::clamp(remaining / dt, -v_cap, v_cap);

      const double a = (v - prev_velocity_[i]) / dt;
      const double a_clamped = std::clamp(a, -c_.max_joint_acceleration[i],
                                          c_.max_joint_acceleration[i]);
      v = prev_velocity_[i] + a_clamped * dt;

      out[i] = prev_target_[i] + v * dt;
      prev_velocity_[i] = v;
      prev_target_[i] = out[i];
    }
    target_ = out;
    return out;
  }

  // Deadman released, watchdog fired, or lead arm unhealthy: stop moving but
  // keep returning a valid command so the controller stays connected.
  std::array<double, kNumJoints> hold(double dt) {
    for (int i = 0; i < kNumJoints; ++i) {
      double v = prev_velocity_[i];
      const double a = std::clamp(-v / std::max(dt, 1e-6),
                                  -c_.max_joint_acceleration[i],
                                  c_.max_joint_acceleration[i]);
      v = v + a * dt;
      if (std::fabs(v) < 1e-6) v = 0.0;
      prev_target_[i] += v * dt;
      prev_velocity_[i] = v;
    }
    filtered_ = prev_target_;
    target_ = prev_target_;
    return target_;
  }

  const std::array<double, kNumJoints>& target() const { return target_; }

  // libfranka rejects a motion that finishes while still moving, so a session
  // may only be ended once every joint has actually come to rest.
  bool isStopped() const {
    for (double v : prev_velocity_)
      if (std::fabs(v) > 1e-6) return false;
    return true;
  }

 private:
  Config c_;
  std::array<double, kNumJoints> filtered_{};
  std::array<double, kNumJoints> target_{};
  std::array<double, kNumJoints> prev_target_{};
  std::array<double, kNumJoints> prev_velocity_{};
};

// Hysteresis (backlash) operator on the raw lead counts.
//
// A servo resting exactly on a quantisation boundary flips between two adjacent
// counts indefinitely -- measured at 134 changes/second on this arm. At scale
// 1.0 one count is 1.5 mrad, so that becomes a ~67 Hz command dither, which the
// Franka's stiff position controller chases and turns into audible buzz while
// the operator is holding perfectly still.
//
// The low-pass alone cannot fix it: first-order rolloff only attenuates ~21 dB
// there, and its output still changes every cycle. This does, by refusing to
// move the held value until the input has travelled more than the band -- after
// which it tracks continuously, offset by the band. No steps, no dead zone in
// the middle of a motion, and single-count chatter simply never propagates.
class LeadDeadband {
 public:
  void apply(std::array<double, kNumServos>& ticks, double band) {
    if (band <= 0.0) return;
    if (!seeded_) {
      held_ = ticks;
      seeded_ = true;
      return;
    }
    for (int i = 0; i < kNumServos; ++i) {
      if (ticks[i] > held_[i] + band) {
        held_[i] = ticks[i] - band;
      } else if (ticks[i] < held_[i] - band) {
        held_[i] = ticks[i] + band;
      }
      ticks[i] = held_[i];
    }
  }

  void reset() { seeded_ = false; }

 private:
  std::array<double, kNumServos> held_{};
  bool seeded_{false};
};

std::array<double, kNumJoints> leadDelta(const LeadSnapshot& now,
                                         const LeadSnapshot& origin,
                                         const Config& c) {
  std::array<double, kNumJoints> out{};
  for (int i = 0; i < kNumJoints; ++i) {
    const int idx = c.servo_id[i] - 1;
    out[i] = (now.ticks[idx] - origin.ticks[idx]) * kTicksToRad;
  }
  return out;
}

// -------------------------------------------------------------- status ----

// Publishes live state for an external viewer. The FCI callback only performs
// atomic stores here -- no allocation, no formatting, no I/O. A separate thread
// does the JSON writing at a human rate.
class StatusPublisher {
 public:
  explicit StatusPublisher(std::string path) : path_(std::move(path)) {}

  void update(int state, bool deadman, const std::array<double, kNumJoints>& q,
              const std::array<double, kNumJoints>& target, double grip_w,
              double grip_t, int64_t lead_seq, double lead_age_ms) {
    state_.store(state);
    deadman_.store(deadman);
    for (int i = 0; i < kNumJoints; ++i) {
      q_[i].store(q[i]);
      target_[i].store(target[i]);
    }
    grip_w_.store(grip_w);
    grip_t_.store(grip_t);
    lead_seq_.store(lead_seq);
    lead_age_ms_.store(lead_age_ms);
  }

  void start() {
    if (path_.empty()) return;
    running_.store(true);
    thread_ = std::thread([this] {
      while (running_.load()) {
        std::ostringstream out;
        out << std::fixed << std::setprecision(4);
        out << "{\"state\":" << state_.load()
            << ",\"deadman\":" << (deadman_.load() ? 1 : 0)
            << ",\"lead_seq\":" << lead_seq_.load()
            << ",\"lead_age_ms\":" << lead_age_ms_.load()
            << ",\"gripper_width\":" << grip_w_.load()
            << ",\"gripper_target\":" << grip_t_.load()
            << ",\"q\":[";
        for (int i = 0; i < kNumJoints; ++i)
          out << (i ? "," : "") << q_[i].load();
        out << "],\"q_target\":[";
        for (int i = 0; i < kNumJoints; ++i)
          out << (i ? "," : "") << target_[i].load();
        out << "]}";

        // Write then rename, so a reader never sees a half-written file.
        const std::string tmp = path_ + ".tmp";
        {
          std::ofstream fh(tmp);
          fh << out.str();
        }
        std::rename(tmp.c_str(), path_.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      std::remove(path_.c_str());
    });
  }

  void stop() {
    running_.store(false);
    if (thread_.joinable()) thread_.join();
  }

  ~StatusPublisher() { stop(); }

 private:
  std::string path_;
  std::atomic<int> state_{0};
  std::atomic<bool> deadman_{false};
  std::array<std::atomic<double>, kNumJoints> q_{};
  std::array<std::atomic<double>, kNumJoints> target_{};
  std::atomic<double> grip_w_{-1.0};
  std::atomic<double> grip_t_{-1.0};
  std::atomic<int64_t> lead_seq_{0};
  std::atomic<double> lead_age_ms_{0.0};
  std::atomic<bool> running_{false};
  std::thread thread_;
};

// ------------------------------------------------------------- gripper ----

// The Franka Hand is commanded from its own thread at a low rate. franka::
// Gripper::move blocks until the motion completes, so it can never be called
// from the 1 kHz callback; and per roadmap section 7 the hand does not need
// servo-rate updates -- a new command is only issued once the operator's
// trigger has moved past a deadband.
class GripperController {
 public:
  GripperController(const std::string& ip, const Config& config)
      : config_(config), gripper_(ip) {}

  void connect() {
    const franka::GripperState state = gripper_.readOnce();
    max_width_ = state.max_width;
    width_.store(state.width);
    commanded_.store(state.width);
    // Seed the target from the MEASURED width. Left at its default of 0.0 this
    // reads as "fully closed", and the thread would slam the hand shut the
    // moment it starts -- before the operator has touched anything.
    open_width_ = config_.gripper_open_width > 0.0
                      ? std::min(config_.gripper_open_width, max_width_)
                      : max_width_;
    target_.store(state.width);
    std::cout << "gripper connected: width=" << state.width
              << " max=" << max_width_ << " open_to=" << open_width_
              << " mode=" << (config_.gripper_binary ? "binary" : "continuous")
              << "\n";
  }

  // Map the lead-arm trigger position onto a hand opening.
  double widthFromTicks(int32_t ticks) const {
    const double span =
        static_cast<double>(config_.gripper_ticks_open - config_.gripper_ticks_closed);
    if (std::fabs(span) < 1.0) return 0.0;
    const double frac =
        (static_cast<double>(ticks) - config_.gripper_ticks_closed) / span;
    return std::clamp(frac, 0.0, 1.0) * open_width_;
  }

  double fractionFromTicks(int32_t ticks) const {
    const double span =
        static_cast<double>(config_.gripper_ticks_open - config_.gripper_ticks_closed);
    if (std::fabs(span) < 1.0) return 1.0;
    return std::clamp(
        (static_cast<double>(ticks) - config_.gripper_ticks_closed) / span,
        0.0, 1.0);
  }

  // Called only while teleop is engaged. The first call also arms the thread:
  // until the operator has taken control the hand must not be commanded at all.
  void setTargetTicks(int32_t ticks) {
    if (config_.gripper_binary) {
      // Hysteresis so a trigger resting near the threshold cannot chatter.
      const double f = fractionFromTicks(ticks);
      bool want_open = want_open_.load();
      if (f > config_.gripper_binary_threshold + 0.08) want_open = true;
      if (f < config_.gripper_binary_threshold - 0.08) want_open = false;
      want_open_.store(want_open);
      target_.store(want_open ? open_width_ : 0.0);
    } else {
      target_.store(widthFromTicks(ticks));
    }
    armed_.store(true);
  }

  double target() const { return target_.load(); }
  double width() const { return width_.load(); }
  double maxWidth() const { return max_width_; }
  int64_t commands() const { return commands_.load(); }
  int64_t preemptions() const { return preemptions_.load(); }
  int64_t errors() const { return errors_.load(); }

  void start() {
    running_.store(true);

    // Mover: owns the blocking call. franka::Gripper::move does not return
    // until the hand has finished travelling, so nothing else can live here.
    move_thread_ = std::thread([this] {
      while (running_.load()) {
        const double want = target_.load();
        const bool changed =
            std::fabs(want - commanded_.load()) >= config_.gripper_min_change;
        if (armed_.load() && changed) {
          commanded_.store(want);
          preempted_.store(false);
          move_active_.store(true);
          ++commands_;
          try {
            if (config_.gripper_binary && want <= 0.0) {
              // Closing on an object: a plain move would stall against it and
              // report failure, so grasp with a force target and an epsilon
              // wide enough to accept any object width.
              gripper_.grasp(0.0, config_.gripper_speed, config_.gripper_force,
                             max_width_, max_width_);
            } else {
              gripper_.move(want, config_.gripper_speed);
            }
          } catch (const franka::Exception&) {
            // An aborted move throws; that is the expected result of a
            // preemption and must not end the arm session.
            ++errors_;
          }
          move_active_.store(false);
        } else {
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
      }
    });

    // Preempt watcher. This does NO network I/O -- it only compares two
    // atomics -- so it must not share a thread with readOnce(), which is a
    // blocking UDP receive gated by the hand's ~5 Hz state publishing. That
    // mistake is what made the first attempt useless: the check ran a few
    // times a second, long after the stale move had already finished.
    preempt_thread_ = std::thread([this] {
      while (running_.load()) {
        if (move_active_.load() && !preempted_.load() &&
            std::fabs(target_.load() - commanded_.load()) >=
                config_.gripper_preempt) {
          preempted_.store(true);
          ++preemptions_;
          try {
            gripper_.stop();
          } catch (const franka::Exception&) {
            ++errors_;
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    });

    // State reader, kept apart because readOnce() blocks on the next datagram.
    state_thread_ = std::thread([this] {
      while (running_.load()) {
        try {
          width_.store(gripper_.readOnce().width);
        } catch (const franka::Exception&) {
          ++errors_;
        }
      }
    });
  }

  void stop() {
    running_.store(false);
    if (move_active_.load()) {
      try {
        gripper_.stop();
      } catch (const franka::Exception&) {
      }
    }
    if (move_thread_.joinable()) move_thread_.join();
    if (preempt_thread_.joinable()) preempt_thread_.join();
    if (state_thread_.joinable()) state_thread_.detach();
  }

  ~GripperController() { stop(); }

 private:
  Config config_;
  franka::Gripper gripper_;
  double max_width_{0.08};
  double open_width_{0.08};
  std::atomic<double> target_{0.0};
  std::atomic<double> width_{0.0};
  std::atomic<double> commanded_{0.0};
  std::atomic<bool> want_open_{true};
  std::atomic<bool> armed_{false};
  std::atomic<bool> move_active_{false};
  std::atomic<bool> preempted_{false};
  std::atomic<int64_t> commands_{0};
  std::atomic<int64_t> preemptions_{0};
  std::atomic<int64_t> errors_{0};
  std::atomic<bool> running_{false};
  std::thread move_thread_;
  std::thread preempt_thread_;
  std::thread state_thread_;
};

// ---------------------------------------------------------------- log -----

struct LogRow {
  int64_t t_ns;
  double dt_s;      // libfranka's period, authoritative -- t_ns jitters
  int64_t lead_seq;
  int state;
  int deadman;
  std::array<double, kNumJoints> q_robot;
  std::array<double, kNumJoints> q_target;
  std::array<double, kNumJoints> lead_delta;
  int32_t gripper_ticks;
  // Observation side, per roadmap section 13.
  std::array<double, kNumJoints> dq_robot;
  std::array<double, kNumJoints> tau_robot;
  std::array<double, 16> O_T_EE;
  std::array<double, 6> O_F_ext;   // external wrench estimate, base frame
  double gripper_width;
  double gripper_target;
};

void writeLog(const std::string& path, const std::vector<LogRow>& rows,
              size_t count) {
  if (path.empty()) return;
  std::ofstream out(path);
  out << "t_ns,dt_s,lead_seq,state,deadman";
  for (int i = 0; i < kNumJoints; ++i) out << ",q_robot" << i;
  for (int i = 0; i < kNumJoints; ++i) out << ",q_target" << i;
  for (int i = 0; i < kNumJoints; ++i) out << ",lead_delta" << i;
  for (int i = 0; i < kNumJoints; ++i) out << ",dq_robot" << i;
  for (int i = 0; i < kNumJoints; ++i) out << ",tau_robot" << i;
  for (int i = 0; i < 16; ++i) out << ",O_T_EE" << i;
  for (int i = 0; i < 6; ++i) out << ",O_F_ext" << i;
  out << ",gripper_ticks,gripper_width,gripper_target\n";
  out << std::setprecision(10);
  for (size_t r = 0; r < count; ++r) {
    const LogRow& row = rows[r];
    out << row.t_ns << "," << row.dt_s << "," << row.lead_seq << ","
        << row.state << "," << row.deadman;
    for (int i = 0; i < kNumJoints; ++i) out << "," << row.q_robot[i];
    for (int i = 0; i < kNumJoints; ++i) out << "," << row.q_target[i];
    for (int i = 0; i < kNumJoints; ++i) out << "," << row.lead_delta[i];
    for (int i = 0; i < kNumJoints; ++i) out << "," << row.dq_robot[i];
    for (int i = 0; i < kNumJoints; ++i) out << "," << row.tau_robot[i];
    for (int i = 0; i < 16; ++i) out << "," << row.O_T_EE[i];
    for (int i = 0; i < 6; ++i) out << "," << row.O_F_ext[i];
    out << "," << row.gripper_ticks << "," << row.gripper_width << ","
        << row.gripper_target << "\n";
  }
  std::cout << "log written: " << path << " (" << count << " rows)\n";
}

// --------------------------------------------------------------- modes ----

void requireTest(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error("selftest failed: " + message);
}

int runSelfTest(const Config& config) {
  // wrap handling
  requireTest(wrapDelta(0, 4095) == 1, "wrap 4095->0");
  requireTest(wrapDelta(4095, 0) == -1, "wrap 0->4095");
  requireTest(wrapDelta(-3, 5) == -8, "signed span");
  requireTest(wrapDelta(100, 50) == 50, "plain delta");

  // velocity limiting
  Config c = config;
  c.enabled.fill(true);
  c.lowpass_hz = 50.0;
  SafetyChain chain(c);
  // A reachable Panda pose. Zeros are NOT valid: J4 is limited to
  // [-3.0718, -0.0698] and can never be zero.
  const std::array<double, kNumJoints> q0 = {0.0, -0.4, 0.0, -2.2, 0.0, 1.9, 0.8};
  chain.seed(q0);
  std::array<double, kNumJoints> big{};
  big.fill(2.0);
  const double dt = 0.001;
  auto out = chain.step(q0, big, dt);
  for (int i = 0; i < kNumJoints; ++i) {
    requireTest(std::fabs(out[i] - q0[i]) <= c.max_joint_velocity[i] * dt * 1.5,
                "velocity limit on first step");
  }

  // acceleration limiting: velocity cannot jump. Seed v_prev with the velocity
  // the first step already reached, not zero.
  double v_prev = (out[0] - q0[0]) / dt;
  for (int k = 0; k < 5; ++k) {
    auto step = chain.step(q0, big, dt);
    const double v = (step[0] - out[0]) / dt;
    requireTest(std::fabs(v - v_prev) <= c.max_joint_acceleration[0] * dt * 1.5,
                "acceleration limit");
    v_prev = v;
    out = step;
  }

  // session clamp
  Config c2 = c;
  c2.max_session_delta = 0.05;
  c2.scale.fill(1.0);
  c2.sign.fill(1.0);
  SafetyChain chain2(c2);
  chain2.seed(q0);
  for (int k = 0; k < 5000; ++k) chain2.step(q0, big, dt);
  for (int i = 0; i < kNumJoints; ++i) {
    requireTest(std::fabs(chain2.target()[i] - q0[i]) <=
                    c2.max_session_delta + 1e-6,
                "session delta clamp");
  }

  // hold decays velocity to zero and freezes
  SafetyChain chain3(c);
  chain3.seed(q0);
  for (int k = 0; k < 50; ++k) chain3.step(q0, big, dt);
  for (int k = 0; k < 2000; ++k) chain3.hold(dt);
  const auto frozen = chain3.target();
  for (int k = 0; k < 100; ++k) chain3.hold(dt);
  for (int i = 0; i < kNumJoints; ++i)
    requireTest(std::fabs(chain3.target()[i] - frozen[i]) < 1e-9,
                "hold freezes target");

  // disabled joints never leave the origin
  Config c4 = c;
  c4.enabled.fill(false);
  c4.enabled[6] = true;
  SafetyChain chain4(c4);
  chain4.seed(q0);
  for (int k = 0; k < 500; ++k) chain4.step(q0, big, dt);
  for (int i = 0; i < 6; ++i)
    requireTest(std::fabs(chain4.target()[i] - q0[i]) < 1e-9,
                "disabled joint held");
  requireTest(std::fabs(chain4.target()[6] - q0[6]) > 1e-6,
              "enabled joint moved");

  // joint limits respected
  Config c5 = c;
  c5.max_session_delta = kCeilSessionDelta;
  SafetyChain chain5(c5);
  std::array<double, kNumJoints> near_limit{};
  for (int i = 0; i < kNumJoints; ++i) near_limit[i] = kQMax[i] - 0.05;
  chain5.seed(near_limit);
  for (int k = 0; k < 5000; ++k) chain5.step(near_limit, big, dt);
  for (int i = 0; i < kNumJoints; ++i)
    requireTest(chain5.target()[i] <= kQMax[i] - kJointLimitMargin + 1e-6,
                "joint upper limit");

  // Hysteresis deadband: single-count chatter must not propagate, but real
  // motion must still track continuously once it leaves the band.
  {
    LeadDeadband db;
    std::array<double, kNumServos> t{};
    t.fill(1000.0);
    db.apply(t, 2.0);                       // seeds
    const double seeded = t[0];

    for (int k = 0; k < 200; ++k) {         // chatter +/-1 count
      std::array<double, kNumServos> c{};
      c.fill(1000.0 + (k % 2));
      db.apply(c, 2.0);
      requireTest(std::fabs(c[0] - seeded) < 1e-9,
                  "deadband suppresses single-count chatter");
    }

    std::array<double, kNumServos> far{};   // move well past the band
    far.fill(1010.0);
    db.apply(far, 2.0);
    requireTest(std::fabs(far[0] - 1008.0) < 1e-9,
                "deadband tracks with a one-band offset");

    double prev = far[0];                   // and tracks smoothly after that
    for (int k = 1; k <= 20; ++k) {
      std::array<double, kNumServos> c{};
      c.fill(1010.0 + k);
      db.apply(c, 2.0);
      requireTest(std::fabs(c[0] - prev - 1.0) < 1e-9,
                  "deadband tracks continuously outside the band");
      prev = c[0];
    }

    LeadDeadband off;                       // disabled must be a pass-through
    std::array<double, kNumServos> raw{};
    raw.fill(1234.0);
    off.apply(raw, 0.0);
    requireTest(std::fabs(raw[0] - 1234.0) < 1e-9, "deadband 0 is transparent");
  }

  // A session may only end at rest.
  SafetyChain chain8(c);
  chain8.seed(q0);
  for (int k = 0; k < 50; ++k) chain8.step(q0, big, dt);
  requireTest(!chain8.isStopped(), "moving chain is not reported stopped");
  for (int k = 0; k < 5000; ++k) chain8.hold(dt);
  requireTest(chain8.isStopped(), "hold brings the chain to rest");

  // Regression: approaching a fixed target must not overshoot it.
  Config c7 = c;
  c7.max_session_delta = 0.05;
  c7.scale.fill(1.0);
  c7.sign.fill(1.0);
  SafetyChain chain7(c7);
  chain7.seed(q0);
  double worst_overshoot = 0.0;
  for (int k = 0; k < 5000; ++k) {
    const auto step = chain7.step(q0, big, dt);
    for (int i = 0; i < kNumJoints; ++i) {
      worst_overshoot =
          std::max(worst_overshoot, (step[i] - q0[i]) - c7.max_session_delta);
    }
  }
  {
    std::ostringstream msg;
    msg << "no overshoot past the session target (worst=" << std::scientific
        << worst_overshoot << " rad)";
    requireTest(worst_overshoot <= 1e-5, msg.str());
  }

  // Regression: a start pose outside the joint envelope must be corrected by
  // bounded motion, never by a single clamped step.
  Config c6 = c;
  SafetyChain chain6(c6);
  std::array<double, kNumJoints> outside = q0;
  outside[3] = 0.0;  // above J4's upper limit of -0.0698
  chain6.seed(outside);
  auto prev = outside;
  for (int k = 0; k < 5000; ++k) {
    const auto step = chain6.step(outside, big, dt);
    requireTest(std::fabs(step[3] - prev[3]) <=
                    c6.max_joint_velocity[3] * dt * 1.5,
                "no jump when seeded outside joint limits");
    prev = step;
  }
  requireTest(chain6.target()[3] <= kQMax[3] - kJointLimitMargin + 1e-6,
              "recovers into the envelope");

  std::cout << "SELFTEST_OK wrap=true velocity=true acceleration=true "
               "session_clamp=true hold_freeze=true joint_mask=true "
               "joint_limits=true no_clamp_jump=true no_overshoot=true "
               "stop_at_rest=true deadband=true\n";
  return 0;
}

int runDry(const Config& config, double duration_s, const std::string& log_path) {
  LeadArmReader lead(config);
  lead.open();
  lead.assertTorqueDisabled();
  DeadmanReader deadman(config.deadman_device, config.deadman_key,
                        config.deadman_grab);
  deadman.open();

  lead.start();
  deadman.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Stand in for the robot: start from a plausible Panda rest pose.
  std::array<double, kNumJoints> q_robot = {0.0, -0.4, 0.0, -2.2, 0.0, 1.9, 0.8};
  SafetyChain chain(config);
  chain.seed(q_robot);
  LeadDeadband deadband;

  State state = State::kReady;
  LeadSnapshot origin{};
  std::array<double, kNumJoints> q_origin = q_robot;

  std::vector<LogRow> rows(static_cast<size_t>(duration_s * 1100) + 1000);
  size_t count = 0;

  const int64_t watchdog_ns = static_cast<int64_t>(config.watchdog_ms) * 1000000LL;
  const int64_t t_start = monotonicNs();
  int64_t last = t_start;

  std::cout << "DRY_RUN_READY (no robot connection)\n";
  while (!g_interrupted.load() &&
         (monotonicNs() - t_start) / 1e9 < duration_s) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const int64_t now = monotonicNs();
    const double dt = std::max((now - last) / 1e9, 1e-6);
    last = now;

    LeadSnapshot snap = lead.snapshot();
    {
      std::array<double, kNumServos> t{};
      for (int i = 0; i < kNumServos; ++i) t[i] = snap.ticks[i];
      deadband.apply(t, config.lead_deadband);
      for (int i = 0; i < kNumServos; ++i)
        snap.ticks[i] = static_cast<int32_t>(std::llround(t[i]));
    }
    const bool fresh = snap.seq > 0 && (now - lead.lastGoodNs()) < watchdog_ns;
    const bool enable = deadman.pressed() && fresh && !g_interrupted.load();

    std::array<double, kNumJoints> delta{};
    if (enable) {
      if (state != State::kTeleop) {
        origin = snap;
        q_origin = chain.target();
        chain.seed(q_origin);
        state = State::kTeleop;
        std::cout << "CLUTCH_ENGAGED state=" << stateName(state)
                  << " lead_seq=" << snap.seq << "\n";
      }
      delta = leadDelta(snap, origin, config);
      chain.step(q_origin, delta, dt);
    } else {
      if (state == State::kTeleop) {
        state = State::kPaused;
        std::cout << "CLUTCH_RELEASED state=" << stateName(state) << " reason="
                  << (deadman.lost() ? "deadman_unplugged"
                      : deadman.pressed() ? "stale_lead" : "deadman") << "\n";
      }
      chain.hold(dt);
    }

    if (count < rows.size()) {
      LogRow& row = rows[count++];
      row.t_ns = now;
      row.dt_s = dt;
      row.lead_seq = snap.seq;
      row.state = static_cast<int>(state);
      row.deadman = deadman.pressed() ? 1 : 0;
      row.q_robot = chain.target();
      row.q_target = chain.target();
      row.lead_delta = delta;
      row.gripper_ticks = snap.ticks[kGripperIndex];
      row.dq_robot.fill(0.0);
      row.tau_robot.fill(0.0);
      row.O_T_EE.fill(0.0);
      row.O_F_ext.fill(0.0);
      row.gripper_width = -1.0;
      row.gripper_target = -1.0;
    }
  }

  lead.stop();
  deadman.stop();
  writeLog(log_path, rows, count);
  std::cout << "dry run done. lead read_failures=" << lead.readFailures()
            << " rejected_jumps=" << lead.rejectedJumps() << "\n";
  return 0;
}

int runRobot(const Config& config, double duration_s,
             const std::string& log_path) {
  LeadArmReader lead(config);
  lead.open();
  lead.assertTorqueDisabled();
  DeadmanReader deadman(config.deadman_device, config.deadman_key,
                        config.deadman_grab);
  deadman.open();

  franka::Robot robot(config.robot_ip);
  const franka::RobotState before = robot.readOnce();
  if (before.robot_mode != franka::RobotMode::kIdle) {
    throw std::runtime_error(
        "preflight refused: robot must be kIdle (brakes open, FCI active)");
  }
  if (before.current_errors) {
    throw std::runtime_error("preflight refused: robot reports current errors");
  }

  std::cout << "preflight ok. q =";
  for (double v : before.q) std::cout << " " << std::fixed << std::setprecision(4) << v;
  std::cout << "\nenabled joints:";
  for (int i = 0; i < kNumJoints; ++i)
    if (config.enabled[i]) std::cout << " J" << (i + 1);
  std::cout << "  scale=" << config.scale[6] << "\n";

  std::unique_ptr<GripperController> gripper;
  if (config.gripper_enabled) {
    gripper = std::make_unique<GripperController>(config.robot_ip, config);
    gripper->connect();
  } else {
    std::cout << "gripper: disabled in config\n";
  }

  StatusPublisher status(config.status_path);
  if (!config.status_path.empty()) {
    std::cout << "publishing status to " << config.status_path << "\n";
  }

  lead.start();
  deadman.start();
  status.start();
  if (gripper) gripper->start();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  SafetyChain chain(config);
  chain.seed(before.q);
  LeadDeadband deadband;

  State state = State::kReady;
  LeadSnapshot origin{};
  std::array<double, kNumJoints> q_origin = before.q;

  std::vector<LogRow> rows(static_cast<size_t>(duration_s * 1100) + 2000);
  size_t count = 0;

  const int64_t watchdog_ns = static_cast<int64_t>(config.watchdog_ms) * 1000000LL;
  double elapsed = 0.0;
  std::array<double, kNumJoints> last_cmd = before.q;
  bool first = true;

  std::cout << "CONTROL_READY hold the SpaceMouse left button to enable\n";

  robot.control([&](const franka::RobotState& robot_state,
                    franka::Duration period) -> franka::JointPositions {
    const double dt = period.toSec() > 0.0 ? period.toSec() : 0.001;
    elapsed += dt;
    const int64_t now = monotonicNs();

    if (first) {
      // First command must be the measured pose, so control starts continuous.
      chain.seed(robot_state.q);
      q_origin = robot_state.q;
      last_cmd = robot_state.q;
      first = false;
      return franka::JointPositions(robot_state.q);
    }

    LeadSnapshot snap = lead.snapshot();
    {
      std::array<double, kNumServos> t{};
      for (int i = 0; i < kNumServos; ++i) t[i] = snap.ticks[i];
      deadband.apply(t, config.lead_deadband);
      for (int i = 0; i < kNumServos; ++i)
        snap.ticks[i] = static_cast<int32_t>(std::llround(t[i]));
    }
    const bool fresh = snap.seq > 0 && (now - lead.lastGoodNs()) < watchdog_ns;
    const bool want_stop = g_interrupted.load() || elapsed >= duration_s;
    const bool enable = deadman.pressed() && fresh && !want_stop;

    std::array<double, kNumJoints> delta{};
    std::array<double, kNumJoints> cmd;
    if (enable) {
      if (state != State::kTeleop) {
        origin = snap;
        q_origin = chain.target();
        state = State::kTeleop;
      }
      delta = leadDelta(snap, origin, config);
      cmd = chain.step(q_origin, delta, dt);
    } else {
      if (state == State::kTeleop) state = State::kPaused;
      cmd = chain.hold(dt);
    }

    // The hand only needs a new setpoint; the blocking move happens on the
    // gripper thread.
    if (gripper && state == State::kTeleop) {
      gripper->setTargetTicks(snap.ticks[kGripperIndex]);
    }

    if (count < rows.size()) {
      LogRow& row = rows[count++];
      row.t_ns = now;
      row.dt_s = dt;
      row.lead_seq = snap.seq;
      row.state = static_cast<int>(state);
      row.deadman = deadman.pressed() ? 1 : 0;
      row.q_robot = robot_state.q;
      row.q_target = cmd;
      row.lead_delta = delta;
      row.gripper_ticks = snap.ticks[kGripperIndex];
      row.dq_robot = robot_state.dq;
      row.tau_robot = robot_state.tau_J;
      row.O_T_EE = robot_state.O_T_EE;
      row.O_F_ext = robot_state.O_F_ext_hat_K;
      row.gripper_width = gripper ? gripper->width() : -1.0;
      row.gripper_target = gripper ? gripper->target() : -1.0;
    }

    status.update(static_cast<int>(state), deadman.pressed(), robot_state.q,
                  cmd, gripper ? gripper->width() : -1.0,
                  gripper ? gripper->target() : -1.0, snap.seq,
                  (now - lead.lastGoodNs()) / 1e6);

    last_cmd = cmd;
    franka::JointPositions output(cmd);
    // Only finish once hold() has actually brought every joint to rest.
    // Finishing mid-motion makes libfranka throw on a non-zero final velocity.
    if (want_stop && chain.isStopped()) {
      output.motion_finished = true;
    }
    return output;
  });

  lead.stop();
  deadman.stop();
  status.stop();
  if (gripper) {
    gripper->stop();
    std::cout << "gripper commands=" << gripper->commands()
              << " preemptions=" << gripper->preemptions()
              << " errors=" << gripper->errors() << "\n";
  }
  writeLog(log_path, rows, count);
  std::cout << "teleop finished. lead read_failures=" << lead.readFailures()
            << " rejected_jumps=" << lead.rejectedJumps() << "\n";
  return 0;
}

void handleSignal(int) { g_interrupted.store(true); }

void printUsage(const char* program) {
  std::cout << "usage:\n"
            << "  " << program << " selftest <config>\n"
            << "  " << program << " dry <config> <seconds> [log.csv]\n"
            << "  " << program << " robot <config> <seconds> [log.csv]\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    printUsage(argv[0]);
    return 2;
  }
  std::signal(SIGINT, handleSignal);
  std::signal(SIGTERM, handleSignal);

  try {
    const std::string mode = argv[1];
    const Config config = loadConfig(argv[2]);

    if (mode == "selftest") return runSelfTest(config);

    if (argc < 4) {
      printUsage(argv[0]);
      return 2;
    }
    const double seconds = std::stod(argv[3]);
    if (seconds <= 0.0 || seconds > 3600.0) {
      std::cerr << "seconds must be within 0..3600\n";
      return 2;
    }
    const std::string log_path = argc > 4 ? argv[4] : "";

    if (mode == "dry") return runDry(config, seconds, log_path);
    if (mode == "robot") return runRobot(config, seconds, log_path);

    printUsage(argv[0]);
    return 2;
  } catch (const franka::Exception& e) {
    std::cerr << "franka error: " << e.what() << "\n";
    return 1;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
}
