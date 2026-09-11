#pragma once
// Robotiq 2F-85/2F-140 Modbus RTU. All serial I/O stays outside the FCI callback.
// Register units are counts (0=open, 255=closed), never inferred millimetres.
#include <array>
#include <cerrno>
#include <cstdint>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <mutex>
#include <poll.h>
#include <sched.h>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace robotiq {
using Clock = std::chrono::steady_clock;
inline uint16_t crc(const uint8_t* data, size_t size) {
  uint16_t result = 0xffff;
  for (size_t i=0; i<size; ++i) {
    result ^= data[i];
    for (int b=0; b<8; ++b) result = result & 1 ? (result >> 1)^0xa001 : result >> 1;
  }
  return result;
}
struct State {
  uint8_t status=0, fault=0, requested=0, position=0, current=0;
  bool active() const { return status & 1; }
  bool ready() const { return active() && ((status >> 4)&3)==3 && fault==0; }
};
class Port {
 public:
  explicit Port(const std::string& path, int slave=9) : slave_(slave) {
    if (slave<1 || slave>247) throw std::runtime_error("Robotiq: invalid slave ID");
    fd_=::open(path.c_str(), O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);
    if (fd_<0) throw std::runtime_error("Robotiq open " + path + ": " + std::strerror(errno));
    try {
      if (flock(fd_, LOCK_EX|LOCK_NB)<0 || ioctl(fd_, TIOCEXCL)<0)
        throw std::runtime_error("Robotiq serial port is busy");
      termios t{};
      if (tcgetattr(fd_, &t)<0) throw std::runtime_error("Robotiq tcgetattr failed");
      cfmakeraw(&t); cfsetispeed(&t,B115200); cfsetospeed(&t,B115200);
      t.c_cflag |= CLOCAL|CREAD; t.c_cflag &= ~CRTSCTS;
      t.c_cc[VMIN]=0; t.c_cc[VTIME]=0;
      if (tcsetattr(fd_,TCSANOW,&t)<0) throw std::runtime_error("Robotiq tcsetattr failed");
      tcflush(fd_,TCIFLUSH);
    } catch (...) { close(); throw; }
  }
  ~Port() { close(); }
  Port(const Port&)=delete;
  Port& operator=(const Port&)=delete;
  State read() {
    auto r=exchange({uint8_t(slave_),4,7,0xd0,0,3},11);
    if (r[2]!=6) throw std::runtime_error("Robotiq: invalid status byte count");
    return {r[3],uint8_t(r[5]&15),r[6],r[7],r[8]};
  }
  void write(uint8_t action, uint8_t position, uint8_t speed, uint8_t force) {
    auto r=exchange({uint8_t(slave_),16,3,0xe8,0,3,6,action,0,0,position,speed,force},8);
    if (r[2]!=3 || r[3]!=0xe8 || r[4]!=0 || r[5]!=3)
      throw std::runtime_error("Robotiq: invalid write acknowledgement");
  }
 private:
  void close() { if(fd_>=0) { ioctl(fd_,TIOCNXCL); ::close(fd_); fd_=-1; } }
  void wait(short events, Clock::time_point deadline) {
    for (;;) {
      int ms=std::chrono::duration_cast<std::chrono::milliseconds>(deadline-Clock::now()).count();
      if(ms<=0) throw std::runtime_error("Robotiq: serial response timeout");
      pollfd p{fd_,events,0}; int n=poll(&p,1,ms);
      if(n<0 && errno==EINTR) continue;
      if(n<0 || (p.revents&(POLLERR|POLLHUP|POLLNVAL)))
        throw std::runtime_error("Robotiq: serial disconnected");
      if(n>0 && (p.revents&events)) return;
    }
  }
  std::vector<uint8_t> exchange(std::vector<uint8_t> request, size_t bytes) {
    std::this_thread::sleep_until(last_+std::chrono::milliseconds(5));
    const auto deadline=Clock::now()+std::chrono::milliseconds(80);
    auto checksum=crc(request.data(),request.size());
    request.push_back(checksum&255); request.push_back(checksum>>8);
    size_t sent=0;
    while(sent<request.size()) {
      wait(POLLOUT,deadline); auto n=::write(fd_,request.data()+sent,request.size()-sent);
      if(n<0 && (errno==EINTR||errno==EAGAIN)) continue;
      if(n<=0) throw std::runtime_error("Robotiq: serial write failed");
      sent+=n;
    }
    std::vector<uint8_t> response(bytes); size_t received=0;
    while(received<bytes) {
      wait(POLLIN,deadline); auto n=::read(fd_,response.data()+received,bytes-received);
      if(n<0 && (errno==EINTR||errno==EAGAIN)) continue;
      if(n<=0) continue;
      received+=n;
      if(received>=2 && (response[1]&0x80)) throw std::runtime_error("Robotiq: Modbus exception response");
    }
    last_=Clock::now();
    if(response[0]!=slave_ || response[1]!=request[1] ||
       crc(response.data(),bytes-2)!=(response[bytes-2]|(response[bytes-1]<<8)))
      throw std::runtime_error("Robotiq: invalid response address/function/CRC");
    return response;
  }
  int fd_=-1, slave_;
  Clock::time_point last_{};
};

inline State initialize(Port& port, const std::function<bool()>& cancelled=[] { return false; }) {
  // Explicit commissioning only: resetting then raising rACT causes automatic
  // finger calibration. Normal connect/start paths never call this function.
  port.write(0,0,0,0);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  auto deadline=Clock::now()+std::chrono::seconds(15);
  while(Clock::now()<deadline) {
    if(cancelled()) { port.write(0,0,0,0); throw std::runtime_error("Robotiq initialization cancelled"); }
    port.write(1,0,0,0);
    auto state=port.read();
    if(state.ready()) return state;
    if(state.fault>=10) throw std::runtime_error("Robotiq activation fault="+std::to_string(state.fault));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  throw std::runtime_error("Robotiq activation timed out");
}

class Controller {
 public:
  Controller(const std::string& path,int slave,int speed,int force)
      : port_(path,slave),speed_(speed),force_(force) {}
  void connect() {
    auto state=port_.read();
    // A communication watchdog does not require reset. Keep the activation bit
    // unchanged and clear GoTo; this never generates an activation rising edge.
    if(state.active() && state.fault==9) {
      port_.write(1,state.position,speed_,force_); state=port_.read();
    }
    if(!state.ready()) throw std::runtime_error("Robotiq 未就绪，fault="+
        std::to_string(state.fault)+"；请先执行 robotiq-init 初始化（会自动开合）。");
    publish(state); command_.store(state.position); // disabled, seeded from actual position
  }
  void request(int position) { command_.store(256u|unsigned(position)); }
  void pause() { command_.fetch_and(255u); }
  int position() const { return position_.load(); }
  int requested() const { return requested_.load(); }
  int fault() const { return fault_.load(); }
  int object() const { return object_.load(); }
  bool healthy() const { return !failed_.load(); }
  std::string failure() const { std::lock_guard<std::mutex> lock(mutex_); return failure_; }
  int64_t commands() const { return commands_.load(); }
  void start() {
    running_=true;
    thread_=std::thread([this] {
      sched_param param{};
      try {
        if(sched_setscheduler(0,SCHED_OTHER,&param)<0) throw std::runtime_error("Robotiq worker scheduler failed");
        unsigned last=~0u; auto last_write=Clock::time_point{};
        while(running_.load()) {
          unsigned command=command_.load();
          if(command!=last || Clock::now()-last_write>=std::chrono::milliseconds(100)) {
            port_.write(command&256 ? 9:1,command&255,speed_,force_);
            ++commands_; last=command; last_write=Clock::now();
          }
          auto state=port_.read(); publish(state);
          if(!state.ready()) throw std::runtime_error("Robotiq runtime fault="+std::to_string(state.fault));
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      } catch(const std::exception& e) {
        { std::lock_guard<std::mutex> lock(mutex_); failure_=e.what(); }
        failed_=true;
      }
      // Stop without opening or resetting. On a broken cable the attempt is
      // bounded by the serial timeout; hardware's own communication fault remains.
      try { port_.write(1,position_.load(),speed_,force_); }
      catch(const std::exception& e) {
        { std::lock_guard<std::mutex> lock(mutex_); if(failure_.empty()) failure_=e.what(); }
        failed_=true;
      }
    });
  }
  void stop() { pause(); running_=false; if(thread_.joinable()) thread_.join(); }
  ~Controller() { stop(); }
 private:
  void publish(const State& s) { position_=s.position; requested_=s.requested; fault_=s.fault; object_=s.status>>6; }
  Port port_;
  uint8_t speed_,force_;
  std::atomic<unsigned> command_{0};
  std::atomic<int> position_{-1},requested_{-1},fault_{0},object_{0};
  std::atomic<bool> running_{false},failed_{false};
  std::atomic<int64_t> commands_{0};
  mutable std::mutex mutex_;
  std::string failure_;
  std::thread thread_;
};
} // namespace robotiq
