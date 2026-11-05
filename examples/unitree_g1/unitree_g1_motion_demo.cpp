#include "unitree_g1_motion_control.hpp"

#include <cerrno>
#include <cmath>
#include <csignal>
#include <fcntl.h>
#include <iostream>
#include <memory>
#include <mutex>
#include <poll.h>
#include <stdexcept>
#include <sys/file.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>
#include "bridge/adapters/unitree_g1/unitree_g1_low_level_protocol.hpp"

namespace {
using namespace g1_demo;
volatile std::sig_atomic_t interrupted = 0;
void signal_handler(int) { interrupted = 1; }
void print_line(const std::string& value) { std::cout << value << std::endl; }

class Terminal final {
 public:
  Terminal() {
    if (tcgetattr(STDIN_FILENO, &saved_) != 0) throw std::runtime_error("Cannot read terminal");
    auto mode = saved_;
    mode.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    mode.c_cc[VMIN] = 0;
    mode.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &mode) != 0)
      throw std::runtime_error("Cannot configure terminal");
  }
  ~Terminal() { tcsetattr(STDIN_FILENO, TCSANOW, &saved_); }
  Terminal(const Terminal&) = delete;
  Terminal& operator=(const Terminal&) = delete;
 private:
  termios saved_{};
};

class InstanceLock final {
 public:
  InstanceLock() {
    fd_ = open("/tmp/adapter_any_robot_g1_motion.lock",
               O_CREAT | O_RDWR | O_NOFOLLOW | O_CLOEXEC, 0600);
    if (fd_ < 0) throw std::runtime_error("Cannot open demo lock");
    if (flock(fd_, LOCK_EX | LOCK_NB) != 0) {
      close(fd_); fd_ = -1;
      throw std::runtime_error("Another motion demo holds the lock");
    }
  }
  ~InstanceLock() { if (fd_ >= 0) close(fd_); }
  InstanceLock(const InstanceLock&) = delete;
  InstanceLock& operator=(const InstanceLock&) = delete;
 private:
  int fd_{-1};
};

class RobotBackend final : public Backend {
 public:
  RobotBackend() : subscriber_("rt/lowstate") {
    loco_.SetTimeout(0.3F);
    loco_.Init();
    switcher_.SetTimeout(0.3F);
    switcher_.Init();
    subscriber_.InitChannel([this](const void* data) {
      if (!data) return;
      const auto& state = *static_cast<const unitree_hg::msg::dds_::LowState_*>(data);
      if (!bridge::adapters::unitree_g1::validate_low_state_crc(state)) return;
      std::lock_guard<std::mutex> lock(mutex_);
      if (!received_ || state.tick() != latest_.tick()) updated_ = Clock::now();
      latest_ = state;
      received_ = true;
    }, 1);
  }
  ~RobotBackend() override { subscriber_.CloseChannel(); }

  bool healthy() override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!received_ || Clock::now() - updated_ > std::chrono::milliseconds(500)) return false;
    const auto& rpy = latest_.imu_state().rpy();
    for (int i = 0; i < 2; ++i)
      if (!std::isfinite(rpy[i]) || std::abs(rpy[i]) > 0.75F) return false;
    for (std::size_t i = 0; i < 29; ++i) {
      const auto& m = latest_.motor_state()[i];
      if (!std::isfinite(m.q()) || !std::isfinite(m.dq()) ||
          !std::isfinite(m.tau_est()) || m.motorstate() != 0 ||
          m.temperature()[0] > 80 || m.temperature()[1] > 100) return false;
    }
    return true;
  }
  State observe() override {
    State state;
    std::string form, name;
    const int mode_code = switcher_.CheckMode(form, name);
    const int fsm_code = loco_.GetFsmId(state.fsm);
    int balance_code = -1;
    if (fsm_code == 0 && state.fsm == 500)
      balance_code = loco_.GetBalanceMode(state.balance);
    print_line("CheckMode code=" + std::to_string(mode_code) + " form=" + form + " name=" + name +
        "; GetFsmId code=" + std::to_string(fsm_code) + " fsm=" + std::to_string(state.fsm) +
        "; GetBalanceMode code=" + std::to_string(balance_code) +
        " balance=" + std::to_string(state.balance));
    state.available = mode_code == 0 && !name.empty() && fsm_code == 0;
    print_line(std::string("LowState fresh/CRC/health checks: ") + (healthy() ? "PASS" : "BLOCKED"));
    return state;
  }
  int send(const Command& command) override {
    if (interrupted && command.action != Action::stop && command.action != Action::shake_end)
      return -10003;
    // A query may have blocked while the operator pressed stop/quit. Leave
    // pending input for the event loop and refuse a new non-stop request.
    pollfd input{STDIN_FILENO, POLLIN, 0};
    if (command.action != Action::stop && command.action != Action::shake_end &&
        poll(&input, 1, 0) > 0) return -10004;
    switch (command.action) {
      case Action::stop: print_line("StopMove"); return loco_.StopMove();
      case Action::zero: print_line("ZeroTorque"); return loco_.ZeroTorque();
      case Action::damp: print_line("Damp"); return loco_.Damp();
      case Action::stand: print_line("StandUp"); return loco_.StandUp();
      case Action::start: print_line("Start (FSM=500)"); return loco_.Start();
      case Action::balance: print_line("BalanceStand"); return loco_.BalanceStand();
      case Action::squat: print_line("Squat"); return loco_.Squat();
      case Action::sit: print_line("Sit"); return loco_.Sit();
      case Action::high: print_line("HighStand"); return loco_.HighStand();
      case Action::low: print_line("LowStand"); return loco_.LowStand();
      case Action::wave: print_line("WaveHand(false)"); return loco_.WaveHand(false);
      case Action::shake_start: print_line("ShakeHand(0)"); return loco_.ShakeHand(0);
      case Action::shake_end: print_line("ShakeHand(1)"); return loco_.ShakeHand(1);
      case Action::velocity:
        print_line("SetVelocity vx=" + std::to_string(command.vx) + " vy=" +
            std::to_string(command.vy) + " yaw=" + std::to_string(command.yaw) +
            " duration=" + std::to_string(command.duration));
        return loco_.SetVelocity(command.vx, command.vy, command.yaw, command.duration);
    }
    return -10002;
  }
 private:
  std::mutex mutex_;
  unitree_hg::msg::dds_::LowState_ latest_;
  Clock::time_point updated_{};
  bool received_{false};
  unitree::robot::g1::LocoClient loco_;
  unitree::robot::b2::MotionSwitcherClient switcher_;
  unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_> subscriber_;
};

class DryBackend final : public Backend {
 public:
  State observe() override {
    print_line("DRY RUN fsm=" + std::to_string(state_.fsm) + " balance=" + std::to_string(state_.balance));
    return state_;
  }
  bool healthy() override { return true; }
  int send(const Command& command) override {
    print_line("DRY RUN action=" + std::to_string(static_cast<int>(command.action)) +
        " vx=" + std::to_string(command.vx) + " vy=" + std::to_string(command.vy) +
        " yaw=" + std::to_string(command.yaw) + " duration=" + std::to_string(command.duration));
    switch (command.action) {
      case Action::zero: state_.fsm = 0; break;
      case Action::damp: state_.fsm = 1; break;
      case Action::stand: state_.fsm = 4; break;
      case Action::start: state_.fsm = 500; break;
      case Action::balance: state_.balance = 0; break;
      case Action::squat: state_.fsm = 2; break;
      case Action::sit: state_.fsm = 3; break;
      default: break;
    }
    return 0;
  }
 private:
  State state_{true, 0, -1};
};

void help() {
  print_line("Usage: unitree_g1_motion_demo [--check | --dry-run | --enable-motion] [--interface eth0]");
  print_line("Default: --check (read only). --dry-run creates NO DDS participants.");
  print_line("i: status   r: visually confirm standing / arm ONE pulse");
  print_line("w/s: +/-0.10 m/s forward/back  a/d: +/-0.08 m/s lateral");
  print_line("j/l: +/-0.15 rad/s yaw; pulse=0.5s; r required before EACH pulse.");
  print_line("space/x: StopMove + retract this demo's handshake; disarm");
  print_line("q/Ctrl+C: best-effort stop then exit. NOT a hardware emergency stop.");
  for (const auto& binding : bindings)
    print_line(std::string(1, binding.key) + ": " + binding.description);
  print_line("StopMove does NOT cancel posture/wave actions. p/z may cause loss of support.");
  print_line("Use manufacturer support fixture and remote; verify posture after every transition.");
}
}  // namespace

int main(int argc, char** argv) {
  std::string mode = "--check", interface = "eth0";
  bool selected = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help") { help(); return 0; }
    if (arg == "--interface" && i + 1 < argc) { interface = argv[++i]; continue; }
    if ((arg == "--check" || arg == "--dry-run" || arg == "--enable-motion") && !selected) {
      selected = true; mode = arg; continue;
    }
    std::cerr << "Invalid or duplicate option: " << arg << '\n'; return 2;
  }
  if (interface.empty() || (mode != "--check" && !isatty(STDIN_FILENO))) {
    std::cerr << "Need a network interface and an interactive TTY for keyboard mode.\n"; return 2;
  }
  for (int signal : {SIGINT, SIGTERM, SIGHUP, SIGQUIT, SIGTSTP})
    std::signal(signal, signal_handler);
  std::signal(SIGPIPE, SIG_IGN);

  try {
    std::unique_ptr<InstanceLock> instance;
    std::unique_ptr<Backend> backend;
    if (mode == "--dry-run") {
      backend = std::make_unique<DryBackend>();
    } else {
      if (mode == "--enable-motion") instance = std::make_unique<InstanceLock>();
      unitree::robot::ChannelFactory::Instance()->Init(0, interface);
      backend = std::make_unique<RobotBackend>();
      for (int i = 0; i < 100 && !interrupted && !backend->healthy(); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (interrupted) return 1;
    const State initial = backend->observe();
    if (mode == "--check") return initial.available && backend->healthy() ? 0 : 1;
    Terminal terminal;
    Controller controller(*backend, print_line);
    help();
    int result = 0;
    try {
      while (!interrupted) {
        controller.tick(Clock::now());
        pollfd descriptor{STDIN_FILENO, POLLIN, 0};
        const int ready = poll(&descriptor, 1, 20);
        if (ready < 0 && errno == EINTR) continue;
        if (ready < 0 || descriptor.revents & (POLLHUP | POLLERR | POLLNVAL)) break;
        if (!(descriptor.revents & POLLIN)) continue;
        char input[64];
        const auto count = read(STDIN_FILENO, input, sizeof(input));
        if (count <= 0) break;
        // Never replay buffered commands after a slow RPC. Stop has priority.
        const std::string batch(input, static_cast<std::size_t>(count));
        if (batch.find('q') != std::string::npos || batch.find('\x04') != std::string::npos) break;
        if (batch.find_first_of(" x") != std::string::npos) controller.stop();
        else if (count == 1 && !interrupted) {
          if (input[0] == '?') { controller.cancel_input(); help(); }
          else if (input[0] == 'i' && !controller.moving()) {
            controller.cancel_input(); backend->observe();
          }
          else controller.key(input[0], Clock::now());
        } else {
          controller.cancel_input();
          print_line("Buffered/pasted input discarded; press one key at a time.");
        }
        // Drop stale motion/action keys, but NEVER drop stop/quit entered
        // while a synchronous SDK request was in flight.
        for (int batches = 0; !interrupted; ++batches) {
          pollfd queued{STDIN_FILENO, POLLIN, 0};
          const int pending = poll(&queued, 1, 0);
          if (pending < 0 && errno == EINTR) continue;
          if (pending < 0 || queued.revents & (POLLHUP | POLLERR | POLLNVAL)) {
            interrupted = 1; break;
          }
          if (!(queued.revents & POLLIN)) break;
          if (batches >= 16) throw std::runtime_error("Input flood; stopping session");
          controller.cancel_input();
          char buffered[256];
          const auto size = read(STDIN_FILENO, buffered, sizeof(buffered));
          if (size <= 0) { interrupted = 1; break; }
          {
            const std::string remaining(buffered, static_cast<std::size_t>(size));
            if (remaining.find('q') != std::string::npos ||
                remaining.find('\x04') != std::string::npos) interrupted = 1;
            if (remaining.find_first_of(" x") != std::string::npos) controller.stop();
          }
        }
      }
    } catch (const std::exception& e) {
      print_line(std::string("Runtime failure: ") + e.what()); result = 1;
    }
    controller.finish();
    return result;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n'; return 1;
  }
}
