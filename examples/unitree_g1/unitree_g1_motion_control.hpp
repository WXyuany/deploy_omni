#pragma once

#include <array>
#include <chrono>
#include <functional>
#include <string>
#include <utility>

namespace g1_demo {
using Clock = std::chrono::steady_clock;
enum class Action { stop, zero, damp, stand, start, balance, squat, sit,
                    high, low, wave, shake_start, shake_end, velocity };
struct Command {
  Action action;
  float vx{0}, vy{0}, yaw{0}, duration{0.5F};
};
struct State {
  bool available{false};
  int fsm{-1};
  int balance{-1};
};
class Backend {
 public:
  virtual ~Backend() = default;
  virtual State observe() = 0;
  virtual bool healthy() = 0;
  virtual int send(const Command&) = 0;
};
struct Binding { char key; Action action; const char* description; };
inline constexpr std::array<Binding, 12> bindings{{
    {'z', Action::zero, "ZeroTorque: FSM 0; requires physical support"},
    {'p', Action::damp, "Damp: FSM 1; does not hold standing posture"},
    {'u', Action::stand, "StandUp: FSM 4"},
    {'t', Action::start, "Start: FSM 500 (locomotion, NOT service startup)"},
    {'b', Action::balance, "BalanceStand: balance mode 0"},
    {'c', Action::squat, "Squat: FSM 2"},
    {'v', Action::sit, "Sit: FSM 3"},
    {'o', Action::high, "HighStand: vendor maximum standing height"},
    {'n', Action::low, "LowStand: vendor minimum standing height"},
    {'h', Action::wave, "WaveHand(false): wave without turning"},
    {'k', Action::shake_start, "ShakeHand(0): extend hand"},
    {'m', Action::shake_end, "ShakeHand(1): retract hand"},
}};

// Pure policy: no DDS, wall-clock sleeps or terminal dependencies.
class Controller {
 public:
  using Log = std::function<void(const std::string&)>;
  Controller(Backend& backend, Log log) : backend_(backend), log_(std::move(log)) {}
  bool armed() const { return armed_; }
  bool moving() const { return moving_; }
  void cancel_input() { armed_ = false; }

  void stop() {
    armed_ = false;
    moving_ = false;
    // A timeout can mean a request executed but the reply was lost.
    // Keep handshake uncertainty until retraction succeeds.
    attempt({Action::stop});
    if (hand_out_ && attempt({Action::shake_end})) hand_out_ = false;
  }

  void finish() noexcept {
    if (!touched_) return;  // Opening and closing the demo performs no writes.
    try { stop(); } catch (...) { log_("Cleanup failed; use physical remote."); }
  }

  void tick(Clock::time_point now) {
    if (moving_ && (now >= stop_at_ || !backend_.healthy())) {
      log_("Pulse ended or state became unavailable; stopping and disarming.");
      stop();
    }
  }

  void key(char key, Clock::time_point now) {
    tick(now);
    if (key == ' ' || key == 'x') { stop(); return; }
    if (moving_) { log_("Pulse active: space/x stops it."); return; }
    if (key == 'r') {
      armed_ = false;
      const auto state = backend_.observe();
      armed_ = state.available && state.fsm == 500 && state.balance == 0 &&
               backend_.healthy() && !hand_out_ && now >= settle_until_;
      log_(armed_ ? "Armed for ONE 0.5 s pulse; operator confirms stable standing."
                  : "Blocked: need fresh state, FSM=500, balance=0, hand retracted and settled posture.");
      return;
    }
    Command move{Action::velocity};
    switch (key) {
      case 'w': move.vx = 0.10F; break;
      case 's': move.vx = -0.10F; break;
      case 'a': move.vy = 0.08F; break;
      case 'd': move.vy = -0.08F; break;
      case 'j': move.yaw = 0.15F; break;
      case 'l': move.yaw = -0.15F; break;
      default: dispatch_action(key, now); return;
    }
    const bool was_armed = armed_;
    armed_ = false;
    if (!was_armed) { log_("Press r after visually confirming stable standing."); return; }
    const auto state = backend_.observe();
    if (!state.available || state.fsm != 500 || state.balance != 0 ||
        !backend_.healthy()) {
      log_("Pre-motion state check failed; disarmed.");
      return;
    }
    // Set before RPC so exceptions and uncertain acknowledgements still trigger cleanup.
    moving_ = true;
    stop_at_ = now + std::chrono::milliseconds(500);
    if (!attempt(move)) stop();
  }

 private:
  void dispatch_action(char key, Clock::time_point now) {
    armed_ = false;
    for (const auto& binding : bindings) {
      if (binding.key != key) continue;
      log_(binding.description);
      execute_action(binding.action, now);
      return;
    }
  }

  static bool allowed(Action action, int fsm) {
    switch (action) {
      case Action::stand: return fsm == 1 || fsm == 2 || fsm == 3 || fsm == 4;
      case Action::start: return fsm == 4 || fsm == 500;
      case Action::squat:
      case Action::sit: return fsm == 4 || fsm == 500;
      default: return fsm == 500;
    }
  }

  void execute_action(Action action, Clock::time_point now) {
    // Explicit support-dependent commands must remain usable even if state queries fail.
    if (action != Action::damp && action != Action::zero) {
      const auto state = backend_.observe();
      if (!state.available || !backend_.healthy() || !allowed(action, state.fsm) ||
          now < settle_until_) {
        log_("Command blocked by state/settling guard; read status and inspect posture.");
        return;
      }
      if (hand_out_ && action != Action::shake_end) {
        log_("Retract hand with m first."); return;
      }
      const bool needs_balance = action == Action::high || action == Action::low ||
          action == Action::wave || action == Action::shake_start;
      if (needs_balance && state.balance != 0) {
        log_("Action requires balance mode 0."); return;
      }
    }
    // After an explicit support-dependent mode switch, never send an automatic
    // arm retraction into an uncertain/zero-torque/passive state.
    if (action == Action::damp || action == Action::zero) hand_out_ = false;
    if (action == Action::shake_start) hand_out_ = true;
    if (attempt({action}) && action == Action::shake_end) hand_out_ = false;
    // Delay reduces accidental double transitions; NOT proof of motion completion.
    settle_until_ = now + std::chrono::seconds(3);
  }

  bool attempt(const Command& command) {
    touched_ = true;
    const int code = backend_.send(command);
    log_("return_code=" + std::to_string(code) +
         (code == 0 ? " (accepted; verify physical result)" : " (failed/uncertain; disarmed)"));
    if (code != 0) armed_ = false;
    return code == 0;
  }
  Backend& backend_;
  Log log_;
  bool armed_{false}, moving_{false}, touched_{false}, hand_out_{false};
  Clock::time_point stop_at_{}, settle_until_{};
};
}  // namespace g1_demo
