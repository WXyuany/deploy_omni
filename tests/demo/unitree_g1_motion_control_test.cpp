#include "../../examples/unitree_g1/unitree_g1_motion_control.hpp"
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <vector>

using namespace g1_demo;
void require(bool condition, const char* message) {
  if (!condition) { std::cerr << message << '\n'; std::exit(1); }
}
struct Fake : Backend {
  State state{true, 500, 0};
  bool fresh{true};
  bool fail_velocity{false};
  bool fail_stop{false};
  bool throw_velocity{false};
  std::vector<Command> sent;
  State observe() override { return state; }
  bool healthy() override { return fresh; }
  int send(const Command& c) override {
    sent.push_back(c);
    if (throw_velocity && c.action == Action::velocity) throw std::runtime_error("transport");
    return (fail_velocity && c.action == Action::velocity) ||
           (fail_stop && c.action == Action::stop) ? 7301 : 0;
  }
};
int main() {
  const auto now = Clock::now();
  const auto silent = [](const std::string&) {};
  {
    Fake f; Controller c(f, silent);
    c.key('w', now); c.finish();
    require(f.sent.empty(), "Startup/unarmed exit must never write");
    c.key('r', now); c.key('w', now);
    require(f.sent.size() == 1 && f.sent[0].action == Action::velocity &&
            f.sent[0].vx == 0.10F && f.sent[0].duration == 0.5F, "bounded forward pulse");
    c.key('w', now); c.key('r', now); c.key('h', now);
    require(f.sent.size() == 1, "repeat/rearm/action blocked during pulse");
    c.tick(now + std::chrono::milliseconds(501));
    require(f.sent.back().action == Action::stop && !c.armed(), "expiry stops and disarms");
    c.key('w', now + std::chrono::seconds(1));
    require(f.sent.size() == 2, "expired pulse cannot restart without r");
  }
  for (char key : {'s', 'a', 'd', 'j', 'l'}) {
    Fake f; Controller c(f, silent);
    c.key('r', now); c.key(key, now);
    require(f.sent.size() == 1, "direction should send one command");
    const auto m = f.sent[0];
    require((key != 's' || m.vx == -0.10F) && (key != 'a' || m.vy == 0.08F) &&
            (key != 'd' || m.vy == -0.08F) && (key != 'j' || m.yaw == 0.15F) &&
            (key != 'l' || m.yaw == -0.15F), "direction/sign mapping");
  }
  for (int problem = 0; problem < 4; ++problem) {
    Fake f; Controller c(f, silent);
    c.key('r', now);
    if (problem == 0) f.fresh = false;
    if (problem == 1) f.state.fsm = 0;
    if (problem == 2) f.state.balance = -1;
    if (problem == 3) f.state.available = false;
    c.key('w', now);
    require(f.sent.empty() && !c.armed(), "recheck state after arming");
  }
  {
    Fake f; Controller c(f, silent);
    c.key('r', now); c.key('a', now); f.fresh = false; c.tick(now);
    require(f.sent.back().action == Action::stop, "stale data stops ongoing pulse");
  }
  {
    Fake f; f.fail_velocity = true; f.fail_stop = true; Controller c(f, silent);
    c.key('r', now); c.key('w', now);
    require(f.sent.size() == 2 && f.sent.back().action == Action::stop && !c.armed(),
            "uncertain velocity reply must attempt stop even if stop fails");
    c.key('p', now);
    require(f.sent.back().action == Action::damp, "explicit damping independent of failed stop");
  }
  {
    Fake f; f.state = {false, -1, -1}; f.fresh = false; Controller c(f, silent);
    c.key('z', now);
    require(f.sent.size() == 1 && f.sent.back().action == Action::zero,
            "one-key zero torque usable when state unavailable");
    c.key('p', now);
    require(f.sent.size() == 2 && f.sent.back().action == Action::damp,
            "one-key damping usable when state unavailable");
  }
  {
    // Assert the public keyboard contract independently of the binding table.
    const std::vector<std::pair<char, Action>> expected{
      {'z', Action::zero}, {'p', Action::damp}, {'u', Action::stand},
      {'t', Action::start}, {'b', Action::balance}, {'c', Action::squat},
      {'v', Action::sit}, {'o', Action::high}, {'n', Action::low},
      {'h', Action::wave}, {'k', Action::shake_start}, {'m', Action::shake_end}};
    for (const auto& entry : expected) {
      Fake f; Controller c(f, silent);
      if (entry.second == Action::stand) f.state.fsm = 1;
      c.key(entry.first, now);
      require(f.sent.size() == 1 && f.sent.front().action == entry.second,
              "each letter directly sends its documented action");
      require(!c.armed(), "posture command disarms walking");
    }
  }
  {
    Fake f; Controller c(f, silent);
    for (char retired : {'!', '1', '2', '3', '4', '5', 'Z', 'P', 'K'}) c.key(retired, now);
    require(f.sent.empty(), "retired keys must not execute actions");
    c.key('h', now); c.key('h', now); c.key('c', now);
    require(f.sent.size() == 1, "single-key actions still respect settling guard");
  }
  {
    Fake f; f.state.fsm = 0; Controller c(f, silent);
    c.key('t', now);
    require(f.sent.empty(), "cannot Start directly from zero torque");
    f.state.fsm = 1; c.key('u', now);
    require(f.sent.back().action == Action::stand, "stand from damp");
    f.state.fsm = 4; c.key('t', now);
    require(f.sent.size() == 1, "settling gate");
    auto later = now + std::chrono::seconds(4);
    c.key('t', later);
    require(f.sent.back().action == Action::start, "Start after standing and settling");
  }
  {
    Fake f; Controller c(f, silent);
    c.key('k', now); c.finish();
    require(f.sent.size() == 3 && f.sent[1].action == Action::stop &&
            f.sent[2].action == Action::shake_end, "handshake retract on exit");
  }
  {
    Fake f; Controller c(f, silent);
    c.key('k', now);
    c.key('p', now); c.finish();
    require(f.sent.back().action == Action::stop, "never retract arm after damping");
  }
  {
    Fake f; f.throw_velocity = true; Controller c(f, silent);
    c.key('r', now);
    try { c.key('w', now); } catch (const std::exception&) {}
    c.finish();
    require(f.sent.back().action == Action::stop, "exception cleanup stops uncertain velocity");
  }
  std::cout << "motion policy tests passed\n";
}
