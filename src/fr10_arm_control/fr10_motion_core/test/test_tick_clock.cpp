#include <iostream>
#include <limits>
#include "fr10_motion_core/tick_clock.hpp"
int main() {
  using namespace fr10_motion;
  bool ok = true;
  auto check = [&](bool valid, const char *name) { if (!valid) { std::cerr << name << '\n'; ok = false; } };
  TickClock clock;
  check(resumeReadiness(true,true,0.,.05) == ResumeDecision::READY, "fresh resume");
  check(resumeReadiness(true,false,.05,.05) == ResumeDecision::WAIT, "bounded observation handover");
  check(resumeReadiness(true,false,.050001,.05) == ResumeDecision::OBSERVATION_EXPIRED,
        "missing sensor returns to safety with a live joint executor");
  check(resumeReadiness(false,true,.050001,.05) == ResumeDecision::FEEDBACK_EXPIRED,
        "missing encoders cannot certify joint braking");
  check(resumeReadiness(true,false,100.,.05) == ResumeDecision::OBSERVATION_EXPIRED,
        "sensor expiry cannot extend the handover deadline");
  check(resumeReadiness(true,true,std::numeric_limits<double>::quiet_NaN(),.05) == ResumeDecision::FEEDBACK_EXPIRED,
        "invalid handover time fails closed");
  check(clock.advance(1, true) == TickDecision::WAIT, "initial simulation boundary");
  for (int i = 1; i < 8; ++i) check(clock.advance(1 + i * .001, true) == TickDecision::WAIT, "subcycle");
  check(clock.advance(1.008, true) == TickDecision::STEP, "8 ms source tick");
  for (int i = 0; i < 10; ++i) check(clock.advance(1.008, true) == TickDecision::WAIT, "pause frozen");
  check(clock.advance(1.016, true) == TickDecision::STEP, "resume one tick");
  check(clock.advance(2, true) == TickDecision::STEP && clock.missedWindows() == 1, "late callback no burst");
  check(clock.advance(2, true) == TickDecision::WAIT, "no second late tick");
  check(clock.advance(.5, true) == TickDecision::BACKWARD_JUMP, "jump invalidates session");
  check(clock.advance(std::numeric_limits<double>::quiet_NaN(), true) == TickDecision::INVALID, "NaN clock");
  TickClock hardware;
  check(hardware.advance(1, false) == TickDecision::STEP &&
        hardware.advance(1.00799, false) == TickDecision::STEP, "hardware scheduler not divided by jitter");
  return ok ? 0 : 1;
}
