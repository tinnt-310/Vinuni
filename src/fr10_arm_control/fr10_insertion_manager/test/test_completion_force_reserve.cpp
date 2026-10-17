#include <iostream>
#include <limits>
#include "fr10_insertion_manager/insertion_state_machine.hpp"

int main() {
  using namespace fr10_insertion;
  bool ok = true;
  const auto check = [&](bool value, const char *name) {
    if (!value) { std::cerr << "FAIL " << name << '\n'; ok = false; }
  };
  InsertionStateConfig config;
  config.preinsert_hold_s = 0.; config.completion_hold_s = .024;
  config.pre_stop_force_error_tolerance_n = .6;
  InsertionStateMachine machine(config);
  check(!machine.requiresFreshWrenchForResume(), "IDLE_HAS_NO_FORCE_RESUME_DEPENDENCY");
  InsertionGoal goal; goal.selected_axes[2] = true; goal.timeout_s = 20.;
  InsertionObservation input;
  const auto reset = [&] {
    machine.reset(); input = {};
    input.dt_s = .008; input.source_time_s = 1.; input.preinsert_reached = input.tare_complete = true;
    input.contact_state = fr10_force::ContactState::CONTACT_CONFIRMED;
    input.actual_depth_m = goal.insertion_depth_m;
    input.axis_force_error_n = .8;
    check(machine.start(goal, 1.), "FORCE_RESERVE_SESSION_START");
  };
  const auto tick = [&] { input.source_time_s += input.dt_s; return machine.update(input); };
  reset();
  check(!machine.requiresFreshWrenchForResume(), "PREINSERT_DOES_NOT_REQUIRE_FORCE_ACTUATION_DATA");
  for (int i=0; i<15; ++i) tick();
  check(machine.state() == InsertionState::VERIFY_INSERTION,
      "MOVING_FORCE_WITHOUT_BRAKING_RESERVE_CANNOT_REPORT_SUCCESS");
  check(machine.requiresFreshWrenchForResume(), "FORCE_MOTION_RETAINS_FRESHNESS_GATE");
  input.axis_force_error_n = .5;
  for (int i=0; i<4; ++i) tick();
  check(machine.state() == InsertionState::SUCCESS,
      "QUALIFIED_FORCE_RESERVE_REQUESTS_CONTROLLED_STOP");
  check(!machine.requiresFreshWrenchForResume(), "BRAKING_MUST_CONTINUE_WITHOUT_FRESH_FT");
  input.axis_force_error_n = .9; input.stop_complete = true;
  check(tick().state == InsertionState::HOLD,
      "STATIONARY_HOLD_RETAINS_ORIGINAL_ONE_NEWTON_TOLERANCE");
  check(machine.requiresFreshWrenchForResume(), "HEALTHY_FORCE_HOLD_STILL_REQUIRES_SENSOR");
  reset(); input.axis_force_error_n = .5;
  for (int i=0; i<15; ++i) tick();
  check(machine.state() == InsertionState::SUCCESS, "UNQUALIFIED_HOLD_SETUP");
  input.axis_force_error_n = 1.4; input.stop_complete = true; input.dt_s = .1;
  InsertionDecision result;
  for (int i=0; i<25 && machine.state() == InsertionState::SUCCESS; ++i) result = tick();
  check(result.state == InsertionState::STOPPING && result.fault == InsertionFault::NO_PROGRESS &&
      result.request_controlled_stop && !result.success && !result.request_online_motion,
      "UNQUALIFIED_STATIONARY_CANDIDATE_FAILS_WITHOUT_AUTO_RESUME_OR_WAITING_GOAL_TIMEOUT");
  check(!machine.requiresFreshWrenchForResume(), "LATCHED_FAILURE_CANNOT_BLOCK_JOINT_STOP");
  check(tick().state == InsertionState::FAILED, "FAILURE_WAITS_FOR_CONTROLLED_STOP_ACK");
  config.pre_stop_force_error_tolerance_n = std::numeric_limits<double>::quiet_NaN();
  try { InsertionStateMachine invalid(config); check(false, "NAN_RESERVE_REJECTED"); }
  catch (const std::invalid_argument &) {}
  return ok ? 0 : 1;
}
