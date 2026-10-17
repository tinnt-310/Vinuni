#include <iostream>

#include "fr10_insertion_manager/insertion_state_machine.hpp"

namespace {

bool check(bool condition, const char *name) {
  if (!condition) std::cerr << "FAIL " << name << '\n';
  return condition;
}

fr10_insertion::InsertionGoal goal() {
  fr10_insertion::InsertionGoal result;
  result.selected_axes[2] = true;
  result.insertion_axis = 2;
  result.insertion_depth_m = 0.01;
  result.insertion_speed_m_s = 0.002;
  result.timeout_s = 5.0;
  return result;
}

} // namespace

int main() {
  bool ok = true;
  fr10_insertion::InsertionStateConfig config;
  config.preinsert_hold_s = 0.016;
  config.completion_hold_s = 0.024;
  fr10_insertion::InsertionStateMachine machine(config);
  auto invalid = goal();
  invalid.selected_axes[0] = true;
  ok &= check(!machine.start(invalid, 0.0),
              "MULTI_AXIS_DEPLOYMENT_REJECTED");
  machine.reset();
  ok &= check(machine.start(goal(), 0.0), "VALID_GOAL_ACCEPTED");

  fr10_insertion::InsertionObservation observation;
  observation.dt_s = 0.008;
  observation.source_time_s = 0.008;
  observation.preinsert_reached = true;
  auto decision = machine.update(observation);
  ok &= check(decision.state ==
                  fr10_insertion::InsertionState::HOLD_PREINSERT,
              "MOVE_TO_PREINSERT_HOLD");
  observation.source_time_s += 0.008;
  machine.update(observation);
  observation.source_time_s += 0.008;
  decision = machine.update(observation);
  ok &= check(decision.state == fr10_insertion::InsertionState::FT_ZERO &&
                  decision.request_tare,
              "HOLD_TO_TARE");
  observation.tare_complete = true;
  observation.source_time_s += 0.008;
  decision = machine.update(observation);
  ok &= check(decision.state == fr10_insertion::InsertionState::APPROACH &&
                  decision.request_online_motion,
              "TARE_TO_APPROACH");

  observation.contact_state =
      fr10_force::ContactState::CONTACT_CONFIRMED;
  observation.source_time_s += 0.008;
  decision = machine.update(observation);
  ok &= check(decision.state ==
                  fr10_insertion::InsertionState::CONTACT_DETECTED,
              "CONTACT_DETECTED");
  observation.source_time_s += 0.008;
  decision = machine.update(observation);
  ok &= check(decision.state ==
                  fr10_insertion::InsertionState::FORCE_INSERTION,
              "CONTACT_TO_FORCE_INSERTION");

  observation.actual_depth_m = 0.01;
  observation.axis_force_error_n = 0.5;
  observation.axis_force_rate_n_s = 1.0;
  observation.source_time_s += 0.008;
  decision = machine.update(observation);
  ok &= check(decision.state ==
                  fr10_insertion::InsertionState::VERIFY_INSERTION,
              "DEPTH_TO_VERIFY");
  observation.actual_depth_m = 0.012;
  for (int cycle=0; cycle<4; ++cycle) {
    observation.source_time_s += 0.008;
    decision = machine.update(observation);
  }
  ok &= check(decision.state == fr10_insertion::InsertionState::VERIFY_INSERTION,
              "OVERSHOOT_IS_NOT_WITHIN_DEPTH_TOLERANCE");
  observation.actual_depth_m = 0.01;
  for (int cycle = 0; cycle < 3; ++cycle) {
    observation.source_time_s += 0.008;
    decision = machine.update(observation);
  }
  ok &= check(decision.state == fr10_insertion::InsertionState::SUCCESS,
              "DEPTH_FORCE_STABILITY_DWELL");
  observation.source_time_s += 0.008;
  decision = machine.update(observation);
  ok &= check(decision.state == fr10_insertion::InsertionState::SUCCESS &&
                  decision.request_controlled_stop && !decision.terminal && !decision.success,
              "SUCCESS_WAITS_FOR_ACKNOWLEDGED_STATIONARY_HOLD");
  observation.stop_complete = true;
  observation.source_time_s += 0.008;
  decision = machine.update(observation);
  ok &= check(decision.state == fr10_insertion::InsertionState::HOLD &&
                  decision.terminal && decision.success,
              "SUCCESS_TO_HOLD");

  machine.reset();
  ok &= check(machine.start(goal(), 1.0), "FAULT_TEST_START");
  observation = {};
  observation.dt_s = 0.008;
  observation.source_time_s = 1.008;
  observation.safety_fault = fr10_force::ForceSafetyFault::STALE_WRENCH;
  decision = machine.update(observation);
  ok &= check(decision.state == fr10_insertion::InsertionState::STOPPING &&
                  decision.request_controlled_stop && !decision.success,
              "FAULT_REQUESTS_CONTROLLED_STOP");
  observation.safety_fault = fr10_force::ForceSafetyFault::NONE;
  observation.stop_complete = true;
  observation.source_time_s += 0.008;
  decision = machine.update(observation);
  ok &= check(decision.state == fr10_insertion::InsertionState::FAILED &&
                  decision.terminal && !decision.success,
              "FAULT_NEVER_REPORTS_SUCCESS");
  machine.reset();
  machine.start(goal(),2.0);
  observation={};observation.dt_s=.008;observation.source_time_s=2.008;
  observation.tracking_valid=false;
  decision=machine.update(observation);
  ok &= check(decision.state==fr10_insertion::InsertionState::STOPPING &&
      decision.safety_fault==fr10_force::ForceSafetyFault::TRACKING_FAILURE &&
      decision.request_controlled_stop,"PREINSERT_TRACKING_STOP_IS_TERMINAL_FAULT");
  observation.source_time_s+=.008;observation.stop_complete=true;
  decision=machine.update(observation);
  ok &= check(decision.state==fr10_insertion::InsertionState::FAILED && decision.terminal &&
      !decision.success,"PREINSERT_TRACKING_FAULT_ACKNOWLEDGED");
  config.process_mode=fr10_insertion::InsertionProcessMode::MATCHED_MATING;
  for(const double depth:{.005,-.005}) {
    fr10_insertion::InsertionStateMachine retract_machine(config);
    auto retract_goal=goal();retract_goal.selected_axes.fill(true);
    retract_goal.completion_behavior=fr10_insertion::CompletionBehavior::RETRACT;
    retract_machine.start(retract_goal,3.0);
    observation={};observation.dt_s=.008;observation.source_time_s=3.008;
    observation.canceled=true;observation.stop_complete=true;
    retract_machine.update(observation);
    observation.canceled=false;observation.retract_requested=true;
    observation.actual_depth_m=depth;observation.mating_valid=false;
    observation.source_time_s+=.008;
    decision=retract_machine.update(observation);
    ok &= check(depth>=0. ? decision.state==fr10_insertion::InsertionState::FAILED &&
       decision.safety_fault==fr10_force::ForceSafetyFault::TRACKING_FAILURE :
       decision.state==fr10_insertion::InsertionState::RETRACT,
       "RETRACT_KEYED_CLEARANCE_REQUIRED_WHILE_INSIDE_SOCKET");
  }
  return ok ? 0 : 1;
}
