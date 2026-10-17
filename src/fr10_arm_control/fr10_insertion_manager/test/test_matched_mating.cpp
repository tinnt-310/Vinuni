#include <iostream>
#include "fr10_insertion_manager/force_session.hpp"

int main() {
  using namespace fr10_insertion;
  using fr10_force::ContactState;
  bool ok = true;
  auto check = [&](bool value, const char *name) { if (!value) { std::cerr << name << '\n'; ok = false; } };
  InsertionStateConfig config;
  config.process_mode = InsertionProcessMode::MATCHED_MATING;
  config.preinsert_hold_s = .008;
  config.alignment_hold_s = .016;
  config.completion_hold_s = .016;
  InsertionStateMachine fsm(config);
  InsertionGoal goal; goal.selected_axes.fill(true); goal.insertion_depth_m = .015;
  check(fsm.start(goal, 1), "fixed Z axis accepts multi-axis compliance");
  auto bad = goal; bad.insertion_axis = 0;
  check(!validInsertionAxes(bad.selected_axes, 0, config.process_mode), "profile axis independent of mask");
  check(!validInsertionAxes(goal.selected_axes, 2, InsertionProcessMode::AXIAL_CONTACT), "axial profile stays single axis");
  InsertionObservation input; input.source_time_s = 1.; input.dt_s = .008;
  auto tick = [&] { input.source_time_s += input.dt_s; return fsm.update(input); };
  input.preinsert_reached = true; tick(); tick();
  input.tare_complete = true;
  check(tick().state == InsertionState::APPROACH_COMPLIANT, "compliant approach phase");
  input.contact_state = ContactState::CONTACT_CONFIRMED;
  check(tick().state == InsertionState::CONTACT_ALIGN, "edge contact alignment");
  input.lateral_force_n = 2.; tick();
  check(fsm.state() == InsertionState::CONTACT_ALIGN, "lateral load blocks advance");
  input.lateral_force_n = 0; input.torque_norm_nm = 0; tick(); tick();
  check(fsm.state() == InsertionState::INSERT_COMPLIANT, "stable alignment resumes insertion");
  input.contact_state = ContactState::NO_CONTACT; input.actual_depth_m = .004;
  check(tick().state == InsertionState::INSERT_COMPLIANT, "clearance gap is not lost contact");
  input.actual_depth_m = .015;
  check(tick().state == InsertionState::SEATING, "actual depth enters seating");
  check(tick().state == InsertionState::SEATING, "nominal arrival alone cannot complete");
  input.contact_state = ContactState::CONTACT_CONFIRMED;
  check(tick().state == InsertionState::VERIFY_INSERTION, "seat contact enters verification");
  input.contact_state = ContactState::NO_CONTACT;
  check(tick().state == InsertionState::SEATING,
        "unverified intermittent seat contact returns to bounded seating, never success");
  input.contact_state = ContactState::CONTACT_CONFIRMED;
  check(tick().state == InsertionState::VERIFY_INSERTION,"confirmed contact restarts verification dwell");
  input.axis_force_error_n = 2; tick(); tick();
  check(fsm.state() == InsertionState::VERIFY_INSERTION, "force error blocks success");
  input.axis_force_error_n = 0; input.mating_valid = false; tick(); tick();
  check(fsm.state() == InsertionState::VERIFY_INSERTION,
        "capture-valid pose cannot complete outside CAD mating clearance");
  input.mating_valid = true;
  input.torque_error_norm_nm = .13; tick(); tick();
  check(fsm.state() == InsertionState::VERIFY_INSERTION,
        "quiet axial force cannot complete with residual seating moment");
  input.torque_error_norm_nm = 0.; input.lateral_force_error_n = 1.1; tick(); tick();
  check(fsm.state() == InsertionState::VERIFY_INSERTION,
        "quiet axial force cannot complete with residual transverse seating force");
  input.lateral_force_error_n = 0.;
  input.torque_norm_nm = .15; tick(); tick();
  check(fsm.state() == InsertionState::SUCCESS,
        "nominal nonzero moment is allowed when full wrench error settles");
  check(tick().state == InsertionState::SUCCESS, "publication does not imply actual HOLD");
  input.stop_complete = true;
  input.torque_error_norm_nm = .11;
  check(!tick().success, "seating moment is rechecked at actual HOLD boundary");
  input.torque_error_norm_nm = 0.;
  input.mating_valid = false;
  check(!tick().success, "seated fit is rechecked at actual HOLD boundary");
  input.mating_valid = true;
  check(tick().success, "stationary executor boundary completes HOLD");
  input.source_time_s += goal.timeout_s+1.;
  check(tick().success, "seated HOLD outlives insertion deadline");
  input.retract_requested = true;
  check(tick().state == InsertionState::RETRACT, "explicit HOLD retract retains ownership until verified retreat");
  input.retract_requested = false;
  input.retract_complete = true;
  check(tick().success, "verified retreat releases HOLD boundary");
  input.stop_complete = false;
  input.safety_fault = fr10_force::ForceSafetyFault::SENSOR_OVERLOAD;
  check(tick().state == InsertionState::STOPPING, "HOLD continues sensor safety monitoring");
  input.safety_fault = fr10_force::ForceSafetyFault::NONE;
  input.torque_norm_nm = 0.;
  input.retract_complete = false;
  fsm.reset(); fsm.start(goal, input.source_time_s);
  input.preinsert_reached = input.tare_complete = true; tick(); tick(); tick();
  input.contact_state = ContactState::NO_CONTACT; input.actual_depth_m = -.02;
  input.dt_s = .008;
  for (int i = 0; i < 260 && fsm.state() != InsertionState::STOPPING; ++i) tick();
  check(fsm.state() == InsertionState::STOPPING && fsm.fault() == InsertionFault::NO_PROGRESS,
        "stuck insertion controlled stop, no automatic retract");
  fsm.reset();fsm.start(goal,input.source_time_s);
  input.planning_failed = true;input.stop_complete = false;
  const auto rejected = tick();
  check(rejected.state == InsertionState::STOPPING && rejected.fault == InsertionFault::PLANNING_FAILED,
        "rejected preinsert returns typed planning failure immediately instead of waiting goal timeout");
  input.planning_failed = false;
  input.stop_complete = true;tick();
  input.retract_requested = true;input.safety_fault = fr10_force::ForceSafetyFault::STALE_WRENCH;
  check(tick().state == InsertionState::FAILED,"explicit recovery retract still rejects stale safety data");
  input.safety_fault = fr10_force::ForceSafetyFault::NONE;
  check(tick().state == InsertionState::RETRACT,"fresh explicit recovery retract clears only the insertion fault");
  input.retract_requested = false;

  input.planning_failed = true; input.stop_complete = false;
  check(tick().fault == InsertionFault::PLANNING_FAILED,
        "rejected recovery retract also requests a typed controlled stop");
  input.planning_failed = false; input.stop_complete = true; tick();
  input.retract_requested = true;
  check(tick().state == InsertionState::RETRACT,
        "a fresh explicit retry can recover from a rejected retract");
  input.retract_requested = false;

  {
    auto settling_config=config;settling_config.no_progress_timeout_s=.05;
    InsertionStateMachine settling(settling_config);
    InsertionObservation sample;sample.dt_s=.008;sample.source_time_s=1.;
    sample.preinsert_reached=sample.tare_complete=true;
    sample.actual_depth_m=goal.insertion_depth_m;
    sample.contact_state=ContactState::CONTACT_CONFIRMED;
    sample.torque_error_norm_nm=.20;
    check(settling.start(goal,1.),"full-wrench progress test start");
    const auto step=[&] {sample.source_time_s+=sample.dt_s;return settling.update(sample);};
    for(unsigned i=0;i<5;++i) step();
    for(unsigned i=0;i<12;++i) {
      sample.torque_error_norm_nm-=.008;step();
      check(settling.state()==InsertionState::VERIFY_INSERTION,
            "moment convergence renews physical progress while depth and axial force remain stationary");
    }
    sample.torque_error_norm_nm=.095;step();step();
    check(settling.state()==InsertionState::SUCCESS,"full-wrench dwell completes after moment converges");
    sample.torque_error_norm_nm=.12;sample.stop_complete=true;
    for(unsigned i=0;i<10 && settling.state()==InsertionState::SUCCESS;++i) step();
    check(settling.fault()==InsertionFault::NO_PROGRESS && settling.state()==InsertionState::STOPPING,
          "stationary moment failure retains the finite watchdog without auto resume");
  }

  fr10_force::ForceSafetyConfig safety;
  safety.max_lateral_correction_norm_m = .002;
  safety.max_rotation_correction_norm_rad = .03490658503988659;
  fr10_force::ForceSafetyMonitor monitor(safety);
  fr10_force::ForceSafetyInput sample;
  sample.raw_sample.valid = true; sample.raw_sample.sequence = 1;
  sample.raw_sample.source_timestamp_s = 1; sample.raw_sample.receipt_steady_s = 1;
  sample.now_steady_s = 1;
  sample.correction.position[0] = sample.correction.position[1] = .0015;
  check(monitor.update(sample).fault == fr10_force::ForceSafetyFault::CORRECTION_LIMIT,
        "2 mm is lateral norm, not per-axis square");
  monitor.reset(); sample.correction.position.fill(0);
  sample.correction.position[3] = sample.correction.position[4] = .026;
  check(monitor.update(sample).fault == fr10_force::ForceSafetyFault::CORRECTION_LIMIT,
        "2 degrees is total rotation vector norm");
  InsertionProgressMonitor progress(.2, .0001, .05);
  check(progress.update(.01, -.03, false, 10.), "negative approach depth initializes progress datum");
  for (int i=0; i<18; ++i) progress.update(.01, -.03-(i%2)*.0002, false, 10.);
  check(!progress.update(.02, -.03, false, 10.), "depth oscillation is not insertion progress");
  progress.reset(); progress.update(.1, 0., true, 20.);
  check(progress.update(.1, 0., true, 18.), "alignment force improvement renews progress without axial plunging");
  check(!progress.update(.21, 0., true, 18.), "stagnant edge alignment times out");
  progress.reset(); progress.update(.1, .015, true, 5., 1);
  check(progress.update(.15, .015, true, 4., 1),
        "seating force convergence is progress at a stationary depth");
  progress.update(.05, .015, false, 0.);
  progress.update(.05, .015, true, 5., 1);
  check(!progress.update(.11, .015, true, 4., 1),
        "phase cycling and repeated force oscillations cannot renew progress");
  InsertionProgressMonitor slow_seating(2., .00001, .05);
  bool advancing = true;
  for (int i=0; i<500; ++i)
    advancing &= slow_seating.update(.008, .0149+i*.008*.000025, true, 5., 1);
  check(advancing, "bounded25 um/s seating remains physical progress at calibrated10 um resolution");
  bool stopped = false;
  for (int i=0; i<260; ++i)
    stopped |= !slow_seating.update(.008, .015+(i%2)*.000001, true, 5., 1);
  check(stopped, "stationary1 um ripple still times out in unchanged2 s window");
  return ok ? 0 : 1;
}
