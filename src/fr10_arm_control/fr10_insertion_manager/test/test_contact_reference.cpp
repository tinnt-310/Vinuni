#include <iostream>
#include <limits>
#include "fr10_insertion_manager/contact_reference.hpp"
#include "fr10_insertion_manager/force_session.hpp"
#include "fr10_motion_core/math_utils.hpp"

class ReferenceEchoIK : public fr10_motion::IKProvider {
public:
  fr10_motion::IKResult solve(const fr10_motion::Pose &pose, const fr10_motion::KinematicState &seed) override {
    pose_ = pose; fr10_motion::IKResult result; result.success = true; result.state = seed;
    result.manipulability = result.condition_number = 1.; return result;
  }
  fr10_motion::IKResult validate(const fr10_motion::KinematicState &target,
      const fr10_motion::KinematicState &) override {
    fr10_motion::IKResult result; result.success = true; result.state = target; return result;
  }
  bool forward(const fr10_motion::KinematicState &, fr10_motion::Pose &pose, std::string *) const override {
    pose = pose_; return true;
  }
private:
  fr10_motion::Pose pose_;
};

int main() {
  using namespace fr10_insertion;
  bool ok = true;
  auto check = [&](bool value, const char *name) {
    if (!value) { std::cerr << "FAIL " << name << '\n'; ok = false; }
  };
  ContactReferenceConfig cfg; cfg.search_radius_m = .0025;
  {
    auto passive_cfg = cfg;
    passive_cfg.spiral_enabled = false;
    ContactReference passive(passive_cfg);
    ContactReferenceObservation sample;
    sample.phase = ContactReferencePhase::ALIGN;
    sample.dt_s = .008;
    sample.source_time_s = 1.;
    sample.depth_m = .001;
    sample.measured_wrench.force_n[2] = 1.;
    for (int i = 0; i < 200; ++i) {
      sample.source_time_s += sample.dt_s;
      passive.observe(sample);
      const auto proposal = passive.propose(sample);
      check(proposal.valid && passive.commit(proposal), "PASSIVE_REFERENCE_ACCEPTS_FRESH_OBSERVATIONS");
      check(passive.state().equilibrium[0] == 0. && passive.state().equilibrium[1] == 0. &&
            passive.state().spiral_angle_rad == 0., "PASSIVE_MODE_NEVER_COMMANDS_A_SPIRAL");
    }
    check(passive.state().mode == ContactReferenceMode::SEARCH,
          "PASSIVE_CONTACT_STILL_ENTERS_LOCATING_PRESSURE_PHASE");
    sample.depth_m = .005;
    sample.actual_correction[0] = .001;
    for (int i = 0; i < 12; ++i) {
      sample.source_time_s += sample.dt_s;
      passive.observe(sample);
      const auto proposal = passive.propose(sample);
      check(proposal.valid && passive.commit(proposal), "PASSIVE_CAPTURE_COMMITS_ONLY_ACCEPTED_OBSERVATIONS");
    }
    check(passive.state().mode == ContactReferenceMode::CAPTURED,
          "PASSIVE_MODE_RETAINS_MEASURED_CONTACT_CAPTURE");
  }
  ContactReference guide(cfg);
  ContactReferenceObservation input;
  input.phase = ContactReferencePhase::ALIGN;
  input.dt_s = .008; input.source_time_s = 1.;
  input.measured_wrench.force_n[2] = 1.;
  auto tick = [&] {
    input.source_time_s += input.dt_s;
    guide.observe(input);
    return guide.propose(input);
  };
  for (int i = 0; i < 70; ++i) {
    auto proposed = tick(); check(proposed.valid && guide.commit(proposed), "PASSIVE_TO_BOUNDED_SEARCH");
  }
  check(guide.state().mode == ContactReferenceMode::SEARCH, "BLOCKED_CONTACT_STARTS_SEARCH_BEFORE_NO_PROGRESS");
  const auto previous = guide.state();
  auto rejected = tick();
  check(guide.state().spiral_angle_rad == previous.spiral_angle_rad &&
        guide.state().last_source_time_s == previous.last_source_time_s,
        "UNACCEPTED_SEARCH_DOES_NOT_ADVANCE_PATH_OR_CLOCK");
  auto next = tick(); check(guide.commit(next), "ACCEPTED_SEARCH_COMMITS");
  check(!guide.commit(rejected), "STALE_PROPOSAL_CANNOT_REPLACE_NEW_REFERENCE");
  // Source clock budget is not extended by a stalled worker/discarded proposal.
  input.source_time_s += cfg.search_budget_s;
  guide.observe(input);
  auto timed_out = guide.propose(input);
  check(timed_out.valid && timed_out.next.mode == ContactReferenceMode::EXHAUSTED,
        "SOURCE_BUDGET_EXPIRES_WITHOUT_COMMITTED_SEARCH_TICKS");
  guide.reset(); input.source_time_s = 1.;
  for (int i = 0; i < 25000 && guide.state().mode != ContactReferenceMode::EXHAUSTED; ++i) {
    input.dt_s = (i % 3 == 0) ? .048 : .008;
    const auto old = guide.state().equilibrium;
    auto proposed = tick();
    check(proposed.valid, "FINITE_SEARCH_PROPOSAL");
    check(std::hypot(proposed.next.equilibrium[0], proposed.next.equilibrium[1]) <= cfg.search_radius_m,
          "RADIAL_BUDGET_NEVER_EXCEEDED");
    check(std::hypot(proposed.next.equilibrium[0] - old[0], proposed.next.equilibrium[1] - old[1]) <=
          cfg.search_speed_m_s * input.dt_s + 1e-12, "SPIRAL_SPEED_BOUNDED_AT_VARIABLE_DT");
    guide.commit(proposed);
  }
  check(guide.state().mode == ContactReferenceMode::EXHAUSTED, "EXHAUSTION_IS_TERMINAL_NOT_RESTARTED");
  auto again = tick(); check(again.next.mode == ContactReferenceMode::EXHAUSTED, "NO_AUTOMATIC_RETRY");
  guide.reset(); input.source_time_s = 1.; input.dt_s = .008;
  input.depth_m = .005; input.actual_correction = {.0015, -.001, 0., .02, -.01, 0.};
  for (int i = 0; i < 12; ++i) { auto acquired = tick(); guide.commit(acquired); }
  check(guide.state().mode == ContactReferenceMode::CAPTURED &&
        guide.state().equilibrium[0] == input.actual_correction[0] && guide.state().equilibrium[3] == 0.,
        "BORE_CAPTURE_DOES_NOT_CERTIFY_FREE_SPACE_ORIENTATION");
  const auto entry = guide.state().equilibrium;
  const auto captured_state = guide.state();
  const auto same_entry = contactEquilibriumAtDepth(captured_state, input.depth_m,
      &input.actual_correction);
  check(std::hypot(same_entry[0]-entry[0], same_entry[1]-entry[1]) < 1e-15,
      "CAPTURED_TRANSPORT_PRESERVES_THE_ENTRY_POINT");
  auto accepted_rotation = input.actual_correction;
  accepted_rotation[3] = -.01; accepted_rotation[4] = -.03;
  const auto transported = contactEquilibriumAtDepth(captured_state, .015, &accepted_rotation);
  const auto old_direction = fr10_motion::math::rotateVector(
      fr10_motion::math::rotationVectorToQuaternion(captured_state.capture_rotation_rad), {0.,0.,1.});
  const auto new_direction = fr10_motion::math::rotateVector(
      fr10_motion::math::rotationVectorToQuaternion({-.01,-.03,0.}), {0.,0.,1.});
  for (std::size_t i=0; i<2; ++i)
    check(std::abs(transported[i] - (entry[i] + .015*new_direction[i]/new_direction[2] -
        input.depth_m*old_direction[i]/old_direction[2])) < 1e-15,
        "CAPTURED_TRANSLATION_FOLLOWS_ACCEPTED_DIRECTION_WITH_FIXED_MOUTH_ANCHOR");
  check(transported[3] == 0. && transported[4] == 0. && transported[5] == 0.,
      "CAPTURED_GEOMETRY_DOES_NOT_FREEZE_A_ROTATIONAL_SPRING_OR_CERTIFY_AXIS");
  accepted_rotation[0] += .001; accepted_rotation[1] -= .001;
  check(contactEquilibriumAtDepth(captured_state, .015, &accepted_rotation) == transported,
      "CAPTURED_TRANSPORT_CANNOT_FOLLOW_MEASURED_LATERAL_BIAS");
  input.actual_correction[0] += .0001;
  for (int i = 0; i < 100; ++i) { auto captured = tick(); guide.commit(captured); }
  check(guide.state().mode == ContactReferenceMode::CAPTURED && guide.state().equilibrium == entry,
        "ENTRY_TRANSLATION_REFERENCE_DOES_NOT_FOLLOW_BIAS");
  input.source_time_s += cfg.search_budget_s;
  guide.observe(input);
  check(guide.propose(input).next.mode == ContactReferenceMode::EXHAUSTED,
        "CAPTURED_ALIGNMENT_HAS_INDEPENDENT_SOURCE_DEADLINE");
  input.source_time_s = guide.state().last_source_time_s;
  input.phase = ContactReferencePhase::SEAT; input.measured_wrench.force_n[2] = 0.;
  for (int i = 0; i < 12; ++i) { auto empty = tick(); guide.commit(empty); }
  check(guide.state().mode == ContactReferenceMode::CAPTURED, "UNLOADED_SHOULDER_CANNOT_QUALIFY_AXIS");
  input.measured_wrench.force_n[2] = 1.; input.measured_wrench.torque_nm[0] = .2;
  for (int i = 0; i < 12; ++i) { auto tilted = tick(); guide.commit(tilted); }
  check(guide.state().mode == ContactReferenceMode::CAPTURED, "UNBALANCED_SHOULDER_CANNOT_QUALIFY_AXIS");
  input.measured_wrench.torque_nm[0] = 0.;
  for (int i = 0; i < 12; ++i) { auto acquired = tick(); guide.commit(acquired); }
  check(guide.state().mode == ContactReferenceMode::ACQUIRED && guide.state().equilibrium == input.actual_correction,
        "LOADED_LOW_MOMENT_DWELL_QUALIFIES_FINAL_CORRIDOR");
  const auto original = guide.state().equilibrium;
  const auto at_acquisition = contactEquilibriumAtDepth(guide.state(), input.depth_m);
  check(at_acquisition == original, "ACQUIRED_CORRIDOR_PRESERVES_ANCHOR_POINT");
  const auto along = contactEquilibriumAtDepth(guide.state(), input.depth_m + .010);
  check(std::hypot(along[0] - original[0], along[1] - original[1]) > .0002 &&
        along[3] == original[3] && along[4] == original[4],
        "ACQUIRED_ROTATION_CHANGES_AXIAL_CORRIDOR_WITHOUT_NEW_SENSOR_ANCHOR");
  input.actual_correction[0] += .0001;
  for (int i = 0; i < 100; ++i) { auto frozen = tick(); guide.commit(frozen); }
  check(guide.state().equilibrium == original, "FINAL_ANCHOR_NEVER_FOLLOWS_SENSOR_BIAS_CONTINUOUSLY");
  guide.reset(); input.actual_correction[0] = .01;
  ContactReferenceProposal invalid;
  for (int i = 0; i < 12; ++i) { invalid = tick(); if (invalid.valid) guide.commit(invalid); }
  check(!invalid.valid, "OUT_OF_ENVELOPE_ACQUISITION_CANNOT_REANCHOR");
  input.actual_correction[0] = std::numeric_limits<double>::quiet_NaN();
  check(!guide.propose(input).valid, "NONFINITE_MEASUREMENT_REJECTED");
  guide.reset(); input.source_time_s = 1.; input.dt_s = .008;
  input.phase = ContactReferencePhase::ALIGN;
  input.actual_correction = {cfg.search_radius_m + .00001, 0., 0., cfg.max_rotation_rad + .00001, 0., 0.};
  for (int i=0; i<12; ++i) { auto entry = tick(); guide.commit(entry); }
  input.phase = ContactReferencePhase::SEAT;
  for (int i=0; i<12; ++i) { auto final = tick(); guide.commit(final); }
  check(guide.state().mode == ContactReferenceMode::ACQUIRED &&
      std::hypot(guide.state().equilibrium[0], guide.state().equilibrium[1]) <= cfg.search_radius_m &&
      fr10_force::norm({guide.state().equilibrium[3], guide.state().equilibrium[4], guide.state().equilibrium[5]}) <= cfg.max_rotation_rad,
      "SMALL_QUALIFIED_OBSERVATION_ERROR_PROJECTS_ANCHOR_INSIDE_UNCHANGED_COMMAND_SET");
  guide.reset(); input.actual_correction[3] = cfg.max_rotation_rad + .0001;
  for (int i=0; i<12; ++i) { invalid = tick(); if (invalid.valid) guide.commit(invalid); }
  check(!invalid.valid, "EXCESS_ANGULAR_MEASUREMENT_ERROR_IS_NOT_PROJECTED_AWAY");
  ContactReference legacy;
  input.actual_correction = {}; input.depth_m = 0.;
  auto unchanged = legacy.propose(input);
  check(unchanged.valid && unchanged.next.equilibrium == fr10_force::Vector6{} &&
        unchanged.next.mode == ContactReferenceMode::PASSIVE, "LEGACY_PROFILE_HAS_NO_SEARCH");
  // Sensor observations continue while a motion target is uncommitted. An
  // intervening moment excursion invalidates an already proposed anchor.
  ContactReference observed(cfg);
  ContactReferenceObservation measured;
  measured.phase = ContactReferencePhase::APPROACH;
  measured.depth_m = .005; measured.source_time_s = 10.; measured.dt_s = .008;
  measured.measured_wrench.force_n[2] = 1.;
  auto observe_tick = [&] {
    measured.source_time_s += measured.dt_s;
    observed.observe(measured);
    return observed.propose(measured);
  };
  for (int i=0; i<12; ++i) observed.commit(observe_tick());
  check(observed.state().mode == ContactReferenceMode::CAPTURED, "OBSERVED_BORE_CAPTURE");
  measured.phase = ContactReferencePhase::SEAT;
  ContactReferenceProposal pending_anchor;
  for (int i=0; i<12; ++i) pending_anchor = observe_tick();
  check(pending_anchor.next.mode == ContactReferenceMode::ACQUIRED && observed.canCommit(pending_anchor),
      "SOURCE_DWELL_QUALIFIES_WITHOUT_ADVANCING_MOTION_REFERENCE");
  measured.measured_wrench.torque_nm[0] = .2;
  measured.source_time_s += .008;
  observed.observe(measured); // No target proposal or commit on this tick.
  check(!observed.canCommit(pending_anchor) && !observed.commit(pending_anchor) &&
      observed.state().mode == ContactReferenceMode::CAPTURED,
      "PENDING_ANCHOR_CANNOT_FORGET_INTERVENING_UNQUALIFIED_SENSOR_OBSERVATION");
  measured.measured_wrench.torque_nm[0] = 0.;
  for (int i=0; i<8; ++i) pending_anchor = observe_tick();
  check(pending_anchor.next.mode == ContactReferenceMode::CAPTURED,
      "POST_EXCURSION_WINDOW_REQUIRES_ITS_OWN_CONTIGUOUS_SOURCE_DWELL");
  for (int i=0; i<5; ++i) pending_anchor = observe_tick();
  check(pending_anchor.next.mode == ContactReferenceMode::ACQUIRED, "STABLE_SOURCE_WINDOW_REQUALIFIES");
  measured.source_time_s += cfg.maximum_observation_gap_s + .001;
  observed.observe(measured);
  check(!observed.canCommit(pending_anchor), "SOURCE_GAP_INVALIDATES_UNISSUED_ANCHOR");
  ForceSessionConfig session_cfg;
  session_cfg.state_machine.process_mode = InsertionProcessMode::MATCHED_MATING;
  session_cfg.state_machine.preinsert_hold_s = .008;
  session_cfg.state_machine.no_progress_timeout_s = .2;
  session_cfg.contact_reference = cfg;
  session_cfg.contact_reference.stall_time_s = .04;
  session_cfg.contact_reference.search_budget_s = .4;
  session_cfg.admittance.selected_axes.fill(true);
  session_cfg.admittance.stiffness = {200.,200.,0.,2.,2.,2.};
  session_cfg.admittance.damping = {1000.,1000.,100000.,10.,10.,10.};
  ForceSession session(session_cfg); ReferenceEchoIK ik;
  InsertionGoal goal; goal.selected_axes.fill(true); goal.timeout_s = 10.;
  fr10_force::CartesianPose final_pose; final_pose.position_m[2] = goal.insertion_depth_m;
  fr10_force::Wrench desired; desired.force_n[2] = 5.;
  check(session.start(goal, final_pose, {0.,0.,1.}, desired, 1, 1, 1.), "SEARCH_SESSION_START");
  ForceSessionInput in;
  in.actual_correction = fr10_force::Vector6{};
  in.preinsert_reached = in.tare_complete = true;
  in.dt_s = .008; in.source_time_s = in.now_steady_s = 1.; in.raw_sample.valid = true;
  auto session_tick = [&] {
    in.source_time_s += .008; in.now_steady_s += .008; ++in.raw_sample.sequence;
    in.raw_sample.source_timestamp_s = in.source_time_s;
    in.raw_sample.receipt_steady_s = in.now_steady_s;
    return session.update(in, ik);
  };
  for (int i = 0; i < 4; ++i) {
    auto out = session_tick(); if (out.online_target_valid) session.commitOnlineTarget(out.online_target.sequence, true);
  }
  in.conditioned_wrench.force_n[2] = in.raw_sample.wrench.force_n[2] = 1.;
  for (int i = 0; i < 9; ++i) {
    auto out = session_tick();
    if (out.online_target_valid) session.commitOnlineTarget(out.online_target.sequence, true);
  }
  check(session.contactReference().state().mode == ContactReferenceMode::SEARCH,
        "SEARCH_COMMITS_THROUGH_SOLE_SESSION_TARGET");
  const auto committed = session.contactReference().state();
  auto out = session_tick();
  check(out.online_target_valid && session.contactReference().state().spiral_angle_rad == committed.spiral_angle_rad,
        "SESSION_SEARCH_IS_PENDING_UNTIL_EXECUTOR_ACK");
  session.commitOnlineTarget(out.online_target.sequence, false);
  check(session.contactReference().state().spiral_angle_rad == committed.spiral_angle_rad,
        "REJECTED_ACTUATION_ROLLS_SEARCH_BACK");
  // Pending validator suppresses proposals but never suppresses source deadline.
  in.allow_motion_proposal = false;
  for (int i = 0; i < 60 && session.stateMachine().state() != InsertionState::STOPPING; ++i) out = session_tick();
  check(out.decision.request_controlled_stop && out.decision.fault == InsertionFault::NO_PROGRESS,
        "SEARCH_SOURCE_DEADLINE_STOPS_WITHOUT_AUTO_RETRY");
  in.stop_complete = true;
  check(session_tick().decision.state == InsertionState::FAILED, "SEARCH_FAILURE_ACKNOWLEDGES_CONTROLLED_STOP");
  // An axial load in moving tool axes is not a lateral disturbance while
  // Rx/Ry alignment is still pending. Legacy fixed-frame behavior is retained.
  auto tilted_cfg = session_cfg;
  tilted_cfg.contact_reference.search_budget_s = 10.;
  tilted_cfg.contact_reference.acquisition_hold_s = .016;
  ForceSession tilted(tilted_cfg);
  auto unsupported_axis = goal; unsupported_axis.insertion_axis = 0;
  check(!tilted.start(unsupported_axis, final_pose, {1.,0.,0.}, desired, 2, 1, 1.),
      "ADAPTIVE_MATCHER_REQUIRES_CANONICAL_TOOL_PLUS_Z_INSERTION_FRAME");
  auto tilted_desired = desired;
  tilted_desired.torque_nm = {.001, .002, .003};
  check(tilted.start(goal, final_pose, {0.,0.,1.}, tilted_desired, 2, 1, 1.), "TILTED_PRESSURE_SESSION_START");
  ForceSessionInput tilt_input;
  tilt_input.actual_correction = fr10_force::Vector6{0.,0.,0.,.02,-.01,0.};
  tilt_input.preinsert_reached = tilt_input.tare_complete = true;
  tilt_input.raw_sample.valid = true; tilt_input.dt_s = .008;
  tilt_input.source_time_s = tilt_input.now_steady_s = 1.;
  tilt_input.actual_depth_m = .005;
  tilt_input.conditioned_wrench.force_n = fr10_motion::math::rotateVector(
      fr10_motion::math::rotationVectorToQuaternion({.02,-.01,0.}), {0.,0.,1.});
  tilt_input.conditioned_wrench.torque_nm = fr10_motion::math::rotateVector(
      fr10_motion::math::rotationVectorToQuaternion({.02,-.01,0.}), tilted_desired.torque_nm);
  ForceSessionOutput tilt_output;
  const auto tilt_tick = [&](bool publish = true) {
    tilt_input.source_time_s += .008; tilt_input.now_steady_s += .008;
    tilt_input.raw_sample.sequence++;
    tilt_input.raw_sample.source_timestamp_s = tilt_input.source_time_s;
    tilt_input.raw_sample.receipt_steady_s = tilt_input.now_steady_s;
    tilt_output = tilted.update(tilt_input, ik);
    if (publish && tilt_output.online_target_valid) tilted.commitOnlineTarget(tilt_output.online_target.sequence, true);
  };
  for (int i=0; i<12; ++i) tilt_tick();
  check(tilted.contactReference().state().mode == ContactReferenceMode::CAPTURED &&
      std::abs(tilt_output.force_error[0]) < 1e-12 && std::abs(tilt_output.force_error[1]) < 1e-12 &&
      std::abs(tilt_output.force_error[2]) < 1e-12,
      "MEASURED_TOOL_PRESSURE_FRAME_AVOIDS_SPURIOUS_XY_ERROR_BEFORE_AXIS_ACQUISITION");
  for (std::size_t i=3; i<6; ++i)
    check(std::abs(tilt_output.force_error[i]) < 1e-12,
        "DESIRED_FORCE_AND_MOMENT_USE_THE_SAME_TOOL_REFERENCE_FRAME");
  const auto previous_pose = tilt_output.corrected_pose;
  tilt_input.conditioned_wrench.force_n[0] += .2;
  tilt_tick();
  fr10_force::Vector3 delta{};
  for (std::size_t i=0; i<3; ++i) delta[i] = tilt_output.corrected_pose.position_m[i] - previous_pose.position_m[i];
  const double length = fr10_force::norm(delta);
  check(tilt_output.online_target_valid && length > 1e-12 && delta[0] < 0.,
      "ACCEPTED_LATERAL_COMPLIANCE_HAS_A_NONAXIAL_TRANSLATION_CHORD");
  for (std::size_t i=0; i<3; ++i)
    check(std::abs(tilt_output.online_target.desired_translation_tangent[i] - delta[i]/length) < 1e-12,
        "EXECUTOR_CARTESIAN_TANGENT_FOLLOWS_ACCEPTED_CORRECTED_PATH");
  tilt_input.actual_depth_m = goal.insertion_depth_m;
  tilt_input.conditioned_wrench.force_n = fr10_motion::math::rotateVector(
      fr10_motion::math::rotationVectorToQuaternion({.02,-.01,0.}), {0.,0.,5.});
  for (int i=0; i<50 && tilted.stateMachine().state() != InsertionState::SEATING; ++i) tilt_tick();
  check(tilted.stateMachine().state() == InsertionState::SEATING, "PENDING_ANCHOR_SEATING_SETUP");
  tilt_tick(); tilt_tick(); tilt_tick(false);
  const auto anchor_sequence = tilt_output.online_target.sequence;
  check(tilt_output.online_target_valid && tilted.canCommitOnlineTarget(anchor_sequence) &&
      tilted.contactReference().state().mode == ContactReferenceMode::CAPTURED,
      "LOADED_ANCHOR_PROPOSAL_AWAITS_ACTUATOR_ADMISSION");
  const auto low_pressure = fr10_motion::math::rotateVector(
      fr10_motion::math::rotationVectorToQuaternion({.02,-.01,0.}), {0.,0.,1.});
  for (std::size_t i=0; i<3; ++i)
    check(std::abs(tilt_output.force_error[i] + 4.*low_pressure[i]) < 1e-12,
        "UNCOMMITTED_AXIS_USES_LOCATING_PRESSURE_DURING_SEATING");
  tilt_input.allow_motion_proposal = false;
  tilt_input.conditioned_wrench.torque_nm[0] = .2;
  tilt_tick(false);
  check(tilt_output.safety.safe() && !tilted.canCommitOnlineTarget(anchor_sequence) &&
      !tilted.commitOnlineTarget(anchor_sequence, false) &&
      tilted.contactReference().state().mode == ContactReferenceMode::CAPTURED,
      "WORKER_PENDING_MOMENT_INVALIDATES_ANCHOR_BEFORE_ACTUATION_WITHOUT_RESTART_OR_FAULT");
  tilt_input.allow_motion_proposal = true;
  tilt_input.conditioned_wrench.torque_nm = fr10_motion::math::rotateVector(
      fr10_motion::math::rotationVectorToQuaternion({.02,-.01,0.}), tilted_desired.torque_nm);
  for (int i=0; i<8 && tilted.contactReference().state().mode != ContactReferenceMode::ACQUIRED; ++i)
    tilt_tick();
  check(tilted.contactReference().state().mode == ContactReferenceMode::ACQUIRED,
      "RECOVERED_LOADED_AXIS_REQUIRES_NEW_DWELL_AND_ACCEPTED_PUBLICATION");
  tilt_tick();
  for (std::size_t i=0; i<3; ++i)
    check(std::abs(tilt_output.force_error[i]) < 1e-12,
        "FULL_SEATING_PRESSURE_BEGINS_AFTER_AXIS_COMMIT");
  return ok ? 0 : 1;
}
