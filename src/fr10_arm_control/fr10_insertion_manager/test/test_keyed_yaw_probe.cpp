#include <algorithm>
#include <cmath>
#include <iostream>
#include "fr10_insertion_manager/contact_reference.hpp"
#include "fr10_insertion_manager/force_session.hpp"

int main() {
  using namespace fr10_insertion;
  bool ok = true;
  const auto check = [&](bool condition, const char *name) {
    if (!condition) { std::cerr << "FAIL " << name << '\n'; ok = false; }
  };
  KeyedYawProbeConfig config; config.drive_torque_nm = .15;
  config.contact_free_half_span_rad = .00828;
  KeyedYawProbe probe(config);
  KeyedYawObservation input; input.valid = true; input.source_time_s = 1.;
  const auto observe = [&] { input.source_time_s += .008; probe.observe(input); };
  observe(); auto start = probe.propose(input, true);
  check(probe.state().mode == KeyedYawProbeMode::IDLE && probe.torque(start.next, 0.) == -.15,
      "PROBE_START_HAS_NO_STATE_OR_JOINT_SIDE_EFFECT");
  observe();
  check(probe.commit(start), "START_ADMISSION_CAN_SURVIVE_NORMAL_WORKER_HANDOVER");
  input.rotation_rad[2] = -.02; input.torque_z_nm = -.12;
  KeyedYawProposal lower;
  for (int i=0; i<9; ++i) { observe(); lower = probe.propose(input, false); }
  check(lower.next.mode == KeyedYawProbeMode::SEEK_POSITIVE && probe.canCommit(lower),
      "LOWER_CONTACT_REQUIRES_CONTIGUOUS_SOURCE_DWELL");
  input.torque_z_nm = 0.; observe();
  check(!probe.canCommit(lower) && !probe.commit(lower),
      "UNISSUED_BOUNDARY_CANNOT_FORGET_CONTACT_LOSS");
  input.torque_z_nm = -.12;
  for (int i=0; i<9; ++i) { observe(); lower = probe.propose(input, false); }
  check(probe.commit(lower) && probe.torque(probe.state(), -.02) == .15,
      "ACCEPTED_LOWER_BOUNDARY_REVERSES_BOUNDED_MOMENT");
  input.rotation_rad[2] = -.005; input.torque_z_nm = .12;
  KeyedYawProposal upper;
  for (int i=0; i<9; ++i) { observe(); upper = probe.propose(input, false); }
  check(upper.next.mode == KeyedYawProbeMode::CENTERING &&
      std::abs(upper.next.center_rad + .0125) < 1e-15 && probe.commit(upper),
      "OPPOSITE_CONTACTS_DEFINE_AN_ESTIMATED_CLEARANCE_MIDPOINT");
  input.rotation_rad[2] = probe.state().center_rad; input.torque_z_nm = 0.;
  KeyedYawProposal centered;
  for (int i=0; i<33; ++i) { observe(); centered = probe.propose(input, false); }
  check(centered.next.mode == KeyedYawProbeMode::COMPLETE && probe.commit(centered) &&
      probe.torque(probe.state(), input.rotation_rad[2]) == 0.,
      "CENTER_DWELL_REMOVES_PROBE_MOMENT_WITHOUT_A_JOINT_COMMAND");
  probe.reset(); input.rotation_rad = {};
  observe(); check(probe.commit(probe.propose(input, true)), "BOUNDARY_SCAN_START");
  input.rotation_rad[2] = -config.max_rotation_rad;
  for (int i=0; i<9; ++i) { observe(); lower = probe.propose(input, false); }
  check(probe.commit(lower), "HARD_ROTATION_ENVELOPE_TRUNCATES_LOWER_SCAN");
  input.rotation_rad[2] += .006; input.torque_z_nm = .12;
  for (int i=0; i<9; ++i) { observe(); upper = probe.propose(input, false); }
  check(upper.next.mode == KeyedYawProbeMode::CENTERING &&
      std::abs(upper.next.center_rad + config.max_rotation_rad) < 1e-15 &&
      upper.next.negative_contact_strength == 0. && upper.next.positive_contact_strength == 1.,
      "QUIET_ENVELOPE_ENDPOINT_IS_NOT_MISTAKEN_FOR_A_PHYSICAL_WALL");
  input.source_time_s += config.maximum_observation_gap_s + .001;
  probe.observe(input);
  check(!probe.canCommit(upper), "SOURCE_GAP_REVOKES_UNISSUED_CENTER_ESTIMATE");
  // Mirror the scan and vary reaction strength continuously. A quiet actuator
  // envelope is not a socket wall, and unequal evidence must not introduce a
  // preferred rotation sign. All transitions still need observed dwell and
  // accepted publication, including a rejected center proposal.
  for (double negative_strength : {0., .25, .75, 1.}) {
    for (double positive_strength : {0., .25, .75, 1.}) {
      probe.reset(); input.rotation_rad = {}; input.torque_z_nm = 0.;
      observe(); check(probe.commit(probe.propose(input, true)), "WEIGHTED_SCAN_START");
      input.rotation_rad[2] = -config.max_rotation_rad;
      input.torque_z_nm = -negative_strength*config.contact_torque_nm;
      for (int i=0; i<9; ++i) { observe(); lower = probe.propose(input, false); }
      check(probe.commit(lower), "WEIGHTED_LOWER_ADMISSION");
      input.rotation_rad[2] = config.max_rotation_rad;
      input.torque_z_nm = positive_strength*config.contact_torque_nm;
      for (int i=0; i<9; ++i) { observe(); upper = probe.propose(input, false); }
      const double expected = (config.max_rotation_rad-config.contact_free_half_span_rad)*
          (positive_strength-negative_strength)/
          std::max(1., negative_strength+positive_strength);
      check(upper.next.mode == KeyedYawProbeMode::CENTERING &&
          std::abs(upper.next.center_rad-expected) < 1e-15 &&
          std::abs(upper.next.center_rad) <= config.max_rotation_rad,
          "CENTER_ESTIMATE_IS_CONTINUOUS_MIRROR_SYMMETRIC_AND_BOUNDED");
      check(probe.state().mode == KeyedYawProbeMode::SEEK_POSITIVE &&
          probe.state().positive_contact_strength == 0.,
          "UNPUBLISHED_CENTER_DOES_NOT_ADVANCE_ENDPOINT_EVIDENCE");
      check(probe.commit(upper), "WEIGHTED_CENTER_ADMISSION");
    }
  }
  // Independent rigid keyed interval, truncated by the immutable 2-degree
  // actuator envelope. Exercise interior errors as well as the extreme input
  // errors: the previous quiet-endpoint heuristic passed2deg but failed1.6deg.
  for (double true_center : {-.03490658503988659, -.027925268031909273,
                             .027925268031909273, .03490658503988659}) {
    probe.reset(); input.rotation_rad = {}; input.torque_z_nm = 0.;
    observe(); check(probe.commit(probe.propose(input, true)), "MECHANICAL_INTERVAL_SCAN_START");
    const double physical_lower = true_center-config.contact_free_half_span_rad;
    const double physical_upper = true_center+config.contact_free_half_span_rad;
    input.rotation_rad[2] = std::max(-config.max_rotation_rad, physical_lower);
    input.torque_z_nm = physical_lower >= -config.max_rotation_rad ? -.12 : 0.;
    for (int i=0; i<9; ++i) { observe(); lower = probe.propose(input, false); }
    check(probe.commit(lower), "MECHANICAL_LOWER_ENDPOINT_ADMISSION");
    input.rotation_rad[2] = std::min(config.max_rotation_rad, physical_upper);
    input.torque_z_nm = physical_upper <= config.max_rotation_rad ? .12 : 0.;
    for (int i=0; i<9; ++i) { observe(); upper = probe.propose(input, false); }
    check(upper.next.mode == KeyedYawProbeMode::CENTERING &&
        std::abs(upper.next.center_rad-true_center) < 1e-15 && probe.commit(upper),
        "MECHANICAL_CLEARANCE_IDENTIFIES_ONE_SIDED_CENTER_AT_INTERIOR_AND_EXTREME_ERRORS");
  }
  KeyedYawProbeConfig unknown_clearance = config;
  unknown_clearance.contact_free_half_span_rad = std::numeric_limits<double>::infinity();
  KeyedYawProbe uncalibrated(unknown_clearance);
  input.rotation_rad = {}; input.torque_z_nm = 0.; input.source_time_s += .008;
  uncalibrated.observe(input); check(uncalibrated.commit(uncalibrated.propose(input, true)), "UNKNOWN_CLEARANCE_START");
  input.rotation_rad[2] = -config.max_rotation_rad;
  for (int i=0; i<9; ++i) { input.source_time_s += .008; uncalibrated.observe(input); lower = uncalibrated.propose(input, false); }
  check(uncalibrated.commit(lower), "UNKNOWN_CLEARANCE_LOWER_ADMISSION");
  input.rotation_rad[2] += .006; input.torque_z_nm = .12;
  for (int i=0; i<9; ++i) { input.source_time_s += .008; uncalibrated.observe(input); upper = uncalibrated.propose(input, false); }
  check(std::abs(upper.next.center_rad-(-config.max_rotation_rad+.003)) < 1e-15,
      "UNKNOWN_MECHANICAL_CLEARANCE_RETAINS_LEGACY_MIDPOINT_API");
  ContactReferenceConfig reference_config;
  reference_config.search_radius_m = .0025;
  reference_config.keyed_yaw = config;
  reference_config.acquisition_hold_s = .016;
  ContactReference reference(reference_config);
  ContactReferenceObservation measured;
  measured.valid = true; measured.phase = ContactReferencePhase::INSERT;
  measured.depth_m = .005; measured.dt_s = .008; measured.source_time_s = 1.;
  const auto tick = [&] {
    measured.source_time_s += .008; reference.observe(measured);
    return reference.propose(measured);
  };
  for (int i=0; i<5; ++i) check(reference.commit(tick()), "ENTRY_CAPTURE_ADMISSION");
  check(reference.state().mode == ContactReferenceMode::CAPTURED, "PROBE_REQUIRES_MEASURED_ENTRY_CAPTURE");
  measured.phase = ContactReferencePhase::SEAT; measured.measured_wrench.force_n[2] = 1.;
  ContactReferenceProposal staged;
  for (int i=0; i<4; ++i) staged = tick();
  check(staged.next.mode == ContactReferenceMode::CAPTURED && staged.yaw_probe.valid &&
      staged.yaw_probe.next.mode == KeyedYawProbeMode::SEEK_NEGATIVE,
      "LOW_MOMENT_ALONE_CANNOT_ACQUIRE_UNOBSERVED_KEYED_YAW");
  measured.measured_wrench.torque_nm[0] = .2; tick();
  check(!reference.canCommit(staged) && reference.yawProbe().state().mode == KeyedYawProbeMode::IDLE,
      "START_RECHECKS_LOADED_WINDOW_BEFORE_ACTUATOR_ADMISSION");
  measured.measured_wrench.torque_nm[0] = 0.;
  for (int i=0; i<5; ++i) check(reference.commit(tick()), "PROBE_START_REQUALIFIED");
  check(reference.yawProbe().state().mode == KeyedYawProbeMode::SEEK_NEGATIVE &&
      reference.state().mode == ContactReferenceMode::CAPTURED,
      "START_AND_SEARCH_STAY_INSIDE_ACCEPTED_REFERENCE_TRANSACTION");
  measured.actual_correction[5] = -reference_config.max_rotation_rad;
  ContactReferenceProposal envelope_endpoint;
  for (int i=0; i<10; ++i) envelope_endpoint = tick();
  check(envelope_endpoint.yaw_probe.next.mode == KeyedYawProbeMode::SEEK_POSITIVE,
      "LOADED_ENVELOPE_ENDPOINT_QUALIFIES_WHILE_WORKER_PENDING");
  measured.measured_wrench.force_n[2] = 0.; tick();
  check(!reference.canCommit(envelope_endpoint) &&
      reference.yawProbe().state().mode == KeyedYawProbeMode::SEEK_NEGATIVE,
      "UNLOADED_ENVELOPE_ENDPOINT_REVOKES_UNISSUED_YAW_PHASE");
  measured.measured_wrench.force_n[2] = 1.;
  measured.actual_correction[5] = -.02; measured.measured_wrench.torque_nm[2] = -.12;
  for (int i=0; i<10; ++i) check(reference.commit(tick()), "LOWER_REFERENCE_TRANSACTION");
  measured.actual_correction[5] = -.005; measured.measured_wrench.torque_nm[2] = .12;
  for (int i=0; i<10; ++i) check(reference.commit(tick()), "UPPER_REFERENCE_TRANSACTION");
  measured.actual_correction[5] = reference.yawProbe().state().center_rad;
  measured.measured_wrench.torque_nm[2] = 0.;
  for (int i=0; i<40; ++i) check(reference.commit(tick()), "CENTER_AND_LOADED_REQUALIFICATION");
  check(reference.state().mode == ContactReferenceMode::ACQUIRED &&
      std::abs(reference.state().equilibrium[5] + .0125) < 1e-15,
      "FINAL_AXIS_REQUIRES_COMPLETED_PROBE_AND_A_NEW_LOADED_DWELL");
  ForceSessionConfig session_config;
  session_config.state_machine.process_mode = InsertionProcessMode::MATCHED_MATING;
  session_config.contact_reference = reference_config;
  session_config.contact.enter_force_n = 1.; session_config.contact.exit_force_n = .5;
  session_config.admittance.drive_deadband[2] = .5;
  session_config.admittance.selected_axes.fill(true);
  session_config.admittance.stiffness[0] = session_config.admittance.stiffness[1] = 200.;
  try { ForceSession invalid(session_config); check(false, "UNREACHABLE_LOCATING_PRESSURE_REJECTED"); }
  catch (const std::invalid_argument &error) {
    check(std::string(error.what()).find("locating pressure") != std::string::npos,
        "LOCATING_PRESSURE_REJECTION_IDENTIFIES_ITS_ROOT_CONTRACT");
  }
  session_config.contact_reference.search_force_n = 2.;
  try { ForceSession valid(session_config); }
  catch (const std::invalid_argument &) { check(false, "QUALIFIABLE_LOCATING_PRESSURE_ACCEPTED"); }

  // A force-only P moment stalls with Coulomb reaction: 0.025/25=1 mrad,
  // exceeding the unchanged50 urad centering tolerance. The hybrid position
  // objective must converge through the actual transactional admittance law.
  for (double reaction : {-.025, .025}) {
    for (double nominal_moment : {0., .015, .2}) {
      KeyedYawProbe centering(config);
      KeyedYawProbeState target; target.mode = KeyedYawProbeMode::CENTERING; target.center_rad = .005;
      fr10_force::AdmittanceConfig dynamics;
      dynamics.mass[5] = .1; dynamics.damping[5] = 20.; dynamics.stiffness[5] = .02;
      dynamics.drive_deadband[5] = 1e-6; dynamics.max_velocity[5] = .002;
      dynamics.max_acceleration[5] = .02; dynamics.max_correction[5] = config.max_rotation_rad;
      dynamics.max_rotation_correction_norm_rad = config.max_rotation_rad;
      dynamics.selected_axes[5] = true;
      fr10_force::AdmittanceModel model(dynamics);
      fr10_force::AdmittanceState initial; initial.correction[5] = target.center_rad+.0025;
      model.reset(initial);
      fr10_force::Vector6 equilibrium{}; equilibrium[5] = target.center_rad;
      double previous_error = .0025;
      for (int i=0; i<1000; ++i) {
        const double previous_yaw = model.state().correction[5];
        const double command = centering.desiredTorque(target, previous_yaw, reaction, nominal_moment);
        check(std::abs(command) <= config.drive_torque_nm, "CENTERING_PRESERVES_PROBE_MOMENT_LIMIT");
        fr10_force::Vector6 error{}; error[5] = command-reaction;
        const auto proposal = model.propose(error, .008, equilibrium);
        check(proposal.valid, "FRICTION_CENTERING_ADMITTANCE_PROPOSAL_VALID");
        if (i%17 == 0) {
          check(model.state().correction[5] == previous_yaw,
              "REJECTED_CENTERING_TARGET_DOES_NOT_ADVANCE_POSITION");
          continue;
        }
        check(model.commit(proposal), "FRICTION_CENTERING_ADVANCES_ONLY_ACCEPTED_TARGET");
        previous_error = model.state().correction[5]-target.center_rad;
      }
      check(std::abs(previous_error) <= config.angle_tolerance_rad,
          "POSITION_CENTERING_OVERCOMES_BOTH_SIGNS_OF_CONTACT_REACTION");
      target.mode = KeyedYawProbeMode::SEEK_NEGATIVE;
      check(centering.desiredTorque(target, 0., reaction, nominal_moment) == nominal_moment-config.drive_torque_nm,
          "CONTACT_SEEK_REMAINS_AN_UNMODIFIED_FORCE_OBJECTIVE");
      target.mode = KeyedYawProbeMode::COMPLETE;
      check(centering.desiredTorque(target, 0., reaction, nominal_moment) == nominal_moment,
          "COMPLETED_PROBE_REMOVES_REACTION_FEEDFORWARD_FOR_LOADED_REQUALIFICATION");
    }
  }
  return ok ? 0 : 1;
}
