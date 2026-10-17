#include "fr10_insertion_manager/force_session.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include "fr10_motion_core/math_utils.hpp"

namespace fr10_insertion {
namespace {

ForceSessionConfig withCorrectionEnvelope(ForceSessionConfig config) {
  for (std::size_t i=0; i<fr10_force::kCartesianDofs; ++i)
    config.admittance.max_correction[i] = std::min(
        config.admittance.max_correction[i], config.safety.max_correction[i]);
  config.admittance.max_lateral_correction_norm_m = std::min(
      config.admittance.max_lateral_correction_norm_m, config.safety.max_lateral_correction_norm_m);
  config.admittance.max_rotation_correction_norm_rad = std::min(
      config.admittance.max_rotation_correction_norm_rad, config.safety.max_rotation_correction_norm_rad);
  config.contact_reference.max_rotation_rad = std::min(
      config.contact_reference.max_rotation_rad, config.admittance.max_rotation_correction_norm_rad);
  config.contact_reference.keyed_yaw.max_yaw_rad = std::min(
      config.contact_reference.keyed_yaw.max_yaw_rad, config.admittance.max_correction[5]);
  return config;
}

fr10_force::Wrench referenceWrench(fr10_force::Wrench desired, const fr10_force::Vector6 &orientation) {
  const auto rotation = fr10_motion::math::rotationVectorToQuaternion(
      {orientation[3], orientation[4], orientation[5]});
  desired.force_n = fr10_motion::math::rotateVector(rotation, desired.force_n);
  desired.torque_nm = fr10_motion::math::rotateVector(rotation, desired.torque_nm);
  return desired;
}

} // namespace

ForceSession::ForceSession(ForceSessionConfig config)
    : config_(withCorrectionEnvelope(config)), state_machine_(config_.state_machine),
      controller_(config_.admittance,
                  config_.rotation_base_from_insertion),
      contact_detector_(config_.contact), safety_monitor_(config_.safety),
      verification_slope_(config_.state_machine.completion_hold_s,config_.safety.stale_timeout_s),
      ik_stepper_(config_.continuous_ik), contact_reference_(config_.contact_reference) {
  const auto scale_valid = [](double scale) {
    return std::isfinite(scale) && scale > 0.0 && scale <= 1.0;
  };
  if (!std::isfinite(config_.target_validity_s) ||
      !(config_.target_validity_s > 0.0) ||
      !std::isfinite(config_.max_nominal_lead_m) || !(config_.max_nominal_lead_m>0.) ||
      !scale_valid(config_.velocity_scale) ||
      !scale_valid(config_.acceleration_scale) ||
      !scale_valid(config_.jerk_scale)) {
    throw std::invalid_argument("force session configuration is invalid");
  }
  // MATCHED_MATING verifies the full wrench. A drive quiet region outside
  // that objective can suppress the correction needed to enter VERIFY.
  // Reject incompatible profiles before any goal can command the robot.
  if (config_.state_machine.process_mode == InsertionProcessMode::MATCHED_MATING) {
    const auto &deadband = config_.admittance.drive_deadband;
    const auto &completion = config_.state_machine;
    if (std::hypot(deadband[0], deadband[1]) > completion.alignment_force_tolerance_n ||
        std::hypot(deadband[3], deadband[4], deadband[5]) > completion.alignment_torque_tolerance_nm ||
        deadband[2] > std::min(completion.force_error_tolerance_n,
                              completion.pre_stop_force_error_tolerance_n))
      throw std::invalid_argument("matched drive deadband exceeds the completion/pre-stop wrench objective");
  }
  if (contact_reference_.enabled()) {
    if ((contact_reference_.yawProbe().enabled() || contact_reference_.lateralProbe(0).enabled()) &&
        config_.contact_reference.search_force_n <=
            std::max(config_.contact.enter_force_n, config_.contact_reference.contact_force_n) +
            config_.admittance.drive_deadband[2])
      throw std::invalid_argument("locating pressure must exceed contact qualification plus axial deadband");
    if (config_.state_machine.process_mode != InsertionProcessMode::MATCHED_MATING ||
        config_.contact_reference.stall_time_s >= config_.state_machine.no_progress_timeout_s ||
        config_.admittance.stiffness[0] <= 0. || config_.admittance.stiffness[1] <= 0. ||
        config_.contact_reference.search_radius_m > config_.safety.max_lateral_correction_norm_m)
      throw std::invalid_argument("search requires matched mode, positive XY stiffness and bounded radial envelope");
    for (std::size_t i = 0; i < 2; ++i)
      if (config_.contact_reference.search_radius_m > config_.admittance.max_correction[i])
        throw std::invalid_argument("search radius exceeds admittance correction envelope");
  }
}

bool ForceSession::start(const InsertionGoal &goal,
                         const fr10_force::CartesianPose &final_pose,
                         const fr10_force::Vector3 &axis_base,
                         const fr10_force::Wrench &desired_wrench,
                         uint64_t generation, uint64_t scene_revision,
                         double source_time_s) {
  const double lateral_drive=config_.contact_reference.lateral_probe.drive_load;
  if (generation == 0 ||
      lateral_drive+std::abs(desired_wrench.force_n[0])>config_.safety.max_force_n[0] ||
      lateral_drive+std::abs(desired_wrench.force_n[1])>config_.safety.max_force_n[1] || !fr10_force::finite(desired_wrench) ||
      (contact_reference_.enabled() && config_.contact_reference.search_force_n > config_.safety.max_force_n[2]) ||
      (config_.contact_reference.keyed_yaw.drive_torque_nm + std::abs(desired_wrench.torque_nm[2]) >
          config_.safety.max_torque_nm[2]) ||
      (contact_reference_.enabled() && goal.insertion_axis != 2) ||
      (contact_reference_.enabled() && !std::all_of(goal.selected_axes.begin(), goal.selected_axes.end(),
          [](bool selected) { return selected; })) ||
      goal.selected_axes != config_.admittance.selected_axes ||
      !nominal_provider_.reset(final_pose, axis_base,
                               goal.preinsert_distance_m,
                               goal.insertion_depth_m,
                               goal.insertion_speed_m_s) ||
      !state_machine_.start(goal, source_time_s)) {
    return false;
  }
  desired_wrench_ = desired_wrench;
  generation_ = generation;
  scene_revision_ = scene_revision;
  online_sequence_ = 0;
  pending_sequence_ = 0;
  deferred_motion_dt_s_ = 0.0;
  pending_controller_result_ = {};
  committed_controller_result_ = {};
  controller_.reset();
  config_.admittance.insertion_axis = goal.insertion_axis;
  config_.safety.insertion_axis = goal.insertion_axis;
  safety_monitor_ = fr10_force::ForceSafetyMonitor(config_.safety);
  controller_.setConfig(config_.admittance);
  contact_reference_.reset();
  pending_reference_ = {};
  contact_detector_.reset();
  safety_monitor_.reset();
  verification_slope_.reset();
  return true;
}

bool ForceSession::reframePreinsert(const fr10_force::CartesianPose &final_pose,
    const fr10_force::Vector3 &axis_base, const fr10_force::RotationMatrix &rotation,
    uint64_t scene_revision) {
  if (state_machine_.state()!=InsertionState::MOVE_PREINSERT || pending_sequence_!=0) return false;
  auto nominal=nominal_provider_;
  const auto &goal=state_machine_.goal();
  if (!nominal.reset(final_pose,axis_base,goal.preinsert_distance_m,
                     goal.insertion_depth_m,goal.insertion_speed_m_s)) return false;
  for(double value:rotation) if(!std::isfinite(value)) return false;
  try { controller_=fr10_force::ForcePositionController(config_.admittance,rotation); }
  catch(const std::invalid_argument &) {return false;}
  config_.rotation_base_from_insertion=rotation;
  nominal_provider_=nominal;
  scene_revision_=scene_revision;
  return true;
}

ForceSessionOutput ForceSession::update(const ForceSessionInput &input,
                                        fr10_motion::IKProvider &provider) {
  ForceSessionOutput output;
  output.actual_depth_m = input.actual_depth_m;
  const auto before = state_machine_.state();
  // During adaptive alignment pressure follows measured tool axes. Otherwise
  // a corrected Rx/Ry turns an axial load into spurious lateral force error.
  // Once acquired, the stationary accepted corridor supplies the force frame.
  const auto wrench_orientation = !contact_reference_.enabled() ? fr10_force::Vector6{} :
      contact_reference_.state().mode == ContactReferenceMode::ACQUIRED ?
          contact_reference_.state().equilibrium : input.actual_correction.value_or(fr10_force::Vector6{});
  // The safety/state clocks run every tick. Motion integration accounts for
  // elapsed ticks while the worker checks the previous uncommitted proposal.
  // This avoids reducing commanded speed to one third merely due to latency.
  const double motion_dt_s = std::min(config_.target_validity_s,
      input.dt_s + deferred_motion_dt_s_);
  if (input.allow_motion_proposal) deferred_motion_dt_s_ = 0.0;
  else deferred_motion_dt_s_ = motion_dt_s;
  const std::size_t axis = state_machine_.goal().insertion_axis;
  output.contact_evidence = contact_detector_.update(input.conditioned_wrench, axis, input.dt_s);
  output.contact_state = output.contact_evidence.axial;

  const bool advance_nominal = before == InsertionState::APPROACH ||
      before == InsertionState::FORCE_INSERTION ||
      before == InsertionState::APPROACH_COMPLIANT ||
      before == InsertionState::INSERT_COMPLIANT ||
      before == InsertionState::SEATING ||
      (before == InsertionState::VERIFY_INSERTION &&
       config_.state_machine.process_mode == InsertionProcessMode::AXIAL_CONTACT);
  auto nominal_proposal = nominal_provider_;
  const auto &goal=state_machine_.goal();
  // A physical shoulder may lie within the accepted depth tolerance, while
  // the acknowledged reference leads measured FK. Brake before that entire
  // uncertainty interval, rather than waiting for the nominal final depth.
  const double nominal_speed=config_.state_machine.process_mode == InsertionProcessMode::MATCHED_MATING
      ? boundedSeatingSpeed(goal.insertion_speed_m_s,goal.insertion_depth_m-input.actual_depth_m,
          config_.max_nominal_lead_m+config_.state_machine.depth_tolerance_m,
          config_.admittance.max_velocity[axis],config_.admittance.max_acceleration[axis])
      : goal.insertion_speed_m_s;
  output.nominal_pose = nominal_proposal.advance(motion_dt_s*nominal_speed/goal.insertion_speed_m_s,advance_nominal,
      config_.state_machine.process_mode == InsertionProcessMode::MATCHED_MATING
          ? input.actual_depth_m - committed_controller_result_.correction.position[axis] + config_.max_nominal_lead_m
          : std::numeric_limits<double>::infinity());
  output.corrected_pose = output.nominal_pose;

  const bool force_enabled = before == InsertionState::CONTACT_DETECTED ||
      before == InsertionState::FORCE_INSERTION ||
      before == InsertionState::VERIFY_INSERTION ||
      before == InsertionState::APPROACH_COMPLIANT ||
      before == InsertionState::CONTACT_ALIGN ||
      before == InsertionState::INSERT_COMPLIANT ||
      before == InsertionState::SEATING;
  auto task_wrench = desired_wrench_;
  const bool seating = before == InsertionState::SEATING || before == InsertionState::VERIFY_INSERTION;
  if (config_.state_machine.process_mode == InsertionProcessMode::MATCHED_MATING &&
      (!seating || (contact_reference_.enabled() &&
          contact_reference_.state().mode != ContactReferenceMode::ACQUIRED))) {
    // Free approach has zero pressure; searching and unqualified seating
    // retain locating pressure. Full seating pressure requires a committed
    // loaded-axis anchor, so it cannot friction-lock an unaligned connector.
    task_wrench.force_n[axis] = contactReferencePending(contact_reference_.state().mode) ||
        (contact_reference_.enabled() && seating)
        ? config_.contact_reference.search_force_n : 0.0;
  }
  task_wrench = referenceWrench(task_wrench, wrench_orientation);
  fr10_force::ForcePositionResult controller_result;
  ContactReferenceProposal reference_proposal;
  ContactReferenceObservation reference_input;
  reference_input.valid = input.raw_sample.valid && input.actual_correction.has_value();
  reference_input.phase = !force_enabled ? ContactReferencePhase::INACTIVE :
      before == InsertionState::CONTACT_ALIGN ? ContactReferencePhase::ALIGN :
      before == InsertionState::SEATING || before == InsertionState::VERIFY_INSERTION ? ContactReferencePhase::SEAT :
      before == InsertionState::INSERT_COMPLIANT ? ContactReferencePhase::INSERT : ContactReferencePhase::APPROACH;
  reference_input.actual_correction = input.actual_correction.value_or(fr10_force::Vector6{});
  reference_input.measured_wrench = input.conditioned_wrench;
  reference_input.nominal_wrench = task_wrench;
  reference_input.depth_m = input.actual_depth_m;
  reference_input.source_time_s = input.source_time_s;
  reference_input.dt_s = input.dt_s;
  contact_reference_.observe(reference_input);
  if (force_enabled) {
    reference_input.dt_s = motion_dt_s;
    reference_proposal = contact_reference_.propose(reference_input);
    const auto desired=contact_reference_.desiredWrench(reference_proposal,reference_input,task_wrench);
    controller_result = controller_.propose(
        output.nominal_pose, desired, input.conditioned_wrench,
        motion_dt_s, reference_proposal.valid ? contactEquilibriumAtDepth(reference_proposal.next,
            nominal_proposal.nominalDepth(), committed_controller_result_.valid ?
                &committed_controller_result_.correction.position : nullptr) : fr10_force::Vector6{});
    if (controller_result.valid) {
      output.corrected_pose = controller_result.corrected_pose;
      output.correction = controller_result.correction;
      output.force_error = controller_result.force_error;
    }
  } else {
    // Keep diagnostics at the last acknowledged corrected pose during HOLD
    // and STOPPING; do not propose or integrate a new force correction.
    if (committed_controller_result_.valid) {
      output.corrected_pose = committed_controller_result_.corrected_pose;
      output.correction = committed_controller_result_.correction;
    }
    const auto desired = referenceWrench(desired_wrench_, wrench_orientation).vector();
    const auto measured = input.conditioned_wrench.vector();
    for (std::size_t dof = 0; dof < 6; ++dof) {
      output.force_error[dof] = desired[dof] - measured[dof];
    }
  }

  fr10_force::ForceSafetyInput safety_input;
  safety_input.raw_sample = input.raw_sample;
  safety_input.conditioned_wrench = input.conditioned_wrench;
  safety_input.correction = output.correction;
  safety_input.depth_m = input.actual_depth_m;
  safety_input.now_steady_s = input.now_steady_s;
  safety_input.contact_required =
      ((before == InsertionState::FORCE_INSERTION || before == InsertionState::VERIFY_INSERTION) &&
       config_.state_machine.process_mode == InsertionProcessMode::AXIAL_CONTACT) ||
      before == InsertionState::SUCCESS;
  safety_input.contact_required = safety_input.contact_required ||
      (before == InsertionState::HOLD &&
       state_machine_.goal().completion_behavior == CompletionBehavior::HOLD);
  safety_input.contact_confirmed = output.contact_state ==
      fr10_force::ContactState::CONTACT_CONFIRMED;
  safety_input.collision_free = input.collision_free;
  safety_input.scene_unchanged = input.scene_unchanged;
  safety_input.command_accepted = input.command_accepted;
  safety_input.conditioned_reference_changed =
      before == InsertionState::FT_ZERO && input.conditioned_reference_changed;
  safety_input.tracking_ok = input.tracking_valid &&
      input.lateral_valid && input.orientation_valid;
  safety_input.robot_ok = input.robot_ok;
  const bool safety_active = before == InsertionState::FT_ZERO ||
      before == InsertionState::APPROACH ||
      before == InsertionState::CONTACT_DETECTED ||
      before == InsertionState::FORCE_INSERTION ||
      before == InsertionState::VERIFY_INSERTION ||
      before == InsertionState::SUCCESS ||
      before == InsertionState::HOLD ||
      before == InsertionState::FAILED ||
      before == InsertionState::RETRACT ||
      before == InsertionState::APPROACH_COMPLIANT ||
      before == InsertionState::CONTACT_ALIGN ||
      before == InsertionState::INSERT_COMPLIANT ||
      before == InsertionState::SEATING;
  if (safety_active) {
    output.safety = safety_monitor_.update(safety_input);
  }
  if (!input.base_stationary && output.safety.safe()) {
    output.safety.fault = fr10_force::ForceSafetyFault::TRACKING_FAILURE;
  }
  if (!input.scene_unchanged && output.safety.safe())
    output.safety.fault = fr10_force::ForceSafetyFault::SCENE_CHANGED;
  if (input.command_source_time_s && !std::isfinite(*input.command_source_time_s)) {
    output.safety.fault = fr10_force::ForceSafetyFault::INVALID_WRENCH;
  }
  if (force_enabled && !controller_result.valid && output.safety.safe()) {
    output.safety.fault = fr10_force::ForceSafetyFault::INVALID_WRENCH;
  }
  if (force_enabled && contact_reference_.enabled() &&
      (!input.actual_correction || !reference_proposal.valid) && output.safety.safe())
    output.safety.fault = fr10_force::ForceSafetyFault::TRACKING_FAILURE;
  output.reference_mode = contact_reference_.state().mode;
  output.yaw_probe_mode = contact_reference_.yawProbe().state().mode;
  for(std::size_t i=0;i<2;++i)output.lateral_probe_modes[i]=contact_reference_.lateralProbe(i).state().mode;

  if(input.conditioned_reference_changed) verification_slope_.reset();
  auto verification_sample=input.raw_sample;
  verification_sample.wrench=input.conditioned_wrench;
  output.verification_force_rate_n_s=verification_slope_.update(verification_sample);

  InsertionObservation observation;
  observation.source_time_s = input.source_time_s;
  observation.dt_s = input.dt_s;
  observation.actual_depth_m = input.actual_depth_m;
  observation.axis_force_error_n = axis < 6 ? output.force_error[axis] : 0.0;
  observation.axis_force_rate_n_s = axis<3 && output.verification_force_rate_n_s
      ? (*output.verification_force_rate_n_s)[axis] : std::numeric_limits<double>::quiet_NaN();
  observation.contact_state = output.contact_state;
  double lateral_squared = 0.0;
  for (std::size_t i=0; i<3; ++i) if (i != axis)
    lateral_squared += input.conditioned_wrench.force_n[i]*input.conditioned_wrench.force_n[i];
  observation.lateral_force_n = std::sqrt(lateral_squared);
  observation.torque_norm_nm = fr10_force::norm(input.conditioned_wrench.torque_nm);
  // Completion uses the nominal goal, excluding transient locating/probe loads.
  // Rotate the goal exactly once into the same canonical frame as feedback.
  const auto nominal_wrench = referenceWrench(desired_wrench_, wrench_orientation);
  double lateral_error_squared = 0.0;
  fr10_force::Vector3 torque_error{};
  for (std::size_t i=0; i<3; ++i) {
    const double force_error = nominal_wrench.force_n[i] - input.conditioned_wrench.force_n[i];
    if (i != axis) lateral_error_squared += force_error * force_error;
    torque_error[i] = nominal_wrench.torque_nm[i] - input.conditioned_wrench.torque_nm[i];
  }
  observation.lateral_force_error_n = std::sqrt(lateral_error_squared);
  observation.torque_error_norm_nm = fr10_force::norm(torque_error);
  observation.safety_fault = output.safety.fault;
  observation.preinsert_reached = input.preinsert_reached;
  observation.planning_failed = input.planning_failed;
  observation.tare_complete = input.tare_complete;
  observation.lateral_valid = input.lateral_valid;
  observation.orientation_valid = input.orientation_valid;
  observation.mating_valid = input.mating_valid;
  observation.tracking_valid = input.tracking_valid;
  observation.robot_fault = !input.robot_ok;
  observation.stop_complete = input.stop_complete;
  observation.retract_complete = input.retract_complete;
  observation.retract_requested = input.retract_requested;
  observation.canceled = input.canceled;
  observation.reference_mode = contact_reference_.state().mode;
  observation.reference_required = contact_reference_.enabled();
  observation.search_deadline_source_s = contact_reference_.state().search_start_source_s +
      config_.contact_reference.search_budget_s;
  output.decision = state_machine_.update(observation);

  if (!input.allow_motion_proposal || !output.decision.request_online_motion ||
      output.decision.request_controlled_stop) {
    return output;
  }

  output.ik = ik_stepper_.step(toMotionPose(output.corrected_pose),
                               input.joint_seed, provider);
  if (!output.ik.success) {
    observation.dt_s = 0.0;
    observation.safety_fault = fr10_force::ForceSafetyFault::IK_FAILURE;
    output.decision = state_machine_.update(observation);
    output.safety.fault = fr10_force::ForceSafetyFault::IK_FAILURE;
    return output;
  }

  output.online_target.position_rad = output.ik.state.position_rad;
  // IK candidates inherit seed derivatives for continuity validation; those
  // derivatives are not a target velocity.  Each online position target must
  // be able to settle.  Ruckig preserves live q/qdot/qddot on retarget while
  // braking toward a zero-derivative endpoint.
  output.online_target.target_velocity_rad_s.fill(0.0);
  output.online_target.target_acceleration_rad_s2.fill(0.0);
  output.online_target.corrected_pose = toMotionPose(output.corrected_pose);
  output.online_target.desired_translation_tangent =
      nominal_provider_.axisBase();
  if (contact_reference_.enabled()) {
    output.online_target.desired_translation_tangent = fr10_motion::math::rotateVector(
        output.corrected_pose.orientation_xyzw, {0.,0.,1.});
    if (committed_controller_result_.valid) {
      fr10_force::Vector3 delta{};
      for (std::size_t i=0; i<3; ++i)
        delta[i] = output.corrected_pose.position_m[i] - committed_controller_result_.corrected_pose.position_m[i];
      const double length = fr10_force::norm(delta);
      if (length > 1e-12)
        for (std::size_t i=0; i<3; ++i) output.online_target.desired_translation_tangent[i] = delta[i]/length;
    }
  }
  output.online_target.velocity_scale = config_.velocity_scale;
  output.online_target.acceleration_scale = config_.acceleration_scale;
  output.online_target.jerk_scale = config_.jerk_scale;
  output.online_target.generation = generation_;
  output.online_target.sequence = ++online_sequence_;
  output.online_target.wrench_sequence = input.raw_sample.sequence;
  output.online_target.scene_revision = scene_revision_;
  output.online_target.timestamp_s = input.command_source_time_s.value_or(input.source_time_s);
  output.online_target.valid_until_s =
      output.online_target.timestamp_s + config_.target_validity_s;
  output.online_target_valid = true;
  pending_sequence_ = output.online_target.sequence;
  pending_controller_result_ = controller_result;
  pending_nominal_provider_ = nominal_proposal;
  pending_reference_ = reference_proposal;
  return output;
}

bool ForceSession::commitOnlineTarget(uint64_t sequence,
                                      bool accepted_and_published) {
  if (sequence == 0 || sequence != pending_sequence_) {
    return false;
  }
  const bool needs_controller_commit = pending_controller_result_.valid;
  // The session has exclusive ownership of both models. Preflight every
  // participant before mutating either, so failed admission cannot partially
  // advance an anchor while leaving admittance and nominal motion behind.
  const bool committed = accepted_and_published && canCommitOnlineTarget(sequence);
  if (committed) {
    if (pending_reference_.valid) contact_reference_.commit(pending_reference_);
    if (needs_controller_commit) controller_.commit(pending_controller_result_);
  }
  if (committed && needs_controller_commit) {
    committed_controller_result_ = pending_controller_result_;
  }
  if (committed) nominal_provider_ = pending_nominal_provider_;
  pending_sequence_ = 0;
  pending_controller_result_ = {};
  pending_reference_ = {};
  return committed;
}

bool ForceSession::canCommitOnlineTarget(uint64_t sequence) const {
  return sequence != 0 && sequence == pending_sequence_ &&
      (!pending_reference_.valid || contact_reference_.canCommit(pending_reference_)) &&
      (!pending_controller_result_.valid || controller_.canCommit(pending_controller_result_));
}

void ForceSession::reset() {
  state_machine_.reset();
  controller_.reset();
  contact_reference_.reset();
  pending_reference_ = {};
  contact_detector_.reset();
  safety_monitor_.reset();
  verification_slope_.reset();
  desired_wrench_ = {};
  generation_ = 0;
  scene_revision_ = 0;
  online_sequence_ = 0;
  pending_sequence_ = 0;
  pending_controller_result_ = {};
  committed_controller_result_ = {};
  deferred_motion_dt_s_ = 0.0;
}

bool ForceSession::active() const {
  const auto state = state_machine_.state();
  return state != InsertionState::IDLE && state != InsertionState::FAILED &&
      state != InsertionState::HOLD;
}

fr10_motion::Pose ForceSession::toMotionPose(
    const fr10_force::CartesianPose &pose) {
  return {pose.position_m, pose.orientation_xyzw};
}

} // namespace fr10_insertion
