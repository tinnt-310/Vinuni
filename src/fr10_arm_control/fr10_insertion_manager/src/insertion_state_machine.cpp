#include "fr10_insertion_manager/insertion_state_machine.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fr10_insertion {
namespace {
double seatingWrenchObjective(const InsertionObservation &input,
    const InsertionStateConfig &config) {
  double objective = std::abs(input.axis_force_error_n) / config.force_error_tolerance_n;
  if (config.process_mode == InsertionProcessMode::MATCHED_MATING) {
    if (!std::isfinite(input.lateral_force_error_n) || input.lateral_force_error_n < 0. ||
        !std::isfinite(input.torque_error_norm_nm) || input.torque_error_norm_nm < 0.)
      return std::numeric_limits<double>::infinity();
    objective = std::max({objective,
        input.lateral_force_error_n / config.alignment_force_tolerance_n,
        input.torque_error_norm_nm / config.alignment_torque_tolerance_nm});
  }
  return std::isfinite(objective) ? objective : std::numeric_limits<double>::infinity();
}

bool seatingQualified(const InsertionObservation &input, const InsertionGoal &goal,
    const InsertionStateConfig &config, double force_tolerance_n) {
  return std::isfinite(input.actual_depth_m) && std::isfinite(input.axis_force_error_n) &&
      std::isfinite(input.axis_force_rate_n_s) &&
      std::abs(input.actual_depth_m - goal.insertion_depth_m) <= config.depth_tolerance_m &&
      std::abs(input.axis_force_error_n) <= force_tolerance_n &&
      seatingWrenchObjective(input, config) <= 1. &&
      std::abs(input.axis_force_rate_n_s) <= config.force_rate_tolerance_n_s &&
      input.contact_state == fr10_force::ContactState::CONTACT_CONFIRMED &&
      input.lateral_valid && input.orientation_valid && input.mating_valid && input.tracking_valid &&
      !input.robot_fault && (!input.reference_required || input.reference_mode == ContactReferenceMode::ACQUIRED);
}
} // namespace

const char *insertionFaultName(InsertionFault fault) {
  switch (fault) {
  case InsertionFault::NONE: return "NONE";
  case InsertionFault::INVALID_GOAL: return "INVALID_GOAL";
  case InsertionFault::TIMEOUT: return "TIMEOUT";
  case InsertionFault::CANCELED: return "CANCELED";
  case InsertionFault::FORCE_SAFETY: return "FORCE_SAFETY";
  case InsertionFault::CLOCK_JUMP: return "CLOCK_JUMP";
  case InsertionFault::NO_PROGRESS: return "NO_PROGRESS";
  case InsertionFault::PLANNING_FAILED: return "PLANNING_FAILED";
  }
  return "UNKNOWN_FAULT";
}

bool validInsertionAxes(const std::array<bool, 6> &selected, std::size_t axis,
                        InsertionProcessMode mode) {
  if (axis >= 3 || !selected[axis]) return false;
  if (mode == InsertionProcessMode::MATCHED_MATING) return axis == 2;
  return std::count(selected.begin(), selected.end(), true) == 1;
}

InsertionStateMachine::InsertionStateMachine(InsertionStateConfig config)
    : config_(config), progress_(config.no_progress_timeout_s, config.minimum_progress_m,
                                config.minimum_alignment_progress_fraction) {
  if (!std::isfinite(config_.preinsert_hold_s) ||
      !std::isfinite(config_.depth_tolerance_m) ||
      !std::isfinite(config_.force_error_tolerance_n) ||
      !std::isfinite(config_.force_rate_tolerance_n_s) ||
      !std::isfinite(config_.completion_hold_s) ||
      config_.preinsert_hold_s < 0.0 ||
      !(config_.depth_tolerance_m > 0.0) ||
      !(config_.force_error_tolerance_n > 0.0) ||
      !(config_.force_rate_tolerance_n_s > 0.0) ||
      !(config_.completion_hold_s > 0.0) ||
      !std::isfinite(config_.alignment_hold_s) || config_.alignment_hold_s <= 0 ||
      !std::isfinite(config_.alignment_force_tolerance_n) || config_.alignment_force_tolerance_n <= 0 ||
      !std::isfinite(config_.alignment_torque_tolerance_nm) || config_.alignment_torque_tolerance_nm <= 0 ||
      !std::isfinite(config_.no_progress_timeout_s) || config_.no_progress_timeout_s <= 0 ||
      !std::isfinite(config_.minimum_progress_m) || config_.minimum_progress_m <= 0 ||
      std::isnan(config_.pre_stop_force_error_tolerance_n) || config_.pre_stop_force_error_tolerance_n <= 0.) {
    throw std::invalid_argument("insertion state configuration is invalid");
  }
}

bool InsertionStateMachine::start(const InsertionGoal &goal,
                                  double source_time_s) {
  const bool valid = state_ == InsertionState::IDLE &&
      std::isfinite(source_time_s) &&
      std::isfinite(goal.preinsert_distance_m) &&
      std::isfinite(goal.insertion_depth_m) &&
      std::isfinite(goal.insertion_speed_m_s) &&
      std::isfinite(goal.desired_axis_force_n) &&
      std::isfinite(goal.timeout_s) && goal.preinsert_distance_m >= 0.0 &&
      goal.insertion_depth_m > 0.0 && goal.insertion_speed_m_s > 0.0 &&
      goal.timeout_s > 0.0 && validInsertionAxes(goal.selected_axes, goal.insertion_axis,
                                                config_.process_mode);
  if (!valid) {
    fault_ = InsertionFault::INVALID_GOAL;
    return false;
  }
  goal_ = goal;
  start_time_s_ = source_time_s;
  previous_time_s_ = source_time_s;
  fault_ = InsertionFault::NONE;
  safety_fault_ = fr10_force::ForceSafetyFault::NONE;
  enter(InsertionState::MOVE_PREINSERT);
  return true;
}

void InsertionStateMachine::enter(InsertionState state) {
  state_ = state;
  phase_elapsed_s_ = 0.0;
  stable_elapsed_s_ = 0.0;
  // Realignment must not reset a physical no-progress watchdog indefinitely.
  if (state == InsertionState::MOVE_PREINSERT || state == InsertionState::FT_ZERO ||
      state == InsertionState::APPROACH_COMPLIANT) progress_.reset();
}

void InsertionStateMachine::fail(
    InsertionFault fault, fr10_force::ForceSafetyFault safety_fault) {
  fault_ = fault;
  safety_fault_ = safety_fault;
  enter(InsertionState::STOPPING);
}

InsertionDecision InsertionStateMachine::update(
    const InsertionObservation &observation) {
  if (state_ == InsertionState::FAILED && observation.retract_requested &&
      observation.safety_fault == fr10_force::ForceSafetyFault::NONE &&
      !observation.robot_fault && observation.tracking_valid &&
      observation.lateral_valid && observation.orientation_valid) {
    // Explicit recovery retract acknowledges the stopped insertion failure;
    // it never clears cabinet alarms or substitutes for fresh safety data.
    fault_ = InsertionFault::NONE;
    safety_fault_ = fr10_force::ForceSafetyFault::NONE;
    goal_.completion_behavior = CompletionBehavior::RETRACT;
    start_time_s_ = observation.source_time_s;
    enter(InsertionState::RETRACT);
  }
  if (state_ == InsertionState::IDLE || state_ == InsertionState::FAILED) {
    return decision(observation.actual_depth_m);
  }
  if (!std::isfinite(observation.source_time_s) ||
      !std::isfinite(observation.dt_s) || !(observation.dt_s >= 0.0) ||
      observation.source_time_s < previous_time_s_ - 1e-12) {
    fail(InsertionFault::CLOCK_JUMP);
    return decision(observation.actual_depth_m);
  }
  previous_time_s_ = observation.source_time_s;
  phase_elapsed_s_ += observation.dt_s;
  if (observation.canceled) {
    fail(InsertionFault::CANCELED);
  } else if ((state_ == InsertionState::MOVE_PREINSERT || state_ == InsertionState::RETRACT) &&
             observation.planning_failed) {
    fail(InsertionFault::PLANNING_FAILED);
  } else if (state_ != InsertionState::HOLD &&
             observation.source_time_s - start_time_s_ > goal_.timeout_s) {
    fail(InsertionFault::TIMEOUT);
  } else if (observation.robot_fault) {
    fail(InsertionFault::FORCE_SAFETY,
         fr10_force::ForceSafetyFault::ROBOT_FAULT);
  } else if (observation.safety_fault !=
             fr10_force::ForceSafetyFault::NONE) {
    fail(InsertionFault::FORCE_SAFETY, observation.safety_fault);
  } else if (!observation.tracking_valid ||
             (state_ == InsertionState::RETRACT &&
              config_.process_mode == InsertionProcessMode::MATCHED_MATING &&
              observation.actual_depth_m >= 0.0 && !observation.mating_valid)) {
    fail(InsertionFault::FORCE_SAFETY,
         fr10_force::ForceSafetyFault::TRACKING_FAILURE);
  }

  if (state_ == InsertionState::STOPPING) {
    if (observation.stop_complete) {
      enter(InsertionState::FAILED);
    }
    return decision(observation.actual_depth_m);
  }

  const bool progress_phase = state_ == InsertionState::APPROACH_COMPLIANT ||
      state_ == InsertionState::CONTACT_ALIGN || state_ == InsertionState::INSERT_COMPLIANT ||
      state_ == InsertionState::SEATING ||
      (state_ == InsertionState::VERIFY_INSERTION && config_.process_mode == InsertionProcessMode::MATCHED_MATING);
  const bool bounded_alignment = contactReferencePending(observation.reference_mode);
  if (observation.reference_mode == ContactReferenceMode::EXHAUSTED ||
      (bounded_alignment && (!std::isfinite(observation.search_deadline_source_s) ||
                  observation.source_time_s >= observation.search_deadline_source_s))) {
    fail(InsertionFault::NO_PROGRESS);
    return decision(observation.actual_depth_m);
  }
  if (progress_phase && !bounded_alignment) {
    const bool seating = state_ == InsertionState::SEATING || state_ == InsertionState::VERIFY_INSERTION;
    const double objective = seating
        ? seatingWrenchObjective(observation, config_)
        : observation.lateral_force_n / config_.alignment_force_tolerance_n +
          observation.torque_norm_nm / config_.alignment_torque_tolerance_nm;
    if (!progress_.update(observation.dt_s, observation.actual_depth_m,
                         seating || state_ == InsertionState::CONTACT_ALIGN,
                         objective, seating ? 1 : 0)) {
      fail(InsertionFault::NO_PROGRESS); return decision(observation.actual_depth_m);
    }
  }

  switch (state_) {
  case InsertionState::MOVE_PREINSERT:
    if (observation.preinsert_reached) enter(InsertionState::HOLD_PREINSERT);
    break;
  case InsertionState::HOLD_PREINSERT:
    if (phase_elapsed_s_ + 1e-12 >= config_.preinsert_hold_s) {
      enter(InsertionState::FT_ZERO);
    }
    break;
  case InsertionState::FT_ZERO:
    if (observation.tare_complete) enter(config_.process_mode == InsertionProcessMode::MATCHED_MATING
        ? InsertionState::APPROACH_COMPLIANT : InsertionState::APPROACH);
    break;
  case InsertionState::APPROACH:
    if (observation.contact_state ==
        fr10_force::ContactState::CONTACT_CONFIRMED) {
      enter(InsertionState::CONTACT_DETECTED);
    }
    break;
  case InsertionState::CONTACT_DETECTED:
    enter(InsertionState::FORCE_INSERTION);
    break;
  case InsertionState::APPROACH_COMPLIANT:
    if (observation.actual_depth_m + config_.depth_tolerance_m >= goal_.insertion_depth_m)
      enter(InsertionState::SEATING);
    else if (observation.contact_state == fr10_force::ContactState::CONTACT_CONFIRMED)
      enter(InsertionState::CONTACT_ALIGN);
    break;
  case InsertionState::CONTACT_ALIGN: {
    const bool aligned = std::isfinite(observation.lateral_force_n) &&
        std::isfinite(observation.torque_norm_nm) &&
        observation.lateral_force_n <= config_.alignment_force_tolerance_n &&
        observation.torque_norm_nm <= config_.alignment_torque_tolerance_nm &&
        observation.tracking_valid;
    stable_elapsed_s_ = aligned ? stable_elapsed_s_+observation.dt_s : 0.0;
    if (stable_elapsed_s_ >= config_.alignment_hold_s) enter(InsertionState::INSERT_COMPLIANT);
    break;
  }
  case InsertionState::INSERT_COMPLIANT:
    if (observation.actual_depth_m + config_.depth_tolerance_m >= goal_.insertion_depth_m)
      enter(InsertionState::SEATING);
    else if (observation.lateral_force_n > config_.alignment_force_tolerance_n ||
             observation.torque_norm_nm > config_.alignment_torque_tolerance_nm)
      enter(InsertionState::CONTACT_ALIGN);
    break;
  case InsertionState::SEATING:
    if (observation.contact_state == fr10_force::ContactState::CONTACT_CONFIRMED &&
        std::abs(observation.actual_depth_m-goal_.insertion_depth_m) <= config_.depth_tolerance_m)
      enter(InsertionState::VERIFY_INSERTION);
    break;
  case InsertionState::FORCE_INSERTION:
    if (observation.actual_depth_m + config_.depth_tolerance_m >=
        goal_.insertion_depth_m) {
      enter(InsertionState::VERIFY_INSERTION);
    }
    break;
  case InsertionState::VERIFY_INSERTION: {
    if (config_.process_mode == InsertionProcessMode::MATCHED_MATING &&
        observation.contact_state != fr10_force::ContactState::CONTACT_CONFIRMED) {
      // Seating has not been accepted yet. Re-establish bounded contact;
      // never declare HOLD from nominal arrival or a vanished seating load.
      // Depth/correction, raw load and physical no-progress guards remain active.
      enter(InsertionState::SEATING);
      break;
    }
    const bool stable = seatingQualified(observation, goal_, config_,
        std::min(config_.force_error_tolerance_n, config_.pre_stop_force_error_tolerance_n));
    stable_elapsed_s_ = stable ? stable_elapsed_s_ + observation.dt_s : 0.0;
    if (stable_elapsed_s_ + 1e-12 >= config_.completion_hold_s) {
      enter(InsertionState::SUCCESS);
    }
    break;
  }
  case InsertionState::SUCCESS:
    if (goal_.completion_behavior == CompletionBehavior::RETRACT) {
      enter(InsertionState::RETRACT);
    } else if (observation.stop_complete &&
               seatingQualified(observation, goal_, config_, config_.force_error_tolerance_n)) {
      // Complete HOLD only at an acknowledged stationary executor boundary.
      // No admittance integration resumes while braking or waiting here.
      enter(InsertionState::HOLD);
    } else if (observation.stop_complete && !progress_.update(observation.dt_s,
        observation.actual_depth_m, true,
        seatingWrenchObjective(observation, config_), 1)) {
      // A stationary but unqualified candidate is still an unfinished session.
      // Use its existing source progress budget; never silently wait until the
      // whole goal timeout, accept weak force, or automatically resume motion.
      fail(InsertionFault::NO_PROGRESS);
    }
    break;
  case InsertionState::RETRACT:
    if (observation.retract_complete) enter(InsertionState::HOLD);
    break;
  case InsertionState::IDLE:
    break;
  case InsertionState::HOLD:
    if (observation.retract_requested && observation.tracking_valid &&
        observation.lateral_valid && observation.orientation_valid) {
      goal_.completion_behavior = CompletionBehavior::RETRACT;
      start_time_s_ = observation.source_time_s;
      enter(InsertionState::RETRACT);
    }
    break;
  case InsertionState::STOPPING:
  case InsertionState::FAILED:
    break;
  }
  return decision(observation.actual_depth_m);
}

InsertionDecision InsertionStateMachine::decision(double depth_m) const {
  InsertionDecision result;
  result.state = state_;
  result.fault = fault_;
  result.safety_fault = safety_fault_;
  result.request_online_motion =
      state_ == InsertionState::APPROACH ||
      state_ == InsertionState::CONTACT_DETECTED ||
      state_ == InsertionState::FORCE_INSERTION ||
      state_ == InsertionState::VERIFY_INSERTION ||
      state_ == InsertionState::APPROACH_COMPLIANT ||
      state_ == InsertionState::CONTACT_ALIGN ||
      state_ == InsertionState::INSERT_COMPLIANT ||
      state_ == InsertionState::SEATING;
  result.request_controlled_stop = state_ == InsertionState::STOPPING ||
      (state_ == InsertionState::SUCCESS &&
       goal_.completion_behavior == CompletionBehavior::HOLD);
  result.request_tare = state_ == InsertionState::FT_ZERO;
  result.terminal = state_ == InsertionState::HOLD ||
      state_ == InsertionState::FAILED;
  result.success = state_ == InsertionState::HOLD &&
      fault_ == InsertionFault::NONE;
  result.progress = goal_.insertion_depth_m > 0.0
      ? std::clamp(depth_m / goal_.insertion_depth_m, 0.0, 1.0)
      : 0.0;
  return result;
}

void InsertionStateMachine::reset() {
  goal_ = InsertionGoal{};
  state_ = InsertionState::IDLE;
  fault_ = InsertionFault::NONE;
  safety_fault_ = fr10_force::ForceSafetyFault::NONE;
  start_time_s_ = 0.0;
  previous_time_s_ = 0.0;
  phase_elapsed_s_ = 0.0;
  stable_elapsed_s_ = 0.0;
  progress_.reset();
}

} // namespace fr10_insertion
