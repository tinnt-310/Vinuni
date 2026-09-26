#include "fr10_pose_controller/motion_controller.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>

#include "fr10_pose_controller/math_utils.hpp"

namespace fr10_motion {
namespace {

template <std::size_t N> bool finiteArray(const std::array<double, N> &values) {
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}

bool validScale(double value) {
  return std::isfinite(value) && value > 0.0 && value <= 1.0;
}

bool feedbackSafetyClear(const RobotState &feedback) {
  return feedback.valid && !feedback.emergency_stop &&
      finiteArray(feedback.joint_position_rad) &&
      finiteArray(feedback.joint_velocity_rad_s) &&
      finiteArray(feedback.joint_acceleration_rad_s2) &&
      !feedback.collision_detected && feedback.main_error_code == 0 &&
      feedback.sub_error_code == 0;
}

constexpr double kMinimumTrajectorySamplePeriodS = 1e-6;

} // namespace

CartesianFaultMonitor::CartesianFaultMonitor(
    const CartesianFaultMonitorConfig &config) {
  setConfig(config);
}

void CartesianFaultMonitor::setConfig(
    const CartesianFaultMonitorConfig &config) {
  const bool valid = config.endpoint_position_tolerance_m > 0.0 &&
      config.tracking_lag_tolerance_m > 0.0 &&
      config.perpendicular_tolerance_m > 0.0 &&
      config.orientation_tolerance_rad > 0.0 && config.persistence_s > 0.0 &&
      std::isfinite(config.endpoint_position_tolerance_m) &&
      std::isfinite(config.tracking_lag_tolerance_m) &&
      std::isfinite(config.perpendicular_tolerance_m) &&
      std::isfinite(config.orientation_tolerance_rad) &&
      std::isfinite(config.persistence_s);
  if (!valid) {
    throw std::invalid_argument(
        "Cartesian fault-monitor limits must be finite and positive");
  }
  config_ = config;
  reset();
}

CartesianFaultState CartesianFaultMonitor::update(
    const CartesianTrackingErrors &errors, bool tracking, double dt) {
  const bool finite = std::isfinite(errors.position_error_m) &&
      std::isfinite(errors.along_track_error_m) &&
      std::isfinite(errors.perpendicular_error_m) &&
      std::isfinite(errors.orientation_error_rad);
  const bool violation = !finite ||
      (tracking ? errors.along_track_error_m >
                      config_.tracking_lag_tolerance_m
                : errors.position_error_m >
                      config_.endpoint_position_tolerance_m) ||
      errors.perpendicular_error_m > config_.perpendicular_tolerance_m ||
      errors.orientation_error_rad > config_.orientation_tolerance_rad;
  if (!violation) {
    reset();
    return state_;
  }
  if (state_ == CartesianFaultState::PERSISTENT_FAULT) {
    return state_;
  }
  const double period_s = dt > 0.0 && std::isfinite(dt) ? dt : 0.0;
  violation_duration_s_ += period_s;
  state_ = violation_duration_s_ + 1e-12 >= config_.persistence_s
               ? CartesianFaultState::PERSISTENT_FAULT
               : CartesianFaultState::WARNING;
  return state_;
}

void CartesianFaultMonitor::reset() {
  state_ = CartesianFaultState::NORMAL;
  violation_duration_s_ = 0.0;
}

MotionExecutor::MotionExecutor(const MotionExecutorConfig &config)
    : config_(config) {
  const double period_s = 1.0 / std::max(1.0, config_.control_rate_hz);
  joint_otg_ = std::make_unique<JointOtg>(
      period_s, config_.limits, config_.max_joint_reference_lead_rad);
}

void MotionExecutor::setConfig(const MotionExecutorConfig &config) {
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
  const double period_s = 1.0 / std::max(1.0, config_.control_rate_hz);
  joint_otg_->setCyclePeriod(period_s);
  joint_otg_->setLimits(config_.limits);
  joint_otg_->setMaxReferenceLead(config_.max_joint_reference_lead_rad);
}

bool MotionExecutor::validTarget(const JointTarget &target) const {
  if (!feedback_initialized_ || state_ == MotionState::FAULT ||
      state_ == MotionState::STOPPING ||
      !finiteArray(target.position_rad) ||
      !finiteArray(target.target_velocity_rad_s) ||
      !finiteArray(target.target_acceleration_rad_s2) ||
      !validScale(target.velocity_scale) ||
      !validScale(target.acceleration_scale) ||
      !validScale(target.jerk_scale) || !std::isfinite(target.timestamp_s)) {
    return false;
  }
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    if (target.position_rad[joint] <
            config_.limits.min_joint_position_rad[joint] ||
        target.position_rad[joint] >
            config_.limits.max_joint_position_rad[joint]) {
      return false;
    }
  }
  return true;
}

bool MotionExecutor::structurallyValidPoseTrajectory(
    const CartesianJointTrajectory &trajectory,
    const MotionExecutorConfig &config) {
  if (trajectory.generation == 0 || trajectory.points.size() < 2 ||
      !std::isfinite(trajectory.planning_timestamp_s) ||
      !finiteArray(trajectory.seed_joint_state.position_rad) ||
      !finiteArray(trajectory.seed_joint_state.velocity_rad_s) ||
      !finiteArray(trajectory.seed_joint_state.acceleration_rad_s2) ||
      !finiteArray(trajectory.path_start.position_m) ||
      !finiteArray(trajectory.path_start.orientation_quat) ||
      !finiteArray(trajectory.path_goal.position_m) ||
      !finiteArray(trajectory.path_goal.orientation_quat) ||
      math::quaternionNormSquared(trajectory.path_start.orientation_quat) <=
          1e-12 ||
      math::quaternionNormSquared(trajectory.path_goal.orientation_quat) <=
          1e-12) {
    return false;
  }
  double previous_time_s = -1.0;
  double previous_path_parameter = -1.0;
  std::array<double, kJointCount> previous_acceleration{};
  bool have_previous_acceleration = false;
  for (const auto &point : trajectory.points) {
    if (!finiteArray(point.joint_state.position_rad) ||
        !finiteArray(point.joint_state.velocity_rad_s) ||
        !finiteArray(point.joint_state.acceleration_rad_s2) ||
        !finiteArray(point.desired_pose.position_m) ||
        !finiteArray(point.desired_pose.orientation_quat) ||
        !finiteArray(point.desired_translation_tangent) ||
        math::quaternionNormSquared(point.desired_pose.orientation_quat) <=
            1e-12 ||
        !std::isfinite(point.time_from_start_s) ||
        !std::isfinite(point.path_parameter) ||
        point.time_from_start_s <= previous_time_s ||
        point.path_parameter + 1e-12 < previous_path_parameter ||
        point.path_parameter < -1e-12 || point.path_parameter > 1.0 + 1e-12) {
      return false;
    }
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      if (point.joint_state.position_rad[joint] <
              config.limits.min_joint_position_rad[joint] ||
          point.joint_state.position_rad[joint] >
              config.limits.max_joint_position_rad[joint] ||
          std::abs(point.joint_state.velocity_rad_s[joint]) >
                  config.limits.max_joint_velocity_rad_s[joint] + 1e-7 ||
          std::abs(point.joint_state.acceleration_rad_s2[joint]) >
                  config.limits.max_joint_acceleration_rad_s2[joint] + 1e-7) {
        return false;
      }
      if (have_previous_acceleration) {
        const double sample_dt = point.time_from_start_s - previous_time_s;
        // Sub-microsecond samples cannot be executed by a 125 Hz position
        // bridge and make finite-difference jerk numerically ill-conditioned.
        if (sample_dt < kMinimumTrajectorySamplePeriodS) {
          return false;
        }
        if (std::abs(point.joint_state.acceleration_rad_s2[joint] -
                     previous_acceleration[joint]) /
                sample_dt >
            config.limits.max_joint_jerk_rad_s3[joint] + 1e-6) {
          return false;
        }
      }
    }
    previous_acceleration = point.joint_state.acceleration_rad_s2;
    have_previous_acceleration = true;
    previous_time_s = point.time_from_start_s;
    previous_path_parameter = point.path_parameter;
  }
  return true;
}

double MotionExecutor::poseActivationStartError(
    const CartesianJointTrajectory &trajectory) const {
  double start_error = 0.0;
  const auto &reference = config_.require_command_ack ? sent_command_ : latest_command_;
  const auto &q = reference.type == CommandType::JOINT_POSITION
      ? reference.joint_position_rad : latest_feedback_.joint_position_rad;
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    start_error = std::max(
        start_error,
        std::abs(trajectory.points.front().joint_state.position_rad[joint] -
                 q[joint]));
  }
  return start_error;
}

std::shared_ptr<const PreparedCartesianTrajectory>
MotionExecutor::preparePoseTrajectory(
    std::shared_ptr<const CartesianJointTrajectory> trajectory) const {
  if (!trajectory) {
    return {};
  }
  MotionExecutorConfig config;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    config = config_;
  }
  if (!structurallyValidPoseTrajectory(*trajectory, config)) {
    return {};
  }
  return std::shared_ptr<const PreparedCartesianTrajectory>(
      new PreparedCartesianTrajectory(std::move(trajectory)));
}

std::shared_ptr<const PreparedCartesianTrajectory> MotionExecutor::prepareJointTrajectory(
    std::shared_ptr<const CartesianJointTrajectory> trajectory, const JointTarget &target) const {
  const auto validated = preparePoseTrajectory(trajectory);
  if (!validated) { return {}; }
  auto prepared = std::shared_ptr<PreparedCartesianTrajectory>(
      new PreparedCartesianTrajectory(std::move(trajectory)));
  prepared->joint_target_ = target;
  return prepared;
}

bool MotionExecutor::submitJointTarget(const JointTarget &target) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!validTarget(target)) {
    return false;
  }

  if (state_ == MotionState::STOPPED) {
    state_ = MotionState::IDLE;
    active_owner_ = MotionOwner::NONE;
    joint_otg_->reset(latest_feedback_.joint_position_rad,
                      latest_feedback_.joint_velocity_rad_s,
                      latest_feedback_.joint_acceleration_rad_s2);
  }
  if (execution_kind_ == ExecutionKind::POSE_TRAJECTORY) {
    const auto &reference = config_.require_command_ack ? sent_command_ : latest_command_;
    joint_otg_->reset(reference.joint_position_rad,
                      reference.joint_velocity_rad_s,
                      reference.joint_acceleration_rad_s2);
  }
  if (!joint_otg_->submitTarget(target)) {
    return false;
  }

  active_owner_ = MotionOwner::JOINT_OTG;
  execution_kind_ = ExecutionKind::JOINT_OTG;
  active_joint_target_ = target;
  active_pose_generation_ = 0;
  active_pose_trajectory_.reset();
  active_pose_point_ = 0;
  active_pose_elapsed_s_ = 0.0;
  active_pose_started_ = false;
  goal_tolerance_samples_ = 0;
  reference_lag_samples_ = 0;
  state_ = MotionState::TRACKING;
  outcome_ = MotionOutcome::NONE;
  latest_status_.state = state_;
  latest_status_.owner = active_owner_;
  latest_status_.active_type = CommandType::JOINT_POSITION;
  latest_status_.active_revision = target.revision;
  latest_status_.active_generation = 0;
  latest_status_.in_goal_tolerance = false;
  latest_status_.cartesian_reference_valid = false;
  return true;
}

bool MotionExecutor::submitPoseTrajectory(
    std::shared_ptr<const CartesianJointTrajectory> trajectory) {
  return submitPreparedPoseTrajectory(preparePoseTrajectory(
      std::move(trajectory)));
}

bool MotionExecutor::submitPreparedPoseTrajectory(
    std::shared_ptr<const PreparedCartesianTrajectory> prepared) {
  return activatePreparedPoseTrajectory(std::move(prepared)) ==
      PoseActivationResult::ACCEPTED;
}

PoseActivationResult MotionExecutor::activatePreparedPoseTrajectory(
    std::shared_ptr<const PreparedCartesianTrajectory> prepared) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!prepared || !prepared->trajectory_ || !feedback_initialized_ ||
      state_ == MotionState::FAULT || state_ == MotionState::STOPPING) {
    return PoseActivationResult::REJECTED;
  }
  if (!config_.require_command_ack && poseActivationStartError(*prepared->trajectory_) >
      config_.activation_position_tolerance_rad) {
    return PoseActivationResult::STALE;
  }
  if (config_.require_command_ack) {
    const auto &seed = prepared->trajectory_->points.front().joint_state;
    const bool have_sent = sent_command_.type == CommandType::JOINT_POSITION;
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const double q = have_sent ? sent_command_.joint_position_rad[i] : latest_feedback_.joint_position_rad[i];
      const double v = have_sent ? sent_command_.joint_velocity_rad_s[i] : latest_feedback_.joint_velocity_rad_s[i];
      const double a = have_sent ? sent_command_.joint_acceleration_rad_s2[i] : latest_feedback_.joint_acceleration_rad_s2[i];
      if (std::abs(seed.position_rad[i] - q) > config_.activation_position_tolerance_rad ||
          std::abs(seed.velocity_rad_s[i] - v) > config_.activation_velocity_tolerance_rad_s ||
          std::abs(seed.acceleration_rad_s2[i] - a) > config_.activation_acceleration_tolerance_rad_s2) {
        return PoseActivationResult::STALE;
      }
    }
  }
  auto trajectory = prepared->trajectory_;
  if (state_ == MotionState::STOPPED) {
    state_ = MotionState::IDLE;
    active_owner_ = MotionOwner::NONE;
  }
  active_owner_ = prepared->joint_target_ ? MotionOwner::JOINT_OTG : MotionOwner::POSE_PATH;
  execution_kind_ = ExecutionKind::POSE_TRAJECTORY;
  active_pose_generation_ = trajectory->generation;
  active_pose_trajectory_ = std::move(trajectory);
  active_pose_point_ = 0;
  active_pose_elapsed_s_ = 0.0;
  active_pose_started_ = false;
  active_joint_target_ = JointTarget{};
  active_joint_target_.position_rad =
      active_pose_trajectory_->points.back().joint_state.position_rad;
  if (prepared->joint_target_) {
    active_joint_target_ = *prepared->joint_target_;
  }
  goal_tolerance_samples_ = 0;
  reference_lag_samples_ = 0;
  state_ = MotionState::TRACKING;
  outcome_ = MotionOutcome::NONE;
  latest_status_ = MotionStatus{};
  latest_status_.state = state_;
  latest_status_.owner = active_owner_;
  latest_status_.active_type = CommandType::JOINT_POSITION;
  latest_status_.active_generation = active_pose_generation_;
  latest_status_.active_revision = active_joint_target_.revision;
  latest_status_.cartesian_reference_valid = true;
  latest_status_.desired_cartesian_pose =
      active_pose_trajectory_->points.front().desired_pose;
  latest_status_.desired_cartesian_translation_tangent =
      active_pose_trajectory_->points.front().desired_translation_tangent;
  latest_status_.cartesian_path_start = active_pose_trajectory_->path_start;
  latest_status_.cartesian_path_goal = active_pose_trajectory_->path_goal;
  return PoseActivationResult::ACCEPTED;
}

bool MotionExecutor::startControlledStopLocked() {
  if (!feedback_initialized_ || state_ == MotionState::FAULT) {
    return false;
  }
  if (state_ == MotionState::STOPPING ||
      execution_kind_ == ExecutionKind::CONTROLLED_STOP) {
    return true;
  }

  const auto &reference = config_.require_command_ack ? sent_command_ : latest_command_;
  const bool have_command = reference.type == CommandType::JOINT_POSITION;
  const auto &position = have_command ? reference.joint_position_rad
                                      : latest_feedback_.joint_position_rad;
  const auto &velocity = have_command ? reference.joint_velocity_rad_s
                                      : latest_feedback_.joint_velocity_rad_s;
  const auto &acceleration =
      have_command ? reference.joint_acceleration_rad_s2
                   : latest_feedback_.joint_acceleration_rad_s2;
  if (!finiteArray(position) || !finiteArray(velocity) ||
      !finiteArray(acceleration)) {
    state_ = MotionState::FAULT;
    latest_status_.state = state_;
    return false;
  }

  joint_otg_->reset(position, velocity, acceleration);
  if (!joint_otg_->startControlledStop()) {
    state_ = MotionState::FAULT;
    latest_status_.state = state_;
    return false;
  }
  execution_kind_ = ExecutionKind::CONTROLLED_STOP;
  active_pose_trajectory_.reset();
  active_pose_point_ = 0;
  active_pose_elapsed_s_ = 0.0;
  active_pose_started_ = false;
  goal_tolerance_samples_ = 0;
  state_ = MotionState::STOPPING;
  outcome_ = MotionOutcome::STOPPED_BEFORE_GOAL;
  latest_status_.state = state_;
  latest_status_.owner = active_owner_;
  latest_status_.active_type = CommandType::JOINT_POSITION;
  latest_status_.cartesian_reference_valid = false;
  latest_status_.in_goal_tolerance = false;
  latest_status_.outcome = outcome_;
  return true;
}

MotionStatus MotionExecutor::update(const RobotState &feedback, double dt) {
  std::lock_guard<std::mutex> lock(mutex_);
  CartesianTrackingReport tracking_report;
  if (cartesian_tracking_buffer_.read(tracking_report) &&
      tracking_report.sequence != consumed_cartesian_tracking_sequence_) {
    consumed_cartesian_tracking_sequence_ = tracking_report.sequence;
    latest_status_.position_error_m = tracking_report.position_error_m;
    latest_status_.angular_error_rad = tracking_report.angular_error_rad;
    latest_status_.cartesian_path_deviation_m =
        tracking_report.path_deviation_m;
    if (tracking_report.fault && active_owner_ == MotionOwner::POSE_PATH) {
      startControlledStopLocked();
    }
  }
  // Always retain the newest hardware safety state. This lets resetFault()
  // prove that an alarm has actually cleared while the executor stays latched
  // in FAULT until an explicit reset request.
  latest_feedback_ = feedback;
  if (!feedbackSafetyClear(feedback)) {
    state_ = MotionState::FAULT;
    reference_lag_samples_ = 0;
    latest_status_.state = state_;
    latest_status_.owner = MotionOwner::NONE;
    latest_command_ = MotionCommand{};
    return latest_status_;
  }
  if (state_ == MotionState::FAULT) {
    latest_status_.state = state_;
    return latest_status_;
  }
  if (!feedback_initialized_) {
    joint_otg_->reset(feedback.joint_position_rad,
                      feedback.joint_velocity_rad_s,
                      feedback.joint_acceleration_rad_s2);
    feedback_initialized_ = true;
    feedback_initialized_atomic_.store(true, std::memory_order_release);
  }
  const double nominal_period_s = 1.0 / config_.control_rate_hz;
  const double period_s = dt > 0.0 && std::isfinite(dt) ? dt : nominal_period_s;
  const bool fresh_feedback = feedback.feedback_sequence == 0 ||
      feedback.feedback_sequence > last_feedback_sequence_;
  if (fresh_feedback) {
    last_feedback_sequence_ = feedback.feedback_sequence;
  }
  const auto &sent_reference = config_.require_command_ack ? sent_command_ : latest_command_;
  double sent_error = 0.0;
  std::size_t lag_joint = 0;
  if (sent_reference.type == CommandType::JOINT_POSITION) {
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      const double error = std::abs(sent_reference.joint_position_rad[joint] -
                                    feedback.joint_position_rad[joint]);
      if (error > sent_error) { sent_error = error; lag_joint = joint; }
    }
  }
  const bool lag_triggered = fresh_feedback && state_ == MotionState::TRACKING &&
      sent_error > config_.max_joint_reference_lead_rad;
  if (lag_triggered) {
    lag_event_.sequence = ++lag_event_sequence_;
    lag_event_.joint = lag_joint;
    lag_event_.feedback_sequence = feedback.feedback_sequence;
    lag_event_.feedback_age_s = feedback.feedback_age_s;
    lag_event_.error_rad = sent_error;
    lag_event_.sent_reference = sent_reference;
    lag_event_.feedback = feedback;
    // Change kind BEFORE stepping: the first braking sample is sent this tick.
    if (!startControlledStopLocked()) {
      state_ = MotionState::FAULT;
      latest_command_ = MotionCommand{};
      latest_status_.state = state_;
      return latest_status_;
    }
  }
  MotionCommand command;
  command.period_s = nominal_period_s;
  command.sequence = ++command_sequence_;
  command.timestamp_s = feedback.timestamp_s;
  MotionStatus status;
  status.state = state_;
  status.owner = active_owner_;
  status.active_type = active_owner_ == MotionOwner::NONE
                           ? CommandType::NONE
                           : CommandType::JOINT_POSITION;
  status.control_period_s = period_s;

  std::array<double, kJointCount> goal_position{};
  bool trajectory_finished = false;
  double candidate_reference_error_rad = 0.0;
  if (execution_kind_ == ExecutionKind::JOINT_OTG) {
    command.type = CommandType::JOINT_POSITION;
    status.active_revision = active_joint_target_.revision;
    if (!joint_otg_->step(feedback, command.joint_position_rad,
                          command.joint_velocity_rad_s,
                          command.joint_acceleration_rad_s2)) {
      state_ = MotionState::FAULT;
      status.state = state_;
      latest_status_ = status;
      return latest_status_;
    }
    candidate_reference_error_rad = joint_otg_->referenceActualErrorRad();
    goal_position = active_joint_target_.position_rad;
    trajectory_finished = joint_otg_->isFinished();
  } else if (execution_kind_ == ExecutionKind::POSE_TRAJECTORY &&
             active_pose_trajectory_) {
    command.type = CommandType::JOINT_POSITION;
    status.active_generation = active_pose_generation_;
    status.active_revision = active_joint_target_.revision;
    const auto &points = active_pose_trajectory_->points;
    const double candidate_elapsed_s =
        active_pose_started_ ? active_pose_elapsed_s_ + nominal_period_s : 0.0;
    const auto upper = std::upper_bound(
        points.begin(), points.end(), candidate_elapsed_s + 1e-12,
        [](double time_s, const CartesianTrajectoryPoint &point) {
          return time_s < point.time_from_start_s;
        });
    const std::size_t candidate_index =
        upper == points.begin()
            ? 0
            : static_cast<std::size_t>(std::distance(points.begin(), upper) -
                                       1);
    const auto &candidate = points[candidate_index];
    double reference_actual_error = 0.0;
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      reference_actual_error = std::max(
          reference_actual_error,
          std::abs(candidate.joint_state.position_rad[joint] -
                   feedback.joint_position_rad[joint]));
    }
    candidate_reference_error_rad = reference_actual_error;
      command.joint_position_rad = candidate.joint_state.position_rad;
      command.joint_velocity_rad_s = candidate.joint_state.velocity_rad_s;
      command.joint_acceleration_rad_s2 =
          candidate.joint_state.acceleration_rad_s2;
      active_pose_point_ = candidate_index;
      active_pose_elapsed_s_ = candidate_elapsed_s;
      active_pose_started_ = true;
    const auto &active_point = points[active_pose_point_];
    status.cartesian_reference_valid = active_owner_ == MotionOwner::POSE_PATH;
    status.desired_cartesian_pose = active_point.desired_pose;
    status.desired_cartesian_translation_tangent =
        active_point.desired_translation_tangent;
    status.cartesian_path_start = active_pose_trajectory_->path_start;
    status.cartesian_path_goal = active_pose_trajectory_->path_goal;
    status.cartesian_path_parameter = active_point.path_parameter;
    goal_position = points.back().joint_state.position_rad;
    trajectory_finished =
        candidate_index + 1 >= points.size();
  } else if (execution_kind_ == ExecutionKind::CONTROLLED_STOP) {
    command.type = CommandType::JOINT_POSITION;
    status.active_revision = active_joint_target_.revision;
    status.active_generation = active_pose_generation_;
    if (!joint_otg_->step(feedback, command.joint_position_rad,
                          command.joint_velocity_rad_s,
                          command.joint_acceleration_rad_s2)) {
      state_ = MotionState::FAULT;
      status.state = state_;
      latest_status_ = status;
      return latest_status_;
    }
    candidate_reference_error_rad = joint_otg_->referenceActualErrorRad();
    goal_position = active_joint_target_.position_rad;
    trajectory_finished = joint_otg_->isFinished();
    if (trajectory_finished) {
      state_ = MotionState::HOLDING;
      status.in_goal_tolerance = false;
    }
  } else {
    command.type = CommandType::NONE;
    command.joint_position_rad = feedback.joint_position_rad;
  }

  status.reference_lagging = sent_error > config_.max_joint_reference_lead_rad;
  status.reference_lag_samples = lag_triggered ? 1 : 0;
  status.lag_event = lag_event_;
  status.max_reference_actual_error_rad = sent_error;
  status.candidate_reference_error_rad = candidate_reference_error_rad;

  if (active_owner_ != MotionOwner::NONE) {
    double max_joint_error = 0.0;
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      status.joint_error_rad[joint] =
          goal_position[joint] - feedback.joint_position_rad[joint];
      max_joint_error =
          std::max(max_joint_error, std::abs(status.joint_error_rad[joint]));
    }
    if (execution_kind_ != ExecutionKind::CONTROLLED_STOP &&
        trajectory_finished &&
        max_joint_error <= config_.joint_goal_tolerance_rad) {
      if (fresh_feedback) { ++goal_tolerance_samples_; }
      if (goal_tolerance_samples_ >= config_.goal_stable_samples) {
        state_ = MotionState::HOLDING;
        outcome_ = MotionOutcome::GOAL_REACHED;
        status.in_goal_tolerance = true;
      }
    } else {
      goal_tolerance_samples_ = 0;
    }
  }

  status.state = state_;
  status.outcome = outcome_;
  latest_command_ = command;
  latest_status_ = status;
  return latest_status_;
}

MotionCommand MotionExecutor::command() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_command_;
}

bool MotionExecutor::acknowledgeCommand(uint64_t sequence) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (latest_command_.type == CommandType::NONE ||
      latest_command_.sequence != sequence) { return false; }
  sent_command_ = latest_command_;
  return true;
}

MotionStatus MotionExecutor::status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_status_;
}

bool MotionExecutor::initialized() const {
  return feedback_initialized_atomic_.load(std::memory_order_acquire);
}

void MotionExecutor::reportCartesianTrackingError(
    double position_error_m, double angular_error_rad,
    double path_deviation_m, bool fault) {
  if (!std::isfinite(position_error_m) || !std::isfinite(angular_error_rad) ||
      !std::isfinite(path_deviation_m)) {
    fault = true;
  }
  CartesianTrackingReport report;
  report.position_error_m = position_error_m;
  report.angular_error_rad = angular_error_rad;
  report.path_deviation_m = path_deviation_m;
  report.fault = fault;
  report.sequence =
      cartesian_tracking_sequence_.fetch_add(1, std::memory_order_relaxed) + 1;
  cartesian_tracking_buffer_.write(report);
}

void MotionExecutor::stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  reference_lag_samples_ = 0;
  if (!startControlledStopLocked() && state_ != MotionState::FAULT) {
    state_ = MotionState::STOPPED;
    active_owner_ = MotionOwner::NONE;
    execution_kind_ = ExecutionKind::NONE;
    latest_command_ = MotionCommand{};
    latest_status_.state = state_;
    latest_status_.active_type = CommandType::NONE;
    latest_status_.owner = MotionOwner::NONE;
  }
}

bool MotionExecutor::resetFault() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != MotionState::FAULT && state_ != MotionState::STOPPED) {
    return false;
  }
  if (!feedbackSafetyClear(latest_feedback_)) {
    return false;
  }
  state_ = MotionState::IDLE;
  active_owner_ = MotionOwner::NONE;
  execution_kind_ = ExecutionKind::NONE;
  feedback_initialized_ = false;
  feedback_initialized_atomic_.store(false, std::memory_order_release);
  goal_tolerance_samples_ = 0;
  reference_lag_samples_ = 0;
  active_pose_generation_ = 0;
  active_pose_trajectory_.reset();
  active_pose_point_ = 0;
  active_pose_elapsed_s_ = 0.0;
  active_pose_started_ = false;
  latest_command_ = MotionCommand{};
  sent_command_ = MotionCommand{};
  outcome_ = MotionOutcome::NONE;
  latest_status_ = MotionStatus{};
  latest_status_.state = state_;
  return true;
}

} // namespace fr10_motion
