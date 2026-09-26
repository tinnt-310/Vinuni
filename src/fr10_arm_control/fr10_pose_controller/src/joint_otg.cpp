#include "fr10_pose_controller/joint_otg.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace fr10_motion {

JointOtg::JointOtg(double cycle_period_s, const KinematicLimits &limits,
                   double max_reference_lead_rad)
    : cycle_period_s_(cycle_period_s), limits_(limits),
      max_reference_lead_rad_(max_reference_lead_rad) {
  if (cycle_period_s_ <= 0.0 || !std::isfinite(cycle_period_s_)) {
    throw std::invalid_argument(
        "JointOtg cycle_period_s must be positive and finite");
  }
  ruckig_ = std::make_unique<ruckig::Ruckig<kJointCount>>(cycle_period_s_);
  setMaxReferenceLead(max_reference_lead_rad_);
  setLimits(limits_);
}

void JointOtg::setMaxReferenceLead(double max_reference_lead_rad) {
  if (!(max_reference_lead_rad > 0.0) ||
      !std::isfinite(max_reference_lead_rad)) {
    throw std::invalid_argument(
        "JointOtg max reference lead must be positive and finite");
  }
  max_reference_lead_rad_ = max_reference_lead_rad;
}

void JointOtg::setLimits(const KinematicLimits &limits) {
  limits_ = limits;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    input_.max_velocity[i] =
        std::max(0.001, limits_.max_joint_velocity_rad_s[i]);
    input_.max_acceleration[i] =
        std::max(0.001, limits_.max_joint_acceleration_rad_s2[i]);
    input_.max_jerk[i] = std::max(0.001, limits_.max_joint_jerk_rad_s3[i]);
  }
  input_.synchronization = ruckig::Synchronization::TimeIfNecessary;
  input_.duration_discretization = ruckig::DurationDiscretization::Discrete;
}

void JointOtg::setCyclePeriod(double period_s) {
  if (period_s <= 0.0 || !std::isfinite(period_s)) {
    return;
  }
  cycle_period_s_ = period_s;
  ruckig_ = std::make_unique<ruckig::Ruckig<kJointCount>>(cycle_period_s_);
}

void JointOtg::reset(const std::array<double, kJointCount> &initial_position) {
  const std::array<double, kJointCount> zero{};
  reset(initial_position, zero, zero);
}

void JointOtg::reset(
    const std::array<double, kJointCount> &initial_position,
    const std::array<double, kJointCount> &initial_velocity,
    const std::array<double, kJointCount> &initial_acceleration) {
  current_position_ = initial_position;
  current_velocity_ = initial_velocity;
  current_acceleration_ = initial_acceleration;

  input_.current_position = initial_position;
  input_.current_velocity = initial_velocity;
  input_.current_acceleration = initial_acceleration;

  input_.target_position = initial_position;
  input_.target_velocity.fill(0.0);
  input_.target_acceleration.fill(0.0);
  input_.control_interface = ruckig::ControlInterface::Position;

  initialized_ = true;
  finished_ = true;
  active_revision_ = 0;
  reference_lagging_ = false;
  reference_lag_cycles_ = 0;
  reference_actual_error_rad_ = 0.0;
}

bool JointOtg::submitTarget(const JointTarget &target) {
  if (!initialized_) {
    reset(target.position_rad);
  }

  // ── Online Retargeting Logic ───────────────────────────────────────────────
  // CRITICAL REQUIREMENT: Do NOT stop or reset velocity to zero when
  // retargeting. Preserve the currently generated position, velocity, and
  // acceleration.
  input_.current_position = current_position_;
  input_.current_velocity = current_velocity_;
  input_.current_acceleration = current_acceleration_;

  input_.target_position = target.position_rad;
  input_.target_velocity = target.target_velocity_rad_s;
  input_.target_acceleration = target.target_acceleration_rad_s2;
  input_.control_interface = ruckig::ControlInterface::Position;

  // Apply scaling to limits
  const double v_scale = std::clamp(target.velocity_scale, 0.01, 1.0);
  const double a_scale = std::clamp(target.acceleration_scale, 0.01, 1.0);
  const double j_scale = std::clamp(target.jerk_scale, 0.01, 1.0);

  for (std::size_t i = 0; i < kJointCount; ++i) {
    input_.max_velocity[i] = limits_.max_joint_velocity_rad_s[i] * v_scale;
    input_.max_acceleration[i] =
        limits_.max_joint_acceleration_rad_s2[i] * a_scale;
    input_.max_jerk[i] = limits_.max_joint_jerk_rad_s3[i] * j_scale;
  }

  active_revision_ = target.revision;
  finished_ = false;
  reference_lagging_ = false;
  reference_lag_cycles_ = 0;
  reference_actual_error_rad_ = 0.0;
  return true;
}

bool JointOtg::startControlledStop() {
  if (!initialized_) {
    return false;
  }

  input_.current_position = current_position_;
  input_.current_velocity = current_velocity_;
  input_.current_acceleration = current_acceleration_;
  input_.target_position = current_position_;
  input_.target_velocity.fill(0.0);
  input_.target_acceleration.fill(0.0);
  input_.control_interface = ruckig::ControlInterface::Velocity;

  // A controlled stop uses the configured engineering acceleration and jerk
  // bounds. Velocity bounds are retained to validate the current state.
  for (std::size_t i = 0; i < kJointCount; ++i) {
    input_.max_velocity[i] =
        std::max(0.001, limits_.max_joint_velocity_rad_s[i]);
    input_.max_acceleration[i] =
        std::max(0.001, limits_.max_joint_acceleration_rad_s2[i]);
    input_.max_jerk[i] =
        std::max(0.001, limits_.max_joint_jerk_rad_s3[i]);
  }
  finished_ = false;
  reference_lagging_ = false;
  reference_lag_cycles_ = 0;
  reference_actual_error_rad_ = 0.0;
  return true;
}

bool JointOtg::step(const RobotState &feedback,
                    std::array<double, kJointCount> &out_position,
                    std::array<double, kJointCount> &out_velocity,
                    std::array<double, kJointCount> &out_acceleration) {
  if (!initialized_) {
    reference_lagging_ = false;
    return false;
  }

  if (finished_) {
    reference_lagging_ = false;
    reference_lag_cycles_ = 0;
    reference_actual_error_rad_ = 0.0;
    out_position = current_position_;
    out_velocity.fill(0.0);
    out_acceleration.fill(0.0);
    return true;
  }

  const auto result = ruckig_->update(input_, output_);
  last_result_ = result;

  if (result < 0) {
    // Calculation failed
    reference_lagging_ = false;
    return false;
  }

  reference_actual_error_rad_ = 0.0;
  if (feedback.valid) {
    for (std::size_t i = 0; i < kJointCount; ++i) {
      reference_actual_error_rad_ = std::max(
          reference_actual_error_rad_,
          std::abs(output_.new_position[i] - feedback.joint_position_rad[i]));
    }
    if (reference_actual_error_rad_ > max_reference_lead_rad_) {
      reference_lagging_ = true;
      if (reference_lag_cycles_ < std::numeric_limits<uint32_t>::max()) {
        ++reference_lag_cycles_;
      }
    }
  }
  // Diagnostic only: never freeze a position stream while retaining nonzero
  // velocity/acceleration. The executor owns the single lag-to-braking policy.
  if (reference_actual_error_rad_ <= max_reference_lead_rad_) {
    reference_lagging_ = false;
    reference_lag_cycles_ = 0;
  }

  current_position_ = output_.new_position;
  current_velocity_ = output_.new_velocity;
  current_acceleration_ = output_.new_acceleration;

  out_position = current_position_;
  out_velocity = current_velocity_;
  out_acceleration = current_acceleration_;

  // Advance output state into input for the next cycle
  output_.pass_to_input(input_);

  if (result == ruckig::Result::Finished) {
    finished_ = true;
  }

  return true;
}

bool JointOtg::isFinished() const { return finished_; }

} // namespace fr10_motion
