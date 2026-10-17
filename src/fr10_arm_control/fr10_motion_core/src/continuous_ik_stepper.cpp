#include "fr10_motion_core/continuous_ik_stepper.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "fr10_motion_core/math_utils.hpp"

namespace fr10_motion {
namespace {

template <std::size_t N>
bool finiteArray(const std::array<double, N> &values) {
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}

bool validConfig(const ContinuousIKConfig &config) {
  return std::isfinite(config.max_joint_step_rad) &&
      config.max_joint_step_rad > 0.0 &&
      std::isfinite(config.max_position_error_m) &&
      config.max_position_error_m >= 0.0 &&
      std::isfinite(config.max_orientation_error_rad) &&
      config.max_orientation_error_rad >= 0.0;
}

} // namespace

ContinuousIKStepper::ContinuousIKStepper(ContinuousIKConfig config) {
  setConfig(config);
}

void ContinuousIKStepper::setConfig(const ContinuousIKConfig &config) {
  if (!validConfig(config)) {
    throw std::invalid_argument("ContinuousIKStepper config is invalid");
  }
  config_ = config;
}

ContinuousIKStepResult ContinuousIKStepper::step(
    const Pose &target, const KinematicState &seed,
    IKProvider &provider) const {
  ContinuousIKStepResult result;
  if (!finiteArray(target.position_m) ||
      !finiteArray(target.orientation_quat) ||
      math::quaternionNormSquared(target.orientation_quat) <= 1e-12 ||
      !finiteArray(seed.position_rad) || !finiteArray(seed.velocity_rad_s) ||
      !finiteArray(seed.acceleration_rad_s2)) {
    return result;
  }

  std::array<IKResult, IKProvider::kMaxCandidates> candidates{};
  result.candidate_count = std::min(
      provider.solveCandidates(target, seed, candidates, nullptr),
      IKProvider::kMaxCandidates);
  if (result.candidate_count == 0) {
    result.failure = ContinuousIKStepFailure::NO_CANDIDATE;
    return result;
  }

  double best_cost = std::numeric_limits<double>::infinity();
  bool continuity_rejected = false;
  bool fk_failed = false;
  bool cartesian_rejected = false;
  for (std::size_t candidate_index = 0;
       candidate_index < result.candidate_count; ++candidate_index) {
    const auto &candidate = candidates[candidate_index];
    if (!candidate.success || !finiteArray(candidate.state.position_rad)) {
      continue;
    }

    double cost = 0.0;
    double maximum_step = 0.0;
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      const double delta = candidate.state.position_rad[joint] -
          seed.position_rad[joint];
      maximum_step = std::max(maximum_step, std::abs(delta));
      cost += delta * delta;
    }
    if (maximum_step > config_.max_joint_step_rad) {
      continuity_rejected = true;
      continue;
    }

    Pose reconstructed;
    if (!provider.forward(candidate.state, reconstructed, nullptr)) {
      fk_failed = true;
      continue;
    }
    double position_error_squared = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const double error = reconstructed.position_m[axis] -
          target.position_m[axis];
      position_error_squared += error * error;
    }
    const double position_error = std::sqrt(position_error_squared);
    const double orientation_error = math::quaternionAngularDistance(
        reconstructed.orientation_quat, target.orientation_quat);
    if (!std::isfinite(position_error) ||
        !std::isfinite(orientation_error) ||
        position_error > config_.max_position_error_m ||
        orientation_error > config_.max_orientation_error_rad) {
      cartesian_rejected = true;
      continue;
    }
    if (cost >= best_cost) {
      continue;
    }

    best_cost = cost;
    result.success = true;
    result.failure = ContinuousIKStepFailure::NONE;
    result.state = candidate.state;
    result.selected_candidate = candidate_index;
    result.maximum_joint_step_rad = maximum_step;
    result.position_error_m = position_error;
    result.orientation_error_rad = orientation_error;
    result.manipulability = candidate.manipulability;
    result.condition_number = candidate.condition_number;
  }

  if (!result.success) {
    if (cartesian_rejected) {
      result.failure = ContinuousIKStepFailure::CARTESIAN_ERROR;
    } else if (fk_failed) {
      result.failure = ContinuousIKStepFailure::FK_FAILURE;
    } else if (continuity_rejected) {
      result.failure = ContinuousIKStepFailure::CONTINUITY_LIMIT;
    } else {
      result.failure = ContinuousIKStepFailure::NO_CANDIDATE;
    }
  }
  return result;
}

} // namespace fr10_motion
