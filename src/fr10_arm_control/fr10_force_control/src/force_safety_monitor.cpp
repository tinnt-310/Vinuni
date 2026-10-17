#include "fr10_force_control/force_safety_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fr10_force {

ForceSafetyMonitor::ForceSafetyMonitor(ForceSafetyConfig config)
    : config_(config) {
  if (!finite(config_.max_force_n) || !finite(config_.max_torque_nm) ||
      !finite(config_.max_force_rate_n_s) ||
      !finite(config_.max_correction) ||
      !std::isfinite(config_.max_depth_m) ||
      !std::isfinite(config_.stale_timeout_s) ||
      !(config_.max_depth_m > 0.0) || !(config_.stale_timeout_s > 0.0) ||
      !(config_.max_lateral_correction_norm_m > 0) ||
      !(config_.max_rotation_correction_norm_rad > 0) || config_.insertion_axis >= 3) {
    throw std::invalid_argument("force safety configuration is invalid");
  }
  for (double limit : config_.max_force_n) {
    if (!(limit > 0.0)) throw std::invalid_argument("force limit invalid");
  }
  for (double limit : config_.max_torque_nm) {
    if (!(limit > 0.0)) throw std::invalid_argument("torque limit invalid");
  }
  for (double limit : config_.max_force_rate_n_s) {
    if (!(limit > 0.0)) throw std::invalid_argument("rate limit invalid");
  }
  for (double limit : config_.max_correction) {
    if (!(limit > 0.0)) throw std::invalid_argument("correction limit invalid");
  }
}

ForceSafetyResult ForceSafetyMonitor::update(const ForceSafetyInput &input) {
  ForceSafetyResult result;
  const auto &sample = input.raw_sample;
  if (!sample.valid || sample.sequence == 0 || !finite(sample.wrench) ||
      !finite(input.conditioned_wrench) ||
      !finite(input.correction.position) ||
      !std::isfinite(sample.source_timestamp_s) ||
      !std::isfinite(sample.receipt_steady_s) ||
      !std::isfinite(input.now_steady_s) || !std::isfinite(input.depth_m)) {
    result.fault = ForceSafetyFault::INVALID_WRENCH;
    return result;
  }
  if (sample.overload) {
    result.fault = ForceSafetyFault::SENSOR_OVERLOAD;
    return result;
  }

  const bool new_sample = !initialized_ || sample.sequence > last_sequence_;
  if (initialized_ && new_sample &&
      sample.source_timestamp_s < last_source_timestamp_s_ - 1e-12) {
    result.fault = ForceSafetyFault::CLOCK_JUMP;
    return result;
  }
  if (new_sample) {
    last_force_rate_n_s_ = {};
    if (initialized_ && !input.conditioned_reference_changed &&
        sample.source_timestamp_s > last_source_timestamp_s_) {
      const double dt_s = sample.source_timestamp_s - last_source_timestamp_s_;
      for (std::size_t axis = 0; axis < 3; ++axis) {
        result.force_rate_n_s[axis] =
            (input.conditioned_wrench.force_n[axis] - last_force_n_[axis]) /
            dt_s;
      }
    }
    last_sequence_ = sample.sequence;
    last_unique_receipt_s_ = sample.receipt_steady_s;
    last_source_timestamp_s_ = sample.source_timestamp_s;
    last_force_n_ = input.conditioned_wrench.force_n;
    initialized_ = true;
    last_force_rate_n_s_ = result.force_rate_n_s;
  }
  result.force_rate_n_s = last_force_rate_n_s_;
  result.source_age_s = std::max(0.0, input.now_steady_s -
                                           last_unique_receipt_s_);
  if (result.source_age_s > config_.stale_timeout_s || !new_sample) {
    // A duplicate is allowed until the receipt watchdog expires, but it never
    // refreshes last_unique_receipt_s_.
    if (result.source_age_s > config_.stale_timeout_s) {
      result.fault = ForceSafetyFault::STALE_WRENCH;
      return result;
    }
  }

  for (std::size_t axis = 0; axis < 3; ++axis) {
    if (std::abs(sample.wrench.force_n[axis]) > config_.max_force_n[axis] ||
        std::abs(input.conditioned_wrench.force_n[axis]) > config_.max_force_n[axis]) {
      result.fault = ForceSafetyFault::FORCE_LIMIT;
      return result;
    }
    if (std::abs(sample.wrench.torque_nm[axis]) > config_.max_torque_nm[axis] ||
        std::abs(input.conditioned_wrench.torque_nm[axis]) > config_.max_torque_nm[axis]) {
      result.fault = ForceSafetyFault::TORQUE_LIMIT;
      return result;
    }
    if (std::abs(result.force_rate_n_s[axis]) >
        config_.max_force_rate_n_s[axis]) {
      result.fault = ForceSafetyFault::FORCE_RATE_LIMIT;
      return result;
    }
  }
  for (std::size_t axis = 0; axis < kCartesianDofs; ++axis) {
    if (std::abs(input.correction.position[axis]) >
        config_.max_correction[axis]) {
      result.fault = ForceSafetyFault::CORRECTION_LIMIT;
      return result;
    }
  }
  double lateral_squared = 0, rotation_squared = 0;
  for (std::size_t i = 0; i < 3; ++i) {
    if (i != config_.insertion_axis) lateral_squared += input.correction.position[i]*input.correction.position[i];
    rotation_squared += input.correction.position[i+3]*input.correction.position[i+3];
  }
  if (std::sqrt(lateral_squared) > config_.max_lateral_correction_norm_m ||
      std::sqrt(rotation_squared) > config_.max_rotation_correction_norm_rad) {
    result.fault = ForceSafetyFault::CORRECTION_LIMIT; return result;
  }
  if (input.depth_m > config_.max_depth_m) {
    result.fault = ForceSafetyFault::DEPTH_LIMIT;
  } else if (input.contact_required && !input.contact_confirmed) {
    result.fault = ForceSafetyFault::LOST_CONTACT;
  } else if (!input.ik_ok) {
    result.fault = ForceSafetyFault::IK_FAILURE;
  } else if (!input.collision_free) {
    result.fault = ForceSafetyFault::COLLISION;
  } else if (!input.scene_unchanged) {
    result.fault = ForceSafetyFault::SCENE_CHANGED;
  } else if (!input.tracking_ok) {
    result.fault = ForceSafetyFault::TRACKING_FAILURE;
  } else if (!input.robot_ok) {
    result.fault = ForceSafetyFault::ROBOT_FAULT;
  } else if (!input.command_accepted) {
    result.fault = ForceSafetyFault::COMMAND_REJECTED;
  }
  return result;
}

void ForceSafetyMonitor::reset() {
  last_sequence_ = 0;
  last_unique_receipt_s_ = 0.0;
  last_source_timestamp_s_ = 0.0;
  last_force_n_ = {};
  last_force_rate_n_s_ = {};
  initialized_ = false;
}

} // namespace fr10_force
