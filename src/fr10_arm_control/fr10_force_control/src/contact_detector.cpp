#include "fr10_force_control/contact_detector.hpp"

#include <cmath>
#include <stdexcept>

namespace fr10_force {

ContactDetector::ContactDetector(ContactDetectorConfig config)
    : config_(config) {
  if (!std::isfinite(config_.enter_force_n) ||
      !std::isfinite(config_.exit_force_n) ||
      !std::isfinite(config_.confirm_time_s) ||
      !std::isfinite(config_.release_time_s) ||
      !(config_.enter_force_n > config_.exit_force_n) ||
      config_.exit_force_n < 0.0 || !(config_.confirm_time_s > 0.0) ||
      !(config_.release_time_s > 0.0) ||
      !std::isfinite(config_.lateral_enter_force_n) || !std::isfinite(config_.lateral_exit_force_n) ||
      !(config_.lateral_enter_force_n > config_.lateral_exit_force_n) || config_.lateral_exit_force_n < 0. ||
      !std::isfinite(config_.tilt_enter_torque_nm) || !std::isfinite(config_.tilt_exit_torque_nm) ||
      !(config_.tilt_enter_torque_nm > config_.tilt_exit_torque_nm) || config_.tilt_exit_torque_nm < 0. ||
      !std::isfinite(config_.torsion_enter_torque_nm) || !std::isfinite(config_.torsion_exit_torque_nm) ||
      !(config_.torsion_enter_torque_nm > config_.torsion_exit_torque_nm) || config_.torsion_exit_torque_nm < 0.) {
    throw std::invalid_argument("contact detector configuration is invalid");
  }
}

ContactState ContactDetector::update(double signed_axis_force_n,
                                     double dt_s) {
  return updateChannel(channels_[0], signed_axis_force_n, config_.enter_force_n, config_.exit_force_n, dt_s);
}

ContactState ContactDetector::updateChannel(Channel &channel, double value,
                                            double enter, double exit, double dt_s) {
  if (!std::isfinite(value) || !std::isfinite(dt_s) ||
      !(dt_s > 0.0)) {
    channel = {};
    return channel.state;
  }
  switch (channel.state) {
  case ContactState::NO_CONTACT:
    if (value >= enter) {
      channel.state = ContactState::POSSIBLE_CONTACT;
      channel.enter_duration_s = dt_s;
    }
    break;
  case ContactState::POSSIBLE_CONTACT:
    if (value >= enter) {
      channel.enter_duration_s += dt_s;
      if (channel.enter_duration_s + 1e-12 >= config_.confirm_time_s) {
        channel.state = ContactState::CONTACT_CONFIRMED;
        channel.exit_duration_s = 0.0;
      }
    } else if (value <= exit) {
      channel = {};
    }
    break;
  case ContactState::CONTACT_CONFIRMED:
    if (value <= exit) {
      channel.exit_duration_s += dt_s;
      if (channel.exit_duration_s + 1e-12 >= config_.release_time_s) {
        channel = {};
      }
    } else {
      channel.exit_duration_s = 0.0;
    }
    break;
  }
  return channel.state;
}

ContactEvidence ContactDetector::update(const Wrench &wrench, std::size_t insertion_axis, double dt_s) {
  if (!finite(wrench) || insertion_axis >= 3 || !std::isfinite(dt_s) || dt_s <= 0.) {
    reset();
    return {};
  }
  double lateral_force = 0., tilt_torque = 0.;
  for (std::size_t i = 0; i < 3; ++i) if (i != insertion_axis) {
    lateral_force = std::hypot(lateral_force, wrench.force_n[i]);
    tilt_torque = std::hypot(tilt_torque, wrench.torque_nm[i]);
  }
  // Keep axial evidence separate: side load or tilt never proves seating.
  return {update(wrench.force_n[insertion_axis], dt_s),
          updateChannel(channels_[1], lateral_force, config_.lateral_enter_force_n,
                        config_.lateral_exit_force_n, dt_s),
          updateChannel(channels_[2], tilt_torque, config_.tilt_enter_torque_nm,
                        config_.tilt_exit_torque_nm, dt_s),
          updateChannel(channels_[3], std::abs(wrench.torque_nm[insertion_axis]),
                        config_.torsion_enter_torque_nm, config_.torsion_exit_torque_nm, dt_s)};
}

void ContactDetector::reset() {
  channels_ = {};
}

} // namespace fr10_force
