#pragma once

#include <memory>
#include <string>

#include "fr10_motion_core/cartesian_trajectory.hpp"
#include "fr10_motion_interfaces/msg/joint_trajectory_command.hpp"

namespace fr10_motion_controller {

struct DecodedTrajectory {
  std::shared_ptr<const fr10_motion::CartesianJointTrajectory> trajectory;
  uint8_t mode{0};
  std::string error;

  explicit operator bool() const { return trajectory != nullptr; }
};

// Converts the typed ROS transport into the ROS-free immutable core type.
// This function runs in the subscription callback, never in update().
DecodedTrajectory decodeTrajectory(
    const fr10_motion_interfaces::msg::JointTrajectoryCommand &message);

} // namespace fr10_motion_controller
