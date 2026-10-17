#pragma once

#include <cstddef>
#include <functional>
#include <string>

#include "fr10_motion_core/cartesian_trajectory.hpp"
#include "fr10_motion_core/ik_provider.hpp"

namespace fr10_motion {

struct TrajectoryTraceLimits {
  bool strict_cartesian{false};
  double position_tolerance_m{0.0};
  double orientation_tolerance_rad{0.0};
  double path_deviation_tolerance_m{0.0};
};

struct TrajectoryTraceResult {
  bool success{false};
  bool canceled{false};
  std::size_t failed_index{0};
  double max_position_error_m{0.0};
  double max_orientation_error_rad{0.0};
  double max_path_deviation_m{0.0};
  std::string message;
};

// Validates every timed joint sample against one IK/collision provider and its
// reconstructed TCP trace. FREE_SPACE reports Cartesian error without making
// the joint-space route a straight-line constraint; INSERTION enforces it.
class TrajectoryTraceValidator {
public:
  using CancelCheck = std::function<bool()>;

  static TrajectoryTraceResult validate(
      const CartesianJointTrajectory &trajectory,
      const KinematicState &seed, IKProvider &provider,
      const TrajectoryTraceLimits &limits,
      const CancelCheck &cancel = {});
};

} // namespace fr10_motion
