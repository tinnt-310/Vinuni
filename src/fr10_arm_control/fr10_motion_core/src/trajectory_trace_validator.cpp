#include "fr10_motion_core/trajectory_trace_validator.hpp"

#include <algorithm>
#include <cmath>

#include "fr10_motion_core/cartesian_path.hpp"
#include "fr10_motion_core/math_utils.hpp"

namespace fr10_motion {

TrajectoryTraceResult TrajectoryTraceValidator::validate(
    const CartesianJointTrajectory &trajectory,
    const KinematicState &seed, IKProvider &provider,
    const TrajectoryTraceLimits &limits, const CancelCheck &cancel) {
  TrajectoryTraceResult result;
  if (trajectory.points.empty()) {
    result.message = "trajectory has no timed samples";
    return result;
  }

  KinematicState previous = seed;
  for (std::size_t index = 0; index < trajectory.points.size(); ++index) {
    result.failed_index = index;
    if (cancel && cancel()) {
      result.canceled = true;
      result.message = "trajectory FK validation canceled";
      return result;
    }

    const auto &point = trajectory.points[index];
    const IKResult checked = provider.validate(point.joint_state, previous);
    if (!checked.success) {
      result.message = checked.message;
      return result;
    }

    Pose reconstructed;
    if (!provider.forward(point.joint_state, reconstructed, &result.message)) {
      return result;
    }
    double squared_position_error = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const double delta = reconstructed.position_m[axis] -
                           point.desired_pose.position_m[axis];
      squared_position_error += delta * delta;
    }
    const double position_error = std::sqrt(squared_position_error);
    const double orientation_error = math::quaternionAngularDistance(
        reconstructed.orientation_quat, point.desired_pose.orientation_quat);
    const double path_deviation = CartesianPath::translationDeviation(
        point.desired_pose, point.desired_translation_tangent,
        reconstructed);
    result.max_position_error_m =
        std::max(result.max_position_error_m, position_error);
    result.max_orientation_error_rad =
        std::max(result.max_orientation_error_rad, orientation_error);
    result.max_path_deviation_m =
        std::max(result.max_path_deviation_m, path_deviation);

    if (!std::isfinite(position_error) || !std::isfinite(orientation_error) ||
        !std::isfinite(path_deviation) ||
        (limits.strict_cartesian &&
         (position_error > limits.position_tolerance_m ||
          orientation_error > limits.orientation_tolerance_rad ||
          path_deviation > limits.path_deviation_tolerance_m))) {
      result.message =
          "time-parameterized FK trace exceeds Cartesian tolerance";
      return result;
    }
    previous = point.joint_state;
  }
  result.success = true;
  return result;
}

} // namespace fr10_motion
