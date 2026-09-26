#include "fr10_pose_controller/linear_insert_planner.hpp"

#include <algorithm>
#include <cmath>

#include "fr10_pose_controller/math_utils.hpp"

namespace fr10_motion {
namespace {

double translationDistance(const Pose &first, const Pose &second) {
  double squared = 0.0;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const double delta = second.position_m[axis] - first.position_m[axis];
    squared += delta * delta;
  }
  return std::sqrt(squared);
}

bool validScale(double scale) {
  return std::isfinite(scale) && scale > 0.0 && scale <= 1.0;
}

} // namespace

const char *linearInsertStopReasonName(LinearInsertStopReason reason) {
  switch (reason) {
  case LinearInsertStopReason::NONE:
    return "NONE";
  case LinearInsertStopReason::INVALID_INPUT:
    return "INVALID_INPUT";
  case LinearInsertStopReason::UNEXPECTED_RESISTANCE:
    return "UNEXPECTED_RESISTANCE";
  case LinearInsertStopReason::PERPENDICULAR_DEVIATION:
    return "PERPENDICULAR_DEVIATION";
  case LinearInsertStopReason::ORIENTATION_ERROR:
    return "ORIENTATION_ERROR";
  case LinearInsertStopReason::POSITION_ERROR:
    return "POSITION_ERROR";
  }
  return "UNKNOWN";
}

LinearInsertPlan LinearInsertPlanner::plan(
    const Pose &pre_insert, const Pose &target,
    const LinearInsertPlannerConfig &config) {
  LinearInsertPlan result;
  result.start_pose = pre_insert;
  result.target_pose = target;
  result.distance_m = translationDistance(pre_insert, target);
  result.linear_speed_scale = config.linear_speed_scale;
  result.angular_speed_scale = config.angular_speed_scale;
  const double orientation_change = math::quaternionAngularDistance(
      pre_insert.orientation_quat, target.orientation_quat);
  if (!(config.max_insertion_distance_m > 0.0) ||
      !(config.max_orientation_change_rad >= 0.0) ||
      !(config.perpendicular_tolerance_m > 0.0) ||
      !(config.position_tolerance_m > 0.0) ||
      !(config.orientation_tolerance_rad > 0.0) ||
      !std::isfinite(result.distance_m) ||
      !std::isfinite(orientation_change) ||
      result.distance_m > config.max_insertion_distance_m ||
      orientation_change > config.max_orientation_change_rad ||
      !validScale(config.linear_speed_scale) ||
      !validScale(config.angular_speed_scale)) {
    result.message =
        "linear insertion distance, orientation, tolerance, or speed is invalid";
    return result;
  }
  if (!CartesianPath::linear(pre_insert, target, config.path,
                             result.cartesian_path, &result.message)) {
    return result;
  }
  result.success = true;
  result.message = "success";
  return result;
}

LinearInsertTracking LinearInsertPlanner::monitor(
    const Pose &path_start, const Pose &path_goal,
    const Pose &desired_pose, const Pose &actual_pose,
    bool unexpected_resistance, const LinearInsertPlannerConfig &config) {
  LinearInsertTracking result;
  result.position_error_m = translationDistance(desired_pose, actual_pose);
  result.orientation_error_rad = math::quaternionAngularDistance(
      desired_pose.orientation_quat, actual_pose.orientation_quat);
  result.perpendicular_error_m = CartesianPath::translationDeviation(
      path_start, path_goal, actual_pose);
  result.unexpected_resistance = unexpected_resistance;
  result.valid = std::isfinite(result.position_error_m) &&
      std::isfinite(result.orientation_error_rad) &&
      std::isfinite(result.perpendicular_error_m);
  if (!result.valid) {
    result.stop_reason = LinearInsertStopReason::INVALID_INPUT;
  } else if (unexpected_resistance) {
    result.stop_reason = LinearInsertStopReason::UNEXPECTED_RESISTANCE;
  } else if (result.perpendicular_error_m >
             config.perpendicular_tolerance_m) {
    result.stop_reason = LinearInsertStopReason::PERPENDICULAR_DEVIATION;
  } else if (result.orientation_error_rad > config.orientation_tolerance_rad) {
    result.stop_reason = LinearInsertStopReason::ORIENTATION_ERROR;
  } else if (result.position_error_m > config.position_tolerance_m) {
    result.stop_reason = LinearInsertStopReason::POSITION_ERROR;
  }
  result.controlled_stop_required =
      result.stop_reason != LinearInsertStopReason::NONE;
  return result;
}

} // namespace fr10_motion
