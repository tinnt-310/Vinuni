#include "fr10_motion_core/pre_insert_planner.hpp"

#include <algorithm>
#include <cmath>

#include "fr10_motion_core/math_utils.hpp"

namespace fr10_motion {

PreInsertPlan PreInsertPlanner::plan(const Pose &target,
                                     const PreInsertPlannerConfig &config) {
  PreInsertPlan result;
  result.target_pose = target;
  result.distance_m = config.distance_m;
  const bool finite_pose =
      std::all_of(target.position_m.begin(), target.position_m.end(),
                  [](double value) { return std::isfinite(value); }) &&
      std::all_of(target.orientation_quat.begin(),
                  target.orientation_quat.end(),
                  [](double value) { return std::isfinite(value); }) &&
      math::quaternionNormSquared(target.orientation_quat) > 1e-12;
  const bool finite_axis =
      std::all_of(config.local_axis.begin(), config.local_axis.end(),
                  [](double value) { return std::isfinite(value); });
  if (!finite_pose || !finite_axis || !std::isfinite(config.distance_m) ||
      !std::isfinite(config.min_distance_m) ||
      !std::isfinite(config.max_distance_m) ||
      config.min_distance_m < 0.0 ||
      config.max_distance_m < config.min_distance_m ||
      config.distance_m < config.min_distance_m ||
      config.distance_m > config.max_distance_m) {
    result.message = "pre-insert pose, axis, or distance limits are invalid";
    return result;
  }

  const double norm = std::sqrt(
      config.local_axis[0] * config.local_axis[0] +
      config.local_axis[1] * config.local_axis[1] +
      config.local_axis[2] * config.local_axis[2]);
  if (!(norm > 1e-12) || !std::isfinite(norm)) {
    result.message = "pre-insert local axis must be non-zero";
    return result;
  }
  const std::array<double, 3> normalized_axis{
      config.local_axis[0] / norm, config.local_axis[1] / norm,
      config.local_axis[2] / norm};
  result.base_axis =
      math::rotateVector(target.orientation_quat, normalized_axis);
  result.pre_insert_pose = target;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    result.pre_insert_pose.position_m[axis] -=
        config.distance_m * result.base_axis[axis];
  }
  result.success = true;
  result.message = "success";
  return result;
}

} // namespace fr10_motion
