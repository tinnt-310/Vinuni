#include <cmath>
#include <string>
#include <vector>

#include "fr10_motion_core/cartesian_path.hpp"
#include "fr10_motion_core/cartesian_route_planner.hpp"
#include "fr10_motion_core/continuous_ik.hpp"
#include "fr10_motion_core/math_utils.hpp"
#include "test_support.hpp"

namespace {

class LinearIKProvider final : public fr10_motion::IKProvider {
public:
  fr10_motion::IKResult
  solve(const fr10_motion::Pose &target,
        const fr10_motion::KinematicState &seed) override {
    (void)seed;
    fr10_motion::IKResult result;
    result.success = true;
    result.state.position_rad[0] = target.position_m[0];
    result.state.position_rad[1] = target.position_m[1];
    result.state.position_rad[2] = target.position_m[2];
    result.manipulability = 1.0;
    result.condition_number = 2.0;
    return result;
  }

  fr10_motion::IKResult
  validate(const fr10_motion::KinematicState &target,
           const fr10_motion::KinematicState &seed) override {
    (void)seed;
    fr10_motion::IKResult result;
    result.success = true;
    result.state = target;
    return result;
  }

  bool forward(const fr10_motion::KinematicState &state,
               fr10_motion::Pose &pose, std::string *error) const override {
    (void)error;
    pose.position_m = {state.position_rad[0], state.position_rad[1],
                       state.position_rad[2]};
    pose.orientation_quat = {0.0, 0.0, 0.0, 1.0};
    return true;
  }
};

class SyntheticBarrierIKProvider final : public fr10_motion::IKProvider {
public:
  fr10_motion::IKResult
  solve(const fr10_motion::Pose &target,
        const fr10_motion::KinematicState & /* seed */) override {
    fr10_motion::IKResult result;
    if (target.position_m[0] >= 0.44 && target.position_m[0] <= 0.56 &&
        std::hypot(target.position_m[1], target.position_m[2]) < 0.03) {
      result.message = "synthetic singularity barrier";
      return result;
    }
    result.success = true;
    result.state.position_rad[0] = target.position_m[0];
    result.state.position_rad[1] = target.position_m[1];
    result.state.position_rad[2] = target.position_m[2];
    result.manipulability = 1.0;
    result.condition_number = 2.0;
    result.message = "success";
    return result;
  }

  fr10_motion::IKResult
  validate(const fr10_motion::KinematicState &target,
           const fr10_motion::KinematicState & /* seed */) override {
    fr10_motion::IKResult result;
    if (target.position_rad[0] >= 0.44 && target.position_rad[0] <= 0.56 &&
        std::hypot(target.position_rad[1], target.position_rad[2]) < 0.03) {
      result.message = "synthetic singularity barrier";
      return result;
    }
    result.success = true;
    result.state = target;
    return result;
  }

  bool forward(const fr10_motion::KinematicState &state,
               fr10_motion::Pose &pose,
               std::string * /* error */) const override {
    pose.position_m = {state.position_rad[0], state.position_rad[1],
                       state.position_rad[2]};
    pose.orientation_quat = {0.0, 0.0, 0.0, 1.0};
    return true;
  }
};

class CylindricalBarrierIKProvider final : public fr10_motion::IKProvider {
public:
  fr10_motion::IKResult
  solve(const fr10_motion::Pose &target,
        const fr10_motion::KinematicState & /* seed */) override {
    fr10_motion::IKResult result;
    if (std::hypot(target.position_m[0], target.position_m[1]) < 0.200) {
      result.message = "synthetic base cylinder barrier";
      return result;
    }
    result.success = true;
    result.state.position_rad[0] = target.position_m[0];
    result.state.position_rad[1] = target.position_m[1];
    result.state.position_rad[2] = target.position_m[2];
    result.manipulability = 1.0;
    result.condition_number = 2.0;
    result.message = "success";
    return result;
  }

  fr10_motion::IKResult
  validate(const fr10_motion::KinematicState &target,
           const fr10_motion::KinematicState & /* seed */) override {
    fr10_motion::IKResult result;
    if (std::hypot(target.position_rad[0], target.position_rad[1]) < 0.200) {
      result.message = "synthetic base cylinder barrier";
      return result;
    }
    result.success = true;
    result.state = target;
    result.manipulability = 1.0;
    result.condition_number = 2.0;
    return result;
  }

  bool forward(const fr10_motion::KinematicState &state,
               fr10_motion::Pose &pose,
               std::string * /* error */) const override {
    pose.position_m = {state.position_rad[0], state.position_rad[1],
                       state.position_rad[2]};
    pose.orientation_quat = {0.0, 0.0, 0.0, 1.0};
    return true;
  }
};

} // namespace

int main() {
  // Resolve sub-microradian rotations without the acos(dot) precision floor.
  // This is the range used by live-boundary FK velocity finite differences.
  fr10_motion_test::Check check;
  const auto base = fr10_motion_test::axisAngle(2, 1.7);
  for (const double angle : {1e-10, 1e-8, 1e-6, 0.2, 3.141592653589793}) {
    const auto rotated = fr10_motion::math::quaternionMultiply(
        base, fr10_motion_test::axisAngle(0, angle));
    auto antipodal = rotated;
    for (auto &value : antipodal) value *= -3.0;
    check.require(std::abs(fr10_motion::math::quaternionAngularDistance(base, rotated)-angle)<1e-14 &&
                      std::abs(fr10_motion::math::quaternionAngularDistance(base, antipodal)-angle)<1e-14,
                  "QUATERNION_SMALL_ANGLE_AND_SIGN_"+std::to_string(angle),
                  "shortest angle must preserve small rotations and quaternion sign/scale invariance");
  }
  check.require(std::isinf(fr10_motion::math::quaternionAngularDistance(
                    base, {0.,0.,0.,0.})) &&
                    std::isinf(fr10_motion::math::quaternionAngularDistance(
                    base, {0.,0.,0.,std::numeric_limits<double>::infinity()})),
                "QUATERNION_INVALID_DISTANCE", "invalid quaternion must fail closed");

  fr10_motion::Pose start;
  start.position_m = {0.20, -0.10, 0.40};
  fr10_motion::Pose goal = start;
  goal.position_m[0] += 0.070;
  fr10_motion::CartesianPathConfig path_config;
  path_config.max_translation_step_m = 0.010;
  path_config.max_rotation_step_rad = 0.10;
  std::vector<fr10_motion::Pose> path;
  std::string error;
  check.require(fr10_motion::CartesianPath::linear(start, goal, path_config,
                                                   path, &error),
                "CART_PATH_CREATE", error);
  bool path_resolution_valid = path.size() >= 2;
  for (std::size_t index = 1; index < path.size(); ++index) {
    path_resolution_valid =
        path_resolution_valid &&
        std::abs(path[index].position_m[0] - path[index - 1].position_m[0]) <=
            0.010 + 1e-12;
  }
  check.require(path_resolution_valid, "CART_PATH_RESOLUTION",
                "linear path exceeded the configured segment resolution");
  check.require(std::abs(path.front().position_m[0] - 0.20) < 1e-12 &&
                    std::abs(path.back().position_m[0] - 0.27) < 1e-12,
                "CART_PATH_ENDPOINTS", "path endpoints changed");
  std::size_t counted_segments = 0;
  check.require(fr10_motion::CartesianPath::linearSegmentCount(
                    start, goal, path_config, counted_segments, &error) &&
                    counted_segments + 1 == path.size(),
                "CART_PATH_SEGMENT_COUNT_PARITY",
                "lightweight segment count differs from generated path");
  auto huge_goal = goal;
  huge_goal.position_m[0] = 1e308;
  counted_segments = 99;
  std::vector<fr10_motion::Pose> huge_path;
  check.require(!fr10_motion::CartesianPath::linearSegmentCount(
                    start, huge_goal, path_config, counted_segments, &error) &&
                    counted_segments == 0 &&
                    !fr10_motion::CartesianPath::linear(
                        start, huge_goal, path_config, huge_path, &error) &&
                    huge_path.empty(),
                "CART_PATH_HUGE_FINITE_REJECT",
                "huge finite coordinates overflowed segment count");

  fr10_motion::Pose rotation_goal = start;
  rotation_goal.orientation_quat =
      fr10_motion_test::axisAngle(2, 0.5 * std::acos(-1.0));
  std::vector<fr10_motion::Pose> rotation_path;
  check.require(fr10_motion::CartesianPath::linear(
                    start, rotation_goal, path_config, rotation_path, &error),
                "CART_PATH_ROTATION_CREATE", error);
  bool rotation_resolution_valid = rotation_path.size() >= 2;
  for (std::size_t index = 1; index < rotation_path.size(); ++index) {
    rotation_resolution_valid =
        rotation_resolution_valid &&
        fr10_motion::math::quaternionAngularDistance(
            rotation_path[index - 1].orientation_quat,
            rotation_path[index].orientation_quat) <= 0.10 + 1e-12;
  }
  check.require(rotation_resolution_valid &&
                    fr10_motion::math::quaternionAngularDistance(
                        rotation_path.back().orientation_quat,
                        rotation_goal.orientation_quat) < 1e-12,
                "CART_PATH_ROTATION_RESOLUTION",
                "SLERP path exceeded angular resolution or changed endpoint");

  fr10_motion::Pose large_rotation_goal = start;
  large_rotation_goal.orientation_quat =
      fr10_motion_test::axisAngle(1, 2.0 * std::acos(-1.0) / 3.0);
  std::vector<fr10_motion::Pose> large_rotation_path;
  check.require(fr10_motion::CartesianPath::linear(
                    start, large_rotation_goal, path_config,
                    large_rotation_path, &error) &&
                    large_rotation_path.size() >= 21 &&
                    fr10_motion::math::quaternionAngularDistance(
                        large_rotation_path.back().orientation_quat,
                        large_rotation_goal.orientation_quat) < 1e-12,
                "CART_PATH_LARGE_ROTATION",
                "120 degree goal was not preserved as dense SLERP");

  fr10_motion::Pose socket;
  socket.position_m = {0.4, 0.2, 0.6};
  const double half_sqrt = std::sqrt(0.5);
  socket.orientation_quat = {0.0, 0.0, half_sqrt, half_sqrt};
  fr10_motion::Pose pre_insert;
  check.require(fr10_motion::CartesianPath::preInsertPose(
                    socket, {1.0, 0.0, 0.0}, 0.070, pre_insert, &error),
                "PRE_INSERT_CREATE", error);
  check.require(
      std::abs(pre_insert.position_m[0] - socket.position_m[0]) < 1e-12 &&
          std::abs(pre_insert.position_m[1] - 0.13) < 1e-12 &&
          fr10_motion::math::quaternionAngularDistance(
              pre_insert.orientation_quat, socket.orientation_quat) < 1e-12,
      "PRE_INSERT_LOCAL_AXIS",
      "local X was not rotated into base Y or orientation changed");

  LinearIKProvider provider;
  fr10_motion::KinematicState seed;
  seed.position_rad[0] = start.position_m[0];
  seed.position_rad[1] = start.position_m[1];
  seed.position_rad[2] = start.position_m[2];
  fr10_motion::ContinuousIKConfig ik_config;
  ik_config.max_joint_step_rad = 0.02;
  ik_config.max_position_error_m = 1e-9;
  ik_config.max_orientation_error_rad = 1e-9;
  const auto continuous =
      fr10_motion::ContinuousIK::solve(path, seed, provider, ik_config);
  check.require(continuous.success, "CONTINUOUS_IK_SUCCESS",
                continuous.message);
  check.require(continuous.joint_path.size() == path.size() &&
                    continuous.max_position_error_m < 1e-9,
                "CONTINUOUS_IK_FK_ERROR", "IK/FK path mismatch");

  ik_config.max_joint_step_rad = 0.005;
  const auto rejected =
      fr10_motion::ContinuousIK::solve(path, seed, provider, ik_config);
  check.require(!rejected.success && rejected.failure_index > 0,
                "CONTINUOUS_IK_JUMP_REJECT",
                "joint continuity barrier did not reject a large step");

  fr10_motion::CartesianRoutePlannerConfig free_space_config;
  free_space_config.mode = fr10_motion::CartesianRouteMode::FREE_SPACE_JOINT;
  free_space_config.path.max_translation_step_m = 0.01;
  free_space_config.path.max_rotation_step_rad = 0.10;
  free_space_config.path.max_waypoints = 1000;
  free_space_config.continuous_ik.max_joint_step_rad = 0.03;
  free_space_config.continuous_ik.max_position_error_m = 1e-9;
  free_space_config.continuous_ik.max_orientation_error_rad = 1e-9;
  free_space_config.limits.min_joint_position_rad.fill(-2.0);
  free_space_config.limits.max_joint_position_rad.fill(2.0);
  const auto free_space_route = fr10_motion::CartesianRoutePlanner::plan(
      start, goal, seed, provider, free_space_config);
  check.require(free_space_route.success &&
                    free_space_route.free_space_joint_route &&
                    free_space_route.used_joint_space_detour &&
                    free_space_route.continuous_ik.joint_path.size() ==
                        free_space_route.cartesian_path.size(),
                "CART_ROUTE_FREE_SPACE_JOINT_MODE", free_space_route.message);

  CylindricalBarrierIKProvider cylindrical_provider;
  fr10_motion::Pose cylindrical_start;
  cylindrical_start.position_m = {-0.50, 0.10, 0.40};
  fr10_motion::Pose cylindrical_goal = cylindrical_start;
  cylindrical_goal.position_m[0] = 0.50;
  fr10_motion::KinematicState cylindrical_seed;
  cylindrical_seed.position_rad[0] = cylindrical_start.position_m[0];
  cylindrical_seed.position_rad[1] = cylindrical_start.position_m[1];
  cylindrical_seed.position_rad[2] = cylindrical_start.position_m[2];
  auto cylindrical_config = free_space_config;
  cylindrical_config.path.max_translation_step_m = 0.005;
  cylindrical_config.continuous_ik.max_joint_step_rad = 0.03;
  cylindrical_config.cylindrical_barrier_radius_m = 0.200;
  cylindrical_config.cylindrical_detour_radius_m = 0.350;
  const auto cylindrical_route = fr10_motion::CartesianRoutePlanner::plan(
      cylindrical_start, cylindrical_goal, cylindrical_seed,
      cylindrical_provider, cylindrical_config);
  const auto cylindrical_repeat = fr10_motion::CartesianRoutePlanner::plan(
      cylindrical_start, cylindrical_goal, cylindrical_seed,
      cylindrical_provider, cylindrical_config);
  bool cylindrical_clearance = cylindrical_route.success &&
      cylindrical_route.used_cylindrical_detour &&
      cylindrical_route.cartesian_path.size() ==
          cylindrical_repeat.cartesian_path.size();
  for (std::size_t index = 0;
       cylindrical_clearance && index < cylindrical_route.cartesian_path.size();
       ++index) {
    const auto &pose = cylindrical_route.cartesian_path[index];
    const auto &repeat = cylindrical_repeat.cartesian_path[index];
    cylindrical_clearance =
        std::hypot(pose.position_m[0], pose.position_m[1]) >= 0.350 - 1e-9 &&
        std::abs(pose.position_m[0] - repeat.position_m[0]) < 1e-15 &&
        std::abs(pose.position_m[1] - repeat.position_m[1]) < 1e-15;
  }
  check.require(cylindrical_clearance,
                "CART_ROUTE_DETERMINISTIC_CYLINDER",
                cylindrical_route.message);

  SyntheticBarrierIKProvider barrier_provider;
  fr10_motion::Pose barrier_start;
  barrier_start.position_m = {0.20, 0.0, 0.0};
  fr10_motion::Pose barrier_goal = barrier_start;
  barrier_goal.position_m[0] = 0.80;
  fr10_motion::KinematicState barrier_seed;
  barrier_seed.position_rad[0] = barrier_start.position_m[0];
  fr10_motion::CartesianRoutePlannerConfig route_config;
  route_config.path.max_translation_step_m = 0.01;
  route_config.path.max_rotation_step_rad = 0.10;
  route_config.path.max_waypoints = 1000;
  route_config.continuous_ik.max_joint_step_rad = 0.03;
  route_config.continuous_ik.max_position_error_m = 1e-10;
  route_config.continuous_ik.max_orientation_error_rad = 1e-10;
  route_config.detour_clearances_m = {0.05};
  route_config.limits.min_joint_position_rad.fill(-2.0);
  route_config.limits.max_joint_position_rad.fill(2.0);
  const auto avoided = fr10_motion::CartesianRoutePlanner::plan(
      barrier_start, barrier_goal, barrier_seed, barrier_provider,
      route_config);
  bool detour_avoids_barrier =
      avoided.success && avoided.used_detour &&
      (avoided.used_joint_space_detour || avoided.detour_clearance_m == 0.05);
  for (const auto &pose : avoided.cartesian_path) {
    if (pose.position_m[0] >= 0.44 && pose.position_m[0] <= 0.56 &&
        std::hypot(pose.position_m[1], pose.position_m[2]) < 0.03) {
      detour_avoids_barrier = false;
    }
  }
  check.require(detour_avoids_barrier &&
                    avoided.direct_failure_message.find("singularity") !=
                        std::string::npos &&
                    std::abs(avoided.cartesian_path.front().position_m[0] -
                             barrier_start.position_m[0]) < 1e-12 &&
                    std::abs(avoided.cartesian_path.back().position_m[0] -
                             barrier_goal.position_m[0]) < 1e-12,
                "CART_ROUTE_SINGULARITY_DETOUR", avoided.message);

  auto insertion_route_config = route_config;
  insertion_route_config.mode =
      fr10_motion::CartesianRouteMode::CARTESIAN_LINEAR;
  insertion_route_config.singularity_avoidance_enabled = false;
  const auto insertion_rejected = fr10_motion::CartesianRoutePlanner::plan(
      barrier_start, barrier_goal, barrier_seed, barrier_provider,
      insertion_route_config);
  check.require(
      !insertion_rejected.success && !insertion_rejected.used_detour &&
          insertion_rejected.detour_attempts == 0 &&
          insertion_rejected.direct_failure_message.find("singularity") !=
              std::string::npos,
      "CART_ROUTE_INSERTION_ZERO_BENDING_REJECT", insertion_rejected.message);

  fr10_motion::Pose forbidden_goal = barrier_start;
  forbidden_goal.position_m[0] = 0.50;
  const auto impossible = fr10_motion::CartesianRoutePlanner::plan(
      barrier_start, forbidden_goal, barrier_seed, barrier_provider,
      route_config);
  check.require(!impossible.success && impossible.detour_attempts > 0,
                "CART_ROUTE_FORBIDDEN_ENDPOINT_REJECT",
                "planner weakened the endpoint singularity barrier");
  return check.result();
}
