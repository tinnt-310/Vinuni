#include <cmath>
#include <iostream>
#include <limits>

#include "fr10_pose_controller/linear_insert_planner.hpp"
#include "fr10_pose_controller/pre_insert_planner.hpp"
#include "test_support.hpp"

int main() {
  fr10_motion_test::Check check;
  fr10_motion::Pose socket;
  socket.position_m = {0.50, 0.20, 0.60};
  const double half_sqrt = std::sqrt(0.5);
  socket.orientation_quat = {0.0, 0.0, half_sqrt, half_sqrt};

  fr10_motion::PreInsertPlannerConfig pre_config;
  pre_config.local_axis = {1.0, 0.0, 0.0};
  pre_config.distance_m = 0.070;
  const auto pre = fr10_motion::PreInsertPlanner::plan(socket, pre_config);
  check.require(pre.success, "INSERT_PRE_PLAN", pre.message);
  check.require(
      std::abs(pre.pre_insert_pose.position_m[0] - 0.50) < 1e-12 &&
          std::abs(pre.pre_insert_pose.position_m[1] - 0.13) < 1e-12 &&
          std::abs(pre.pre_insert_pose.position_m[2] - 0.60) < 1e-12,
      "INSERT_PRE_LOCAL_AXIS", "socket-local X was not rotated into base Y");
  check.require(
      fr10_motion::math::quaternionAngularDistance(
          socket.orientation_quat,
          pre.pre_insert_pose.orientation_quat) < 1e-12,
      "INSERT_PRE_ORIENTATION", "pre-insertion changed socket orientation");

  fr10_motion::LinearInsertPlannerConfig linear_config;
  linear_config.path.max_translation_step_m = 0.005;
  linear_config.path.max_rotation_step_rad = 0.01;
  linear_config.path.max_waypoints = 100;
  linear_config.max_insertion_distance_m = 0.100;
  linear_config.max_orientation_change_rad = 1e-6;
  linear_config.perpendicular_tolerance_m = 0.001;
  linear_config.position_tolerance_m = 0.002;
  linear_config.orientation_tolerance_rad = 0.01;
  linear_config.linear_speed_scale = 0.10;
  linear_config.angular_speed_scale = 0.10;
  const auto insertion = fr10_motion::LinearInsertPlanner::plan(
      pre.pre_insert_pose, socket, linear_config);
  check.require(insertion.success, "INSERT_LINEAR_PLAN", insertion.message);
  bool insertion_resolution_ok = insertion.cartesian_path.size() >= 15;
  for (std::size_t index = 1; index < insertion.cartesian_path.size(); ++index) {
    double squared_step = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const double delta =
          insertion.cartesian_path[index].position_m[axis] -
          insertion.cartesian_path[index - 1].position_m[axis];
      squared_step += delta * delta;
    }
    insertion_resolution_ok =
        insertion_resolution_ok && std::sqrt(squared_step) <= 0.005 + 1e-12;
  }
  check.require(insertion_resolution_ok &&
                    std::abs(insertion.distance_m - 0.070) < 1e-12,
                "INSERT_LINEAR_RESOLUTION",
                "70 mm insertion exceeded the configured 5 mm resolution");
  bool straight = true;
  for (const auto &pose : insertion.cartesian_path) {
    straight = straight &&
        fr10_motion::CartesianPath::translationDeviation(
            insertion.start_pose, insertion.target_pose, pose) < 1e-12;
  }
  check.require(straight, "INSERT_LINEAR_STRAIGHT",
                "planned insertion curved away from the socket line");

  fr10_motion::Pose desired = fr10_motion::CartesianPath::interpolate(
      insertion.start_pose, insertion.target_pose, 0.5);
  fr10_motion::Pose actual = desired;
  actual.position_m[0] += 0.0015;
  auto tracking = fr10_motion::LinearInsertPlanner::monitor(
      insertion.start_pose, insertion.target_pose, desired, actual, false,
      linear_config);
  check.require(tracking.controlled_stop_required &&
                    tracking.valid &&
                    tracking.perpendicular_error_m > 0.001 &&
                    tracking.stop_reason == fr10_motion::
                                                LinearInsertStopReason::
                                                    PERPENDICULAR_DEVIATION,
                "INSERT_PATH_DEVIATION_STOP",
                "perpendicular deviation did not report its stop reason");
  tracking = fr10_motion::LinearInsertPlanner::monitor(
      insertion.start_pose, insertion.target_pose, desired, desired, true,
      linear_config);
  check.require(tracking.controlled_stop_required &&
                    tracking.valid && tracking.unexpected_resistance &&
                    tracking.stop_reason ==
                        fr10_motion::LinearInsertStopReason::
                            UNEXPECTED_RESISTANCE,
                "INSERT_RESISTANCE_STOP",
                "unexpected resistance did not report its stop reason");
  actual = desired;
  actual.position_m[0] = std::numeric_limits<double>::quiet_NaN();
  tracking = fr10_motion::LinearInsertPlanner::monitor(
      insertion.start_pose, insertion.target_pose, desired, actual, false,
      linear_config);
  check.require(!tracking.valid && tracking.controlled_stop_required &&
                    tracking.stop_reason ==
                        fr10_motion::LinearInsertStopReason::INVALID_INPUT &&
                    std::string(fr10_motion::linearInsertStopReasonName(
                        tracking.stop_reason)) == "INVALID_INPUT",
                "INSERT_INVALID_TRACKING_STOP",
                "non-finite tracking input lacked a structured stop reason");

  pre_config.distance_m = 0.30;
  check.require(!fr10_motion::PreInsertPlanner::plan(socket, pre_config).success,
                "INSERT_DISTANCE_REJECT",
                "out-of-range pre-insertion distance was accepted");
  return check.result();
}
