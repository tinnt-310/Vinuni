#include <cmath>
#include <iostream>
#include <limits>

#include "fr10_motion_core/linear_insert_planner.hpp"
#include "fr10_motion_core/pre_insert_planner.hpp"
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

  const auto forward_axis = fr10_motion::LinearInsertPlanner::validateAxis(
      pre.pre_insert_pose, socket, pre.pre_insert_pose, socket, linear_config);
  check.require(forward_axis.success && forward_axis.straight_segment.success &&
                    forward_axis.axis_deviation_m < 1e-12 &&
                    forward_axis.forward_projection > 0.0,
                "INSERT_AXIS_FORWARD", "aligned forward insertion was rejected");
  const auto retract_axis = fr10_motion::LinearInsertPlanner::validateAxis(
      socket, pre.pre_insert_pose, socket, pre.pre_insert_pose, linear_config);
  check.require(retract_axis.success &&
                    retract_axis.axis_deviation_m < 1e-12 &&
                    retract_axis.forward_projection > 0.0,
                "RETRACT_AXIS_FORWARD", "aligned retraction was rejected");
  auto offset_start = pre.pre_insert_pose;
  offset_start.position_m[0] += 0.002;
  const auto lateral_axis = fr10_motion::LinearInsertPlanner::validateAxis(
      offset_start, socket, pre.pre_insert_pose, socket, linear_config);
  check.require(!lateral_axis.success &&
                    lateral_axis.axis_deviation_m >
                        linear_config.perpendicular_tolerance_m,
                "INSERT_AXIS_LATERAL", "off-axis insertion was accepted");
  auto overshot = socket;
  overshot.position_m[1] += 0.01;
  const auto reverse_axis = fr10_motion::LinearInsertPlanner::validateAxis(
      overshot, socket, pre.pre_insert_pose, socket, linear_config);
  check.require(!reverse_axis.success &&
                    reverse_axis.forward_projection < -1e-9,
                "INSERT_AXIS_REVERSE", "backward insertion was accepted");
  auto invalid_start = pre.pre_insert_pose;
  invalid_start.position_m[0] = std::numeric_limits<double>::quiet_NaN();
  const auto invalid_axis = fr10_motion::LinearInsertPlanner::validateAxis(
      invalid_start, socket, pre.pre_insert_pose, socket, linear_config);
  check.require(!invalid_axis.success,
                "INSERT_AXIS_NONFINITE", "non-finite insertion was accepted");

  const auto lightweight_forward = fr10_motion::LinearInsertPlanner::checkAxis(
      pre.pre_insert_pose, socket, pre.pre_insert_pose, socket, linear_config);
  const auto lightweight_lateral = fr10_motion::LinearInsertPlanner::checkAxis(
      offset_start, socket, pre.pre_insert_pose, socket, linear_config);
  const auto lightweight_reverse = fr10_motion::LinearInsertPlanner::checkAxis(
      overshot, socket, pre.pre_insert_pose, socket, linear_config);
  const auto lightweight_invalid = fr10_motion::LinearInsertPlanner::checkAxis(
      invalid_start, socket, pre.pre_insert_pose, socket, linear_config);
  check.require(lightweight_forward.success == forward_axis.success &&
                    lightweight_lateral.success == lateral_axis.success &&
                    lightweight_reverse.success == reverse_axis.success &&
                    lightweight_invalid.success == invalid_axis.success &&
                    lightweight_lateral.message.find("perpendicular") !=
                        std::string::npos &&
                    lightweight_reverse.message.find("reversed") !=
                        std::string::npos &&
                    std::abs(lightweight_forward.distance_m -
                             insertion.distance_m) < 1e-12,
                "INSERT_AXIS_LIGHTWEIGHT_PARITY",
                "forward=" + lightweight_forward.message +
                    " lateral=" + lightweight_lateral.message +
                    " reverse=" + lightweight_reverse.message +
                    " invalid=" + lightweight_invalid.message);
  auto capped_config = linear_config;
  capped_config.path.max_waypoints = 2;
  const auto capped_check = fr10_motion::LinearInsertPlanner::checkAxis(
      pre.pre_insert_pose, socket, pre.pre_insert_pose, socket, capped_config);
  const auto capped_plan = fr10_motion::LinearInsertPlanner::plan(
      pre.pre_insert_pose, socket, capped_config);
  check.require(!capped_check.success && !capped_plan.success &&
                    capped_check.message == capped_plan.message,
                "INSERT_AXIS_CAPACITY_PARITY",
                "lightweight check accepted an over-capacity path");

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
