#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "extended_test_support.hpp"
#include "fr10_pose_controller/linear_insert_planner.hpp"
#include "fr10_pose_controller/pre_insert_planner.hpp"

namespace {

using Case = std::function<bool()>;
constexpr double kEngineeringPathToleranceM = 0.002;
constexpr double kNumericalTolerance = 1e-10;

bool pathAcceptable(const std::string &id,
                    const fr10_motion_test::ExecutedPathMetrics &metrics) {
  fr10_motion_test::printPathMetrics(id, metrics);
  return metrics.success &&
         metrics.max_perpendicular_error_m <= kEngineeringPathToleranceM &&
         metrics.max_cartesian_error_m <= kEngineeringPathToleranceM &&
         metrics.endpoint_error_m <= 1e-5 &&
         metrics.max_orientation_error_rad <= 0.002 &&
         metrics.min_path_parameter >= -1e-6 &&
         metrics.max_path_parameter <= 1.0 + 1e-6;
}

bool baseAxisCase(std::size_t axis, double sign, const std::string &id) {
  fr10_motion::Fr10AnalyticalIKProvider provider(
      fr10_motion_test::analyticalConfig());
  const auto seed = fr10_motion_test::nominalState();
  fr10_motion::Pose start;
  if (!provider.forward(seed, start, nullptr)) {
    return false;
  }
  auto goal = start;
  goal.position_m[axis] += sign * 0.050;
  fr10_motion::CartesianPathConfig path_config;
  path_config.max_translation_step_m = 0.002;
  path_config.max_rotation_step_rad = 0.01;
  return pathAcceptable(
      id, fr10_motion_test::executeAnalyticalPath(start, goal, seed,
                                                  path_config));
}

bool rotatedSocketCase(const std::array<double, 3> &local_axis,
                       const std::string &id) {
  fr10_motion::Fr10AnalyticalIKProvider provider(
      fr10_motion_test::analyticalConfig());
  auto nominal = fr10_motion_test::nominalState();
  fr10_motion::Pose socket;
  if (!provider.forward(nominal, socket, nullptr)) {
    return false;
  }
  socket.orientation_quat = fr10_motion_test::rpyQuaternion(
      15.0 * fr10_motion::kDegToRad, 25.0 * fr10_motion::kDegToRad,
      -40.0 * fr10_motion::kDegToRad);
  const auto socket_solution = provider.solve(socket, nominal);
  if (!socket_solution.success) {
    std::cerr << "DETAIL " << id << " socket_ik="
              << socket_solution.message << "\n";
    return false;
  }
  fr10_motion::PreInsertPlannerConfig pre_config;
  pre_config.local_axis = local_axis;
  pre_config.distance_m = 0.050;
  const auto pre = fr10_motion::PreInsertPlanner::plan(socket, pre_config);
  if (!pre.success) {
    return false;
  }
  const auto expected_axis = pre.base_axis;
  std::array<double, 3> measured_axis{};
  for (std::size_t axis = 0; axis < 3; ++axis) {
    measured_axis[axis] =
        (socket.position_m[axis] - pre.pre_insert_pose.position_m[axis]) /
        pre.distance_m;
  }
  double axis_error_squared = 0.0;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const double delta = measured_axis[axis] - expected_axis[axis];
    axis_error_squared += delta * delta;
  }
  fr10_motion::CartesianPathConfig path_config;
  path_config.max_translation_step_m = 0.002;
  path_config.max_rotation_step_rad = 0.01;
  const auto pre_solution = provider.solve(pre.pre_insert_pose, socket_solution.state);
  if (!pre_solution.success) {
    std::cerr << "DETAIL " << id << " pre_insert_ik="
              << pre_solution.message << "\n";
    return false;
  }
  const auto metrics = fr10_motion_test::executeAnalyticalPath(
      pre.pre_insert_pose, socket, pre_solution.state, path_config);
  std::cout << "METRIC " << id
            << " transformed_axis_error=" << std::sqrt(axis_error_squared)
            << " base_axis=" << expected_axis[0] << "," << expected_axis[1]
            << "," << expected_axis[2] << "\n";
  return std::sqrt(axis_error_squared) < 1e-10 &&
         pathAcceptable(id, metrics);
}

bool distanceMatrix() {
  fr10_motion::Pose socket;
  socket.position_m = {0.40, -0.20, 0.55};
  socket.orientation_quat = fr10_motion_test::rpyQuaternion(
      0.20, -0.35, 0.60);
  bool ok = true;
  for (double distance : {0.010, 0.015, 0.020, 0.030, 0.050, 0.070}) {
    fr10_motion::PreInsertPlannerConfig config;
    config.distance_m = distance;
    config.local_axis = {0.0, 0.0, 1.0};
    const auto plan = fr10_motion::PreInsertPlanner::plan(socket, config);
    const double measured = fr10_motion_test::positionDistance(
        socket, plan.pre_insert_pose);
    const double orientation =
        fr10_motion::math::quaternionAngularDistance(
            socket.orientation_quat, plan.pre_insert_pose.orientation_quat);
    ok = ok && plan.success && std::abs(measured - distance) < 1e-12 &&
         orientation < 1e-12;
    std::cout << "METRIC PI_PREINSERT_DISTANCE_MATRIX configured_m="
              << distance << " measured_m=" << measured
              << " orientation_error_rad=" << orientation << "\n";
  }
  return ok;
}

bool orientationPreservation() {
  fr10_motion::Fr10AnalyticalIKProvider provider(
      fr10_motion_test::analyticalConfig());
  const auto seed = fr10_motion_test::nominalState();
  fr10_motion::Pose start;
  provider.forward(seed, start, nullptr);
  auto goal = start;
  goal.position_m[1] += 0.050;
  const auto metrics =
      fr10_motion_test::executeAnalyticalPath(start, goal, seed);
  fr10_motion_test::printPathMetrics("PI_ORIENTATION_PRESERVATION", metrics);
  return metrics.success && metrics.max_orientation_error_rad <= 0.002;
}

bool waypointDensity() {
  fr10_motion::Pose start;
  start.position_m = {0.25, -0.20, 0.50};
  auto goal = start;
  goal.position_m[0] += 0.047;
  goal.orientation_quat = fr10_motion_test::axisAngle(1, 0.23);
  fr10_motion::CartesianPathConfig config;
  config.max_translation_step_m = 0.003;
  config.max_rotation_step_rad = 0.017;
  std::vector<fr10_motion::Pose> path;
  std::string error;
  bool ok = fr10_motion::CartesianPath::linear(start, goal, config, path,
                                                &error);
  double max_translation = 0.0;
  double max_rotation = 0.0;
  for (std::size_t index = 1; index < path.size(); ++index) {
    max_translation = std::max(
        max_translation,
        fr10_motion_test::positionDistance(path[index - 1], path[index]));
    max_rotation = std::max(
        max_rotation,
        fr10_motion::math::quaternionAngularDistance(
            path[index - 1].orientation_quat,
            path[index].orientation_quat));
  }
  std::cout << "METRIC PI_WAYPOINT_DENSITY count=" << path.size()
            << " max_translation_step_m=" << max_translation
            << " max_rotation_step_rad=" << max_rotation << "\n";
  return ok && path.size() > 2 &&
         max_translation <= config.max_translation_step_m + 1e-12 &&
         max_rotation <= config.max_rotation_step_rad + 1e-12;
}

bool noJointShortcut() {
  fr10_motion::Fr10AnalyticalIKProvider provider(
      fr10_motion_test::analyticalConfig());
  const auto seed = fr10_motion_test::nominalState();
  fr10_motion::Pose start;
  provider.forward(seed, start, nullptr);

  double worst_naive_deviation = 0.0;
  fr10_motion_test::ExecutedPathMetrics selected;
  for (const std::array<double, 3> delta :
       {std::array<double, 3>{0.10, 0.00, 0.00},
        std::array<double, 3>{0.00, 0.10, 0.00},
        std::array<double, 3>{0.00, 0.00, 0.10},
        std::array<double, 3>{0.06, -0.06, 0.04}}) {
    auto goal = start;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      goal.position_m[axis] += delta[axis];
    }
    const auto metrics =
        fr10_motion_test::executeAnalyticalPath(start, goal, seed);
    if (!metrics.success || metrics.ik.joint_path.size() < 2) {
      continue;
    }
    const auto &first = metrics.ik.joint_path.front();
    const auto &last = metrics.ik.joint_path.back();
    double naive_deviation = 0.0;
    for (int sample = 0; sample <= 100; ++sample) {
      const double fraction = static_cast<double>(sample) / 100.0;
      fr10_motion::KinematicState state;
      for (std::size_t joint = 0; joint < fr10_motion::kJointCount; ++joint) {
        state.position_rad[joint] =
            first.position_rad[joint] +
            fraction * (last.position_rad[joint] - first.position_rad[joint]);
      }
      fr10_motion::Pose pose;
      provider.forward(state, pose, nullptr);
      naive_deviation = std::max(
          naive_deviation,
          fr10_motion::CartesianPath::translationDeviation(start, goal, pose));
    }
    if (naive_deviation > worst_naive_deviation) {
      worst_naive_deviation = naive_deviation;
      selected = metrics;
    }
  }
  std::cout << "METRIC PI_NO_JOINT_SHORTCUT naive_e_perp_max_m="
            << worst_naive_deviation
            << " constrained_e_perp_max_m="
            << selected.max_perpendicular_error_m
            << " constrained_samples=" << selected.timed_samples << "\n";
  return selected.success && selected.timed_samples > 2 &&
         selected.max_perpendicular_error_m <= kEngineeringPathToleranceM &&
         worst_naive_deviation > 1e-4 &&
         selected.max_perpendicular_error_m < worst_naive_deviation;
}

std::map<std::string, Case> cases() {
  return {
      {"PI_BASE_POS_X", [] { return baseAxisCase(0, 1.0, "PI_BASE_POS_X"); }},
      {"PI_BASE_NEG_X", [] { return baseAxisCase(0, -1.0, "PI_BASE_NEG_X"); }},
      {"PI_BASE_POS_Y", [] { return baseAxisCase(1, 1.0, "PI_BASE_POS_Y"); }},
      {"PI_BASE_NEG_Y", [] { return baseAxisCase(1, -1.0, "PI_BASE_NEG_Y"); }},
      {"PI_BASE_POS_Z", [] { return baseAxisCase(2, 1.0, "PI_BASE_POS_Z"); }},
      {"PI_BASE_NEG_Z", [] { return baseAxisCase(2, -1.0, "PI_BASE_NEG_Z"); }},
      {"PI_ROTATED_LOCAL_Z", [] {
         return rotatedSocketCase({0.0, 0.0, 1.0}, "PI_ROTATED_LOCAL_Z");
       }},
      {"PI_ROTATED_LOCAL_X", [] {
         return rotatedSocketCase({1.0, 0.0, 0.0}, "PI_ROTATED_LOCAL_X");
       }},
      {"PI_PREINSERT_DISTANCE_MATRIX", distanceMatrix},
      {"PI_ORIENTATION_PRESERVATION", orientationPreservation},
      {"PI_WAYPOINT_DENSITY", waypointDensity},
      {"PI_NO_JOINT_SHORTCUT", noJointShortcut},
  };
}

} // namespace

int main(int argc, char **argv) {
  const auto registry = cases();
  if (argc == 3 && std::string(argv[1]) == "--case") {
    const auto found = registry.find(argv[2]);
    if (found == registry.end()) {
      std::cerr << "Unknown case: " << argv[2] << "\n";
      return 2;
    }
    const bool passed = found->second();
    std::cout << (passed ? "PASS " : "FAIL ") << found->first << "\n";
    return passed ? 0 : 1;
  }
  bool all_passed = true;
  for (const auto &[id, run] : registry) {
    const bool passed = run();
    std::cout << (passed ? "PASS " : "FAIL ") << id << "\n";
    all_passed = all_passed && passed;
  }
  return all_passed ? 0 : 1;
}
