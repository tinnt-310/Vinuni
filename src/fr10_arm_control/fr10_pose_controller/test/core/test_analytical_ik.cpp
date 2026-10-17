#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>

#include "fr10_motion_core/fr10_analytical_ik.hpp"
#include "fr10_motion_core/cartesian_route_planner.hpp"
#include "test_support.hpp"

namespace {

double positionError(const fr10_motion::Pose &first,
                     const fr10_motion::Pose &second) {
  double squared = 0.0;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const double delta = first.position_m[axis] - second.position_m[axis];
    squared += delta * delta;
  }
  return std::sqrt(squared);
}

fr10_motion::Fr10AnalyticalIKConfig configuration() {
  fr10_motion::Fr10AnalyticalIKConfig config;
  config.limits = fr10_motion_test::limits();
  config.limits.min_joint_position_rad.fill(-2.0 * std::acos(-1.0));
  config.limits.max_joint_position_rad.fill(2.0 * std::acos(-1.0));
  config.wrist_min_abs_rad = 0.05;
  config.elbow_min_abs_rad = 0.05;
  config.max_wrist_reach_m = 10.0;
  config.min_wrist_radius_m = 0.0;
  config.max_jacobian_condition = 1e8;
  config.min_target_reach_m = 0.01;
  config.max_target_reach_m = 3.0;
  return config;
}

} // namespace

int main() {
  fr10_motion_test::Check check;
  fr10_motion::Fr10AnalyticalIKProvider provider(configuration());
  fr10_motion::KinematicState seed;
  seed.position_rad =
      {-1.23224948, -1.94007846, 1.85129480,
       -2.38987928, -0.39137653, -0.66787197};
  fr10_motion::Pose target;
  check.require(provider.forward(seed, target), "ANALYTICAL_FK",
                "closed-form FK rejected a finite state");

  fr10_motion::Fr10AnalyticalIKSolutions branches;
  std::string error;
  check.require(provider.solveAll(target, seed, branches, &error),
                "ANALYTICAL_BRANCH_ENUMERATION", error);
  check.require(branches.count == 8, "ANALYTICAL_EIGHT_BRANCHES",
                "the nonsingular reference target did not enumerate 8 branches");
  double maximum_position_error = 0.0;
  double maximum_orientation_error = 0.0;
  for (std::size_t index = 0; index < branches.count; ++index) {
    fr10_motion::Pose reconstructed;
    const bool forward_ok = provider.forward(branches.states[index],
                                              reconstructed);
    maximum_position_error =
        std::max(maximum_position_error,
                 forward_ok ? positionError(target, reconstructed)
                            : std::numeric_limits<double>::infinity());
    maximum_orientation_error = std::max(
        maximum_orientation_error,
        forward_ok ? fr10_motion::math::quaternionAngularDistance(
                         target.orientation_quat,
                         reconstructed.orientation_quat)
                   : std::numeric_limits<double>::infinity());
  }
  check.require(maximum_position_error < 1e-9 &&
                    maximum_orientation_error < 1e-7,
                "ANALYTICAL_IK_FK_CLOSURE",
                "one or more analytical branches failed FK closure");

  const auto selected = provider.solve(target, seed);
  double maximum_seed_difference = 0.0;
  for (std::size_t joint = 0; joint < fr10_motion::kJointCount; ++joint) {
    maximum_seed_difference = std::max(
        maximum_seed_difference,
        std::abs(selected.state.position_rad[joint] -
                 seed.position_rad[joint]));
  }
  check.require(selected.success && maximum_seed_difference < 1e-7,
                "ANALYTICAL_SEEDED_CONTINUITY",
                "weighted branch selection did not retain the seed branch");

  fr10_motion::Pose unreachable = target;
  unreachable.position_m = {4.0, 0.0, 0.0};
  check.require(!provider.workspaceReachable(unreachable) &&
                    !provider.solve(unreachable, seed).success,
                "ANALYTICAL_WORKSPACE_REJECT",
                "early spherical bounds accepted a 4 m target");

  constexpr std::size_t kBenchmarkIterations = 50000;
  for (std::size_t index = 0; index < 1000; ++index) {
    (void)provider.solve(target, seed);
  }
  const auto benchmark_start = std::chrono::steady_clock::now();
  std::size_t solved = 0;
  for (std::size_t index = 0; index < kBenchmarkIterations; ++index) {
    solved += provider.solve(target, seed).success ? 1U : 0U;
  }
  const double benchmark_us =
      std::chrono::duration<double, std::micro>(
          std::chrono::steady_clock::now() - benchmark_start)
          .count() /
      static_cast<double>(kBenchmarkIterations);
  check.require(solved == kBenchmarkIterations,
                "ANALYTICAL_BENCHMARK_SOLVES",
                "benchmark contained failed IK solves");
  check.require(benchmark_us < 75.0, "ANALYTICAL_LATENCY_TARGET",
                "mean closed-form solve latency exceeded 15 us");
  const auto stats = provider.statistics();
  std::cout << "METRIC analytical_branches=" << branches.count
            << " max_fk_position_error_m=" << maximum_position_error
            << " max_fk_orientation_error_rad=" << maximum_orientation_error
            << " mean_external_solve_us=" << benchmark_us
            << " mean_internal_solve_us=" << stats.mean_solve_time_us
            << " max_internal_solve_us=" << stats.max_solve_time_us << "\n";

  fr10_motion::Fr10AnalyticalIKConfig deployed_config;
  deployed_config.limits = fr10_motion_test::limits();
  deployed_config.limits.min_joint_position_rad =
      {-3.0543, -4.6251, -2.8274, -4.6251, -3.0543, -3.0543};
  deployed_config.limits.max_joint_position_rad =
      {3.0543, 1.4835, 2.8274, 1.4835, 3.0543, 3.0543};
  deployed_config.wrist_min_abs_rad = 8.0 * std::acos(-1.0) / 180.0;
  deployed_config.elbow_min_abs_rad = 15.0 * std::acos(-1.0) / 180.0;
  deployed_config.max_wrist_reach_m = 1.30;
  deployed_config.min_wrist_radius_m = 0.200;
  deployed_config.max_jacobian_condition = 30.0;
  deployed_config.min_target_reach_m = 0.060;
  deployed_config.max_target_reach_m = 1.750;
  fr10_motion::Fr10AnalyticalIKProvider deployed_provider(deployed_config);
  fr10_motion::KinematicState simmachine_seed;
  simmachine_seed.position_rad =
      {2.505024715079205, -1.3790183682727644, 1.2897626574968133,
       -1.4815368875934192, -1.5707963267948966, 2.417758785111707};
  fr10_motion::Pose simmachine_start;
  const bool simmachine_fk =
      deployed_provider.forward(simmachine_seed, simmachine_start, nullptr);
  fr10_motion::Pose horizontal_goal = simmachine_start;
  const double half_sqrt = std::sqrt(0.5);
  // tool0 +Z -> base +Y (horizontal insertion), tool0 +Y -> base +Z
  // (connector top face remains parallel to the ground).
  horizontal_goal.orientation_quat = {0.0, half_sqrt, half_sqrt, 0.0};
  fr10_motion::CartesianRoutePlannerConfig deployed_route_config;
  deployed_route_config.path.max_translation_step_m = 0.005;
  deployed_route_config.path.max_rotation_step_rad = 0.02;
  deployed_route_config.path.max_waypoints = 2000;
  deployed_route_config.continuous_ik.max_joint_step_rad = 0.35;
  deployed_route_config.continuous_ik.max_position_error_m = 0.001;
  deployed_route_config.continuous_ik.max_orientation_error_rad = 0.01;
  deployed_route_config.detour_clearances_m = {0.05, 0.10, 0.15};
  deployed_route_config.limits = deployed_config.limits;
  const auto horizontal_route =
      simmachine_fk
          ? fr10_motion::CartesianRoutePlanner::plan(
                simmachine_start, horizontal_goal, simmachine_seed,
                deployed_provider, deployed_route_config)
          : fr10_motion::CartesianRoutePlan{};
  const double requested_rotation =
      fr10_motion::math::quaternionAngularDistance(
          simmachine_start.orientation_quat,
          horizontal_goal.orientation_quat);
  check.require(simmachine_fk && horizontal_route.success &&
                    !horizontal_route.used_detour &&
                    requested_rotation > 2.0 &&
                    horizontal_route.continuous_ik.max_condition_number <
                        deployed_config.max_jacobian_condition,
                "ANALYTICAL_SIMMACHINE_LARGE_ROTATION_ROUTE",
                horizontal_route.message);
  std::cout << "METRIC simmachine_large_rotation_deg="
            << requested_rotation * 180.0 / std::acos(-1.0)
            << " route="
            << (horizontal_route.used_detour ? "detour" : "direct")
            << " waypoints=" << horizontal_route.cartesian_path.size()
            << " condition="
            << horizontal_route.continuous_ik.max_condition_number << "\n";
  return check.result();
}
