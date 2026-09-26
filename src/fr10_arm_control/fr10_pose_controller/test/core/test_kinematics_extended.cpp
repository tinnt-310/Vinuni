#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <string>
#include <vector>

#include "extended_test_support.hpp"

namespace {

enum class Outcome { PASS, FAIL, SKIP };
using Case = std::function<Outcome()>;

double percentile(const std::vector<double> &sorted, double fraction) {
  if (sorted.empty()) {
    return 0.0;
  }
  const double scaled = fraction * static_cast<double>(sorted.size() - 1);
  const std::size_t lower = static_cast<std::size_t>(std::floor(scaled));
  const std::size_t upper = static_cast<std::size_t>(std::ceil(scaled));
  const double blend = scaled - static_cast<double>(lower);
  return sorted[lower] + blend * (sorted[upper] - sorted[lower]);
}

Outcome ikFkConsistency() {
  fr10_motion::Fr10AnalyticalIKProvider provider(
      fr10_motion_test::analyticalConfig());
  const auto nominal = fr10_motion_test::nominalState();
  constexpr std::size_t samples = 1000;
  std::size_t successes = 0;
  double max_position_error = 0.0;
  double max_orientation_error = 0.0;
  for (std::size_t sample = 0; sample < samples; ++sample) {
    auto state = nominal;
    const double phase = 0.017 * static_cast<double>(sample);
    for (std::size_t joint = 0; joint < fr10_motion::kJointCount; ++joint) {
      state.position_rad[joint] +=
          0.12 * std::sin(phase * static_cast<double>(joint + 1));
    }
    fr10_motion::Pose target;
    if (!provider.forward(state, target, nullptr)) {
      continue;
    }
    const auto solved = provider.solve(target, state);
    if (!solved.success) {
      continue;
    }
    fr10_motion::Pose reconstructed;
    if (!provider.forward(solved.state, reconstructed, nullptr)) {
      continue;
    }
    ++successes;
    max_position_error = std::max(
        max_position_error,
        fr10_motion_test::positionDistance(target, reconstructed));
    max_orientation_error = std::max(
        max_orientation_error,
        fr10_motion::math::quaternionAngularDistance(
            target.orientation_quat, reconstructed.orientation_quat));
  }
  const double ratio = static_cast<double>(successes) / samples;
  std::cout << "METRIC K_IK_FK_CONSISTENCY samples=" << samples
            << " success_ratio=" << ratio
            << " max_position_error_m=" << max_position_error
            << " max_orientation_error_rad=" << max_orientation_error
            << "\n";
  return ratio >= 0.999 && max_position_error < 1e-9 &&
                 max_orientation_error < 1e-6
             ? Outcome::PASS
             : Outcome::FAIL;
}

Outcome continuousBranch() {
  fr10_motion::Fr10AnalyticalIKProvider provider(
      fr10_motion_test::analyticalConfig());
  const auto seed = fr10_motion_test::nominalState();
  fr10_motion::Pose start;
  provider.forward(seed, start, nullptr);
  auto goal = start;
  goal.position_m[0] += 0.060;
  goal.position_m[1] -= 0.040;
  goal.position_m[2] += 0.030;
  fr10_motion::CartesianPathConfig path_config;
  path_config.max_translation_step_m = 0.002;
  std::vector<fr10_motion::Pose> path;
  std::string error;
  if (!fr10_motion::CartesianPath::linear(start, goal, path_config, path,
                                           &error)) {
    return Outcome::FAIL;
  }
  fr10_motion::ContinuousIKConfig config;
  config.max_joint_step_rad = 0.08;
  config.max_position_error_m = 1e-6;
  config.max_orientation_error_rad = 1e-6;
  const auto result =
      fr10_motion::ContinuousIK::solve(path, seed, provider, config);
  double max_step = 0.0;
  bool branch_preserved = result.success;
  for (std::size_t index = 1; index < result.joint_path.size(); ++index) {
    for (std::size_t joint = 0; joint < fr10_motion::kJointCount; ++joint) {
      const double step = std::abs(
          result.joint_path[index].position_rad[joint] -
          result.joint_path[index - 1].position_rad[joint]);
      max_step = std::max(max_step, step);
      branch_preserved = branch_preserved && step <= config.max_joint_step_rad;
    }
    branch_preserved = branch_preserved &&
        std::signbit(result.joint_path[index].position_rad[2]) ==
            std::signbit(result.joint_path[index - 1].position_rad[2]) &&
        std::signbit(result.joint_path[index].position_rad[4]) ==
            std::signbit(result.joint_path[index - 1].position_rad[4]);
  }
  std::cout << "METRIC K_CONTINUOUS_BRANCH waypoints=" << path.size()
            << " max_joint_step_rad=" << max_step
            << " max_condition=" << result.max_condition_number << "\n";
  return branch_preserved && result.joint_path.size() == path.size()
             ? Outcome::PASS
             : Outcome::FAIL;
}

Outcome wristBarrier(double offset_from_threshold, bool expected_success,
                     const std::string &id) {
  auto config = fr10_motion_test::analyticalConfig();
  config.max_jacobian_condition = 1e9;
  fr10_motion::Fr10AnalyticalIKProvider provider(config);
  auto state = fr10_motion_test::nominalState();
  state.position_rad[4] = -(config.wrist_min_abs_rad + offset_from_threshold);
  const auto result = provider.validate(state, state);
  std::cout << "METRIC " << id << " q5_rad=" << state.position_rad[4]
            << " threshold_rad=" << config.wrist_min_abs_rad
            << " accepted=" << (result.success ? 1 : 0)
            << " reason=" << result.message << "\n";
  const bool correct_reason = expected_success ||
      result.message.find("wrist singularity barrier") != std::string::npos;
  return result.success == expected_success && correct_reason
             ? Outcome::PASS
             : Outcome::FAIL;
}

Outcome elbowBarrier(double offset_from_threshold, bool expected_success,
                     const std::string &id) {
  auto config = fr10_motion_test::analyticalConfig();
  config.max_jacobian_condition = 1e9;
  config.max_wrist_reach_m = 10.0;
  config.min_wrist_radius_m = 0.0;
  fr10_motion::Fr10AnalyticalIKProvider provider(config);
  auto state = fr10_motion_test::nominalState();
  state.position_rad[2] = config.elbow_min_abs_rad + offset_from_threshold;
  const auto result = provider.validate(state, state);
  std::cout << "METRIC " << id << " q3_rad=" << state.position_rad[2]
            << " threshold_rad=" << config.elbow_min_abs_rad
            << " accepted=" << (result.success ? 1 : 0)
            << " reason=" << result.message << "\n";
  const bool correct_reason = expected_success ||
      result.message.find("elbow singularity barrier") != std::string::npos;
  return result.success == expected_success && correct_reason
             ? Outcome::PASS
             : Outcome::FAIL;
}

Outcome radialBarrier() {
  auto config = fr10_motion_test::analyticalConfig();
  config.max_jacobian_condition = 1e9;
  config.max_wrist_reach_m = 10.0;
  config.wrist_min_abs_rad = 0.0;
  config.elbow_min_abs_rad = 0.0;
  fr10_motion::Fr10AnalyticalIKProvider provider(config);
  const auto nominal = fr10_motion_test::nominalState();
  const auto outside = provider.validate(nominal, nominal);
  bool found_inside = false;
  fr10_motion::KinematicState inside;
  std::string reason;
  double minimum_observed_radius = std::numeric_limits<double>::infinity();
  fr10_motion::KinematicState minimum_radius_state;
  uint32_t random_state = 20260829U;
  auto unit_random = [&random_state]() {
    random_state = random_state * 1664525U + 1013904223U;
    return static_cast<double>(random_state) /
           static_cast<double>(std::numeric_limits<uint32_t>::max());
  };
  for (std::size_t sample = 0; sample < 250000 && !found_inside; ++sample) {
    auto candidate = nominal;
    for (std::size_t joint = 1; joint <= 4; ++joint) {
      const double lower = config.limits.min_joint_position_rad[joint];
      const double upper = config.limits.max_joint_position_rad[joint];
      candidate.position_rad[joint] = lower + unit_random() * (upper - lower);
    }
    fr10_motion::Pose tip_pose;
    if (provider.forward(candidate, tip_pose, nullptr)) {
      const auto tool_offset = fr10_motion::math::rotateVector(
          tip_pose.orientation_quat,
          config.wrist3_to_tip.position_m);
      const double wrist_x = tip_pose.position_m[0] - tool_offset[0];
      const double wrist_y = tip_pose.position_m[1] - tool_offset[1];
      const double radius = std::hypot(wrist_x, wrist_y);
      if (radius < minimum_observed_radius) {
        minimum_observed_radius = radius;
        minimum_radius_state = candidate;
      }
    }
    const auto result = provider.validate(candidate, candidate);
    if (!result.success &&
        result.message.find("wrist-link barrier") != std::string::npos) {
      found_inside = true;
      inside = candidate;
      reason = result.message;
    }
  }
  std::cout << "METRIC K_RADIAL_BARRIER outside_accepted="
            << (outside.success ? 1 : 0)
            << " inside_found=" << (found_inside ? 1 : 0)
            << " inside_q2=" << inside.position_rad[1]
            << " inside_q3=" << inside.position_rad[2]
            << " min_observed_radius_m=" << minimum_observed_radius
            << " min_radius_q2=" << minimum_radius_state.position_rad[1]
            << " min_radius_q3=" << minimum_radius_state.position_rad[2]
            << " reason=" << reason << "\n";
  return outside.success && found_inside ? Outcome::PASS : Outcome::FAIL;
}

Outcome jacobianSigmaTrend() {
  auto config = fr10_motion_test::analyticalConfig();
  config.wrist_min_abs_rad = 0.0;
  config.elbow_min_abs_rad = 0.0;
  config.max_jacobian_condition = 1e9;
  fr10_motion::Fr10AnalyticalIKProvider provider(config);
  const auto normal = fr10_motion_test::nominalState();
  auto near_singular = normal;
  near_singular.position_rad[4] = 0.01;
  const auto normal_metrics =
      fr10_motion_test::finiteDifferenceJacobian(provider, normal);
  const auto near_metrics =
      fr10_motion_test::finiteDifferenceJacobian(provider, near_singular);
  std::cout << "METRIC K_JACOBIAN_SIGMA_TREND normal_sigma_min="
            << normal_metrics.sigma_min
            << " near_sigma_min=" << near_metrics.sigma_min
            << " normal_condition=" << normal_metrics.condition
            << " near_condition=" << near_metrics.condition << "\n";
  return normal_metrics.valid && near_metrics.valid &&
                 near_metrics.sigma_min < normal_metrics.sigma_min &&
                 near_metrics.condition > normal_metrics.condition
             ? Outcome::PASS
             : Outcome::FAIL;
}

Outcome conditionLimit() {
  auto config = fr10_motion_test::analyticalConfig();
  config.wrist_min_abs_rad = 0.0;
  config.elbow_min_abs_rad = 0.0;
  fr10_motion::Fr10AnalyticalIKProvider provider(config);
  const auto normal = fr10_motion_test::nominalState();
  auto near_singular = normal;
  near_singular.position_rad[4] = 0.01;
  const auto normal_result = provider.validate(normal, normal);
  const auto near_result = provider.validate(near_singular, near_singular);
  std::cout << "METRIC K_JACOBIAN_CONDITION normal_condition="
            << normal_result.condition_number
            << " normal_accepted=" << (normal_result.success ? 1 : 0)
            << " near_condition=" << near_result.condition_number
            << " near_accepted=" << (near_result.success ? 1 : 0)
            << " reason=" << near_result.message << "\n";
  return normal_result.success && !near_result.success &&
                 (near_result.message.find("condition-number") !=
                      std::string::npos ||
                  near_result.message.find("singular") != std::string::npos)
             ? Outcome::PASS
             : Outcome::FAIL;
}

Outcome unreachablePose() {
  fr10_motion::Fr10AnalyticalIKProvider provider(
      fr10_motion_test::analyticalConfig());
  fr10_motion::Pose target;
  target.position_m = {5.0, 0.0, 0.0};
  const bool early_rejected = !provider.workspaceReachable(target);
  const auto result = provider.solve(target, fr10_motion_test::nominalState());
  std::cout << "METRIC K_UNREACHABLE early_rejected="
            << (early_rejected ? 1 : 0)
            << " solve_success=" << (result.success ? 1 : 0)
            << " reason=" << result.message << "\n";
  return early_rejected && !result.success ? Outcome::PASS : Outcome::FAIL;
}

Outcome jointLimitBoundary() {
  auto config = fr10_motion_test::analyticalConfig();
  fr10_motion::Fr10AnalyticalIKProvider provider(config);
  auto inside = fr10_motion_test::nominalState();
  inside.position_rad[0] = config.limits.max_joint_position_rad[0] - 1e-9;
  auto exact = inside;
  exact.position_rad[0] = config.limits.max_joint_position_rad[0];
  auto outside = exact;
  outside.position_rad[0] += 1e-9;
  const auto inside_result = provider.validate(inside, inside);
  const auto exact_result = provider.validate(exact, exact);
  const auto outside_result = provider.validate(outside, outside);
  std::cout << "METRIC K_JOINT_LIMIT inside="
            << (inside_result.success ? 1 : 0)
            << " exact=" << (exact_result.success ? 1 : 0)
            << " outside=" << (outside_result.success ? 1 : 0)
            << " outside_reason=" << outside_result.message << "\n";
  return inside_result.success && exact_result.success &&
                 !outside_result.success &&
                 outside_result.message.find("joint limit") !=
                     std::string::npos
             ? Outcome::PASS
             : Outcome::FAIL;
}

Outcome analyticalBenchmark() {
  fr10_motion::Fr10AnalyticalIKProvider provider(
      fr10_motion_test::analyticalConfig());
  constexpr std::size_t sample_count = 10000;
  std::vector<fr10_motion::Pose> targets(sample_count);
  std::vector<fr10_motion::KinematicState> seeds(sample_count);
  const auto nominal = fr10_motion_test::nominalState();
  for (std::size_t sample = 0; sample < sample_count; ++sample) {
    seeds[sample] = nominal;
    const double phase = 0.009 * static_cast<double>(sample);
    for (std::size_t joint = 0; joint < fr10_motion::kJointCount; ++joint) {
      seeds[sample].position_rad[joint] +=
          0.10 * std::sin(phase * static_cast<double>(joint + 1));
    }
    provider.forward(seeds[sample], targets[sample], nullptr);
  }
  std::vector<double> durations_us;
  durations_us.reserve(sample_count);
  std::size_t successes = 0;
  double max_position_error = 0.0;
  double max_orientation_error = 0.0;
  for (std::size_t sample = 0; sample < sample_count; ++sample) {
    const auto started = std::chrono::steady_clock::now();
    const auto result = provider.solve(targets[sample], seeds[sample]);
    const auto finished = std::chrono::steady_clock::now();
    durations_us.push_back(
        std::chrono::duration<double, std::micro>(finished - started).count());
    if (!result.success) {
      continue;
    }
    ++successes;
    fr10_motion::Pose reconstructed;
    provider.forward(result.state, reconstructed, nullptr);
    max_position_error = std::max(
        max_position_error,
        fr10_motion_test::positionDistance(targets[sample], reconstructed));
    max_orientation_error = std::max(
        max_orientation_error,
        fr10_motion::math::quaternionAngularDistance(
            targets[sample].orientation_quat,
            reconstructed.orientation_quat));
  }
  std::sort(durations_us.begin(), durations_us.end());
  const double mean = std::accumulate(durations_us.begin(),
                                      durations_us.end(), 0.0) /
                      durations_us.size();
  double squared_sum = 0.0;
  for (double duration : durations_us) {
    const double delta = duration - mean;
    squared_sum += delta * delta;
  }
  const double stddev = std::sqrt(squared_sum / durations_us.size());
  const double success_ratio =
      static_cast<double>(successes) / sample_count;
  std::cout << std::setprecision(9)
            << "METRIC K_ANALYTICAL_BENCHMARK seed=20260829 samples="
            << sample_count << " success_ratio=" << success_ratio
            << " mean_us=" << mean
            << " median_us=" << percentile(durations_us, 0.50)
            << " stddev_us=" << stddev
            << " p95_us=" << percentile(durations_us, 0.95)
            << " p99_us=" << percentile(durations_us, 0.99)
            << " max_us=" << durations_us.back()
            << " max_position_error_m=" << max_position_error
            << " max_orientation_error_rad=" << max_orientation_error
            << "\n";
  return success_ratio >= 0.999 && percentile(durations_us, 0.99) < 85.0 &&
                 max_position_error < 1e-9 && max_orientation_error < 1e-6
             ? Outcome::PASS
             : Outcome::FAIL;
}

Outcome numericalBenchmarkInactive() {
  std::cout
      << "SKIP K_MOVEIT_NUMERICAL_NOT_ACTIVE reason=the production path uses "
         "analytical IK; MoveIt is retained as independent FK validation and "
         "has no deterministic standalone numerical-IK benchmark fixture\n";
  return Outcome::SKIP;
}

std::map<std::string, Case> cases() {
  return {
      {"K_IK_FK_CONSISTENCY", ikFkConsistency},
      {"K_CONTINUOUS_BRANCH", continuousBranch},
      {"K_WRIST_OUTSIDE", [] {
         return wristBarrier(0.02, true, "K_WRIST_OUTSIDE");
       }},
      {"K_WRIST_BOUNDARY", [] {
         return wristBarrier(0.0, true, "K_WRIST_BOUNDARY");
       }},
      {"K_WRIST_INSIDE", [] {
         return wristBarrier(-0.02, false, "K_WRIST_INSIDE");
       }},
      {"K_ELBOW_OUTSIDE", [] {
         return elbowBarrier(0.02, true, "K_ELBOW_OUTSIDE");
       }},
      {"K_ELBOW_INSIDE", [] {
         return elbowBarrier(-0.02, false, "K_ELBOW_INSIDE");
       }},
      {"K_RADIAL_BARRIER", radialBarrier},
      {"K_JACOBIAN_SIGMA_TREND", jacobianSigmaTrend},
      {"K_JACOBIAN_CONDITION", conditionLimit},
      {"K_UNREACHABLE", unreachablePose},
      {"K_JOINT_LIMIT", jointLimitBoundary},
      {"K_ANALYTICAL_BENCHMARK", analyticalBenchmark},
      {"K_MOVEIT_NUMERICAL_NOT_ACTIVE", numericalBenchmarkInactive},
  };
}

} // namespace

int main(int argc, char **argv) {
  const auto registry = cases();
  if (argc != 3 || std::string(argv[1]) != "--case") {
    std::cerr << "Usage: " << argv[0] << " --case CASE_ID\n";
    return 2;
  }
  const auto found = registry.find(argv[2]);
  if (found == registry.end()) {
    std::cerr << "Unknown case: " << argv[2] << "\n";
    return 2;
  }
  const Outcome outcome = found->second();
  if (outcome == Outcome::SKIP) {
    return 77;
  }
  std::cout << (outcome == Outcome::PASS ? "PASS " : "FAIL ")
            << found->first << "\n";
  return outcome == Outcome::PASS ? 0 : 1;
}
