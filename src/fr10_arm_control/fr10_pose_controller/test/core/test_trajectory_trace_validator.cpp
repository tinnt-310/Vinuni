#include "fr10_motion_core/trajectory_trace_validator.hpp"
#include "test_support.hpp"

#include <limits>

namespace {
class FakeProvider final : public fr10_motion::IKProvider {
public:
  fr10_motion::IKResult solve(const fr10_motion::Pose &,
                              const fr10_motion::KinematicState &) override {
    return {};
  }

  fr10_motion::IKResult validate(
      const fr10_motion::KinematicState &target,
      const fr10_motion::KinematicState &) override {
    fr10_motion::IKResult result;
    result.success = target.position_rad[0] < 0.5;
    result.message = result.success ? "" : "blocked joint sample";
    return result;
  }

  bool forward(const fr10_motion::KinematicState &state,
               fr10_motion::Pose &pose,
               std::string * = nullptr) const override {
    pose.position_m = {state.position_rad[0], 0.0, 0.0};
    pose.orientation_quat = {0.0, 0.0, 0.0, 1.0};
    return true;
  }
};
} // namespace

int main() {
  fr10_motion_test::Check check;
  FakeProvider provider;
  fr10_motion::KinematicState seed;
  fr10_motion::CartesianJointTrajectory trajectory;
  fr10_motion::CartesianTrajectoryPoint first;
  first.desired_translation_tangent = {1.0, 0.0, 0.0};
  trajectory.points.push_back(first);
  auto second = first;
  second.joint_state.position_rad[0] = 0.1;
  second.desired_pose.position_m[0] = 0.1;
  trajectory.points.push_back(second);
  const fr10_motion::TrajectoryTraceLimits strict{true, 0.01, 0.01, 0.01};

  const auto valid = fr10_motion::TrajectoryTraceValidator::validate(
      trajectory, seed, provider, strict);
  check.require(valid.success && valid.max_position_error_m < 1e-12,
                "valid", "matching timed FK trace must pass");

  trajectory.points[1].desired_pose.position_m[0] = 0.2;
  const auto strict_error = fr10_motion::TrajectoryTraceValidator::validate(
      trajectory, seed, provider, strict);
  check.require(!strict_error.success && strict_error.failed_index == 1 &&
                    strict_error.max_position_error_m > 0.09,
                "strict", "insertion must reject excessive TCP error");
  auto free_space = strict;
  free_space.strict_cartesian = false;
  const auto diagnostic_only = fr10_motion::TrajectoryTraceValidator::validate(
      trajectory, seed, provider, free_space);
  check.require(diagnostic_only.success &&
                    diagnostic_only.max_position_error_m > 0.09,
                "free_space", "joint-space routes retain Cartesian telemetry");

  trajectory.points[1].desired_pose.position_m = {0.1, 0.1, 0.0};
  const auto off_axis = fr10_motion::TrajectoryTraceValidator::validate(
      trajectory, seed, provider, strict);
  check.require(!off_axis.success && off_axis.max_path_deviation_m > 0.09,
                "off_axis", "insertion rejects perpendicular deviation");

  trajectory.points[1].desired_pose.position_m[0] =
      std::numeric_limits<double>::quiet_NaN();
  const auto nonfinite = fr10_motion::TrajectoryTraceValidator::validate(
      trajectory, seed, provider, free_space);
  check.require(!nonfinite.success, "nonfinite",
                "even free-space telemetry must reject nonfinite errors");

  trajectory.points[1] = second;
  const auto canceled = fr10_motion::TrajectoryTraceValidator::validate(
      trajectory, seed, provider, strict, [] { return true; });
  check.require(!canceled.success && canceled.canceled,
                "canceled", "worker cancellation must not accept a plan");

  trajectory.points[1].joint_state.position_rad[0] = 0.6;
  const auto blocked = fr10_motion::TrajectoryTraceValidator::validate(
      trajectory, seed, provider, strict);
  check.require(!blocked.success && blocked.message == "blocked joint sample",
                "collision", "provider rejection propagates to planner");
  return check.result();
}
