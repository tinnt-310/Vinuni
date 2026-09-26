#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

#include "extended_test_support.hpp"

namespace {

std::string failureReason(const std::string &message, bool workspace_reachable) {
  if (!workspace_reachable || message.find("reach") != std::string::npos ||
      message.find("workspace") != std::string::npos) {
    return "WORKSPACE_LIMIT";
  }
  if (message.find("joint limit") != std::string::npos) {
    return "JOINT_LIMIT";
  }
  if (message.find("singular") != std::string::npos ||
      message.find("barrier") != std::string::npos ||
      message.find("condition") != std::string::npos) {
    return "SINGULARITY_LIMIT";
  }
  return "IK_FAILED";
}

} // namespace

int main(int argc, char **argv) {
  if (argc != 8 || std::string(argv[1]) != "--sample") {
    std::cerr << "Usage: " << argv[0]
              << " --sample dx_m dy_m dz_m droll_rad dpitch_rad dyaw_rad\n";
    return 2;
  }
  std::array<double, 6> injected{};
  for (std::size_t index = 0; index < injected.size(); ++index) {
    char *end = nullptr;
    injected[index] = std::strtod(argv[index + 2], &end);
    if (end == argv[index + 2] || *end != '\0' ||
        !std::isfinite(injected[index])) {
      std::cerr << "invalid finite sample value\n";
      return 2;
    }
  }

  fr10_motion::Fr10AnalyticalIKProvider provider(
      fr10_motion_test::analyticalConfig());
  const auto seed = fr10_motion_test::nominalState();
  fr10_motion::Pose target;
  if (!provider.forward(seed, target, nullptr)) {
    return 2;
  }
  for (std::size_t axis = 0; axis < 3; ++axis) {
    target.position_m[axis] += injected[axis];
  }
  const auto perturbation = fr10_motion_test::rpyQuaternion(
      injected[3], injected[4], injected[5]);
  target.orientation_quat = fr10_motion_test::quaternionMultiply(
      perturbation, target.orientation_quat);

  const bool workspace_reachable = provider.workspaceReachable(target);
  const auto started = std::chrono::steady_clock::now();
  const auto solution = provider.solve(target, seed);
  const auto finished = std::chrono::steady_clock::now();
  const double execution_time_us =
      std::chrono::duration<double, std::micro>(finished - started).count();

  double max_joint_step = 0.0;
  double position_error = 0.0;
  double orientation_error = 0.0;
  fr10_motion_test::JacobianMetrics jacobian;
  if (solution.success) {
    for (std::size_t joint = 0; joint < fr10_motion::kJointCount; ++joint) {
      max_joint_step = std::max(
          max_joint_step,
          std::abs(solution.state.position_rad[joint] -
                   seed.position_rad[joint]));
    }
    fr10_motion::Pose reconstructed;
    provider.forward(solution.state, reconstructed, nullptr);
    position_error =
        fr10_motion_test::positionDistance(target, reconstructed);
    orientation_error = fr10_motion::math::quaternionAngularDistance(
        target.orientation_quat, reconstructed.orientation_quat);
    jacobian =
        fr10_motion_test::finiteDifferenceJacobian(provider, solution.state);
  }
  const std::string failure_reason =
      solution.success ? "NONE"
                       : failureReason(solution.message, workspace_reachable);

  std::cout << std::setprecision(17)
            << "{\"accepted\":" << (solution.success ? "true" : "false")
            << ",\"failure_reason\":\"" << failure_reason << "\""
            << ",\"ik_success\":" << (solution.success ? "true" : "false")
            << ",\"workspace_reachable\":"
            << (workspace_reachable ? "true" : "false")
            << ",\"target_pose\":{\"position_m\":["
            << target.position_m[0] << "," << target.position_m[1] << ","
            << target.position_m[2] << "],\"quaternion_xyzw\":["
            << target.orientation_quat[0] << ","
            << target.orientation_quat[1] << ","
            << target.orientation_quat[2] << ","
            << target.orientation_quat[3] << "]}"
            << ",\"max_joint_step_rad\":" << max_joint_step
            << ",\"sigma_min\":" << jacobian.sigma_min
            << ",\"jacobian_condition_number\":"
            << (jacobian.valid ? jacobian.condition : 0.0)
            << ",\"max_cartesian_error_m\":" << position_error
            << ",\"max_orientation_error_rad\":" << orientation_error
            << ",\"execution_time_us\":" << execution_time_us
            << ",\"solver_message\":\"" << solution.message << "\"}\n";
  return 0;
}
