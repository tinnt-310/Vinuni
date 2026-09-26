#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <string>

#include "fr10_pose_controller/core_types.hpp"
#include "fr10_pose_controller/math_utils.hpp"

namespace fr10_motion::diagnostics {

struct UnifiedTrackingDiagnostic {
  Pose target_tcp;
  Pose actual_tcp;
  Pose target_flange;
  Pose actual_flange;
  std::array<double, kJointCount> target_joint_rad{};
  std::array<double, kJointCount> actual_joint_rad{};
  std::array<double, kJointCount> joint_error_rad{};
  double tcp_position_error_m{0.0};
  double tcp_orientation_error_rad{0.0};
  double flange_position_error_m{0.0};
  double flange_orientation_error_rad{0.0};
  double max_joint_error_rad{0.0};
};

inline Pose composePose(const Pose &base_to_first,
                        const Pose &first_to_second) {
  Pose result;
  const auto rotated_translation = math::rotateVector(
      base_to_first.orientation_quat, first_to_second.position_m);
  for (std::size_t axis = 0; axis < 3; ++axis) {
    result.position_m[axis] =
        base_to_first.position_m[axis] + rotated_translation[axis];
  }
  result.orientation_quat = math::normalizedQuaternion(math::quaternionMultiply(
      base_to_first.orientation_quat, first_to_second.orientation_quat));
  return result;
}

inline Pose inversePose(const Pose &pose) {
  Pose result;
  result.orientation_quat = math::quaternionConjugate(
      math::normalizedQuaternion(pose.orientation_quat));
  const std::array<double, 3> negative_translation{
      -pose.position_m[0], -pose.position_m[1], -pose.position_m[2]};
  result.position_m =
      math::rotateVector(result.orientation_quat, negative_translation);
  return result;
}

inline double translationDistance(const Pose &first, const Pose &second) {
  double squared_distance = 0.0;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const double delta = first.position_m[axis] - second.position_m[axis];
    squared_distance += delta * delta;
  }
  return std::sqrt(squared_distance);
}

inline UnifiedTrackingDiagnostic makeUnifiedTrackingDiagnostic(
    const Pose &target_tcp, const Pose &actual_flange,
    const Pose &flange_to_tcp,
    const std::array<double, kJointCount> &target_joint_rad,
    const std::array<double, kJointCount> &actual_joint_rad) {
  UnifiedTrackingDiagnostic diagnostic;
  diagnostic.target_tcp = target_tcp;
  diagnostic.actual_flange = actual_flange;
  diagnostic.target_flange =
      composePose(target_tcp, inversePose(flange_to_tcp));
  diagnostic.actual_tcp = composePose(actual_flange, flange_to_tcp);
  diagnostic.target_joint_rad = target_joint_rad;
  diagnostic.actual_joint_rad = actual_joint_rad;
  diagnostic.tcp_position_error_m =
      translationDistance(diagnostic.target_tcp, diagnostic.actual_tcp);
  diagnostic.tcp_orientation_error_rad =
      math::quaternionAngularDistance(diagnostic.target_tcp.orientation_quat,
                                      diagnostic.actual_tcp.orientation_quat);
  diagnostic.flange_position_error_m =
      translationDistance(diagnostic.target_flange, diagnostic.actual_flange);
  diagnostic.flange_orientation_error_rad = math::quaternionAngularDistance(
      diagnostic.target_flange.orientation_quat,
      diagnostic.actual_flange.orientation_quat);
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    diagnostic.joint_error_rad[joint] =
        math::wrapToPi(target_joint_rad[joint] - actual_joint_rad[joint]);
    diagnostic.max_joint_error_rad =
        std::max(diagnostic.max_joint_error_rad,
                 std::abs(diagnostic.joint_error_rad[joint]));
  }
  return diagnostic;
}

inline std::array<double, 6> poseForFairinoDisplay(const Pose &pose) {
  // FAIRINO v3.9.3 robot_types.h defines RX/RY/RZ as rotations about the
  // fixed X/Y/Z axes in degrees. This is the fixed-axis XYZ convention
  // represented by R = Rz(yaw) * Ry(pitch) * Rx(roll).
  const auto rpy = math::quaternionToFixedXyzRpy(pose.orientation_quat);
  return {pose.position_m[0], pose.position_m[1], pose.position_m[2],
          rpy[0] * kRadToDeg, rpy[1] * kRadToDeg, rpy[2] * kRadToDeg};
}

inline std::string
formatUnifiedTrackingBlock(const char *state, const char *mode,
                           double control_rate_hz,
                           const UnifiedTrackingDiagnostic &diagnostic) {
  const auto target_tcp = poseForFairinoDisplay(diagnostic.target_tcp);
  const auto actual_tcp = poseForFairinoDisplay(diagnostic.actual_tcp);
  const auto target_flange = poseForFairinoDisplay(diagnostic.target_flange);
  const auto actual_flange = poseForFairinoDisplay(diagnostic.actual_flange);
  std::array<double, kJointCount> target_joint_deg{};
  std::array<double, kJointCount> actual_joint_deg{};
  std::array<double, kJointCount> joint_error_deg{};
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    target_joint_deg[joint] = diagnostic.target_joint_rad[joint] * kRadToDeg;
    actual_joint_deg[joint] = diagnostic.actual_joint_rad[joint] * kRadToDeg;
    joint_error_deg[joint] = diagnostic.joint_error_rad[joint] * kRadToDeg;
  }

  std::array<char, 4096> buffer{};
  const int length = std::snprintf(
      buffer.data(), buffer.size(),
      "[%s | %s | %.1f Hz]\n\n"
      "       TARGET TCP   : [%.4f, %.4f, %.4f, %.2f, %.2f, %.2f]\n"
      "       ACTUAL TCP   : [%.4f, %.4f, %.4f, %.2f, %.2f, %.2f]\n"
      "       TCP ERROR    : %.2f mm | %.2f deg\n\n"
      "       TARGET FLANGE: [%.4f, %.4f, %.4f, %.2f, %.2f, %.2f]\n"
      "       ACTUAL FLANGE: [%.4f, %.4f, %.4f, %.2f, %.2f, %.2f]\n"
      "       FLANGE ERROR : %.2f mm | %.2f deg\n\n"
      "       TARGET JOINT : [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] deg\n"
      "       ACTUAL JOINT : [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] deg\n"
      "       JOINT ERROR  : [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] deg\n"
      "       MAX J ERROR  : %.2f deg",
      state, mode, control_rate_hz, target_tcp[0], target_tcp[1], target_tcp[2],
      target_tcp[3], target_tcp[4], target_tcp[5], actual_tcp[0], actual_tcp[1],
      actual_tcp[2], actual_tcp[3], actual_tcp[4], actual_tcp[5],
      diagnostic.tcp_position_error_m * 1000.0,
      diagnostic.tcp_orientation_error_rad * kRadToDeg, target_flange[0],
      target_flange[1], target_flange[2], target_flange[3], target_flange[4],
      target_flange[5], actual_flange[0], actual_flange[1], actual_flange[2],
      actual_flange[3], actual_flange[4], actual_flange[5],
      diagnostic.flange_position_error_m * 1000.0,
      diagnostic.flange_orientation_error_rad * kRadToDeg,
      target_joint_deg[0],
      target_joint_deg[1], target_joint_deg[2], target_joint_deg[3],
      target_joint_deg[4], target_joint_deg[5], actual_joint_deg[0],
      actual_joint_deg[1], actual_joint_deg[2], actual_joint_deg[3],
      actual_joint_deg[4], actual_joint_deg[5], joint_error_deg[0],
      joint_error_deg[1], joint_error_deg[2], joint_error_deg[3],
      joint_error_deg[4], joint_error_deg[5],
      diagnostic.max_joint_error_rad * kRadToDeg);
  if (length <= 0) {
    return {};
  }
  const int safe_length = std::min<int>(
      length, static_cast<int>(buffer.size() - 1));
  return std::string(buffer.data(), static_cast<std::size_t>(safe_length));
}

inline std::string
unifiedTrackingLogKey(const char *state, const char *mode,
                      const UnifiedTrackingDiagnostic &diagnostic) {
  // Reuse the displayed precision for change detection, but deliberately
  // ignore small loop-rate fluctuations. Motion/state changes still alter the
  // key, while an unchanged IDLE/HOLDING snapshot is logged only once.
  return formatUnifiedTrackingBlock(state, mode, 0.0, diagnostic);
}

inline bool shouldEmitChangedTrackingLog(const std::string &key,
                                         std::string &last_key) {
  if (key.empty() || key == last_key) {
    return false;
  }
  last_key = key;
  return true;
}

} // namespace fr10_motion::diagnostics
