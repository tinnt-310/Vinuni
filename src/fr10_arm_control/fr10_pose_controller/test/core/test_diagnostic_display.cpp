#include <cmath>
#include <iostream>
#include <string>

#include "fr10_pose_controller/diagnostic_display.hpp"
#include "test_support.hpp"

namespace {
constexpr double kPi = 3.14159265358979323846;

bool near(double first, double second, double tolerance = 1e-10) {
  return std::abs(first - second) <= tolerance;
}
} // namespace

int main() {
  fr10_motion_test::Check check;

  fr10_motion::Pose actual_flange;
  actual_flange.position_m = {1.0, 2.0, 3.0};
  actual_flange.orientation_quat =
      fr10_motion::math::fixedXyzRpyToQuaternion({0.0, 0.0, 0.5 * kPi});
  fr10_motion::Pose flange_to_tcp;
  flange_to_tcp.position_m = {0.060, 0.0, 0.175};
  flange_to_tcp.orientation_quat =
      fr10_motion::math::fixedXyzRpyToQuaternion({0.2, -0.1, 0.3});
  const fr10_motion::Pose target_tcp =
      fr10_motion::diagnostics::composePose(actual_flange, flange_to_tcp);

  std::array<double, fr10_motion::kJointCount> target_joint{};
  std::array<double, fr10_motion::kJointCount> actual_joint{};
  target_joint[0] = kPi - 0.01;
  actual_joint[0] = -kPi + 0.01;
  target_joint[1] = 15.0 * fr10_motion::kDegToRad;
  actual_joint[1] = 14.5 * fr10_motion::kDegToRad;

  const auto diagnostic =
      fr10_motion::diagnostics::makeUnifiedTrackingDiagnostic(
          target_tcp, actual_flange, flange_to_tcp, target_joint, actual_joint);
  check.require(
      near(diagnostic.actual_tcp.position_m[0], 1.0) &&
          near(diagnostic.actual_tcp.position_m[1], 2.060) &&
          near(diagnostic.actual_tcp.position_m[2], 3.175),
      "DIAGNOSTIC_ACTUAL_TCP_COMPOSITION",
      "actual TCP did not include the rotated flange-to-TCP translation");
  check.require(
      fr10_motion::diagnostics::translationDistance(diagnostic.target_flange,
                                                    actual_flange) < 1e-10 &&
          diagnostic.tcp_position_error_m < 1e-10 &&
          diagnostic.tcp_orientation_error_rad < 1e-10 &&
          diagnostic.flange_position_error_m < 1e-10 &&
          diagnostic.flange_orientation_error_rad < 1e-10,
      "DIAGNOSTIC_RIGID_TRANSFORM_ROUND_TRIP",
      "target flange inverse transform did not round-trip the target TCP");
  check.require(
      near(diagnostic.joint_error_rad[0], -0.02) &&
          near(diagnostic.joint_error_rad[1], 0.5 * fr10_motion::kDegToRad) &&
          near(diagnostic.max_joint_error_rad, 0.02),
      "DIAGNOSTIC_WRAPPED_JOINT_ERROR",
      "joint diagnostics used naive rather than wrapped error");

  const auto displayed_flange =
      fr10_motion::diagnostics::poseForFairinoDisplay(actual_flange);
  check.require(near(displayed_flange[3], 0.0, 1e-9) &&
                    near(displayed_flange[4], 0.0, 1e-9) &&
                    near(displayed_flange[5], 90.0, 1e-9),
                "DIAGNOSTIC_FAIRINO_FIXED_XYZ_RPY",
                "quaternion display did not follow fixed-axis XYZ degrees");

  const std::string block =
      fr10_motion::diagnostics::formatUnifiedTrackingBlock(
          "TRACKING", "CARTESIAN", 125.0, diagnostic);
  const std::array<const char *, 10> required_order{
      "TARGET TCP",    "ACTUAL TCP",   "TCP ERROR",    "TARGET FLANGE",
      "ACTUAL FLANGE", "FLANGE ERROR", "TARGET JOINT", "ACTUAL JOINT",
      "JOINT ERROR",  "MAX J ERROR"};
  std::size_t position = 0;
  bool order_valid = block.find("[TRACKING | CARTESIAN | 125.0 Hz]") == 0;
  for (const char *label : required_order) {
    const std::size_t found = block.find(label, position);
    order_valid = order_valid && found != std::string::npos;
    if (found != std::string::npos) {
      position = found + std::string(label).size();
    }
  }
  check.require(order_valid, "DIAGNOSTIC_REQUIRED_DISPLAY_ORDER",
                "unified block labels were missing or rearranged");
  check.require(block.find("-1.15") != std::string::npos &&
                    block.find("0.50") != std::string::npos,
                "DIAGNOSTIC_JOINT_DEGREES",
                "wrapped joint error was not formatted in degrees");

  std::string last_log_key;
  const std::string tracking_key =
      fr10_motion::diagnostics::unifiedTrackingLogKey(
          "HOLDING", "JOINT", diagnostic);
  check.require(
      fr10_motion::diagnostics::shouldEmitChangedTrackingLog(tracking_key,
                                                             last_log_key) &&
          !fr10_motion::diagnostics::shouldEmitChangedTrackingLog(
              tracking_key, last_log_key),
      "DIAGNOSTIC_DEDUP_UNCHANGED",
      "an unchanged displayed snapshot was emitted more than once");

  auto moved_flange = actual_flange;
  moved_flange.position_m[0] += 0.001;
  const auto moved_diagnostic =
      fr10_motion::diagnostics::makeUnifiedTrackingDiagnostic(
          target_tcp, moved_flange, flange_to_tcp, target_joint, actual_joint);
  const std::string moved_key =
      fr10_motion::diagnostics::unifiedTrackingLogKey(
          "HOLDING", "JOINT", moved_diagnostic);
  check.require(
      fr10_motion::diagnostics::shouldEmitChangedTrackingLog(moved_key,
                                                             last_log_key),
      "DIAGNOSTIC_DEDUP_CHANGED_POSE",
      "a changed displayed pose was incorrectly suppressed");
  const std::string state_key =
      fr10_motion::diagnostics::unifiedTrackingLogKey(
          "TRACKING", "JOINT", moved_diagnostic);
  check.require(
      fr10_motion::diagnostics::shouldEmitChangedTrackingLog(state_key,
                                                             last_log_key),
      "DIAGNOSTIC_DEDUP_CHANGED_STATE",
      "a state transition was incorrectly suppressed");

  std::cout << "METRIC flange_tcp_translation_m="
            << flange_to_tcp.position_m[0] << ","
            << flange_to_tcp.position_m[1] << ","
            << flange_to_tcp.position_m[2] << " wrapped_j1_error_deg="
            << diagnostic.joint_error_rad[0] * fr10_motion::kRadToDeg
            << " display_bytes=" << block.size() << "\n";
  return check.result();
}
