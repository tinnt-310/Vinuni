#include "fr10_motion_core/joint_target_decoder.hpp"
#include "test_support.hpp"

#include <limits>
#include <string>
#include <vector>

int main() {
  fr10_motion_test::Check check;
  const std::vector<std::string> expected{"j1", "j2", "j3", "j4", "j5", "j6"};
  const std::vector<std::string> reversed{"j6", "j5", "j4", "j3", "j2", "j1"};
  const std::vector<double> degrees{60.0, 50.0, 40.0, 30.0, 20.0, 10.0};

  const auto decoded = fr10_motion::decodeJointTarget(
      reversed, degrees, expected, "degrees");
  check.require(decoded.error == fr10_motion::JointTargetDecodeError::NONE,
                "reordered_names", "all six named joints should decode");
  for (std::size_t i = 0; i < fr10_motion::kJointCount; ++i) {
    const double expected_rad = (i + 1) * 10.0 * fr10_motion::kDegToRad;
    check.require(fr10_motion_test::near(decoded.position_rad[i], expected_rad,
                                         1e-12),
                  "degrees", "joint order and degree conversion");
  }

  const auto radians = fr10_motion::decodeJointTarget(
      expected, std::vector<double>(fr10_motion::kJointCount, 0.5), expected,
      "radians");
  check.require(radians.error == fr10_motion::JointTargetDecodeError::NONE &&
                    fr10_motion_test::near(radians.position_rad[0], 0.5, 1e-12),
                "radians", "radian input must not be converted");

  auto short_names = expected;
  short_names.pop_back();
  check.require(fr10_motion::decodeJointTarget(short_names, degrees, expected,
                                                "degrees").error ==
                    fr10_motion::JointTargetDecodeError::COUNT,
                "count", "partial joint messages are rejected");
  auto duplicate_names = expected;
  duplicate_names.back() = "j5";
  check.require(fr10_motion::decodeJointTarget(duplicate_names, degrees,
                                                expected, "degrees").error ==
                    fr10_motion::JointTargetDecodeError::NAMES,
                "duplicate_names", "a missing j6 cannot be replaced by j5");
  auto invalid_values = degrees;
  invalid_values[2] = std::numeric_limits<double>::quiet_NaN();
  check.require(fr10_motion::decodeJointTarget(expected, invalid_values,
                                                expected, "degrees").error ==
                    fr10_motion::JointTargetDecodeError::NONFINITE,
                "nan", "NaN command values are rejected");
  invalid_values[2] = std::numeric_limits<double>::infinity();
  check.require(fr10_motion::decodeJointTarget(expected, invalid_values,
                                                expected, "degrees").error ==
                    fr10_motion::JointTargetDecodeError::NONFINITE,
                "inf", "infinite command values are rejected");
  check.require(fr10_motion::decodeJointTarget(expected, degrees, expected,
                                                "invalid").error ==
                    fr10_motion::JointTargetDecodeError::UNIT,
                "unit", "unsupported input units are rejected");
  return check.result();
}
