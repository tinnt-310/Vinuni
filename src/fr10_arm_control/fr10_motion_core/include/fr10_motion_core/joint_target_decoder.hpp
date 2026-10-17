#pragma once

#include <array>
#include <cmath>
#include <string>
#include <string_view>
#include <vector>

#include "fr10_motion_core/core_types.hpp"

namespace fr10_motion {

enum class JointTargetDecodeError { NONE, COUNT, NAMES, NONFINITE, UNIT };

struct JointTargetDecodeResult {
  JointTargetDecodeError error{JointTargetDecodeError::COUNT};
  std::array<double, kJointCount> position_rad{};
};

// Decode a named joint command without ROS types or mutable name-map state.
// Input order may differ from controller order, but every joint must occur
// exactly once and every numeric command must be finite.
inline JointTargetDecodeResult decodeJointTarget(
    const std::vector<std::string> &names,
    const std::vector<double> &positions,
    const std::vector<std::string> &expected_names,
    std::string_view input_unit) {
  JointTargetDecodeResult result;
  if (names.size() != kJointCount || positions.size() != kJointCount ||
      expected_names.size() != kJointCount) {
    return result;
  }
  if (input_unit != "degrees" && input_unit != "radians") {
    result.error = JointTargetDecodeError::UNIT;
    return result;
  }
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    bool found = false;
    std::size_t index = 0;
    for (std::size_t source = 0; source < kJointCount; ++source) {
      if (names[source] == expected_names[joint]) {
        if (found) {
          result.error = JointTargetDecodeError::NAMES;
          return result;
        }
        found = true;
        index = source;
      }
    }
    if (!found) {
      result.error = JointTargetDecodeError::NAMES;
      return result;
    }
    const double value = positions[index];
    if (!std::isfinite(value)) {
      result.error = JointTargetDecodeError::NONFINITE;
      return result;
    }
    result.position_rad[joint] =
        input_unit == "degrees" ? value * kDegToRad : value;
  }
  result.error = JointTargetDecodeError::NONE;
  return result;
}

} // namespace fr10_motion
