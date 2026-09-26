#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <string>

#include "fr10_pose_controller/core_types.hpp"
#include "fr10_pose_controller/math_utils.hpp"

namespace fr10_motion_test {

inline fr10_motion::KinematicLimits limits() {
  fr10_motion::KinematicLimits result;
  result.min_joint_position_rad =
      {-3.0543, -4.6251, -2.8274, -4.6251, -3.0543, -3.0543};
  result.max_joint_position_rad =
      {3.0543, 1.4835, 2.8274, 1.4835, 3.0543, 3.0543};
  result.max_joint_velocity_rad_s.fill(0.30);
  result.max_joint_acceleration_rad_s2.fill(0.80);
  result.max_joint_jerk_rad_s3.fill(4.0);
  result.max_linear_velocity_m_s = 0.20;
  result.max_linear_acceleration_m_s2 = 0.80;
  result.max_linear_jerk_m_s3 = 4.0;
  result.max_angular_velocity_rad_s = 0.40;
  result.max_angular_acceleration_rad_s2 = 1.20;
  result.max_angular_jerk_rad_s3 = 6.0;
  return result;
}

inline bool near(double first, double second, double tolerance) {
  return std::abs(first - second) <= tolerance;
}

inline double norm3(const std::array<double, 3> &value) {
  return std::sqrt(value[0] * value[0] + value[1] * value[1] +
                   value[2] * value[2]);
}

inline double quaternionDistance(const std::array<double, 4> &first,
                                 const std::array<double, 4> &second) {
  return fr10_motion::math::quaternionAngularDistance(first, second);
}

inline std::array<double, 4> axisAngle(std::size_t axis, double angle_rad) {
  std::array<double, 4> quaternion{0.0, 0.0, 0.0,
                                    std::cos(0.5 * angle_rad)};
  quaternion.at(axis) = std::sin(0.5 * angle_rad);
  return quaternion;
}

class Check {
public:
  void require(bool condition, const std::string &test_id,
               const std::string &reason) {
    if (condition) {
      std::cout << "PASS " << test_id << "\n";
      return;
    }
    ++failures_;
    std::cerr << "FAIL " << test_id << ": " << reason << "\n";
  }

  int result() const { return failures_ == 0 ? 0 : 1; }
  int failures() const { return failures_; }

private:
  int failures_{0};
};

} // namespace fr10_motion_test
