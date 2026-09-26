#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace fr10_motion::math {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kHalfPi = 0.5 * kPi;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kQuaternionNormEpsilon = 1e-12;
constexpr double kDefaultControlPeriodS = 0.008;

[[nodiscard]] inline double wrapToPi(double angle) {
  angle = std::remainder(angle, kTwoPi);
  return angle <= -kPi ? angle + kTwoPi : angle;
}

[[nodiscard]] inline double norm3(const std::array<double, 3> &v) {
  return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

[[nodiscard]] inline double distance3D(const std::array<double, 3> &first,
                                       const std::array<double, 3> &second) {
  const double dx = second[0] - first[0];
  const double dy = second[1] - first[1];
  const double dz = second[2] - first[2];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

[[nodiscard]] inline bool validScale(double scale) {
  return std::isfinite(scale) && scale > 0.0 && scale <= 1.0;
}

[[nodiscard]] inline double quaternionNormSquared(const std::array<double, 4> &quaternion) {
  double norm_squared = 0.0;
  for (const double value : quaternion) {
    norm_squared += value * value;
  }
  return norm_squared;
}

[[nodiscard]] inline bool normalizeQuaternion(std::array<double, 4> &quaternion) {
  const double norm_squared = quaternionNormSquared(quaternion);
  if (!(norm_squared > kQuaternionNormEpsilon) || !std::isfinite(norm_squared)) {
    return false;
  }
  const double inverse_norm = 1.0 / std::sqrt(norm_squared);
  for (double &value : quaternion) {
    value *= inverse_norm;
  }
  return true;
}

[[nodiscard]] inline std::array<double, 4>
normalizedQuaternion(std::array<double, 4> quaternion) {
  if (!normalizeQuaternion(quaternion)) {
    return {0.0, 0.0, 0.0, 1.0};
  }
  return quaternion;
}

[[nodiscard]] inline double quaternionDot(const std::array<double, 4> &first,
                                       const std::array<double, 4> &second) {
  return first[0] * second[0] + first[1] * second[1] + first[2] * second[2] +
         first[3] * second[3];
}

[[nodiscard]] inline std::array<double, 4>
quaternionMultiply(const std::array<double, 4> &first,
                   const std::array<double, 4> &second) {
  return {first[3] * second[0] + first[0] * second[3] + first[1] * second[2] -
              first[2] * second[1],
          first[3] * second[1] - first[0] * second[2] + first[1] * second[3] +
              first[2] * second[0],
          first[3] * second[2] + first[0] * second[1] - first[1] * second[0] +
              first[2] * second[3],
          first[3] * second[3] - first[0] * second[0] - first[1] * second[1] -
              first[2] * second[2]};
}

[[nodiscard]] inline std::array<double, 4>
quaternionConjugate(const std::array<double, 4> &quaternion) {
  return {-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3]};
}

[[nodiscard]] inline std::array<double, 4> quaternionSlerp(std::array<double, 4> first,
                                                         std::array<double, 4> second,
                                                         double fraction) {
  if (!normalizeQuaternion(first) || !normalizeQuaternion(second) ||
      !std::isfinite(fraction)) {
    return {0.0, 0.0, 0.0, 1.0};
  }
  fraction = std::clamp(fraction, 0.0, 1.0);
  double dot = quaternionDot(first, second);
  if (dot < 0.0) {
    for (double &value : second) {
      value = -value;
    }
    dot = -dot;
  }

  if (dot > 0.9995) {
    std::array<double, 4> result{};
    for (std::size_t i = 0; i < result.size(); ++i) {
      result[i] = first[i] + fraction * (second[i] - first[i]);
    }
    if (!normalizeQuaternion(result)) {
      return {0.0, 0.0, 0.0, 1.0};
    }
    return result;
  }

  const double theta = std::acos(std::clamp(dot, -1.0, 1.0));
  const double sin_theta = std::sin(theta);
  const double first_scale = std::sin((1.0 - fraction) * theta) / sin_theta;
  const double second_scale = std::sin(fraction * theta) / sin_theta;
  return {first_scale * first[0] + second_scale * second[0],
          first_scale * first[1] + second_scale * second[1],
          first_scale * first[2] + second_scale * second[2],
          first_scale * first[3] + second_scale * second[3]};
}

// Interpolates the complementary SO(3) arc. The returned endpoint may be the
// negated quaternion representation of the same physical goal orientation.
// Callers must avoid the identical-orientation case, where the long arc is
// not uniquely defined.
[[nodiscard]] inline std::array<double, 4> quaternionSlerpLong(std::array<double, 4> first,
                                                             std::array<double, 4> second,
                                                             double fraction) {
  if (!normalizeQuaternion(first) || !normalizeQuaternion(second) ||
      !std::isfinite(fraction)) {
    return {0.0, 0.0, 0.0, 1.0};
  }
  fraction = std::clamp(fraction, 0.0, 1.0);
  double dot = quaternionDot(first, second);
  if (dot > 0.0) {
    for (double &value : second) {
      value = -value;
    }
    dot = -dot;
  }
  if (dot < -0.9995) {
    return quaternionSlerp(first, second, fraction);
  }
  const double theta = std::acos(std::clamp(dot, -1.0, 1.0));
  const double sin_theta = std::sin(theta);
  const double first_scale = std::sin((1.0 - fraction) * theta) / sin_theta;
  const double second_scale = std::sin(fraction * theta) / sin_theta;
  return {first_scale * first[0] + second_scale * second[0],
          first_scale * first[1] + second_scale * second[1],
          first_scale * first[2] + second_scale * second[2],
          first_scale * first[3] + second_scale * second[3]};
}

[[nodiscard]] inline std::array<double, 3>
rotateVector(const std::array<double, 4> &quaternion,
             const std::array<double, 3> &vector) {
  const auto normalized = normalizedQuaternion(quaternion);
  const std::array<double, 4> pure{vector[0], vector[1], vector[2], 0.0};
  const auto rotated = quaternionMultiply(quaternionMultiply(normalized, pure),
                                          quaternionConjugate(normalized));
  return {rotated[0], rotated[1], rotated[2]};
}

[[nodiscard]] inline double quaternionAngularDistance(const std::array<double, 4> &first,
                                                      const std::array<double, 4> &second) {
  const double first_norm = quaternionNormSquared(first);
  const double second_norm = quaternionNormSquared(second);
  if (!(first_norm > kQuaternionNormEpsilon) || !(second_norm > kQuaternionNormEpsilon)) {
    return std::numeric_limits<double>::infinity();
  }
  const double dot =
      quaternionDot(first, second) / std::sqrt(first_norm * second_norm);
  return 2.0 * std::acos(std::clamp(std::abs(dot), 0.0, 1.0));
}

[[nodiscard]] inline std::array<double, 3>
quaternionToRotationVector(std::array<double, 4> quaternion) {
  quaternion = normalizedQuaternion(quaternion);
  if (quaternion[3] < 0.0) {
    for (double &value : quaternion) {
      value = -value;
    }
  }
  const double vector_norm =
      std::sqrt(quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] +
                quaternion[2] * quaternion[2]);
  if (vector_norm < kQuaternionNormEpsilon) {
    return {0.0, 0.0, 0.0};
  }
  const double angle =
      2.0 * std::atan2(vector_norm, std::clamp(quaternion[3], -1.0, 1.0));
  return {quaternion[0] * angle / vector_norm,
          quaternion[1] * angle / vector_norm,
          quaternion[2] * angle / vector_norm};
}

[[nodiscard]] inline std::array<double, 4>
rotationVectorToQuaternion(const std::array<double, 3> &rotation) {
  const double angle =
      std::sqrt(rotation[0] * rotation[0] + rotation[1] * rotation[1] +
                rotation[2] * rotation[2]);
  if (angle < kQuaternionNormEpsilon) {
    return {0.0, 0.0, 0.0, 1.0};
  }
  const double scale = std::sin(0.5 * angle) / angle;
  return {rotation[0] * scale, rotation[1] * scale, rotation[2] * scale,
          std::cos(0.5 * angle)};
}

[[nodiscard]] inline std::array<double, 3>
quaternionToFixedXyzRpy(const std::array<double, 4> &quaternion) {
  const auto normalized = normalizedQuaternion(quaternion);
  const double x = normalized[0];
  const double y = normalized[1];
  const double z = normalized[2];
  const double w = normalized[3];
  const double sin_pitch = std::clamp(2.0 * (w * y - z * x), -1.0, 1.0);

  if (std::abs(sin_pitch) > 0.99999) {
    const double pitch = std::copysign(kHalfPi, sin_pitch);
    const double yaw =
        std::atan2(2.0 * (x * y - w * z), 1.0 - 2.0 * (y * y + z * z));
    return {0.0, pitch, yaw};
  }

  const double roll =
      std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  const double pitch = std::asin(sin_pitch);
  const double yaw =
      std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  return {roll, pitch, yaw};
}

[[nodiscard]] inline std::array<double, 4>
fixedXyzRpyToQuaternion(const std::array<double, 3> &rpy) {
  const double cr = std::cos(rpy[0] * 0.5);
  const double sr = std::sin(rpy[0] * 0.5);
  const double cp = std::cos(rpy[1] * 0.5);
  const double sp = std::sin(rpy[1] * 0.5);
  const double cy = std::cos(rpy[2] * 0.5);
  const double sy = std::sin(rpy[2] * 0.5);
  return {sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
          cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy};
}

} // namespace fr10_motion::math
