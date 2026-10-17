#include "fr10_force_control/wrench_transform.hpp"

#include <cmath>
#include <limits>

namespace fr10_force {

bool WrenchTransform::valid(const RigidTransform &transform) {
  if (!finite(transform.rotation_target_from_source) ||
      !finite(transform.target_to_source_m)) {
    return false;
  }
  // Require SO(3), not just finite/unit rows: reflections or repeated rows
  // change wrench handedness and must fail closed. Fixed-size, no SVD.
  for (std::size_t row = 0; row < 3; ++row) {
    double squared_norm = 0.0;
    for (std::size_t column = 0; column < 3; ++column) {
      const double value =
          transform.rotation_target_from_source[row * 3 + column];
      squared_norm += value * value;
    }
    if (std::abs(squared_norm - 1.0) > 1e-6) {
      return false;
    }
  }
  const auto &r = transform.rotation_target_from_source;
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t other = row + 1; other < 3; ++other) {
      double dot = 0.0;
      for (std::size_t column = 0; column < 3; ++column) {
        dot += r[row * 3 + column] * r[other * 3 + column];
      }
      if (std::abs(dot) > 1e-6) return false;
    }
  }
  const double determinant = r[0]*(r[4]*r[8]-r[5]*r[7]) -
      r[1]*(r[3]*r[8]-r[5]*r[6]) + r[2]*(r[3]*r[7]-r[4]*r[6]);
  if (std::abs(determinant - 1.0) > 1e-6) return false;
  return true;
}

Wrench WrenchTransform::apply(const Wrench &source,
                              const RigidTransform &transform) {
  if (!finite(source) || !valid(transform)) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    return {{nan, nan, nan}, {nan, nan, nan}};
  }
  Wrench target;
  target.force_n = rotate(transform.rotation_target_from_source,
                          source.force_n);
  target.torque_nm = rotate(transform.rotation_target_from_source,
                            source.torque_nm);
  const auto moment = cross(transform.target_to_source_m, target.force_n);
  for (std::size_t axis = 0; axis < 3; ++axis) {
    target.torque_nm[axis] += moment[axis];
  }
  return target;
}

} // namespace fr10_force
