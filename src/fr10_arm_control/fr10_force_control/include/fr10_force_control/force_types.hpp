#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace fr10_force {

constexpr std::size_t kCartesianDofs = 6;
constexpr std::size_t kTranslationDofs = 3;

using Vector3 = std::array<double, 3>;
using Vector6 = std::array<double, 6>;
using Quaternion = std::array<double, 4>;
using RotationMatrix = std::array<double, 9>;

struct CartesianPose {
  Vector3 position_m{};
  Quaternion orientation_xyzw{{0.0, 0.0, 0.0, 1.0}};
};

struct Wrench {
  Vector3 force_n{};
  Vector3 torque_nm{};

  [[nodiscard]] Vector6 vector() const {
    return {force_n[0], force_n[1], force_n[2],
            torque_nm[0], torque_nm[1], torque_nm[2]};
  }

  static Wrench fromVector(const Vector6 &value) {
    return {{value[0], value[1], value[2]},
            {value[3], value[4], value[5]}};
  }
};

struct WrenchSample {
  Wrench wrench;
  double source_timestamp_s{0.0};
  double receipt_steady_s{0.0};
  uint64_t sequence{0};
  bool valid{false};
  bool overload{false};
};

struct CartesianCorrection {
  Vector6 position{};
  Vector6 velocity{};
  Vector6 acceleration{};
};

struct RigidTransform {
  // Row-major rotation mapping a vector from source to target.
  RotationMatrix rotation_target_from_source{{1.0, 0.0, 0.0,
                                               0.0, 1.0, 0.0,
                                               0.0, 0.0, 1.0}};
  // Vector from target origin to source origin, expressed in target.
  Vector3 target_to_source_m{};
};

template <std::size_t N>
[[nodiscard]] inline bool finite(const std::array<double, N> &value) {
  return std::all_of(value.begin(), value.end(),
                     [](double element) { return std::isfinite(element); });
}

[[nodiscard]] inline bool finite(const Wrench &wrench) {
  return finite(wrench.force_n) && finite(wrench.torque_nm);
}

// Raw safety evidence between controller ticks; conditioning processes the
// individual source samples separately before controller-rate consumption. Brief impulses/invalid packets cannot be
// overwritten by a subsequent normal packet from an oversampled sensor.
struct RawWrenchEnvelope {
  Wrench peak;
  bool valid{true};
  bool populated{false};
  void observe(const Wrench &value, bool sample_valid=true) {
    populated=true;valid=valid && sample_valid && finite(value);
    if(!finite(value)) return;
    for(std::size_t i=0;i<3;++i) {
      if(std::abs(value.force_n[i])>std::abs(peak.force_n[i])) peak.force_n[i]=value.force_n[i];
      if(std::abs(value.torque_nm[i])>std::abs(peak.torque_nm[i])) peak.torque_nm[i]=value.torque_nm[i];
    }
  }
  void reset() { *this=RawWrenchEnvelope{}; }
};

[[nodiscard]] inline bool finite(const CartesianPose &pose) {
  return finite(pose.position_m) && finite(pose.orientation_xyzw);
}

[[nodiscard]] inline double norm(const Vector3 &value) {
  return std::sqrt(value[0] * value[0] + value[1] * value[1] +
                   value[2] * value[2]);
}

[[nodiscard]] inline Vector3 cross(const Vector3 &left,
                                   const Vector3 &right) {
  return {left[1] * right[2] - left[2] * right[1],
          left[2] * right[0] - left[0] * right[2],
          left[0] * right[1] - left[1] * right[0]};
}

[[nodiscard]] inline Vector3 rotate(const RotationMatrix &rotation,
                                    const Vector3 &value) {
  return {rotation[0] * value[0] + rotation[1] * value[1] +
              rotation[2] * value[2],
          rotation[3] * value[0] + rotation[4] * value[1] +
              rotation[5] * value[2],
          rotation[6] * value[0] + rotation[7] * value[1] +
              rotation[8] * value[2]};
}

} // namespace fr10_force
