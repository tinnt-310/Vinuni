#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>

#include "fr10_motion_core/core_types.hpp"

namespace fr10_motion {

struct KinematicState {
  std::array<double, kJointCount> position_rad{};
  std::array<double, kJointCount> velocity_rad_s{};
  std::array<double, kJointCount> acceleration_rad_s2{};
};

struct IKResult {
  bool success{false};
  KinematicState state;
  double manipulability{0.0};
  double condition_number{0.0};
  std::string message;
};

class IKProvider {
public:
  static constexpr std::size_t kMaxCandidates = 8;
  virtual ~IKProvider() = default;

  [[nodiscard]] static bool barrierSafe(double seed_value, double target_value,
                                        double minimum_abs) {
    if (minimum_abs <= 0.0) {
      return true;
    }
    if (std::abs(target_value) < minimum_abs) {
      return false;
    }
    if (std::abs(seed_value) < minimum_abs) {
      return true;
    }
    return std::signbit(seed_value) == std::signbit(target_value);
  }

  [[nodiscard]] static bool checkJointLimits(const KinematicState &target,
                                             const KinematicLimits &limits,
                                             std::string &error) {
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      if (!std::isfinite(target.position_rad[joint]) ||
          target.position_rad[joint] < limits.min_joint_position_rad[joint] ||
          target.position_rad[joint] > limits.max_joint_position_rad[joint]) {
        error = "joint limit violation";
        return false;
      }
    }
    return true;
  }

  [[nodiscard]] virtual IKResult solve(const Pose &target,
                                       const KinematicState &seed) = 0;
  [[nodiscard]] virtual IKResult validate(const KinematicState &target,
                                          const KinematicState &seed) = 0;

  // Providers with closed-form branch enumeration override this method.
  // Numerical/synthetic providers retain the single-candidate default.
  [[nodiscard]] virtual std::size_t solveCandidates(
      const Pose &target, const KinematicState &seed,
      std::array<IKResult, kMaxCandidates> &candidates,
      std::string *error = nullptr) {
    candidates = {};
    IKResult result = solve(target, seed);
    if (!result.success) {
      if (error != nullptr) {
        *error = result.message;
      }
      return 0;
    }
    candidates[0] = std::move(result);
    return 1;
  }

  [[nodiscard]] virtual bool forward(const KinematicState &state, Pose &pose,
                                     std::string *error = nullptr) const = 0;
};

} // namespace fr10_motion
