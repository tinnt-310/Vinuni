#pragma once

#include <array>
#include <cstddef>
#include <string>

#include "fr10_motion_core/ik_provider.hpp"

namespace fr10_motion_planning {

// Keeps closed-form branch search deterministic while requiring every
// candidate to pass an independent model/collision validator. This adapter is
// for managed planning workers and is never called from the 125 Hz tick.
class CollisionValidatedIKProvider final : public fr10_motion::IKProvider {
public:
  CollisionValidatedIKProvider(fr10_motion::IKProvider &primary,
                               fr10_motion::IKProvider &validator)
      : primary_(primary), validator_(validator) {}

  fr10_motion::IKResult solve(
      const fr10_motion::Pose &target,
      const fr10_motion::KinematicState &seed) override {
    std::array<fr10_motion::IKResult, kMaxCandidates> candidates{};
    std::string error;
    if (solveCandidates(target, seed, candidates, &error) == 0) {
      fr10_motion::IKResult result;
      result.message = error;
      return result;
    }
    return candidates[0];
  }

  std::size_t solveCandidates(
      const fr10_motion::Pose &target,
      const fr10_motion::KinematicState &seed,
      std::array<fr10_motion::IKResult, kMaxCandidates> &candidates,
      std::string *error = nullptr) override {
    std::array<fr10_motion::IKResult, kMaxCandidates> primary_candidates{};
    std::string primary_error;
    const std::size_t count = primary_.solveCandidates(
        target, seed, primary_candidates, &primary_error);
    candidates = {};
    std::size_t accepted = 0;
    std::string validation_error;
    for (std::size_t index = 0; index < count; ++index) {
      auto validation =
          validator_.validate(primary_candidates[index].state, seed);
      if (!validation.success) {
        validation_error = validation.message;
        continue;
      }
      candidates[accepted] = primary_candidates[index];
      candidates[accepted].manipulability = validation.manipulability;
      candidates[accepted].condition_number = validation.condition_number;
      candidates[accepted].message = "success";
      ++accepted;
    }
    if (accepted == 0 && error != nullptr) {
      *error = validation_error.empty() ? primary_error : validation_error;
    }
    return accepted;
  }

  fr10_motion::IKResult validate(
      const fr10_motion::KinematicState &target,
      const fr10_motion::KinematicState &seed) override {
    const auto primary_result = primary_.validate(target, seed);
    if (!primary_result.success) return primary_result;
    return validator_.validate(target, seed);
  }

  bool forward(const fr10_motion::KinematicState &state,
               fr10_motion::Pose &pose,
               std::string *error = nullptr) const override {
    return primary_.forward(state, pose, error);
  }

private:
  fr10_motion::IKProvider &primary_;
  fr10_motion::IKProvider &validator_;
};

} // namespace fr10_motion_planning

