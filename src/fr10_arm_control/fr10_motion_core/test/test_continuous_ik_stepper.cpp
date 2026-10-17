#include <array>
#include <cmath>
#include <iostream>

#include "fr10_motion_core/continuous_ik_stepper.hpp"

namespace {

class FixedProvider final : public fr10_motion::IKProvider {
public:
  fr10_motion::IKResult solve(
      const fr10_motion::Pose &target,
      const fr10_motion::KinematicState &seed) override {
    std::array<fr10_motion::IKResult, kMaxCandidates> candidates{};
    const auto count = solveCandidates(target, seed, candidates, nullptr);
    return count == 0 ? fr10_motion::IKResult{} : candidates[0];
  }

  std::size_t solveCandidates(
      const fr10_motion::Pose &, const fr10_motion::KinematicState &,
      std::array<fr10_motion::IKResult, kMaxCandidates> &candidates,
      std::string *) override {
    candidates = {};
    candidates[0].success = true;
    candidates[0].state.position_rad[0] = 0.2;
    candidates[0].manipulability = 0.2;
    candidates[1].success = true;
    candidates[1].state.position_rad[0] = 0.05;
    candidates[1].manipulability = 0.5;
    return 2;
  }

  fr10_motion::IKResult validate(
      const fr10_motion::KinematicState &target,
      const fr10_motion::KinematicState &) override {
    fr10_motion::IKResult result;
    result.success = true;
    result.state = target;
    return result;
  }

  bool forward(const fr10_motion::KinematicState &,
               fr10_motion::Pose &pose, std::string *) const override {
    pose = expected_pose;
    return forward_ok;
  }

  fr10_motion::Pose expected_pose;
  bool forward_ok{true};
};

bool check(bool condition, const char *name) {
  if (!condition) {
    std::cerr << "FAIL " << name << '\n';
  }
  return condition;
}

} // namespace

int main() {
  bool ok = true;
  FixedProvider provider;
  provider.expected_pose.position_m = {0.1, 0.2, 0.3};
  fr10_motion::ContinuousIKConfig config;
  config.max_joint_step_rad = 0.1;
  config.max_position_error_m = 1e-8;
  config.max_orientation_error_rad = 1e-8;
  fr10_motion::ContinuousIKStepper stepper(config);
  fr10_motion::KinematicState seed;
  const auto selected = stepper.step(provider.expected_pose, seed, provider);
  ok &= check(selected.success && selected.selected_candidate == 1 &&
                  std::abs(selected.state.position_rad[0] - 0.05) < 1e-12,
              "STEPPER_SELECTS_CONTINUOUS_NEAREST_BRANCH");

  config.max_joint_step_rad = 0.01;
  stepper.setConfig(config);
  const auto discontinuous = stepper.step(provider.expected_pose, seed, provider);
  ok &= check(!discontinuous.success &&
                  discontinuous.failure ==
                      fr10_motion::ContinuousIKStepFailure::CONTINUITY_LIMIT,
              "STEPPER_REJECTS_DISCONTINUITY");

  auto invalid_pose = provider.expected_pose;
  invalid_pose.orientation_quat.fill(0.0);
  const auto invalid = stepper.step(invalid_pose, seed, provider);
  ok &= check(!invalid.success &&
                  invalid.failure ==
                      fr10_motion::ContinuousIKStepFailure::INVALID_INPUT,
              "STEPPER_REJECTS_INVALID_POSE");
  return ok ? 0 : 1;
}
