#pragma once

#include <cstddef>

#include "fr10_motion_core/continuous_ik.hpp"

namespace fr10_motion {

enum class ContinuousIKStepFailure : int {
  NONE = 0,
  INVALID_INPUT,
  NO_CANDIDATE,
  CONTINUITY_LIMIT,
  FK_FAILURE,
  CARTESIAN_ERROR
};

struct ContinuousIKStepResult {
  bool success{false};
  ContinuousIKStepFailure failure{ContinuousIKStepFailure::INVALID_INPUT};
  KinematicState state;
  std::size_t candidate_count{0};
  std::size_t selected_candidate{0};
  double maximum_joint_step_rad{0.0};
  double position_error_m{0.0};
  double orientation_error_rad{0.0};
  double manipulability{0.0};
  double condition_number{0.0};
};

// Selects one IK branch for one online Cartesian sample. Candidate storage is
// fixed at IKProvider::kMaxCandidates and the class has no ROS/MoveIt surface.
class ContinuousIKStepper {
public:
  explicit ContinuousIKStepper(ContinuousIKConfig config = {});

  [[nodiscard]] ContinuousIKStepResult step(
      const Pose &target, const KinematicState &seed,
      IKProvider &provider) const;

  void setConfig(const ContinuousIKConfig &config);
  const ContinuousIKConfig &config() const { return config_; }

private:
  ContinuousIKConfig config_;
};

} // namespace fr10_motion
