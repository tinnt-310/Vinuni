#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

#include "fr10_pose_controller/ik_provider.hpp"

namespace fr10_motion {

struct ContinuousIKConfig {
  double max_joint_step_rad{0.35};
  double max_position_error_m{0.001};
  double max_orientation_error_rad{0.01};
};

struct ContinuousIKResult {
  bool success{false};
  std::vector<KinematicState> joint_path;
  std::size_t failure_index{0};
  double max_position_error_m{0.0};
  double max_orientation_error_rad{0.0};
  double min_manipulability{0.0};
  double max_condition_number{0.0};
  std::string message;
};

class ContinuousIK {
public:
  using CancelCheck = std::function<bool()>;

  static ContinuousIKResult solve(const std::vector<Pose> &cartesian_path,
                                  const KinematicState &seed,
                                  IKProvider &provider,
                                  const ContinuousIKConfig &config,
                                  const CancelCheck &cancel = {});
};

} // namespace fr10_motion
