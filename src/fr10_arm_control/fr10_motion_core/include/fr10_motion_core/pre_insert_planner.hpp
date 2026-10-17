#pragma once

#include <array>
#include <string>

#include "fr10_motion_core/core_types.hpp"

namespace fr10_motion {

struct PreInsertPlannerConfig {
  std::array<double, 3> local_axis{{0.0, 0.0, 1.0}};
  double distance_m{0.070};
  double min_distance_m{0.001};
  double max_distance_m{0.250};
};

struct PreInsertPlan {
  bool success{false};
  Pose target_pose;
  Pose pre_insert_pose;
  std::array<double, 3> base_axis{};
  double distance_m{0.0};
  std::string message;
};

class PreInsertPlanner {
public:
  static PreInsertPlan plan(const Pose &target,
                            const PreInsertPlannerConfig &config);
};

} // namespace fr10_motion
