#pragma once

#include <string>
#include <vector>

#include "fr10_motion_core/cartesian_path.hpp"

namespace fr10_motion {

struct LinearInsertPlannerConfig {
  CartesianPathConfig path;
  double max_insertion_distance_m{0.250};
  double max_orientation_change_rad{0.02};
  double perpendicular_tolerance_m{0.002};
  double position_tolerance_m{0.002};
  double orientation_tolerance_rad{0.02};
  double linear_speed_scale{0.10};
  double angular_speed_scale{0.10};
};

struct LinearInsertPlan {
  bool success{false};
  std::vector<Pose> cartesian_path;
  Pose start_pose;
  Pose target_pose;
  double distance_m{0.0};
  double linear_speed_scale{0.0};
  double angular_speed_scale{0.0};
  std::string message;
};

struct LinearInsertAxisValidation {
  bool success{false};
  LinearInsertPlan straight_segment;
  double axis_deviation_m{0.0};
  double forward_projection{0.0};
};

struct LinearInsertAxisCheck {
  bool success{false};
  double distance_m{0.0};
  double axis_deviation_m{0.0};
  double forward_projection{0.0};
  std::string message;
};

enum class LinearInsertStopReason : int {
  NONE = 0,
  INVALID_INPUT,
  UNEXPECTED_RESISTANCE,
  PERPENDICULAR_DEVIATION,
  ORIENTATION_ERROR,
  POSITION_ERROR
};

const char *linearInsertStopReasonName(LinearInsertStopReason reason);

struct LinearInsertTracking {
  double position_error_m{0.0};
  double orientation_error_rad{0.0};
  double perpendicular_error_m{0.0};
  bool valid{false};
  bool unexpected_resistance{false};
  bool controlled_stop_required{false};
  LinearInsertStopReason stop_reason{LinearInsertStopReason::NONE};
};

class LinearInsertPlanner {
public:
  static LinearInsertPlan plan(const Pose &pre_insert, const Pose &target,
                               const LinearInsertPlannerConfig &config);

  // Constant-time segment/axis check; does not materialize waypoints.
  static LinearInsertAxisCheck checkAxis(
      const Pose &current, const Pose &target, const Pose &expected_start,
      const Pose &expected_goal, const LinearInsertPlannerConfig &config);

  // Full legacy validation, including an allocated straight-line path.
  static LinearInsertAxisValidation validateAxis(
      const Pose &current, const Pose &target, const Pose &expected_start,
      const Pose &expected_goal, const LinearInsertPlannerConfig &config);

  static LinearInsertTracking monitor(
      const Pose &path_start, const Pose &path_goal,
      const Pose &desired_pose, const Pose &actual_pose,
      bool unexpected_resistance, const LinearInsertPlannerConfig &config);
};

} // namespace fr10_motion
