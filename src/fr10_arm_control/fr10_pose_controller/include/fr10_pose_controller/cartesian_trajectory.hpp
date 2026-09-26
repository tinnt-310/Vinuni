#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "fr10_pose_controller/core_types.hpp"
#include "fr10_pose_controller/ik_provider.hpp"

namespace fr10_motion {

enum class PoseExecutionPolicy : int { FREE_SPACE = 0, INSERTION };

const char *poseExecutionPolicyName(PoseExecutionPolicy policy);

enum class TimeParameterizationStatus : int {
  FAILED = 0,
  SUCCESS,
  CANCELED,
  REQUIRES_CONTROLLED_STOP
};

enum class TimeParameterConstraint : int {
  NONE = 0,
  JOINT_VELOCITY_SLOPE,
  JOINT_ACCELERATION,
  JOINT_JERK,
  LINEAR_VELOCITY_DERIVATIVE,
  LINEAR_ACCELERATION,
  LINEAR_JERK,
  ANGULAR_VELOCITY_DERIVATIVE,
  ANGULAR_ACCELERATION,
  ANGULAR_JERK
};

const char *timeParameterConstraintName(TimeParameterConstraint constraint);

struct CartesianTrajectoryPoint {
  KinematicState joint_state;
  Pose desired_pose;
  std::array<double, 3> desired_translation_tangent{};
  double path_parameter{0.0};
  double time_from_start_s{0.0};
};

struct CartesianJointTrajectory {
  std::vector<CartesianTrajectoryPoint> points;
  Pose path_start;
  Pose path_goal;
  uint64_t generation{0};
  // Activation metadata used to reject and re-plan a trajectory whose seed
  // became stale while the asynchronous worker was planning.
  double planning_timestamp_s{0.0};
  KinematicState seed_joint_state;
};

struct PathTimeParameterizerConfig {
  double sample_period_s{0.008};
  KinematicLimits limits;
  PoseExecutionPolicy policy{PoseExecutionPolicy::FREE_SPACE};
  double joint_velocity_scale{1.0};
  double joint_acceleration_scale{1.0};
  double joint_jerk_scale{1.0};
  // Cartesian scales are velocity factors k. Acceleration uses sqrt(k),
  // while jerk uses k, preserving a proportional operational S-curve triad.
  double linear_scale{1.0};
  double angular_scale{1.0};
  double minimum_ramp_time_s{0.20};
  std::size_t max_samples{250000};
  std::size_t max_scaling_iterations{40};
};

struct PathTimeDiagnostics {
  PoseExecutionPolicy policy{PoseExecutionPolicy::FREE_SPACE};
  double trajectory_distance_m{0.0};
  double trajectory_rotation_rad{0.0};
  double duration_s{0.0};
  std::array<double, kJointCount> joint_velocity_ratio{};
  std::array<double, kJointCount> joint_acceleration_ratio{};
  std::array<double, kJointCount> joint_jerk_ratio{};
  std::size_t max_joint_velocity_index{0};
  std::size_t max_joint_acceleration_index{0};
  std::size_t max_joint_jerk_index{0};
  double max_joint_velocity_ratio{0.0};
  double max_joint_acceleration_ratio{0.0};
  double max_joint_jerk_ratio{0.0};
  double linear_velocity_ratio{0.0};
  double linear_acceleration_ratio{0.0};
  double linear_jerk_ratio{0.0};
  double angular_velocity_ratio{0.0};
  double angular_acceleration_ratio{0.0};
  double angular_jerk_ratio{0.0};
  double max_translation_derivative_m{0.0};
  double max_rotation_derivative_rad{0.0};
  std::array<double, kJointCount> max_joint_path_slope_rad{};
  TimeParameterConstraint initial_velocity_constraint{
      TimeParameterConstraint::NONE};
  TimeParameterConstraint limiting_constraint{TimeParameterConstraint::NONE};
  std::size_t scaling_iterations{0};
};

struct PathTimeParameterizationResult {
  bool success{false};
  TimeParameterizationStatus status{TimeParameterizationStatus::FAILED};
  CartesianJointTrajectory trajectory;
  double duration_s{0.0};
  std::array<double, kJointCount> max_joint_velocity_by_joint_rad_s{};
  std::array<double, kJointCount> max_joint_acceleration_by_joint_rad_s2{};
  std::array<double, kJointCount> max_joint_jerk_by_joint_rad_s3{};
  double max_joint_velocity_rad_s{0.0};
  double max_joint_acceleration_rad_s2{0.0};
  double max_joint_jerk_rad_s3{0.0};
  double max_linear_velocity_m_s{0.0};
  double max_linear_acceleration_m_s2{0.0};
  double max_linear_jerk_m_s3{0.0};
  double max_angular_velocity_rad_s{0.0};
  double max_angular_acceleration_rad_s2{0.0};
  double max_angular_jerk_rad_s3{0.0};
  PathTimeDiagnostics diagnostics;
  std::string message;
};

// Converts a uniformly sampled Cartesian/continuous-IK path into a dense,
// timestamped joint trajectory. A single jerk-limited scalar progress law is
// applied to a C2 joint spline, so the path geometry and time law remain
// explicit and MotionExecutor never shortcuts directly from q0 to qN.
class PathTimeParameterizer {
public:
  using CancelCheck = std::function<bool()>;

  static PathTimeParameterizationResult
  parameterize(const std::vector<Pose> &cartesian_path,
               const std::vector<KinematicState> &joint_path,
               const KinematicState &initial_state,
               const PathTimeParameterizerConfig &config,
               uint64_t generation = 0, const CancelCheck &cancel = {});
};

} // namespace fr10_motion
