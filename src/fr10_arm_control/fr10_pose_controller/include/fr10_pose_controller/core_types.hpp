#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace fr10_motion {

constexpr std::size_t kJointCount = 6;
constexpr double kRadToDeg = 57.295779513082320876;
constexpr double kDegToRad = 0.017453292519943295769;
constexpr double kDefaultControlPeriodS = 0.008;

// ── Target Families
// ────────────────────────────────────────────────────────────

// The complete public control-mode surface. Pose staging/insertion choices
// are execution policies and deliberately do not extend this enum.
enum class ControlMode : int { JOINT = 0, POSE };

inline const char *controlModeName(ControlMode mode) {
  switch (mode) {
  case ControlMode::JOINT:
    return "JOINT";
  case ControlMode::POSE:
    return "POSE";
  }
  return "UNKNOWN";
}

enum class CommandType : int {
  NONE = 0,
  JOINT_POSITION
};

inline const char *commandTypeName(CommandType type) {
  switch (type) {
  case CommandType::NONE:
    return "NONE";
  case CommandType::JOINT_POSITION:
    return "JOINT_POSITION";
  }
  return "UNKNOWN";
}

enum class MotionOwner : int {
  NONE = 0,
  JOINT_OTG,
  POSE_PATH
};

inline const char *motionOwnerName(MotionOwner owner) {
  switch (owner) {
  case MotionOwner::NONE:
    return "NONE";
  case MotionOwner::JOINT_OTG:
    return "JOINT_OTG";
  case MotionOwner::POSE_PATH:
    return "POSE_PATH";
  }
  return "UNKNOWN";
}

struct JointTarget {
  std::array<double, kJointCount> position_rad{};
  std::array<double, kJointCount> target_velocity_rad_s{};
  std::array<double, kJointCount> target_acceleration_rad_s2{};
  double velocity_scale{1.0};
  double acceleration_scale{1.0};
  double jerk_scale{1.0};
  uint64_t revision{0};
  double timestamp_s{0.0};
};

struct Pose {
  std::array<double, 3> position_m{};
  // Quaternion in [x, y, z, w] order.
  std::array<double, 4> orientation_quat{{0.0, 0.0, 0.0, 1.0}};
};

struct CartesianTarget {
  std::array<double, 3> position_m{};
  // Quaternion: [x, y, z, w]
  std::array<double, 4> orientation_quat{{0.0, 0.0, 0.0, 1.0}};
  std::array<double, 3> target_linear_velocity_m_s{};
  std::array<double, 3> target_angular_velocity_rad_s{};
  double linear_speed_scale{1.0};
  double angular_speed_scale{1.0};
  uint64_t generation{0};
  double timestamp_s{0.0};

  [[nodiscard]] Pose toPose() const {
    return {position_m, orientation_quat};
  }

  static CartesianTarget fromPose(const Pose &p) {
    CartesianTarget target;
    target.position_m = p.position_m;
    target.orientation_quat = p.orientation_quat;
    return target;
  }
};

// ── Robot State Feedback
// ───────────────────────────────────────────────────────

enum class RobotMode : int {
  AUTO = 0,
  MANUAL = 1,
  UNKNOWN = -1
};

struct RobotState {
  std::array<double, kJointCount> joint_position_rad{};
  std::array<double, kJointCount> joint_velocity_rad_s{};
  std::array<double, kJointCount> joint_acceleration_rad_s2{};
  std::array<double, kJointCount> joint_effort_Nm{};

  std::array<double, 3> tcp_position_m{};
  std::array<double, 4> tcp_orientation_quat{{0.0, 0.0, 0.0, 1.0}};
  // Twist: [vx, vy, vz, wx, wy, wz]
  std::array<double, 6> tcp_twist{};
  // Wrench: [Fx, Fy, Fz, Tx, Ty, Tz]
  std::array<double, 6> tcp_wrench{};

  double timestamp_s{0.0};
  bool valid{false};
  // Source sample identity, not the number of reads of the backend cache.
  uint64_t feedback_sequence{0};
  double received_steady_s{0.0};
  double feedback_age_s{0.0};
  bool safety_known{false};
  uint64_t sdk_read_errors{0};
  uint64_t sdk_write_errors{0};
  bool emergency_stop{false};
  bool collision_detected{false};
  int robot_mode{0}; // 0 = Auto, 1 = Manual (see RobotMode)
  int main_error_code{0};
  int sub_error_code{0};
};

// ── Low-Level Output Command
// ───────────────────────────────────────────────────

struct MotionCommand {
  // Pose execution is deliberately represented by these joint fields too.
  CommandType type{CommandType::NONE};
  std::array<double, kJointCount> joint_position_rad{};
  std::array<double, kJointCount> joint_velocity_rad_s{};
  std::array<double, kJointCount> joint_acceleration_rad_s2{};
  std::array<double, kJointCount> joint_effort_Nm{};

  double period_s{kDefaultControlPeriodS};
  uint64_t sequence{0};
  double timestamp_s{0.0};
};

// ── Motion Controller Status
// ───────────────────────────────────────────────────

enum class MotionState : int {
  IDLE = 0,
  TRACKING,
  STOPPING,
  HOLDING,
  FAULT,
  STOPPED
};

inline const char *motionStateName(MotionState state) {
  switch (state) {
  case MotionState::IDLE:
    return "IDLE";
  case MotionState::TRACKING:
    return "TRACKING";
  case MotionState::STOPPING:
    return "STOPPING";
  case MotionState::HOLDING:
    return "HOLDING";
  case MotionState::FAULT:
    return "FAULT";
  case MotionState::STOPPED:
    return "STOPPED";
  }
  return "UNKNOWN";
}

enum class MotionOutcome : int { NONE = 0, GOAL_REACHED, STOPPED_BEFORE_GOAL };

inline const char *motionOutcomeName(MotionOutcome outcome) {
  switch (outcome) {
  case MotionOutcome::NONE:
    return "NONE";
  case MotionOutcome::GOAL_REACHED:
    return "GOAL_REACHED";
  case MotionOutcome::STOPPED_BEFORE_GOAL:
    return "STOPPED_BEFORE_GOAL";
  }
  return "UNKNOWN";
}

struct ReferenceLagEvent {
  uint64_t sequence{0};
  uint64_t feedback_sequence{0};
  std::size_t joint{0};
  double error_rad{0.0};
  double feedback_age_s{0.0};
  MotionCommand sent_reference;
  RobotState feedback;
};

struct MotionStatus {
  MotionState state{MotionState::IDLE};
  MotionOwner owner{MotionOwner::NONE};
  CommandType active_type{CommandType::NONE};
  uint64_t active_revision{0};
  uint64_t active_generation{0};
  double position_error_m{0.0};
  double angular_error_rad{0.0};
  std::array<double, kJointCount> joint_error_rad{};
  double max_reference_actual_error_rad{0.0};
  uint32_t reference_lag_samples{0};
  double control_period_s{0.0};
  bool reference_lagging{false};
  bool in_goal_tolerance{false};
  MotionOutcome outcome{MotionOutcome::NONE};
  ReferenceLagEvent lag_event;
  double candidate_reference_error_rad{0.0};
  bool cartesian_reference_valid{false};
  Pose desired_cartesian_pose;
  std::array<double, 3> desired_cartesian_translation_tangent{};
  Pose cartesian_path_start;
  Pose cartesian_path_goal;
  double cartesian_path_parameter{0.0};
  double cartesian_path_deviation_m{0.0};
};

// ── Kinematic & Dynamics Limits
// ────────────────────────────────────────────────

struct KinematicLimits {
  std::array<double, kJointCount> min_joint_position_rad{
      {-std::numeric_limits<double>::infinity(),
       -std::numeric_limits<double>::infinity(),
       -std::numeric_limits<double>::infinity(),
       -std::numeric_limits<double>::infinity(),
       -std::numeric_limits<double>::infinity(),
       -std::numeric_limits<double>::infinity()}};
  std::array<double, kJointCount> max_joint_position_rad{
      {std::numeric_limits<double>::infinity(),
       std::numeric_limits<double>::infinity(),
       std::numeric_limits<double>::infinity(),
       std::numeric_limits<double>::infinity(),
       std::numeric_limits<double>::infinity(),
       std::numeric_limits<double>::infinity()}};
  std::array<double, kJointCount> max_joint_velocity_rad_s{};
  std::array<double, kJointCount> max_joint_acceleration_rad_s2{};
  std::array<double, kJointCount> max_joint_jerk_rad_s3{};

  double max_linear_velocity_m_s{0.5};
  double max_linear_acceleration_m_s2{2.0};
  double max_linear_jerk_m_s3{10.0};

  double max_angular_velocity_rad_s{1.0};
  double max_angular_acceleration_rad_s2{4.0};
  double max_angular_jerk_rad_s3{20.0};
};

} // namespace fr10_motion
