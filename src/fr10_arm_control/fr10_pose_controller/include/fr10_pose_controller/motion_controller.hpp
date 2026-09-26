#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <utility>

#include "fr10_pose_controller/cartesian_trajectory.hpp"
#include "fr10_pose_controller/core_types.hpp"
#include "fr10_pose_controller/joint_otg.hpp"
#include "fr10_pose_controller/realtime_buffer.hpp"

namespace fr10_motion {

enum class PoseActivationResult : int { ACCEPTED = 0, STALE, REJECTED };

enum class CartesianFaultState : int { NORMAL = 0, WARNING, PERSISTENT_FAULT };

struct CartesianTrackingErrors {
  double position_error_m{0.0};
  double along_track_error_m{0.0};
  double perpendicular_error_m{0.0};
  double orientation_error_rad{0.0};
};

struct CartesianFaultMonitorConfig {
  double endpoint_position_tolerance_m{0.002};
  double tracking_lag_tolerance_m{0.005};
  double perpendicular_tolerance_m{0.002};
  double orientation_tolerance_rad{0.02};
  double persistence_s{0.024};
};

// Debounces orthogonal Cartesian tracking errors outside the 125 Hz executor.
// A violation first enters WARNING and only latches after persistence_s.
class CartesianFaultMonitor {
public:
  explicit CartesianFaultMonitor(
      const CartesianFaultMonitorConfig &config = {});

  CartesianFaultState update(const CartesianTrackingErrors &errors,
                             bool tracking, double dt);
  void reset();
  void setConfig(const CartesianFaultMonitorConfig &config);
  CartesianFaultState state() const { return state_; }
  double violationDuration() const { return violation_duration_s_; }

private:
  CartesianFaultMonitorConfig config_;
  CartesianFaultState state_{CartesianFaultState::NORMAL};
  double violation_duration_s_{0.0};
};

class PreparedCartesianTrajectory {
public:
  const CartesianJointTrajectory &trajectory() const { return *trajectory_; }
  long trajectoryUseCount() const { return trajectory_.use_count(); }

private:
  friend class MotionExecutor;
  explicit PreparedCartesianTrajectory(
      std::shared_ptr<const CartesianJointTrajectory> trajectory)
      : trajectory_(std::move(trajectory)) {}

  std::shared_ptr<const CartesianJointTrajectory> trajectory_;
  std::optional<JointTarget> joint_target_;
};

struct MotionExecutorConfig {
  double control_rate_hz{125.0};
  KinematicLimits limits;
  double joint_goal_tolerance_rad{0.005};
  double max_joint_reference_lead_rad{kDegToRad};
  int reference_lag_persistence_samples{3};
  int goal_stable_samples{5};
  // Production calls acknowledgeCommand only after successful publication.
  // Offline synchronous consumers may leave this false.
  bool require_command_ack{false};
  double activation_position_tolerance_rad{1e-4};
  double activation_velocity_tolerance_rad_s{1e-3};
  double activation_acceleration_tolerance_rad_s2{1e-2};
};

// Common high-rate joint command executor. Joint goals use online Ruckig;
// Pose goals use an already validated, time-parameterized joint buffer.
// Cartesian geometry, IK, and trajectory construction live outside this class.
class MotionExecutor {
public:
  explicit MotionExecutor(const MotionExecutorConfig &config);
  ~MotionExecutor() = default;

  bool submitJointTarget(const JointTarget &target);
  std::shared_ptr<const PreparedCartesianTrajectory> preparePoseTrajectory(
      std::shared_ptr<const CartesianJointTrajectory> trajectory) const;
  std::shared_ptr<const PreparedCartesianTrajectory> prepareJointTrajectory(
      std::shared_ptr<const CartesianJointTrajectory> trajectory,
      const JointTarget &target) const;
  bool submitPreparedPoseTrajectory(
      std::shared_ptr<const PreparedCartesianTrajectory> trajectory);
  PoseActivationResult activatePreparedPoseTrajectory(
      std::shared_ptr<const PreparedCartesianTrajectory> trajectory);
  // Convenience non-real-time API. Production ROS activation uses the two
  // methods above so structural validation never runs in the 125 Hz callback.
  bool submitPoseTrajectory(
      std::shared_ptr<const CartesianJointTrajectory> trajectory);

  MotionStatus update(const RobotState &feedback, double dt);

  MotionCommand command() const;
  bool acknowledgeCommand(uint64_t sequence);
  MotionStatus status() const;
  bool initialized() const;

  void reportCartesianTrackingError(double position_error_m,
                                    double angular_error_rad,
                                    double path_deviation_m,
                                    bool fault);

  void stop();
  bool resetFault();

  void setConfig(const MotionExecutorConfig &config);
  const MotionExecutorConfig &getConfig() const { return config_; }

private:
  enum class ExecutionKind : int {
    NONE = 0,
    JOINT_OTG,
    POSE_TRAJECTORY,
    CONTROLLED_STOP
  };

  struct CartesianTrackingReport {
    double position_error_m{0.0};
    double angular_error_rad{0.0};
    double path_deviation_m{0.0};
    uint64_t sequence{0};
    bool fault{false};
  };

  bool validTarget(const JointTarget &target) const;
  static bool structurallyValidPoseTrajectory(
      const CartesianJointTrajectory &trajectory,
      const MotionExecutorConfig &config);
  double poseActivationStartError(
      const CartesianJointTrajectory &trajectory) const;
  bool startControlledStopLocked();

  mutable std::mutex mutex_;
  MotionExecutorConfig config_;
  std::unique_ptr<JointOtg> joint_otg_;

  MotionOwner active_owner_{MotionOwner::NONE};
  ExecutionKind execution_kind_{ExecutionKind::NONE};
  MotionState state_{MotionState::IDLE};
  JointTarget active_joint_target_;
  uint64_t active_pose_generation_{0};
  std::shared_ptr<const CartesianJointTrajectory> active_pose_trajectory_;
  std::size_t active_pose_point_{0};
  double active_pose_elapsed_s_{0.0};
  bool active_pose_started_{false};

  MotionCommand latest_command_;
  MotionCommand sent_command_;
  ReferenceLagEvent lag_event_;
  uint64_t lag_event_sequence_{0};
  uint64_t last_feedback_sequence_{0};
  MotionOutcome outcome_{MotionOutcome::NONE};
  MotionStatus latest_status_;
  uint64_t command_sequence_{0};
  bool feedback_initialized_{false};
  std::atomic<bool> feedback_initialized_atomic_{false};
  RobotState latest_feedback_;
  int goal_tolerance_samples_{0};
  uint32_t reference_lag_samples_{0};
  RealtimeBuffer<CartesianTrackingReport> cartesian_tracking_buffer_;
  std::atomic<uint64_t> cartesian_tracking_sequence_{0};
  uint64_t consumed_cartesian_tracking_sequence_{0};
};

using MotionControllerConfig = MotionExecutorConfig;
using MotionController = MotionExecutor;

} // namespace fr10_motion
