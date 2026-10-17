#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <unordered_map>
#include <moveit_msgs/msg/collision_object.hpp>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "fr10_motion_core/cartesian_path.hpp"
#include "fr10_motion_core/cartesian_route_planner.hpp"
#include "fr10_motion_core/cartesian_tracking.hpp"
#include "fr10_motion_core/cartesian_trajectory.hpp"
#include "fr10_motion_planning/collision_validated_ik_provider.hpp"
#include "fr10_motion_core/continuous_ik.hpp"
#include "fr10_motion_core/diagnostic_display.hpp"
#include "fr10_motion_core/fr10_analytical_ik.hpp"
#include "fr10_insertion_manager/insertion_ros_adapter.hpp"
#include "fr10_insertion_manager/parking_ros_adapter.hpp"
#include "fr10_motion_core/insertion_sequence_state.hpp"
#include "fr10_motion_core/stop_reason_policy.hpp"
#include "fr10_motion_core/joint_target_decoder.hpp"
#include "fr10_motion_core/linear_insert_planner.hpp"
#include "fr10_motion_core/math_utils.hpp"
#include "fr10_motion_core/motion_controller.hpp"
#include "fr10_motion_core/motion_coordinator.hpp"
#include "fr10_motion_core/tick_clock.hpp"
#include <fr10_motion_interfaces/msg/motion_command_status.hpp>
#include "fr10_pose_controller/motion_adapter_runtime.hpp"
#include "fr10_pose_controller/force_runtime_config.hpp"
#include "fr10_motion_planning/moveit_ik_provider.hpp"
#include "fr10_pose_controller/node_parameters.hpp"
#include "fr10_motion_core/pre_insert_planner.hpp"
#include "fr10_motion_core/realtime_buffer.hpp"
#include "fr10_motion_core/robot_backend.hpp"
#include "fr10_pose_controller/ros_control_backend.hpp"
#include "fr10_motion_core/trajectory_trace_validator.hpp"

namespace fr10_pose_controller::detail {
using namespace std::chrono_literals;

inline std::size_t policyIndex(fr10_motion::PoseExecutionPolicy policy) {
  return static_cast<std::size_t>(policy);
}

using fr10_motion::StopReason;

inline const char *stopReasonName(StopReason reason) {
  switch (reason) {
  case StopReason::NONE:
    return "NONE";
  case StopReason::USER_INTERRUPT:
    return "USER_INTERRUPT";
  case StopReason::GOAL_REACHED:
    return "GOAL_REACHED";
  case StopReason::SINGULARITY:
    return "SINGULARITY";
  case StopReason::HARDWARE_ALARM:
    return "HARDWARE_ALARM";
  case StopReason::COMM_FAILURE:
    return "COMM_FAILURE";
  case StopReason::IK_FAILURE:
    return "IK_FAILURE";
  case StopReason::PATH_DEVIATION:
    return "PATH_DEVIATION";
  case StopReason::REFERENCE_LAG:
    return "REFERENCE_LAG";
  case StopReason::SCENE_CHANGED:
    return "SCENE_CHANGED";
  case StopReason::CONTROLLED_STOP:
    return "CONTROLLED_STOP";
  case StopReason::RETARGET_TIMING:
    return "RETARGET_TIMING";
  case StopReason::STALE_TRAJECTORY:
    return "STALE_TRAJECTORY";
  }
  return "UNKNOWN";
}

class MotionAdapter final : public rclcpp::Node {
  struct KinematicsTask {
    fr10_motion::ControlMode control_mode{fr10_motion::ControlMode::JOINT};
    fr10_motion::JointTarget joint_target;
    fr10_motion::CartesianTarget pose_target;
    fr10_motion::RobotState seed;
    fr10_motion::PoseExecutionPolicy policy{
        fr10_motion::PoseExecutionPolicy::FREE_SPACE};
    uint64_t sequence{0};
    uint64_t command_id{0};
    fr10_motion::PlanningBoundary boundary;
    unsigned replan_attempt{0};
    unsigned timing_stop_attempts{0};
    std::optional<fr10_insertion::InsertionRosAdapter::PendingGoal> force_goal;
  };

  struct KinematicsResult {
    fr10_motion::ControlMode control_mode{fr10_motion::ControlMode::JOINT};
    fr10_motion::JointTarget joint_target;
    fr10_motion::CartesianTarget pose_target;
    fr10_motion::PoseExecutionPolicy policy{
        fr10_motion::PoseExecutionPolicy::FREE_SPACE};
    uint64_t sequence{0};
    uint64_t command_id{0};
    fr10_motion::PlanningBoundary boundary;
    unsigned replan_attempt{0};
    unsigned timing_stop_attempts{0};
    uint64_t scene_revision{0};
    double manipulability{0.0};
    double condition_number{std::numeric_limits<double>::infinity()};
    std::size_t ik_waypoint_count{0};
    std::shared_ptr<const fr10_motion::PreparedCartesianTrajectory>
        pose_trajectory;
    double trajectory_duration_s{0.0};
    double planned_max_position_error_m{0.0};
    double planned_max_orientation_error_rad{0.0};
    double planned_max_path_deviation_m{0.0};
    bool used_kinematic_detour{false};
    bool used_alternate_orientation{false};
    bool used_joint_space_detour{false};
    double detour_clearance_m{0.0};
    fr10_motion::PathTimeDiagnostics timing_diagnostics;
    bool requires_controlled_stop_replan{false};
    bool force_corridor_validated{false};
    bool force_planning_failed{false};
    std::optional<fr10_insertion::InsertionRosAdapter::PendingGoal> force_goal;
  };

  struct LoopMetrics {
    uint64_t samples{0};
    double elapsed_s{0.0};
    double mean_abs_jitter_ms{0.0};
    double max_abs_jitter_ms{0.0};
    double max_cycle_period_ms{0.0};
    double max_tick_execution_ms{0.0};
    double max_feedback_age_ms{0.0};
    uint64_t tick_deadline_misses{0};
    uint64_t scheduler_gaps{0};
    uint64_t max_consecutive_tick_misses{0};
    // Fixed 0.5 ms bins; overflow is retained in the last bin.
    std::array<uint32_t, 128> period_histogram{};
    std::array<uint32_t, 128> execution_histogram{};
    double max_delta_joint_velocity_rad_s{0.0};
    double max_delta_linear_velocity_m_s{0.0};
    uint64_t command_write_errors{0};
  };

  struct DiagnosticsSnapshot {
    fr10_motion::MotionStatus status;
    fr10_motion::RobotState feedback;
    fr10_motion::MotionCommand command;
    LoopMetrics loop;
  };

  struct StatusLogSnapshot {
    fr10_motion::MotionStatus status;
    fr10_motion::RobotState feedback;
    fr10_motion::MotionCommand command;
    fr10_motion::Pose actual_flange;
    fr10_motion::Pose final_pose_goal;
    LoopMetrics loop;
    bool actual_flange_valid{false};
    bool final_pose_goal_valid{false};
    const char *force_phase{nullptr};
  };

  struct AcceptedTargetSnapshot {
    fr10_motion::JointTarget joint_target;
    fr10_motion::CartesianTarget cartesian_target;
    bool have_joint_target{false};
    bool have_cartesian_target{false};
  };

  struct DisplayPoseGoalSnapshot {
    fr10_motion::Pose pose;
    bool valid{false};
  };

  struct ForceTickSubmission {
    uint64_t online_sequence{0};
    bool accepted{false};
  };

public:
  explicit MotionAdapter(const rclcpp::NodeOptions &options);

  ~MotionAdapter() override;

  void initialize();

  void requestStopAll(StopReason reason = StopReason::USER_INTERRUPT);

  void cleanupWorkers();

  bool stopRequested() const;

  void beginGracefulShutdown();

  double shutdownTimeoutS() const;

  bool shutdownBackend();

  void requestControlledStop(StopReason reason);

  bool feedbackSnapshot(fr10_motion::RobotState &snapshot) const;

  void controlTick();

  void pathMonitorTick();

  void initializeKinematics();

  void queueKinematicsTask(const KinematicsTask &task);
  void seedPlanningBoundary(KinematicsTask &task);

  void clearRejectedStagingGeneration(uint64_t generation);

  void logTimeParameterDiagnostics(
      uint64_t generation,
      const fr10_motion::PathTimeParameterizationResult &result) const;

  bool publishKinematicsResult(const KinematicsResult &result);

  void kinematicsWorkerLoop();

  void consumeKinematicsResult();

  void queuePostStopReplanIfReady(const fr10_motion::MotionStatus &status,
                                  const fr10_motion::RobotState &feedback,
                                  const fr10_motion::MotionCommand &command);

  void controlledStopCallback(
      const std_msgs::msg::Empty::ConstSharedPtr & /*message*/);

  void jointTargetCallback(
      const sensor_msgs::msg::JointState::ConstSharedPtr &message);

  void poseTargetCallback(
      const geometry_msgs::msg::PoseStamped::ConstSharedPtr &message);

  void freePoseTargetCallback(
      const geometry_msgs::msg::PoseStamped::ConstSharedPtr &message);

  void publishCommandEvents();

  std::optional<uint64_t> queuePoseTarget(
      const fr10_motion::Pose &pose,
      fr10_motion::PoseExecutionPolicy policy,
      bool preserve_orientation = false,
      std::optional<fr10_insertion::InsertionRosAdapter::PendingGoal> force_goal = std::nullopt);

  bool resolveForceTaskFrame(KinematicsTask &task,
                            planning_scene::PlanningSceneConstPtr &scene);
  bool validateForceCorridor(const KinematicsTask &task,
                            const fr10_motion::KinematicState &preinsert_seed,
                            const fr10_motion::CartesianRoutePlanner::CancelCheck &cancel = {});

  bool
  queueStrictInsertionTarget(fr10_motion::CartesianTarget target,
                             const fr10_motion::Pose &expected_path_start,
                             const fr10_motion::Pose &expected_path_goal,
                             const fr10_motion::RobotState &seed,
                             const fr10_motion::MotionCommand &live_reference);

  void
  queueInsertionAfterStagingIfReady(const fr10_motion::MotionStatus &status,
                                    const fr10_motion::RobotState &feedback,
                                    const fr10_motion::MotionCommand &command);

  void
  retractCallback(const std_msgs::msg::Empty::ConstSharedPtr & /*message*/);

  void initializeDiagnosticFrames();

  bool populateCartesianFeedback(fr10_motion::RobotState &feedback,
                                 fr10_motion::Pose &actual_flange) const;

  void updateContinuityMetrics(const fr10_motion::MotionCommand &command);

  void publishTcpPose(const fr10_motion::RobotState &feedback);

  void diagnosticsTick();

  void publishStatus(const fr10_motion::MotionStatus &status,
                     const fr10_motion::RobotState &feedback,
                     const fr10_motion::MotionCommand &command,
                     const LoopMetrics &loop);

  void statusLogTick();

  struct ForceValidationRequest {
    fr10_motion::OnlineJointTarget target;
    fr10_motion::KinematicState seed;
  };
  struct ForceValidationResult {
    fr10_motion::OnlineJointTarget target;
    bool valid{false};
  };
  void validateOnlineForceTarget(const ForceValidationRequest &request);
  bool forceGoalCanStart();

  void consumePendingForceGoal();

  bool forcePreinsertReached(const fr10_motion::MotionStatus &status,
                             const fr10_motion::RobotState &feedback,
                             const fr10_motion::MotionCommand &command);

  ForceTickSubmission updateForceRuntime(
      const fr10_motion::RobotState &feedback,
      const fr10_motion::MotionStatus &status,
      const fr10_motion::MotionCommand &command);

private:
  fr10_pose_controller::NodeParameters params_;
  fr10_pose_controller::ForceRuntimeConfig force_runtime_config_;
  std::unique_ptr<fr10_motion::MotionExecutor> motion_executor_;
  std::unique_ptr<fr10_insertion::InsertionRosAdapter> force_adapter_;
  std::atomic<uint64_t> force_generation_{0};
  uint64_t force_staging_generation_{0};
  uint64_t force_retract_generation_{0};
  bool force_preinsert_reached_{false};
  bool force_alignment_queued_{false};
  std::mutex task_geometry_mutex_;
  std::unordered_map<std::string,moveit_msgs::msg::CollisionObject> task_geometry_;
  rclcpp::Subscription<moveit_msgs::msg::CollisionObject>::SharedPtr task_geometry_sub_;
  bool force_corridor_validated_{false};
  bool force_validation_pending_{false};
  uint64_t pending_force_validation_sequence_{0};
  double pending_force_validation_deadline_s_{0.};
  uint64_t consumed_force_validation_sequence_{0};
  std::optional<ForceValidationRequest> pending_force_validation_;
  fr10_motion::RealtimeBuffer<ForceValidationResult> force_validation_result_;
  planning_scene::PlanningSceneConstPtr force_validation_scene_; // worker owned

  bool force_retract_queued_{false};
  bool force_retract_requested_last_tick_{false};
  std::atomic<uint64_t> failed_planning_generation_{0};
  std::unique_ptr<fr10_insertion::ParkingRosAdapter> force_parking_;
  std::optional<fr10_insertion::InsertionRosAdapter::PendingGoal> force_admission_goal_;
  double force_admission_source_s_{0};
  bool force_requires_base_feedback_{false};
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr force_joint_trace_pub_;
  std_msgs::msg::Float64MultiArray force_joint_trace_message_;
  double last_force_tick_clock_s_{0.0};
  double force_resume_steady_s_{0.0};
  std::unique_ptr<fr10_motion::RobotBackend> backend_;
  std::atomic<bool> backend_shutdown_{false};
  std::atomic<bool> backend_ready_announced_{false};
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> shutdown_in_progress_{false};
  std::atomic<StopReason> stop_reason_{StopReason::NONE};
  std::atomic<StopReason> pending_event_{StopReason::NONE};
  std::atomic<StopReason> stop_latch_{StopReason::NONE};
  std::atomic<uint64_t> path_stop_generation_{0};
  uint64_t handled_lag_event_{0};
  uint64_t reported_lag_event_{0};
  std::atomic<fr10_motion::ControlMode> active_control_mode_{
      fr10_motion::ControlMode::JOINT};
  std::atomic<fr10_motion::PoseExecutionPolicy> active_pose_policy_{
      fr10_motion::PoseExecutionPolicy::FREE_SPACE};
  std::atomic<uint64_t> scene_revision_{1};
  std::atomic<uint64_t> active_scene_revision_{0};

  std::string last_status_log_key_;
  fr10_motion::CartesianFaultMonitor path_fault_monitor_;
  fr10_motion::PoseExecutionPolicy monitored_pose_policy_{
      fr10_motion::PoseExecutionPolicy::FREE_SPACE};
  bool path_monitor_policy_initialized_{false};

  fr10_motion::RealtimeBuffer<fr10_motion::RobotState> feedback_buffer_;
  fr10_motion::RealtimeBuffer<fr10_motion::Pose> tcp_feedback_buffer_;
  fr10_motion::RealtimeBuffer<fr10_motion::MotionCommand>
      latest_command_buffer_;
  fr10_motion::Pose flange_to_tcp_;

  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  const moveit::core::JointModelGroup *joint_model_group_{nullptr};
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  std::unique_ptr<fr10_motion_planning::MoveItIKProvider> moveit_ik_provider_;
  std::unique_ptr<fr10_motion::Fr10AnalyticalIKProvider>
      analytical_ik_provider_;
  std::unique_ptr<fr10_motion_planning::CollisionValidatedIKProvider>
      collision_validated_ik_provider_;
  fr10_motion::IKProvider *path_ik_provider_{nullptr};

  std::mutex kinematics_mutex_;
  std::condition_variable kinematics_cv_;
  std::optional<KinematicsTask> pending_kinematics_task_;
  std::mutex reversal_mutex_;
  std::optional<KinematicsTask> post_stop_replan_task_;
  fr10_motion::InsertionSequenceState insertion_sequence_state_;
  std::thread kinematics_worker_;
  // Worker-owned retirement: large trajectory destruction never occurs on
  // the tick that replaces or aborts a plan.
  std::vector<std::shared_ptr<const fr10_motion::PreparedCartesianTrajectory>> retained_trajectories_;
  std::atomic<uint64_t> kinematics_task_sequence_{0};

  fr10_motion::RealtimeBuffer<KinematicsResult> kinematics_result_buffer_;
  uint64_t consumed_kinematics_result_sequence_{0};
  AcceptedTargetSnapshot accepted_target_writer_;
  fr10_motion::RealtimeBuffer<AcceptedTargetSnapshot> accepted_target_buffer_;
  fr10_motion::RealtimeBuffer<DisplayPoseGoalSnapshot>
      display_pose_goal_buffer_;

  std::unique_ptr<fr10_motion::MotionCoordinator> coordinator_;
  fr10_motion::TickClock tick_clock_;
  std::atomic<uint32_t> planning_horizon_cycles_{63};
  double last_logic_source_s_{0.0};
  bool simulation_paused_{false};

  fr10_motion::RealtimeBuffer<DiagnosticsSnapshot> diagnostics_buffer_;
  fr10_motion::RealtimeBuffer<StatusLogSnapshot> status_log_buffer_;

  fr10_motion::MotionState previous_reported_state_{
      fr10_motion::MotionState::IDLE};

  int consecutive_feedback_failures_{0};
  uint64_t command_write_errors_{0};
  fr10_motion::MotionCommand previous_command_;
  bool have_previous_command_{false};
  double max_delta_joint_velocity_rad_s_{0.0};
  double max_delta_linear_velocity_m_s_{0.0};
  uint64_t loop_samples_{0};
  double loop_elapsed_s_{0.0};
  double loop_abs_jitter_sum_ms_{0.0};
  double max_abs_jitter_ms_{0.0};
  double max_cycle_period_ms_{0.0};
  double max_tick_execution_ms_{0.0};
  double max_feedback_age_ms_{0.0};
  uint64_t tick_deadline_misses_{0};
  uint64_t scheduler_gaps_{0};
  uint64_t consecutive_tick_misses_{0};
  uint64_t max_consecutive_tick_misses_{0};
  std::array<uint32_t, 128> period_histogram_{};
  std::array<uint32_t, 128> execution_histogram_{};
  std::chrono::steady_clock::time_point last_tick_{};

  rclcpp::CallbackGroup::SharedPtr control_callback_group_;
  rclcpp::CallbackGroup::SharedPtr target_callback_group_;
  rclcpp::CallbackGroup::SharedPtr diagnostics_callback_group_;
  rclcpp::CallbackGroup::SharedPtr path_monitor_callback_group_;
  rclcpp::CallbackGroup::SharedPtr feedback_callback_group_;
  rclcpp::CallbackGroup::SharedPtr stop_callback_group_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_guard_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<fr10_motion_interfaces::msg::MotionCommandStatus>::SharedPtr command_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      tracking_diagnostics_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tcp_pose_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr free_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr retract_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr controlled_stop_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr path_monitor_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr status_log_timer_;
  geometry_msgs::msg::PoseStamped tcp_pose_message_;
  std_msgs::msg::String status_message_;
  std::chrono::steady_clock::time_point path_monitor_last_tick_{};
};

} // namespace fr10_pose_controller::detail
