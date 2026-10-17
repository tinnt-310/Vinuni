#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "fr10_force_control/wrench_compensation.hpp"
#include "fr10_force_control/wrench_filter.hpp"
#include "fr10_force_control/wrench_tare.hpp"
#include "fr10_insertion_manager/force_session.hpp"
#include "fr10_insertion_manager/insertion_geometry_monitor.hpp"
#include "fr10_motion_interfaces/action/insert.hpp"
#include "fr10_motion_interfaces/msg/wrench_source_status.hpp"
#include "fr10_motion_interfaces/srv/tare_wrench.hpp"

namespace fr10_insertion {

struct WrenchAdapterConfig {
  std::string wrench_topic{"/ft_sensor/wrench"};
  std::string source_status_topic{"/ft_sensor/source_status"};
  bool require_source_status{false};
  std::string source_frame{"charging_tcp"};
  std::string target_frame{"charging_tcp"};
  fr10_force::RigidTransform target_from_source;
  fr10_force::WrenchCompensationConfig compensation;
  fr10_force::WrenchFilterConfig filter;
  fr10_force::WrenchTareConfig tare;
  double tare_max_joint_velocity_rad_s{0.005};
  double tare_pose_tolerance_m{0.0005};
  double tare_orientation_tolerance_rad{0.002};
  double lateral_tolerance_m{0.002};
  double orientation_tolerance_rad{0.0174532925199433};
  double mating_clearance_m{0.0};
  double mating_feature_lever_m{0.0};
};

class InsertionRosAdapter {
public:
  using Insert = fr10_motion_interfaces::action::Insert;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Insert>;
  using AcceptGuard = std::function<bool()>;

  struct PendingGoal {
    InsertionGoal goal;
    fr10_force::CartesianPose final_pose;
    bool preinsert_alignment{true};
    int64_t measurement_ns{0};
    std::optional<fr10_motion::Pose> task_final_pose; // Immutable once anchored.
    fr10_force::Wrench desired_wrench;
    fr10_force::Vector3 max_force_n;
    fr10_force::Vector3 max_torque_nm;
    std::string profile_id;
  };

  struct RuntimeInput {
    fr10_motion::KinematicState joint_seed;
    // IK continuity seed may be commanded; tare must use measured velocity.
    std::optional<fr10_force::Vector6> measured_joint_velocity_rad_s;
    fr10_force::CartesianPose actual_pose;
    bool actual_pose_valid{false};
    uint64_t current_scene_revision{0};
    bool allow_motion_proposal{true};
    bool preinsert_reached{false};
    bool planning_failed{false};
    bool stop_complete{false};
    bool retract_complete{false};
    bool collision_free{true};
    bool tracking_valid{true};
    bool robot_ok{true};
    bool base_stationary{true};
  };

  InsertionRosAdapter(rclcpp::Node *node, ForceSessionConfig base_config,
                      std::string base_frame,
                      WrenchAdapterConfig wrench_adapter,
                      std::string profile_id, bool enabled,
                      AcceptGuard accept_guard);

  bool busy() const;
  // Thread-safe display snapshot; nullptr when no force session owns motion.
  // Returned labels have static storage. Never reads the control-owned FSM.
  const char *diagnosticPhase() const;
  bool submitPoseGoal(const Insert::Goal &goal);

  bool freshUniqueWrench() const;
  bool requiresFreshWrenchForResume() const;
  std::optional<PendingGoal> takePendingGoal();
  bool startSession(const PendingGoal &goal,
                    const fr10_force::Vector3 &axis_base,
                    uint64_t generation, uint64_t scene_revision,
                    double source_time_s);
  bool reframePreinsert(const PendingGoal &goal, uint64_t scene_revision);
  void rejectPending(uint16_t code, const std::string &message);

  ForceSessionOutput update(const RuntimeInput &input,
                            fr10_motion::IKProvider &provider);
  bool commitOnlineTarget(uint64_t sequence,
                          bool accepted_and_published);
  bool canCommitOnlineTarget(uint64_t sequence) const;
  void discardOnlineTarget(uint64_t sequence);
  void requestCancel();
  bool requestHoldRetract();
  bool cancelRequested() const { return cancel_requested_.load(std::memory_order_acquire); }

  uint64_t generation() const { return generation_; }
  InsertionState state() const;
  bool retractRequested() const;
  const PendingGoal &activeGoal() const { return active_goal_; }
  const fr10_force::CartesianPose &preinsertPose() const {
    return preinsert_pose_;
  }
  const fr10_force::CartesianPose &retractPose() const { return retract_pose_; }

private:
  struct WrenchBuffer {
    fr10_force::WrenchSample sample;
    std::string frame_id;
  };

  rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const Insert::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
      const std::shared_ptr<GoalHandle> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);
  void enqueueGoal(const Insert::Goal &goal);

  void wrenchCallback(
      const geometry_msgs::msg::WrenchStamped::ConstSharedPtr &message);
  void tareCallback(
      const std::shared_ptr<fr10_motion_interfaces::srv::TareWrench::Request>
          request,
      std::shared_ptr<fr10_motion_interfaces::srv::TareWrench::Response>
          response);
  void publish(const ForceSessionOutput &output, double source_age_s);
  void complete(const ForceSessionOutput &output,
                const fr10_force::CartesianPose &actual_pose);
  static uint16_t resultCode(const InsertionDecision &decision);
  static double steadyNow();

  rclcpp::Node *node_;
  ForceSessionConfig base_config_;
  std::string base_frame_;
  std::string wrench_source_frame_;
  std::string wrench_frame_;
  std::string insertion_axes_frame_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr insertion_tf_pub_;
  tf2_msgs::msg::TFMessage insertion_tf_message_;
  fr10_force::CartesianPose last_conditioned_tcp_pose_;
  fr10_force::RigidTransform wrench_target_from_source_;
  double lateral_tolerance_m_;
  double orientation_tolerance_rad_;
  double mating_clearance_m_;
  double mating_feature_lever_m_;
  std::string profile_id_;
  bool enabled_{false};
  bool simulation_time_{false};
  AcceptGuard accept_guard_;

  rclcpp_action::Server<Insert>::SharedPtr action_server_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
      wrench_sub_;
  rclcpp::Subscription<fr10_motion_interfaces::msg::WrenchSourceStatus>::SharedPtr driver_status_sub_;
  std::atomic<bool> driver_status_seen_{false};
  bool require_source_status_{false};
  std::atomic<bool> driver_online_{false};
  std::atomic<bool> driver_overload_{false};
  std::atomic<double> driver_status_receipt_s_{0.0};
  uint64_t driver_status_sequence_{0};
  rclcpp::Service<fr10_motion_interfaces::srv::TareWrench>::SharedPtr
      tare_service_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr
      filtered_wrench_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr
      force_error_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      nominal_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      corrected_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      correction_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr contact_state_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_;
  rclcpp::Publisher<fr10_motion_interfaces::msg::WrenchSourceStatus>::SharedPtr
      source_status_pub_;

  mutable std::mutex goal_mutex_;
  std::shared_ptr<GoalHandle> goal_handle_;
  PendingGoal pending_goal_;
  PendingGoal active_goal_;
  bool pending_available_{false};
  bool internal_goal_{false};
  bool admission_reserved_{false};
  rclcpp::Publisher<Insert::Result>::SharedPtr result_pub_;
  rclcpp::Publisher<Insert::Feedback>::SharedPtr feedback_pub_;

  std::atomic<bool> cancel_requested_{false};
  std::atomic<bool> hold_owned_{false};
  std::atomic<bool> hold_retract_requested_{false};
  bool completion_reported_{false};
  std::atomic<bool> tare_requested_{false};
  std::atomic<bool> tare_reset_bias_{false};
  std::atomic<uint64_t> tare_request_generation_{0};
  std::atomic<InsertionState> published_state_{InsertionState::IDLE};

  mutable std::mutex wrench_mutex_;
  WrenchBuffer wrench_writer_;
  // Bounded raw-source batch: anti-alias conditioning before 125 Hz consumption.
  // Validated physics step >=50 us: at most1000 samples per50 ms.
  // Overflow remains a fault; capacity never licenses stale receipt time.
  static constexpr std::size_t kWrenchBatchCapacity = 1024;
  std::array<fr10_force::WrenchSample,kWrenchBatchCapacity> wrench_batch_{};
  std::size_t wrench_batch_size_{0}; // protected by wrench_mutex_
  fr10_force::RawWrenchEnvelope wrench_envelope_; // protected by wrench_mutex_
  uint64_t wrench_sequence_{0};
  int64_t last_wrench_stamp_ns_{-1};
  fr10_force::WrenchSample last_raw_sample_;
  fr10_force::Wrench last_conditioned_wrench_;
  bool have_conditioned_wrench_{false};
  fr10_force::WrenchCompensator compensator_;
  fr10_force::WrenchFilter filter_;
  fr10_force::WrenchTare tare_;
  double tare_max_joint_velocity_rad_s_;
  double tare_pose_tolerance_m_;
  double tare_orientation_tolerance_rad_;
  uint64_t consumed_wrench_sequence_{0};
  double last_conditioned_source_time_s_{0.0};

  std::unique_ptr<ForceSession> session_;
  fr10_force::Vector3 axis_base_{};
  fr10_force::CartesianPose contact_pose_;
  fr10_force::CartesianPose preinsert_pose_;
  fr10_force::CartesianPose retract_pose_;
  std::optional<InsertionCorridor> retract_corridor_;
  uint64_t generation_{0};
  uint64_t scene_revision_{0};
  uint64_t tare_generation_{0};
  bool tare_complete_{false};
  double last_source_time_s_{0.0};
  double peak_force_n_{0.0};
  double peak_torque_nm_{0.0};
  bool previous_command_accepted_{true};
};

} // namespace fr10_insertion
