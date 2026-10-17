#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/empty.hpp>

#include "fr10_motion_controller/trajectory_decoder.hpp"
#include "fr10_motion_core/motion_controller.hpp"
#include "fr10_motion_interfaces/msg/controller_state.hpp"
#include "fr10_motion_interfaces/msg/joint_trajectory_command.hpp"

namespace fr10_motion_controller {

class Fr10MotionController : public controller_interface::ControllerInterface {
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  enum class DetailCode : uint8_t {
    READY = 0,
    TRACKING,
    GOAL_REACHED,
    CONTROLLED_STOP,
    TIMING_VIOLATION,
    STALE_TRAJECTORY,
    EXECUTOR_FAULT,
    DRIVER_ERROR
  };

  struct PendingTrajectory {
    std::shared_ptr<const fr10_motion::PreparedCartesianTrajectory> prepared;
    uint64_t generation{0};
    uint8_t mode{0};
  };

  void trajectoryCallback(
      fr10_motion_interfaces::msg::JointTrajectoryCommand::ConstSharedPtr message);
  void healthCallback(std_msgs::msg::Float64MultiArray::ConstSharedPtr message);
  void publishState();
  bool loadConfig();
  void holdMeasuredPosition();
  static const char *detailText(DetailCode code);

  std::vector<std::string> joint_names_{"j1", "j2", "j3", "j4", "j5", "j6"};
  fr10_motion::MotionExecutorConfig executor_config_;
  double nominal_period_s_{fr10_motion::kDefaultControlPeriodS};
  double maximum_cycle_gap_s_{0.016};
  double state_publish_rate_hz_{20.0};

  std::unique_ptr<fr10_motion::MotionExecutor> executor_;
  // Structural validation is deliberately isolated from the update executor,
  // so a planning callback can never contend for the update mutex.
  std::unique_ptr<fr10_motion::MotionExecutor> preparation_executor_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<const PendingTrajectory>>
      pending_buffer_;
  std::atomic<uint64_t> newest_received_generation_{0};
  uint64_t consumed_generation_{0};
  uint64_t feedback_sequence_{0};
  std::atomic<bool> accepting_commands_{false};
  std::atomic<bool> stop_requested_{false};
  std::array<double, fr10_motion::kJointCount> previous_measured_velocity_{};
  bool have_previous_measured_velocity_{false};

  rclcpp::Subscription<
      fr10_motion_interfaces::msg::JointTrajectoryCommand>::SharedPtr
      trajectory_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr health_sub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_service_;
  rclcpp::Publisher<fr10_motion_interfaces::msg::ControllerState>::SharedPtr
      state_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  std::atomic<uint8_t> state_snapshot_{
      fr10_motion_interfaces::msg::ControllerState::IDLE};
  std::atomic<uint64_t> generation_snapshot_{0};
  std::atomic<uint64_t> command_sequence_snapshot_{0};
  std::atomic<uint64_t> driver_write_sequence_{0};
  std::atomic<uint64_t> driver_success_sequence_{0};
  std::atomic<double> progress_snapshot_{0.0};
  std::array<std::atomic<double>, fr10_motion::kJointCount>
      reference_position_snapshot_{};
  std::array<std::atomic<double>, fr10_motion::kJointCount>
      reference_velocity_snapshot_{};
  std::array<std::atomic<double>, fr10_motion::kJointCount>
      reference_acceleration_snapshot_{};
  std::atomic<double> reference_error_snapshot_{0.0};
  std::atomic<double> cycle_period_snapshot_{0.0};
  std::atomic<double> cycle_jitter_snapshot_{0.0};
  std::atomic<double> driver_gap_snapshot_{0.0};
  std::atomic<int32_t> driver_error_snapshot_{0};
  std::atomic<DetailCode> detail_snapshot_{DetailCode::READY};
};

} // namespace fr10_motion_controller
