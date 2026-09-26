#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "fr10_pose_controller/core_types.hpp"
#include "fr10_pose_controller/realtime_buffer.hpp"
#include "fr10_pose_controller/robot_backend.hpp"

namespace fr10_motion
{

class RosControlBackend : public RobotBackend
{
public:
  RosControlBackend(
    const rclcpp::Node::SharedPtr & node,
    const std::vector<std::string> & joint_names,
    const std::string & command_topic = "/fairino10_controller/commands",
    const std::string & joint_state_topic = "/joint_states",
    const rclcpp::CallbackGroup::SharedPtr & callback_group = nullptr,
    double feedback_timeout_s = 0.050,
    bool require_driver_health = true);

  ~RosControlBackend() override = default;

  bool initialize() override;
  bool shutdown() override;
  bool isReady() const override;

  bool writeCommand(const MotionCommand & command) override;
  bool readState(RobotState & state) override;

  const char * name() const override { return "RosControlBackend"; }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr & message);

  rclcpp::Node::SharedPtr node_;
  std::vector<std::string> joint_names_;
  std::string command_topic_;
  std::string joint_state_topic_;
  double feedback_timeout_s_;
  bool require_driver_health_;
  struct DriverHealth {
    std::array<double, 16> data{};
    double received_s{0.0};
  };
  RealtimeBuffer<DriverHealth> health_buffer_;
  uint64_t last_health_sequence_{0};
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr health_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  RealtimeBuffer<RobotState> state_buffer_;
  RobotState writer_state_;
  bool have_joint_state_{false};
  std::atomic<bool> active_{false};
  std::atomic<bool> ready_{false};
  std::array<std::size_t, kJointCount> joint_state_index_map_{};
  std::vector<std::string> joint_state_layout_;
  bool joint_state_index_map_valid_{false};
  std_msgs::msg::Float64MultiArray command_message_;
};

} // namespace fr10_motion
