#pragma once

#include <atomic>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <fr10_motion_interfaces/msg/vehicle_parking_lease.hpp>
#include <fr10_motion_interfaces/msg/vehicle_parking_state.hpp>
#include "fr10_insertion_manager/base_hold_monitor.hpp"
#include "fr10_motion_core/realtime_buffer.hpp"

namespace fr10_insertion {

// No actuator publisher. The existing vehicle controller arbitrates its own
// drive command under a renewable lease; this adapter only observes feedback.
class ParkingRosAdapter {
public:
  ParkingRosAdapter(rclcpp::Node *node, const std::string &odom_topic,
                    const std::string &feedback_frame = "odom", bool twist_from_pose = false,
                    double pose_window_s = .016);
  ~ParkingRosAdapter();
  void acquire();
  void release(); // Called only after the arm reaches a stationary boundary.
  BaseHoldResult update();
  bool freshFeedback() const;
  bool acquired() const { return owner_id_.load() != 0; }
private:
  struct Status { uint64_t owner{0}, sequence{0}; uint8_t state{0}; double receipt_s{0}; };
  void heartbeat();
  rclcpp::Node *node_;
  uint64_t client_owner_id_{0};
  std::atomic<uint64_t> owner_id_{0};
  uint64_t published_owner_{0}, sequence_{0}; // Timer only.
  int64_t last_odom_source_ns_{0}; // Subscriber only.
  BaseHoldMonitor monitor_; // Control tick only.
  BasePoseVelocity pose_velocity_; // Subscriber only.
  fr10_motion::RealtimeBuffer<BaseFeedback> feedback_;
  fr10_motion::RealtimeBuffer<Status> status_;
  rclcpp::CallbackGroup::SharedPtr feedback_group_;
  rclcpp::executors::SingleThreadedExecutor feedback_executor_;
  std::thread feedback_thread_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<fr10_motion_interfaces::msg::VehicleParkingState>::SharedPtr status_sub_;
  rclcpp::Publisher<fr10_motion_interfaces::msg::VehicleParkingLease>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace fr10_insertion
