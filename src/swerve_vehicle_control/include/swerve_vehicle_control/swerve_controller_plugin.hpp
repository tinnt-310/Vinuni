#ifndef MY_UR_CONTROL_CPP__SWERVE_CONTROLLER_PLUGIN_HPP_
#define MY_UR_CONTROL_CPP__SWERVE_CONTROLLER_PLUGIN_HPP_

#include <memory>
#include <string>
#include <vector>
#include <array>
#include <atomic>
#include <mutex>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

namespace my_ur_control_cpp
{

// ============================================================
// SwerveDriveController — ros2_control ControllerInterface Plugin
// ============================================================
// Replaces: swerve_drive Node + position_controllers + velocity_controllers
// Benefits:
//   1. Direct memory access to hardware interfaces (no pub/sub latency)
//   2. 100Hz fixed-rate update (via controller_manager)
//   3. Moore-Penrose pseudoinverse forward kinematics (all 4 wheels)
//   4. Optional PID steering servo at hardware rate
//   5. publish_tf parameter for EKF compatibility
// ============================================================

// Command interface indices
enum CmdIdx : size_t
{
  STEER0 = 0,   // Revolute1_to_base_link/position
  STEER1 = 1,   // Revolute2_to_base_link/position
  STEER2 = 2,   // Revolute3_to_base_link/position
  STEER3 = 3,   // Revolute4_to_base_link/position
  DRIVE0 = 4,   // Wheel1_to_Revolute1/velocity
  DRIVE1 = 5,   // Wheel2_to_Revolute2/velocity
  DRIVE2 = 6,   // Wheel3_to_Revolute3/velocity
  DRIVE3 = 7,   // Wheel4_to_Revolute4/velocity
  NUM_CMD = 8
};

// State interface indices
enum StateIdx : size_t
{
  STEER_POS0 = 0,   // Revolute1_to_base_link/position
  STEER_POS1 = 1,   // Revolute2_to_base_link/position
  STEER_POS2 = 2,   // Revolute3_to_base_link/position
  STEER_POS3 = 3,   // Revolute4_to_base_link/position
  STEER_VEL0 = 4,   // Revolute1_to_base_link/velocity
  STEER_VEL1 = 5,   // Revolute2_to_base_link/velocity
  STEER_VEL2 = 6,   // Revolute3_to_base_link/velocity
  STEER_VEL3 = 7,   // Revolute4_to_base_link/velocity
  DRIVE_VEL0 = 8,   // Wheel1_to_Revolute1/velocity
  DRIVE_VEL1 = 9,   // Wheel2_to_Revolute2/velocity
  DRIVE_VEL2 = 10,  // Wheel3_to_Revolute3/velocity
  DRIVE_VEL3 = 11,  // Wheel4_to_Revolute4/velocity
  DRIVE_POS0 = 12,  // Wheel1_to_Revolute1/position
  DRIVE_POS1 = 13,  // Wheel2_to_Revolute2/position
  DRIVE_POS2 = 14,  // Wheel3_to_Revolute3/position
  DRIVE_POS3 = 15,  // Wheel4_to_Revolute4/position
  NUM_STATE = 16
};

class SwerveDriveController : public controller_interface::ControllerInterface
{
public:
  SwerveDriveController() = default;
  ~SwerveDriveController() override = default;

  // ControllerInterface lifecycle overrides
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // === Parameters ===
  double wheel_separation_x_;   // Wheelbase (m)
  double wheel_separation_y_;   // Track width (m)
  double wheel_radius_;         // Wheel radius (m)
  double singularity_threshold_;
  double max_steering_speed_;
  double cmd_vel_timeout_;
  bool publish_tf_;
  bool enable_pid_;
  double steering_pid_kp_, steering_pid_ki_, steering_pid_kd_;
  double odom_cov_x_, odom_cov_y_, odom_cov_yaw_;

  // === Robot geometry ===
  // Wheel positions relative to robot center: {x, y}
  std::array<std::pair<double, double>, 4> wheel_positions_;

  // === Odometry state ===
  double x_{0.0}, y_{0.0}, theta_{0.0};
  rclcpp::Time last_publish_time_;
  bool first_update_{true};

  // === Steering state ===
  std::array<double, 4> prev_steering_angles_{};

  // === Latest cmd_vel (written by sub callback, read by update) ===
  std::mutex cmd_mutex_;
  geometry_msgs::msg::Twist::SharedPtr last_cmd_;
  rclcpp::Time last_cmd_time_;
  std::atomic<bool> new_cmd_received_{false};

  // === PID state ===
  std::array<double, 4> pid_integral_{};
  std::array<double, 4> pid_prev_error_{};

  // === ROS 2 communication ===
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // === Helper methods ===
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /// Compute inverse kinematics: cmd_vel → steering angles + drive velocities
  void compute_inverse_kinematics(
    double vx, double vy, double omega,
    double dt,
    std::array<double, 4> & steer_angles,
    std::array<double, 4> & drive_speeds);

  /// Compute forward kinematics via Moore-Penrose pseudoinverse
  /// Uses all 4 wheel states for least-squares vx/vy/omega estimate
  void compute_forward_kinematics(
    const std::array<double, 4> & steer_angles,
    const std::array<double, 4> & drive_velocities,
    double & vx_out, double & vy_out, double & omega_out);

  /// PID compute for a single steering servo
  double pid_compute(int idx, double setpoint, double actual, double dt);

  /// Unwrap angle to shortest path
  static double unwrap_angle(double target, double prev);
};

}  // namespace my_ur_control_cpp

#endif  // MY_UR_CONTROL_CPP__SWERVE_CONTROLLER_PLUGIN_HPP_
