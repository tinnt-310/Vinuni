#include "swerve_vehicle_control/swerve_controller_plugin.hpp"

#include <cmath>
#include <algorithm>
#include <limits>
#include <Eigen/Dense>

#include <lifecycle_msgs/msg/state.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace my_ur_control_cpp
{

// ============================================================
// LIFECYCLE: on_init
// ============================================================
controller_interface::CallbackReturn SwerveDriveController::on_init()
{
  try
  {
    // === Declare parameters ===
    auto & node = *get_node();

    node.declare_parameter("wheel_separation_x", 0.80);
    node.declare_parameter("wheel_separation_y", 0.70);
    node.declare_parameter("wheel_radius", 0.210);
    node.declare_parameter("singularity_threshold", 0.08);
    node.declare_parameter("max_steering_speed", 3.0);
    node.declare_parameter("cmd_vel_timeout", 0.5);
    node.declare_parameter("publish_tf", true);
    node.declare_parameter("enable_pid", false);
    node.declare_parameter("steering_pid.kp", 50.0);
    node.declare_parameter("steering_pid.ki", 0.0);
    node.declare_parameter("steering_pid.kd", 5.0);
    node.declare_parameter("odom_covariance.x", 0.03);
    node.declare_parameter("odom_covariance.y", 0.03);
    node.declare_parameter("odom_covariance.yaw", 0.02);

    RCLCPP_INFO(node.get_logger(), "SwerveDriveController: on_init() — parameters declared");
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "SwerveDriveController on_init exception: %s\n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================
// INTERFACE CONFIGURATION: command_interfaces
// ============================================================
controller_interface::InterfaceConfiguration SwerveDriveController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // 4 steering position commands
  config.names.emplace_back("Revolute1_to_base_link/position");
  config.names.emplace_back("Revolute2_to_base_link/position");
  config.names.emplace_back("Revolute3_to_base_link/position");
  config.names.emplace_back("Revolute4_to_base_link/position");

  // 4 drive velocity commands
  config.names.emplace_back("Wheel1_to_Revolute1/velocity");
  config.names.emplace_back("Wheel2_to_Revolute2/velocity");
  config.names.emplace_back("Wheel3_to_Revolute3/velocity");
  config.names.emplace_back("Wheel4_to_Revolute4/velocity");

  return config;
}

// ============================================================
// INTERFACE CONFIGURATION: state_interfaces
// ============================================================
controller_interface::InterfaceConfiguration SwerveDriveController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Steering position feedback (4)
  config.names.emplace_back("Revolute1_to_base_link/position");
  config.names.emplace_back("Revolute2_to_base_link/position");
  config.names.emplace_back("Revolute3_to_base_link/position");
  config.names.emplace_back("Revolute4_to_base_link/position");

  // Steering velocity feedback (4)
  config.names.emplace_back("Revolute1_to_base_link/velocity");
  config.names.emplace_back("Revolute2_to_base_link/velocity");
  config.names.emplace_back("Revolute3_to_base_link/velocity");
  config.names.emplace_back("Revolute4_to_base_link/velocity");

  // Drive velocity feedback (4)
  config.names.emplace_back("Wheel1_to_Revolute1/velocity");
  config.names.emplace_back("Wheel2_to_Revolute2/velocity");
  config.names.emplace_back("Wheel3_to_Revolute3/velocity");
  config.names.emplace_back("Wheel4_to_Revolute4/velocity");

  // Drive position feedback (4, used for odometry wheel travel)
  config.names.emplace_back("Wheel1_to_Revolute1/position");
  config.names.emplace_back("Wheel2_to_Revolute2/position");
  config.names.emplace_back("Wheel3_to_Revolute3/position");
  config.names.emplace_back("Wheel4_to_Revolute4/position");

  return config;
}

// ============================================================
// LIFECYCLE: on_configure
// ============================================================
controller_interface::CallbackReturn SwerveDriveController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto & node = *get_node();

  // === Read parameters ===
  wheel_separation_x_ = node.get_parameter("wheel_separation_x").as_double();
  wheel_separation_y_ = node.get_parameter("wheel_separation_y").as_double();
  wheel_radius_ = node.get_parameter("wheel_radius").as_double();
  singularity_threshold_ = node.get_parameter("singularity_threshold").as_double();
  max_steering_speed_ = node.get_parameter("max_steering_speed").as_double();
  cmd_vel_timeout_ = node.get_parameter("cmd_vel_timeout").as_double();
  publish_tf_ = node.get_parameter("publish_tf").as_bool();
  enable_pid_ = node.get_parameter("enable_pid").as_bool();
  steering_pid_kp_ = node.get_parameter("steering_pid.kp").as_double();
  steering_pid_ki_ = node.get_parameter("steering_pid.ki").as_double();
  steering_pid_kd_ = node.get_parameter("steering_pid.kd").as_double();
  odom_cov_x_ = node.get_parameter("odom_covariance.x").as_double();
  odom_cov_y_ = node.get_parameter("odom_covariance.y").as_double();
  odom_cov_yaw_ = node.get_parameter("odom_covariance.yaw").as_double();

  // === Wheel positions relative to robot center ===
  // Order: FL (0), FR (1), RL (2), RR (3)
  double L2 = wheel_separation_x_ / 2.0;
  double W2 = wheel_separation_y_ / 2.0;
  wheel_positions_[0] = { L2,  W2};  // Front Left
  wheel_positions_[1] = { L2, -W2};  // Front Right
  wheel_positions_[2] = {-L2,  W2};  // Rear Left
  wheel_positions_[3] = {-L2, -W2};  // Rear Right

  // === Create ROS 2 comms ===
  // Subscribe to cmd_vel_nav (from velocity_smoother / Nav2 pipeline)
  cmd_vel_sub_ = node.create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_nav", rclcpp::SystemDefaultsQoS(),
    std::bind(&SwerveDriveController::cmd_vel_callback, this, std::placeholders::_1));

  // Odometry publisher
  odom_pub_ = node.create_publisher<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::SystemDefaultsQoS());

  // TF broadcaster (if enabled)
  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
  }

  RCLCPP_INFO(node.get_logger(),
    "SwerveDriveController configured: L=%.3f W=%.3f r=%.3f steer_max=%.1f PID=%s tf=%s",
    wheel_separation_x_, wheel_separation_y_, wheel_radius_,
    max_steering_speed_,
    enable_pid_ ? "ON" : "OFF",
    publish_tf_ ? "ON" : "OFF");

  return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================
// LIFECYCLE: on_activate
// ============================================================
controller_interface::CallbackReturn SwerveDriveController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Verify we have the correct number of command and state interfaces
  if (command_interfaces_.size() != NUM_CMD) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "SwerveDriveController: expected %zu command interfaces, got %zu",
      NUM_CMD, command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (state_interfaces_.size() != NUM_STATE) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "SwerveDriveController: expected %zu state interfaces, got %zu",
      NUM_STATE, state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Log the interface names for verification
  auto & logger = *get_node();
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    RCLCPP_DEBUG(logger.get_logger(), "  Cmd[%zu] = %s", i,
      command_interfaces_[i].get_name().c_str());
  }
  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    RCLCPP_DEBUG(logger.get_logger(), "  State[%zu] = %s", i,
      state_interfaces_[i].get_name().c_str());
  }

  // Reset odometry
  x_ = 0.0; y_ = 0.0; theta_ = 0.0;
  first_update_ = true;

  // Reset steering history
  prev_steering_angles_.fill(0.0);

  // Reset PID state
  pid_integral_.fill(0.0);
  pid_prev_error_.fill(0.0);

  // Reset cmd_vel
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    last_cmd_.reset();
    new_cmd_received_ = false;
  }

  RCLCPP_INFO(get_node()->get_logger(), "SwerveDriveController activated — odometry reset");
  return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================
// LIFECYCLE: on_deactivate
// ============================================================
controller_interface::CallbackReturn SwerveDriveController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Zero all command interfaces to stop the robot safely
  for (auto & interface : command_interfaces_) {
    interface.set_value(0.0);
  }

  RCLCPP_INFO(get_node()->get_logger(), "SwerveDriveController deactivated — commands zeroed");
  return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================
// LIFECYCLE: on_cleanup
// ============================================================
controller_interface::CallbackReturn SwerveDriveController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  cmd_vel_sub_.reset();
  odom_pub_.reset();
  tf_broadcaster_.reset();

  RCLCPP_INFO(get_node()->get_logger(), "SwerveDriveController cleaned up");
  return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================
// CALLBACK: cmd_vel subscriber
// ============================================================
void SwerveDriveController::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_mutex_);
  last_cmd_ = msg;
  last_cmd_time_ = get_node()->now();
  new_cmd_received_ = true;
}

// ============================================================
// UPDATE: Main control loop (called at controller_manager update_rate)
// ============================================================
controller_interface::return_type SwerveDriveController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  double dt = period.seconds();
  if (dt <= 0.0 || dt > 0.1) {
    dt = 0.01;  // safety clamp
  }

  // --- Step 1: Read latest cmd_vel ---
  double vx_cmd = 0.0, vy_cmd = 0.0, omega_cmd = 0.0;
  bool cmd_timeout = false;

  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    if (last_cmd_) {
      double elapsed = (time - last_cmd_time_).seconds();
      if (elapsed > cmd_vel_timeout_) {
        cmd_timeout = true;
      } else {
        vx_cmd = last_cmd_->linear.x;
        vy_cmd = last_cmd_->linear.y;
        omega_cmd = last_cmd_->angular.z;
      }
    }
  }

  if (cmd_timeout) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "cmd_vel timeout (%.1fs) — stopping robot", cmd_vel_timeout_);
  }

  // --- Step 2: Read actual state interfaces ---
  std::array<double, 4> steer_pos{};   // actual steering angles
  std::array<double, 4> steer_vel{};   // actual steering velocities
  std::array<double, 4> drive_vel{};   // actual drive velocities
  std::array<double, 4> drive_pos{};   // actual drive positions

  for (int i = 0; i < 4; ++i) {
    steer_pos[i] = state_interfaces_[STEER_POS0 + i].get_value();
    steer_vel[i] = state_interfaces_[STEER_VEL0 + i].get_value();
    drive_vel[i] = state_interfaces_[DRIVE_VEL0 + i].get_value();
    drive_pos[i] = state_interfaces_[DRIVE_POS0 + i].get_value();
  }

  // --- Step 3: Inverse Kinematics (cmd_vel → joint targets) ---
  std::array<double, 4> target_steer{};
  std::array<double, 4> target_drive_speed{};

  compute_inverse_kinematics(vx_cmd, vy_cmd, omega_cmd, dt,
    target_steer, target_drive_speed);

  // --- Step 4: Write command interfaces ---
  for (int i = 0; i < 4; ++i) {
    // Steering: position command
    double steer_cmd = target_steer[i];

    // Optional PID servo: setpoint = target_steer, feedback = steer_pos
    if (enable_pid_) {
      steer_cmd = pid_compute(i, target_steer[i], steer_pos[i], dt);
    }

    command_interfaces_[STEER0 + i].set_value(steer_cmd);
    command_interfaces_[DRIVE0 + i].set_value(target_drive_speed[i]);
  }

  // --- Step 5: Forward Kinematics (wheel states → robot velocity) ---
  double vx_actual = 0.0, vy_actual = 0.0, omega_actual = 0.0;
  compute_forward_kinematics(steer_pos, drive_vel, vx_actual, vy_actual, omega_actual);

  // --- Step 6: Odometry Integration ---
  if (first_update_) {
    first_update_ = false;
    last_publish_time_ = time;
  } else {
    double dt_odom = (time - last_publish_time_).seconds();
    if (dt_odom > 0.0) {
      // Dead reckoning using actual wheel velocities
      double delta_x = (vx_actual * std::cos(theta_) - vy_actual * std::sin(theta_)) * dt_odom;
      double delta_y = (vx_actual * std::sin(theta_) + vy_actual * std::cos(theta_)) * dt_odom;
      double delta_th = omega_actual * dt_odom;

      x_ += delta_x;
      y_ += delta_y;
      theta_ += delta_th;
    }
    last_publish_time_ = time;
  }

  // Unwrap theta
  if (theta_ > M_PI) theta_ -= 2.0 * M_PI;
  if (theta_ < -M_PI) theta_ += 2.0 * M_PI;

  // --- Step 7: Publish Odometry ---
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta_);

  auto odom = nav_msgs::msg::Odometry();
  odom.header.stamp = time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";

  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  // Velocity in odometry frame
  odom.twist.twist.linear.x = vx_actual;
  odom.twist.twist.linear.y = vy_actual;
  odom.twist.twist.angular.z = omega_actual;

  // Covariance — tight because we have direct encoder access
  for (int i = 0; i < 36; ++i) odom.pose.covariance[i] = 0.0;
  odom.pose.covariance[0] = odom_cov_x_;
  odom.pose.covariance[7] = odom_cov_y_;
  odom.pose.covariance[35] = odom_cov_yaw_;

  odom_pub_->publish(odom);

  // --- Step 8: Publish TF (if enabled) ---
  if (publish_tf_ && tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = time;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_footprint";

    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
  }

  return controller_interface::return_type::OK;
}

// ============================================================
// INVERSE KINEMATICS
// ============================================================
void SwerveDriveController::compute_inverse_kinematics(
  double vx, double vy, double omega,
  double dt,
  std::array<double, 4> & steer_angles,
  std::array<double, 4> & drive_speeds)
{
  for (int i = 0; i < 4; ++i) {
    double wx = wheel_positions_[i].first;
    double wy = wheel_positions_[i].second;

    // Wheel velocity in robot frame
    double wheel_vx = vx - omega * wy;
    double wheel_vy = vy + omega * wx;

    double target_speed = std::hypot(wheel_vx, wheel_vy);
    double target_angle = std::atan2(wheel_vy, wheel_vx);

    // === Singularity handling ===
    if (target_speed < singularity_threshold_) {
      target_angle = prev_steering_angles_[i];
      target_speed = 0.0;
    } else {
      // === Shortest path optimization ===
      double delta = target_angle - prev_steering_angles_[i];
      delta = std::fmod(delta + M_PI, 2.0 * M_PI);
      if (delta < 0) delta += 2.0 * M_PI;
      delta -= M_PI;

      // If delta > 90°, flip the wheel direction instead
      if (std::abs(delta) > M_PI / 2.0) {
        delta -= (delta > 0) ? M_PI : -M_PI;
        target_speed = -target_speed;
      }

      // === Slew rate limiting ===
      double max_delta = max_steering_speed_ * dt;
      delta = std::clamp(delta, -max_delta, max_delta);

      target_angle = prev_steering_angles_[i] + delta;
    }

    prev_steering_angles_[i] = target_angle;
    steer_angles[i] = target_angle;
    drive_speeds[i] = target_speed / wheel_radius_;
  }
}

// ============================================================
// FORWARD KINEMATICS — Moore-Penrose Pseudoinverse (3×8)
// ============================================================
// Builds the system: A * [vx, vy, omega]^T = b
// Where A is 8×3 (4 wheels × 2 equations each)
//   row 2i:   [1, 0, -wy_i]
//   row 2i+1: [0, 1,  wx_i]
// And b is 8×1:
//   b[2i]   = wvx_i = drive_vel_i * r * cos(steer_i)
//   b[2i+1] = wvy_i = drive_vel_i * r * sin(steer_i)
//
// Solved via: x = (A^T * A)^{-1} * A^T * b
// The 3×3 matrix A^T*A is inverted analytically.
// ============================================================
void SwerveDriveController::compute_forward_kinematics(
  const std::array<double, 4> & steer_angles,
  const std::array<double, 4> & drive_velocities,
  double & vx_out, double & vy_out, double & omega_out)
{
  // Build the 8×3 system using Eigen
  Eigen::Matrix<double, 8, 3> A;
  Eigen::Matrix<double, 8, 1> b;

  for (int i = 0; i < 4; ++i) {
    double wx = wheel_positions_[i].first;
    double wy = wheel_positions_[i].second;

    double wheel_speed = drive_velocities[i] * wheel_radius_;
    double wvx = wheel_speed * std::cos(steer_angles[i]);
    double wvy = wheel_speed * std::sin(steer_angles[i]);

    A(2 * i, 0) = 1.0;
    A(2 * i, 1) = 0.0;
    A(2 * i, 2) = -wy;

    A(2 * i + 1, 0) = 0.0;
    A(2 * i + 1, 1) = 1.0;
    A(2 * i + 1, 2) = wx;

    b(2 * i) = wvx;
    b(2 * i + 1) = wvy;
  }

  // Moore-Penrose: x = (A^T * A)^{-1} * A^T * b
  Eigen::Matrix<double, 3, 8> AT = A.transpose();
  Eigen::Matrix<double, 3, 3> ATA = AT * A;

  // Solve using full-pivoting LU decomposition for numerical stability
  Eigen::Matrix<double, 3, 1> x = ATA.fullPivLu().solve(AT * b);

  vx_out = x(0);
  vy_out = x(1);
  omega_out = x(2);
}

// ============================================================
// PID COMPUTE
// ============================================================
double SwerveDriveController::pid_compute(
  int idx, double setpoint, double actual, double dt)
{
  // Compute shortest-path error
  double raw_error = setpoint - actual;
  raw_error = std::fmod(raw_error + M_PI, 2.0 * M_PI);
  if (raw_error < 0) raw_error += 2.0 * M_PI;
  raw_error -= M_PI;

  pid_integral_[idx] += raw_error * dt;
  // Anti-windup
  pid_integral_[idx] = std::clamp(pid_integral_[idx], -1.0, 1.0);

  double derivative = (raw_error - pid_prev_error_[idx]) / dt;
  pid_prev_error_[idx] = raw_error;

  double output = steering_pid_kp_ * raw_error
                + steering_pid_ki_ * pid_integral_[idx]
                + steering_pid_kd_ * derivative;

  return output;
}

// ============================================================
// UNWRAP ANGLE
// ============================================================
double SwerveDriveController::unwrap_angle(double target, double prev)
{
  double diff = target - prev;
  while (diff > M_PI)  diff -= 2.0 * M_PI;
  while (diff < -M_PI) diff += 2.0 * M_PI;
  return prev + diff;
}

}  // namespace my_ur_control_cpp

// ============================================================
// PLUGINLIB EXPORT
// ============================================================
PLUGINLIB_EXPORT_CLASS(my_ur_control_cpp::SwerveDriveController,
                       controller_interface::ControllerInterface)
