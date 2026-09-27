#include "swerve_vehicle_control/swerve_controller_plugin.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <pluginlib/class_list_macros.hpp>

namespace swerve_vehicle_control
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
// Match the position command-interface bounds in config/ros2_control.xacro.
constexpr double kSteerMin = -3.14;
constexpr double kSteerMax = 3.14;
// The interface rounds pi to 3.14; accept that 1.6 mrad endpoint approximation.
constexpr double kSteerLimitTolerance = 0.002;
constexpr double kMaxPeriod = 0.1;
constexpr double kUnobservedVariance = 1.0e6;
}  // namespace

controller_interface::CallbackReturn SwerveDriveController::on_init()
{
  try {
    auto & node = *get_node();
    node.declare_parameter("wheel_separation_x", 0.80);
    node.declare_parameter("wheel_separation_y", 0.70);
    node.declare_parameter("wheel_radius", 0.210);
    node.declare_parameter("singularity_threshold", 0.0001);
    node.declare_parameter("max_steering_speed", 3.0);
    node.declare_parameter("max_wheel_rate", 10.0);
    node.declare_parameter("cmd_vel_timeout", 0.5);
    node.declare_parameter("publish_tf", true);
    node.declare_parameter("odom_covariance.x", 0.03);
    node.declare_parameter("odom_covariance.y", 0.03);
    node.declare_parameter("odom_covariance.yaw", 0.02);
    node.declare_parameter("twist_covariance.x", 0.05);
    node.declare_parameter("twist_covariance.y", 0.05);
    node.declare_parameter("twist_covariance.yaw", 0.04);
  } catch (const std::exception & e) {
    fprintf(stderr, "SwerveDriveController on_init exception: %s\n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration SwerveDriveController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.emplace_back("Revolute1_to_base_link/position");
  config.names.emplace_back("Revolute2_to_base_link/position");
  config.names.emplace_back("Revolute3_to_base_link/position");
  config.names.emplace_back("Revolute4_to_base_link/position");
  config.names.emplace_back("Wheel1_to_Revolute1/velocity");
  config.names.emplace_back("Wheel2_to_Revolute2/velocity");
  config.names.emplace_back("Wheel3_to_Revolute3/velocity");
  config.names.emplace_back("Wheel4_to_Revolute4/velocity");
  return config;
}

controller_interface::InterfaceConfiguration SwerveDriveController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.emplace_back("Revolute1_to_base_link/position");
  config.names.emplace_back("Revolute2_to_base_link/position");
  config.names.emplace_back("Revolute3_to_base_link/position");
  config.names.emplace_back("Revolute4_to_base_link/position");
  config.names.emplace_back("Revolute1_to_base_link/velocity");
  config.names.emplace_back("Revolute2_to_base_link/velocity");
  config.names.emplace_back("Revolute3_to_base_link/velocity");
  config.names.emplace_back("Revolute4_to_base_link/velocity");
  config.names.emplace_back("Wheel1_to_Revolute1/velocity");
  config.names.emplace_back("Wheel2_to_Revolute2/velocity");
  config.names.emplace_back("Wheel3_to_Revolute3/velocity");
  config.names.emplace_back("Wheel4_to_Revolute4/velocity");
  config.names.emplace_back("Wheel1_to_Revolute1/position");
  config.names.emplace_back("Wheel2_to_Revolute2/position");
  config.names.emplace_back("Wheel3_to_Revolute3/position");
  config.names.emplace_back("Wheel4_to_Revolute4/position");
  return config;
}

controller_interface::CallbackReturn SwerveDriveController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto & node = *get_node();
  wheel_separation_x_ = node.get_parameter("wheel_separation_x").as_double();
  wheel_separation_y_ = node.get_parameter("wheel_separation_y").as_double();
  wheel_radius_ = node.get_parameter("wheel_radius").as_double();
  singularity_threshold_ = node.get_parameter("singularity_threshold").as_double();
  max_steering_speed_ = node.get_parameter("max_steering_speed").as_double();
  max_wheel_rate_ = node.get_parameter("max_wheel_rate").as_double();
  cmd_vel_timeout_ = node.get_parameter("cmd_vel_timeout").as_double();
  publish_tf_ = node.get_parameter("publish_tf").as_bool();
  odom_cov_x_ = node.get_parameter("odom_covariance.x").as_double();
  odom_cov_y_ = node.get_parameter("odom_covariance.y").as_double();
  odom_cov_yaw_ = node.get_parameter("odom_covariance.yaw").as_double();
  twist_cov_x_ = node.get_parameter("twist_covariance.x").as_double();
  twist_cov_y_ = node.get_parameter("twist_covariance.y").as_double();
  twist_cov_yaw_ = node.get_parameter("twist_covariance.yaw").as_double();

  const auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };
  const auto nonnegative = [](double v) { return std::isfinite(v) && v >= 0.0; };
  if (!positive(wheel_separation_x_) || !positive(wheel_separation_y_) ||
      !positive(wheel_radius_) || !positive(singularity_threshold_) ||
      !positive(max_steering_speed_) || !positive(max_wheel_rate_) ||
      max_wheel_rate_ > 10.0 || !positive(cmd_vel_timeout_) ||
      !nonnegative(odom_cov_x_) || !nonnegative(odom_cov_y_) ||
      !nonnegative(odom_cov_yaw_) || !nonnegative(twist_cov_x_) ||
      !nonnegative(twist_cov_y_) || !nonnegative(twist_cov_yaw_)) {
    RCLCPP_ERROR(node.get_logger(), "Invalid swerve geometry, limit, timeout or covariance parameter");
    return controller_interface::CallbackReturn::ERROR;
  }

  const double half_length = wheel_separation_x_ / 2.0;
  const double half_width = wheel_separation_y_ / 2.0;
  wheel_positions_ = {{{half_length, half_width}, {half_length, -half_width},
    {-half_length, half_width}, {-half_length, -half_width}}};

  // All upstream sources must normalize to TwistStamped on this one input.
  cmd_vel_sub_ = node.create_subscription<geometry_msgs::msg::TwistStamped>(
    "/cmd_vel_nav", rclcpp::SystemDefaultsQoS(),
    std::bind(&SwerveDriveController::cmd_vel_callback, this, std::placeholders::_1));
  odom_pub_ = node.create_publisher<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::SystemDefaultsQoS());
  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
  }
  RCLCPP_INFO(node.get_logger(),
    "Swerve configured: L=%.3f W=%.3f r=%.3f steer=%.2f wheel=%.2f rad/s tf=%s",
    wheel_separation_x_, wheel_separation_y_, wheel_radius_, max_steering_speed_,
    max_wheel_rate_, publish_tf_ ? "ON" : "OFF");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveDriveController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (command_interfaces_.size() != NUM_CMD || state_interfaces_.size() != NUM_STATE) {
    RCLCPP_ERROR(get_node()->get_logger(), "Swerve interface count mismatch: command=%zu state=%zu",
      command_interfaces_.size(), state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < 4; ++i) {
    const double actual = state_interfaces_[STEER_POS0 + i].get_value();
    if (!std::isfinite(actual) || actual < kSteerMin - kSteerLimitTolerance ||
        actual > kSteerMax + kSteerLimitTolerance) {
      RCLCPP_ERROR(get_node()->get_logger(), "Steering %zu is invalid or outside command limits", i);
      return controller_interface::CallbackReturn::ERROR;
    }
    prev_steering_angles_[i] = std::clamp(actual, kSteerMin, kSteerMax);
  }
  x_ = 0.0;
  y_ = 0.0;
  theta_ = 0.0;
  first_update_ = true;
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    last_cmd_.reset();
  }
  stop_drive_and_hold_steering();
  RCLCPP_INFO(get_node()->get_logger(), "Swerve activated with steering commands seeded from encoders");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveDriveController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // A zero position command would rotate the steering joints. Hold measured positions.
  for (size_t i = 0; i < 4; ++i) {
    const double actual = state_interfaces_[STEER_POS0 + i].get_value();
    if (std::isfinite(actual) && actual >= kSteerMin && actual <= kSteerMax) {
      prev_steering_angles_[i] = std::clamp(actual, kSteerMin, kSteerMax);
    }
  }
  stop_drive_and_hold_steering();
  RCLCPP_INFO(get_node()->get_logger(), "Swerve deactivated; drive commands zeroed");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveDriveController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  cmd_vel_sub_.reset();
  odom_pub_.reset();
  tf_broadcaster_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

void SwerveDriveController::cmd_vel_callback(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  const auto & twist = msg->twist;
  const bool valid = std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) &&
    std::isfinite(twist.linear.z) && std::isfinite(twist.angular.x) &&
    std::isfinite(twist.angular.y) && std::isfinite(twist.angular.z);
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    if (valid) {
      last_cmd_ = msg;
      last_cmd_time_ = get_node()->now();
    } else {
      last_cmd_.reset();
    }
  }
  if (!valid) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Rejected nonfinite TwistStamped; drive will stop");
  }
}

void SwerveDriveController::stop_drive_and_hold_steering()
{
  for (size_t i = 0; i < 4; ++i) {
    command_interfaces_[STEER0 + i].set_value(prev_steering_angles_[i]);
    command_interfaces_[DRIVE0 + i].set_value(0.0);
  }
}

controller_interface::return_type SwerveDriveController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  if (!std::isfinite(dt) || dt <= 0.0 || dt > kMaxPeriod) {
    stop_drive_and_hold_steering();
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Invalid controller period %.6f s; holding steering and stopping drive", dt);
    return controller_interface::return_type::OK;
  }

  double vx_cmd = 0.0, vy_cmd = 0.0, omega_cmd = 0.0;
  bool cmd_timeout = false;
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    if (last_cmd_) {
      const double age = (time - last_cmd_time_).seconds();
      if (std::isfinite(age) && age >= 0.0 && age <= cmd_vel_timeout_) {
        vx_cmd = last_cmd_->twist.linear.x;
        vy_cmd = last_cmd_->twist.linear.y;
        omega_cmd = last_cmd_->twist.angular.z;
      } else {
        cmd_timeout = true;
      }
    }
  }
  if (cmd_timeout) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "TwistStamped timeout or clock discontinuity; stopping drive");
  }

  std::array<double, 4> steer_pos{};
  std::array<double, 4> drive_vel{};
  for (size_t i = 0; i < 4; ++i) {
    steer_pos[i] = state_interfaces_[STEER_POS0 + i].get_value();
    drive_vel[i] = state_interfaces_[DRIVE_VEL0 + i].get_value();
    if (!std::isfinite(steer_pos[i]) || steer_pos[i] < kSteerMin - kSteerLimitTolerance ||
        steer_pos[i] > kSteerMax + kSteerLimitTolerance || !std::isfinite(drive_vel[i])) {
      stop_drive_and_hold_steering();
      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "Invalid steering or drive feedback; controller stopped");
      return controller_interface::return_type::ERROR;
    }
  }

  std::array<double, 4> target_steer{};
  std::array<double, 4> target_drive_rate{};
  compute_inverse_kinematics(vx_cmd, vy_cmd, omega_cmd, dt, steer_pos,
    target_steer, target_drive_rate);
  for (size_t i = 0; i < 4; ++i) {
    if (!std::isfinite(target_steer[i]) || !std::isfinite(target_drive_rate[i])) {
      stop_drive_and_hold_steering();
      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "Nonfinite swerve command rejected");
      return controller_interface::return_type::ERROR;
    }
    command_interfaces_[STEER0 + i].set_value(target_steer[i]);
    command_interfaces_[DRIVE0 + i].set_value(target_drive_rate[i]);
  }

  double vx_actual = 0.0, vy_actual = 0.0, omega_actual = 0.0;
  compute_forward_kinematics(steer_pos, drive_vel, vx_actual, vy_actual, omega_actual);
  if (!std::isfinite(vx_actual) || !std::isfinite(vy_actual) || !std::isfinite(omega_actual)) {
    stop_drive_and_hold_steering();
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Nonfinite odometry estimate; controller stopped");
    return controller_interface::return_type::ERROR;
  }
  if (first_update_) {
    first_update_ = false;
  } else {
    const double mid = theta_ + 0.5 * omega_actual * dt;
    x_ += (vx_actual * std::cos(mid) - vy_actual * std::sin(mid)) * dt;
    y_ += (vx_actual * std::sin(mid) + vy_actual * std::cos(mid)) * dt;
    theta_ = std::remainder(theta_ + omega_actual * dt, kTwoPi);
  }

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta_);
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();
  odom.twist.twist.linear.x = vx_actual;
  odom.twist.twist.linear.y = vy_actual;
  odom.twist.twist.angular.z = omega_actual;
  odom.pose.covariance[0] = odom_cov_x_;
  odom.pose.covariance[7] = odom_cov_y_;
  odom.pose.covariance[14] = kUnobservedVariance;
  odom.pose.covariance[21] = kUnobservedVariance;
  odom.pose.covariance[28] = kUnobservedVariance;
  odom.pose.covariance[35] = odom_cov_yaw_;
  odom.twist.covariance[0] = twist_cov_x_;
  odom.twist.covariance[7] = twist_cov_y_;
  odom.twist.covariance[14] = kUnobservedVariance;
  odom.twist.covariance[21] = kUnobservedVariance;
  odom.twist.covariance[28] = kUnobservedVariance;
  odom.twist.covariance[35] = twist_cov_yaw_;
  odom_pub_->publish(odom);

  if (publish_tf_ && tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped t;
    t.header = odom.header;
    t.child_frame_id = odom.child_frame_id;
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(t);
  }
  return controller_interface::return_type::OK;
}

void SwerveDriveController::compute_inverse_kinematics(
  double vx, double vy, double omega, double dt,
  const std::array<double, 4> & actual_steering,
  std::array<double, 4> & steer_angles,
  std::array<double, 4> & drive_speeds)
{
  double peak_rate = 0.0;
  for (size_t i = 0; i < 4; ++i) {
    const double module_vx = vx - omega * wheel_positions_[i].second;
    const double module_vy = vy + omega * wheel_positions_[i].first;
    const double speed = std::hypot(module_vx, module_vy);
    double target = prev_steering_angles_[i];
    double signed_speed = speed;
    if (speed >= singularity_threshold_) {
      const double heading = std::atan2(module_vy, module_vx);
      double best_travel = std::numeric_limits<double>::infinity();
      // A continuous URDF joint has a bounded command interface in this model.
      // Compare physically reachable, unwrapped position commands; wrapping a
      // bounded servo across +/-pi can otherwise request nearly a full turn.
      for (int flipped = 0; flipped < 2; ++flipped) {
        const double base = heading + (flipped ? kPi : 0.0);
        for (int turns = -1; turns <= 1; ++turns) {
          const double candidate = base + turns * kTwoPi;
          if (candidate < kSteerMin - kSteerLimitTolerance ||
              candidate > kSteerMax + kSteerLimitTolerance) continue;
          const double bounded = std::clamp(candidate, kSteerMin, kSteerMax);
          const double travel = std::abs(bounded - actual_steering[i]);
          if (travel < best_travel) {
            best_travel = travel;
            target = bounded;
            signed_speed = flipped ? -speed : speed;
          }
        }
      }
      if (!std::isfinite(best_travel)) signed_speed = 0.0;
    }
    const double max_step = max_steering_speed_ * dt;
    const double step = std::clamp(target - prev_steering_angles_[i], -max_step, max_step);
    const double command = std::clamp(prev_steering_angles_[i] + step, kSteerMin, kSteerMax);
    prev_steering_angles_[i] = command;
    steer_angles[i] = command;
    // Use the final desired rolling direction, never the slew-limited command.
    const double alignment = std::max(0.0,
      std::cos(std::remainder(target - actual_steering[i], kTwoPi)));
    drive_speeds[i] = signed_speed * alignment / wheel_radius_;
    peak_rate = std::max(peak_rate, std::abs(drive_speeds[i]));
  }
  if (peak_rate > max_wheel_rate_) {
    const double scale = max_wheel_rate_ / peak_rate;
    for (double & rate : drive_speeds) rate *= scale;
  }
}

void SwerveDriveController::compute_forward_kinematics(
  const std::array<double, 4> & steer_angles,
  const std::array<double, 4> & drive_velocities,
  double & vx_out, double & vy_out, double & omega_out)
{
  vx_out = 0.0;
  vy_out = 0.0;
  omega_out = 0.0;
  double yaw_denominator = 0.0;
  // The four configured wheel positions are symmetric about the body origin,
  // making this exactly the ordinary least-squares pseudoinverse solution.
  for (size_t i = 0; i < 4; ++i) {
    const double wheel_speed = drive_velocities[i] * wheel_radius_;
    const double wx = wheel_speed * std::cos(steer_angles[i]);
    const double wy = wheel_speed * std::sin(steer_angles[i]);
    vx_out += wx * 0.25;
    vy_out += wy * 0.25;
    omega_out += -wheel_positions_[i].second * wx + wheel_positions_[i].first * wy;
    yaw_denominator += wheel_positions_[i].first * wheel_positions_[i].first +
      wheel_positions_[i].second * wheel_positions_[i].second;
  }
  omega_out /= yaw_denominator;
}

}  // namespace swerve_vehicle_control

PLUGINLIB_EXPORT_CLASS(swerve_vehicle_control::SwerveDriveController,
                       controller_interface::ControllerInterface)
