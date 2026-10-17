#include "swerve_vehicle_control/swerve_controller_plugin.hpp"

#include <algorithm>
#include <chrono>
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
    if (!node.has_parameter("wheel_separation_x")) node.declare_parameter("wheel_separation_x", 0.80);
    if (!node.has_parameter("wheel_separation_y")) node.declare_parameter("wheel_separation_y", 0.70);
    if (!node.has_parameter("wheel_radius")) node.declare_parameter("wheel_radius", 0.210);
    if (!node.has_parameter("wheel_offset_x")) node.declare_parameter("wheel_offset_x", 0.0);
    if (!node.has_parameter("singularity_threshold")) node.declare_parameter("singularity_threshold", 0.0001);
    if (!node.has_parameter("max_steering_speed")) node.declare_parameter("max_steering_speed", 3.0);
    if (!node.has_parameter("max_wheel_rate")) node.declare_parameter("max_wheel_rate", 10.0);
    if (!node.has_parameter("cmd_vel_timeout")) node.declare_parameter("cmd_vel_timeout", 0.5);
    if (!node.has_parameter("publish_tf")) node.declare_parameter("publish_tf", true);
    if (!node.has_parameter("odom_covariance.x")) node.declare_parameter("odom_covariance.x", 0.03);
    if (!node.has_parameter("odom_covariance.y")) node.declare_parameter("odom_covariance.y", 0.03);
    if (!node.has_parameter("odom_covariance.yaw")) node.declare_parameter("odom_covariance.yaw", 0.02);
    if (!node.has_parameter("twist_covariance.x")) node.declare_parameter("twist_covariance.x", 0.05);
    if (!node.has_parameter("twist_covariance.y")) node.declare_parameter("twist_covariance.y", 0.05);
    if (!node.has_parameter("twist_covariance.yaw")) node.declare_parameter("twist_covariance.yaw", 0.04);
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
  wheel_offset_x_ = node.get_parameter("wheel_offset_x").as_double();
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
      !positive(wheel_radius_) || !nonnegative(wheel_offset_x_) ||
      !positive(singularity_threshold_) ||
      !positive(max_steering_speed_) || !positive(max_wheel_rate_) ||
      max_wheel_rate_ > 10.0 || !positive(cmd_vel_timeout_) ||
      cmd_vel_timeout_ >= static_cast<double>(std::numeric_limits<int64_t>::max()) * 1.0e-9 ||
      !nonnegative(odom_cov_x_) || !nonnegative(odom_cov_y_) ||
      !nonnegative(odom_cov_yaw_) || !nonnegative(twist_cov_x_) ||
      !nonnegative(twist_cov_y_) || !nonnegative(twist_cov_yaw_)) {
    RCLCPP_ERROR(node.get_logger(), "Invalid swerve geometry, limit, timeout or covariance parameter");
    return controller_interface::CallbackReturn::ERROR;
  }

  cmd_vel_timeout_ns_ = static_cast<int64_t>(cmd_vel_timeout_ * 1.0e9);
  if (cmd_vel_timeout_ns_ <= 0) return controller_interface::CallbackReturn::ERROR;

  const double half_length = wheel_separation_x_ / 2.0;
  const double half_width = wheel_separation_y_ / 2.0;
  steering_pivots_ = {{{half_length, half_width}, {half_length, -half_width},
    {-half_length, half_width}, {-half_length, -half_width}}};

  // All upstream sources must normalize to TwistStamped on this one input.
  cmd_vel_sub_ = node.create_subscription<geometry_msgs::msg::TwistStamped>(
    "/cmd_vel_nav", rclcpp::SystemDefaultsQoS(),
    std::bind(&SwerveDriveController::cmd_vel_callback, this, std::placeholders::_1));
  odom_pub_ = node.create_publisher<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::SystemDefaultsQoS());
  parking_sub_ = node.create_subscription<fr10_motion_interfaces::msg::VehicleParkingLease>(
    "/vehicle/parking_lease", rclcpp::QoS(10).reliable(),
    [this](fr10_motion_interfaces::msg::VehicleParkingLease::ConstSharedPtr message) {
      ParkingRequest request;
      request.owner_id = message->owner_id;
      request.sequence = message->sequence;
      request.release = message->release;
      request.duration_ns = static_cast<int64_t>(message->lease_duration.sec)*1000000000 +
        message->lease_duration.nanosec;
      request.receipt_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
      if (message->lease_duration.nanosec >= 1000000000) return;
      parking_request_buffer_.writeFromNonRT(request);
    });
  parking_pub_ = node.create_publisher<fr10_motion_interfaces::msg::VehicleParkingState>(
    "/vehicle/parking_state", rclcpp::QoS(10).reliable());
  odom_timer_ = node.create_wall_timer(std::chrono::milliseconds(33),
    std::bind(&SwerveDriveController::publish_odometry, this));
  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
  }
  RCLCPP_INFO(node.get_logger(),
    "Swerve configured: L=%.3f W=%.3f r=%.3f offset=%.6f steer=%.2f wheel=%.2f rad/s tf=%s",
    wheel_separation_x_, wheel_separation_y_, wheel_radius_, wheel_offset_x_, max_steering_speed_,
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
  command_buffer_.writeFromNonRT(VelocityCommand{});
  command_time_gate_.reset();
  odom_sample_gate_.reset();
  odom_stamp_ns_.store(0, std::memory_order_release);
  odom_ready_.store(false, std::memory_order_release);
  stop_drive_and_hold_steering();
  accepting_commands_.store(true, std::memory_order_release);
  RCLCPP_INFO(get_node()->get_logger(), "Swerve activated with steering commands seeded from encoders");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveDriveController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  accepting_commands_.store(false, std::memory_order_release);
  odom_ready_.store(false, std::memory_order_release);
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
  accepting_commands_.store(false, std::memory_order_release);
  cmd_vel_sub_.reset();
  parking_sub_.reset();
  parking_pub_.reset();
  odom_timer_.reset();
  odom_pub_.reset();
  tf_broadcaster_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

void SwerveDriveController::cmd_vel_callback(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  if (!accepting_commands_.load(std::memory_order_acquire)) return;
  if (parking_inhibited_.load(std::memory_order_acquire)) {
    parking_rejected_commands_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  const auto & twist = msg->twist;
  const bool finite = std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) &&
    std::isfinite(twist.linear.z) && std::isfinite(twist.angular.x) &&
    std::isfinite(twist.angular.y) && std::isfinite(twist.angular.z);
  const int64_t received_ns = get_node()->now().nanoseconds();
  const auto & stamp = msg->header.stamp;
  const int64_t source_ns = source_stamp_ns(stamp.sec, stamp.nanosec);
  const bool timely = finite && command_time_gate_.accept(
    msg->header.frame_id, source_ns, received_ns, cmd_vel_timeout_ns_);
  VelocityCommand command;
  if (finite && timely) {
    command.vx = twist.linear.x;
    command.vy = twist.linear.y;
    command.omega = twist.angular.z;
    command.source_ns = source_ns;
    command.received_ns = received_ns;
    command.receipt_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    command.valid = true;
  }
  command_buffer_.writeFromNonRT(command);
  if (!finite || !timely) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Rejected nonfinite, wrong-frame, stale, future or replayed TwistStamped; drive will stop");
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
  const int64_t steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  if (const auto *request = parking_request_buffer_.readFromRT())
    parking_lease_.apply(*request, steady_ns);
  // Set admission before reading the drive command. Even a callback racing
  // this update cannot move the vehicle while the lease owns the actuators.
  parking_inhibited_.store(parking_lease_.inhibited(), std::memory_order_release);
  if (!command_time_gate_.observe_update(
        time.nanoseconds(), steady_ns, cmd_vel_timeout_ns_)) {
    odom_ready_.store(false, std::memory_order_release);
    stop_drive_and_hold_steering();
    return controller_interface::return_type::OK;
  }
  if (!std::isfinite(dt) || dt <= 0.0 || dt > kMaxPeriod) {
    odom_ready_.store(false, std::memory_order_release);
    stop_drive_and_hold_steering();
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Invalid controller period %.6f s; holding steering and stopping drive", dt);
    return controller_interface::return_type::OK;
  }

  double vx_cmd = 0.0, vy_cmd = 0.0, omega_cmd = 0.0;
  bool cmd_timeout = false;
  const auto * command = command_buffer_.readFromRT();
  if (command && command->valid && parking_lease_.commandAllowed(command->receipt_steady_ns)) {
    if (command_time_gate_.usable(command->source_ns, command->received_ns,
                                  time.nanoseconds(), cmd_vel_timeout_ns_)) {
      vx_cmd = command->vx;
      vy_cmd = command->vy;
      omega_cmd = command->omega;
    } else {
      cmd_timeout = true;
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
  const bool stationary = std::abs(vx_actual) <= 1e-4 &&
    std::abs(vy_actual) <= 1e-4 && std::abs(omega_actual) <= 1e-4;
  parking_snapshot_sequence_.fetch_add(1, std::memory_order_acq_rel);
  parking_owner_.store(parking_lease_.owner(), std::memory_order_relaxed);
  parking_sequence_.store(parking_lease_.sequence(), std::memory_order_relaxed);
  parking_state_.store(!parking_lease_.inhibited() ? 0 :
    (parking_lease_.expired(steady_ns) ? 3 : (stationary ? 2 : 1)), std::memory_order_release);
  parking_snapshot_sequence_.fetch_add(1, std::memory_order_release);
  if (first_update_) {
    first_update_ = false;
  } else {
    const double mid = theta_ + 0.5 * omega_actual * dt;
    x_ += (vx_actual * std::cos(mid) - vy_actual * std::sin(mid)) * dt;
    y_ += (vx_actual * std::sin(mid) + vy_actual * std::cos(mid)) * dt;
    theta_ = std::remainder(theta_ + omega_actual * dt, kTwoPi);
  }

  if (!std::isfinite(x_) || !std::isfinite(y_) || !std::isfinite(theta_)) {
    stop_drive_and_hold_steering();
    return controller_interface::return_type::ERROR;
  }
  odom_sequence_.fetch_add(1, std::memory_order_acq_rel);
  odom_snapshot_[0].store(x_, std::memory_order_relaxed);
  odom_snapshot_[1].store(y_, std::memory_order_relaxed);
  odom_snapshot_[2].store(theta_, std::memory_order_relaxed);
  odom_snapshot_[3].store(vx_actual, std::memory_order_relaxed);
  odom_snapshot_[4].store(vy_actual, std::memory_order_relaxed);
  odom_snapshot_[5].store(omega_actual, std::memory_order_relaxed);
  odom_stamp_ns_.store(time.nanoseconds(), std::memory_order_relaxed);
  odom_sequence_.fetch_add(1, std::memory_order_release);
  odom_ready_.store(true, std::memory_order_release);
  return controller_interface::return_type::OK;
}

void SwerveDriveController::publish_odometry()
{
  if (parking_pub_) {
    fr10_motion_interfaces::msg::VehicleParkingState message;
    message.header.stamp = get_node()->now();
    for (int attempt = 0; attempt < 3; ++attempt) {
      const auto before = parking_snapshot_sequence_.load(std::memory_order_acquire);
      if (before & 1U) continue;
      message.owner_id = parking_owner_.load(std::memory_order_relaxed);
      message.sequence = parking_sequence_.load(std::memory_order_relaxed);
      message.state = parking_state_.load(std::memory_order_relaxed);
      if (before != parking_snapshot_sequence_.load(std::memory_order_acquire)) continue;
      message.rejected_drive_commands = parking_rejected_commands_.load();
      parking_pub_->publish(message);
      break;
    }
  }
  if (!odom_ready_.load(std::memory_order_acquire)) return;
  std::array<double, 6> state{};
  int64_t stamp_ns = 0;
  uint64_t snapshot_sequence = 0;
  bool consistent = false;
  for (int attempt = 0; attempt < 3; ++attempt) {
    const auto before = odom_sequence_.load(std::memory_order_acquire);
    if (before & 1U) continue;
    for (size_t i = 0; i < state.size(); ++i) {
      state[i] = odom_snapshot_[i].load(std::memory_order_relaxed);
    }
    stamp_ns = odom_stamp_ns_.load(std::memory_order_relaxed);
    const auto after = odom_sequence_.load(std::memory_order_acquire);
    if (before == after) {consistent = true; snapshot_sequence = after; break;}
  }
  if (!consistent || !odom_ready_.load(std::memory_order_acquire) ||
      !odom_sample_gate_.should_publish(snapshot_sequence, stamp_ns)) return;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, state[2]);
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = rclcpp::Time(stamp_ns);
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";
  odom.pose.pose.position.x = state[0];
  odom.pose.pose.position.y = state[1];
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();
  odom.twist.twist.linear.x = state[3];
  odom.twist.twist.linear.y = state[4];
  odom.twist.twist.angular.z = state[5];
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
    t.transform.translation.x = state[0];
    t.transform.translation.y = state[1];
    t.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(t);
  }
}

void SwerveDriveController::compute_inverse_kinematics(
  double vx, double vy, double omega, double dt,
  const std::array<double, 4> & actual_steering,
  std::array<double, 4> & steer_angles,
  std::array<double, 4> & drive_speeds)
{
  double peak_rate = 0.0;
  for (size_t i = 0; i < 4; ++i) {
    const auto center = wheel_center_position(
      steering_pivots_[i], actual_steering[i], wheel_offset_x_);
    const auto velocity = module_velocity(PlanarTwist{vx, vy, omega}, center);
    const double module_vx = velocity.x;
    const double module_vy = velocity.y;
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
  std::array<WheelPosition, 4> wheel_centers{};
  std::array<PlanarVector, 4> wheel_velocities{};
  for (size_t i = 0; i < 4; ++i) {
    wheel_centers[i] = wheel_center_position(
      steering_pivots_[i], steer_angles[i], wheel_offset_x_);
    const double wheel_speed = drive_velocities[i] * wheel_radius_;
    wheel_velocities[i] = {
      wheel_speed * std::cos(steer_angles[i]),
      wheel_speed * std::sin(steer_angles[i])};
  }
  const auto twist = estimate_body_twist(wheel_centers, wheel_velocities);
  vx_out = twist.vx;
  vy_out = twist.vy;
  omega_out = twist.omega;
}

}  // namespace swerve_vehicle_control

PLUGINLIB_EXPORT_CLASS(swerve_vehicle_control::SwerveDriveController,
                       controller_interface::ControllerInterface)
