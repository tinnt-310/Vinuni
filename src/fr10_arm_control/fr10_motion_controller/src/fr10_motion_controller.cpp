#include "fr10_motion_controller/fr10_motion_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace fr10_motion_controller {
namespace {

constexpr std::size_t kPositionStateOffset = 0;
constexpr std::size_t kVelocityStateOffset = fr10_motion::kJointCount;
constexpr std::size_t kStateInterfaceCount = 2 * fr10_motion::kJointCount;
constexpr double kMaximumExactInteger = 9007199254740991.0;

bool exactCounter(double value) {
  return std::isfinite(value) && value >= 0.0 &&
         value <= kMaximumExactInteger && std::trunc(value) == value;
}

bool finitePositive(double value) {
  return std::isfinite(value) && value > 0.0;
}

template <typename Array>
bool copySix(const std::vector<double> &source, Array &destination,
             bool positive) {
  if (source.size() != fr10_motion::kJointCount) {
    return false;
  }
  for (std::size_t i = 0; i < source.size(); ++i) {
    if (!std::isfinite(source[i]) || (positive && source[i] <= 0.0)) {
      return false;
    }
    destination[i] = source[i];
  }
  return true;
}

} // namespace

controller_interface::CallbackReturn Fr10MotionController::on_init() {
  try {
    auto &node = *get_node();
    node.declare_parameter("joints", joint_names_);
    node.declare_parameter("control_rate_hz", 125.0);
    node.declare_parameter("maximum_cycle_gap_s", 0.016);
    node.declare_parameter("state_publish_rate_hz", 20.0);
    node.declare_parameter("joint_goal_tolerance_rad", 0.002);
    node.declare_parameter("max_joint_reference_lead_rad",
                           fr10_motion::kDegToRad);
    node.declare_parameter("goal_stable_samples", 5);
    node.declare_parameter(
        "limits.min_joint_position_rad",
        std::vector<double>{-3.0543, -4.6251, -2.8274,
                            -4.6251, -3.0543, -3.0543});
    node.declare_parameter(
        "limits.max_joint_position_rad",
        std::vector<double>{3.0543, 1.4835, 2.8274,
                            1.4835, 3.0543, 3.0543});
    node.declare_parameter(
        "limits.max_joint_velocity_rad_s",
        std::vector<double>{1.5, 1.5, 1.5, 2.0, 2.0, 2.0});
    node.declare_parameter(
        "limits.max_joint_acceleration_rad_s2",
        std::vector<double>{3.5, 3.5, 3.5, 4.5, 4.5, 4.5});
    node.declare_parameter(
        "limits.max_joint_jerk_rad_s3",
        std::vector<double>{20.0, 20.0, 20.0, 30.0, 30.0, 30.0});
  } catch (const std::exception &error) {
    fprintf(stderr, "Fr10MotionController on_init: %s\n", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
Fr10MotionController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint : joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  return config;
}

controller_interface::InterfaceConfiguration
Fr10MotionController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const char *interface :
       {hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY}) {
    for (const auto &joint : joint_names_) {
      config.names.push_back(joint + "/" + interface);
    }
  }
  return config;
}

bool Fr10MotionController::loadConfig() {
  auto &node = *get_node();
  joint_names_ = node.get_parameter("joints").as_string_array();
  const double control_rate_hz =
      node.get_parameter("control_rate_hz").as_double();
  maximum_cycle_gap_s_ =
      node.get_parameter("maximum_cycle_gap_s").as_double();
  state_publish_rate_hz_ =
      node.get_parameter("state_publish_rate_hz").as_double();
  executor_config_.joint_goal_tolerance_rad =
      node.get_parameter("joint_goal_tolerance_rad").as_double();
  executor_config_.max_joint_reference_lead_rad =
      node.get_parameter("max_joint_reference_lead_rad").as_double();
  executor_config_.goal_stable_samples =
      static_cast<int>(node.get_parameter("goal_stable_samples").as_int());
  executor_config_.reference_lag_persistence_samples = 1;
  executor_config_.require_command_ack = false;
  executor_config_.control_rate_hz = control_rate_hz;
  if (joint_names_.size() != fr10_motion::kJointCount ||
      !finitePositive(control_rate_hz) ||
      !finitePositive(maximum_cycle_gap_s_) ||
      maximum_cycle_gap_s_ < 1.0 / control_rate_hz ||
      !finitePositive(state_publish_rate_hz_) ||
      !finitePositive(executor_config_.joint_goal_tolerance_rad) ||
      !finitePositive(executor_config_.max_joint_reference_lead_rad) ||
      executor_config_.goal_stable_samples < 1) {
    return false;
  }
  nominal_period_s_ = 1.0 / control_rate_hz;
  return copySix(
             node.get_parameter("limits.min_joint_position_rad")
                 .as_double_array(),
             executor_config_.limits.min_joint_position_rad, false) &&
         copySix(
             node.get_parameter("limits.max_joint_position_rad")
                 .as_double_array(),
             executor_config_.limits.max_joint_position_rad, false) &&
         copySix(
             node.get_parameter("limits.max_joint_velocity_rad_s")
                 .as_double_array(),
             executor_config_.limits.max_joint_velocity_rad_s, true) &&
         copySix(
             node.get_parameter("limits.max_joint_acceleration_rad_s2")
                 .as_double_array(),
             executor_config_.limits.max_joint_acceleration_rad_s2, true) &&
         copySix(
             node.get_parameter("limits.max_joint_jerk_rad_s3")
                 .as_double_array(),
             executor_config_.limits.max_joint_jerk_rad_s3, true);
}

controller_interface::CallbackReturn Fr10MotionController::on_configure(
    const rclcpp_lifecycle::State &) {
  if (!loadConfig()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Invalid FR10 controller rate, limits, joints or tolerance");
    return controller_interface::CallbackReturn::ERROR;
  }
  executor_ = std::make_unique<fr10_motion::MotionExecutor>(executor_config_);
  preparation_executor_ =
      std::make_unique<fr10_motion::MotionExecutor>(executor_config_);

  auto &node = *get_node();
  trajectory_sub_ = node.create_subscription<
      fr10_motion_interfaces::msg::JointTrajectoryCommand>(
      "/fr10_motion/trajectory", rclcpp::QoS(1).reliable(),
      std::bind(&Fr10MotionController::trajectoryCallback, this,
                std::placeholders::_1));
  health_sub_ = node.create_subscription<std_msgs::msg::Float64MultiArray>(
      "/fairino_driver/health", rclcpp::QoS(1).reliable(),
      std::bind(&Fr10MotionController::healthCallback, this,
                std::placeholders::_1));
  stop_service_ = node.create_service<std_srvs::srv::Empty>(
      "/fr10_motion/controlled_stop",
      [this](const std_srvs::srv::Empty::Request::SharedPtr,
             std_srvs::srv::Empty::Response::SharedPtr) {
        stop_requested_.store(true, std::memory_order_release);
      });
  state_pub_ = node.create_publisher<
      fr10_motion_interfaces::msg::ControllerState>(
      "/fr10_motion/controller_state", rclcpp::QoS(10).reliable());
  state_timer_ = node.create_wall_timer(
      std::chrono::duration<double>(1.0 / state_publish_rate_hz_),
      std::bind(&Fr10MotionController::publishState, this));
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Fr10MotionController::on_activate(
    const rclcpp_lifecycle::State &) {
  if (command_interfaces_.size() != fr10_motion::kJointCount ||
      state_interfaces_.size() != kStateInterfaceCount) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "FR10 interface mismatch: command=%zu state=%zu",
                 command_interfaces_.size(), state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  for (std::size_t joint = 0; joint < fr10_motion::kJointCount; ++joint) {
    const double measured =
        state_interfaces_[kPositionStateOffset + joint].get_value();
    if (!std::isfinite(measured)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "FR10 joint %zu has invalid measured position", joint);
      return controller_interface::CallbackReturn::ERROR;
    }
    command_interfaces_[joint].set_value(measured);
    const double measured_velocity =
        state_interfaces_[kVelocityStateOffset + joint].get_value();
    if (!std::isfinite(measured_velocity)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "FR10 joint %zu has invalid measured velocity", joint);
      return controller_interface::CallbackReturn::ERROR;
    }
    previous_measured_velocity_[joint] = measured_velocity;
  }
  executor_ = std::make_unique<fr10_motion::MotionExecutor>(executor_config_);
  pending_buffer_.writeFromNonRT(
      std::shared_ptr<const PendingTrajectory>{});
  newest_received_generation_.store(0, std::memory_order_release);
  consumed_generation_ = 0;
  feedback_sequence_ = 0;
  have_previous_measured_velocity_ = false;
  stop_requested_.store(false, std::memory_order_release);
  detail_snapshot_.store(DetailCode::READY, std::memory_order_release);
  accepting_commands_.store(true, std::memory_order_release);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Fr10MotionController::on_deactivate(
    const rclcpp_lifecycle::State &) {
  accepting_commands_.store(false, std::memory_order_release);
  holdMeasuredPosition();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Fr10MotionController::on_cleanup(
    const rclcpp_lifecycle::State &) {
  accepting_commands_.store(false, std::memory_order_release);
  trajectory_sub_.reset();
  health_sub_.reset();
  stop_service_.reset();
  state_timer_.reset();
  state_pub_.reset();
  executor_.reset();
  preparation_executor_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

void Fr10MotionController::trajectoryCallback(
    fr10_motion_interfaces::msg::JointTrajectoryCommand::ConstSharedPtr
        message) {
  if (!accepting_commands_.load(std::memory_order_acquire) ||
      !preparation_executor_) {
    return;
  }
  const uint64_t newest =
      newest_received_generation_.load(std::memory_order_acquire);
  if (message->generation <= newest) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Rejected non-increasing trajectory generation %lu <= %lu",
                message->generation, newest);
    return;
  }
  auto decoded = decodeTrajectory(*message);
  if (!decoded) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Rejected trajectory generation %lu: %s",
                 message->generation, decoded.error.c_str());
    return;
  }
  std::shared_ptr<const fr10_motion::PreparedCartesianTrajectory> prepared;
  if (message->mode ==
      fr10_motion_interfaces::msg::JointTrajectoryCommand::JOINT) {
    fr10_motion::JointTarget target;
    target.position_rad =
        decoded.trajectory->points.back().joint_state.position_rad;
    target.target_velocity_rad_s =
        decoded.trajectory->points.back().joint_state.velocity_rad_s;
    target.target_acceleration_rad_s2 =
        decoded.trajectory->points.back().joint_state.acceleration_rad_s2;
    target.revision = message->generation;
    target.timestamp_s = decoded.trajectory->planning_timestamp_s;
    prepared = preparation_executor_->prepareJointTrajectory(
        decoded.trajectory, target);
  } else {
    prepared =
        preparation_executor_->preparePoseTrajectory(decoded.trajectory);
  }
  if (!prepared) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Trajectory generation %lu violates configured limits",
                 message->generation);
    return;
  }
  auto pending = std::make_shared<PendingTrajectory>();
  pending->prepared = std::move(prepared);
  pending->generation = message->generation;
  pending->mode = message->mode;
  pending_buffer_.writeFromNonRT(
      std::shared_ptr<const PendingTrajectory>(std::move(pending)));
  newest_received_generation_.store(message->generation,
                                    std::memory_order_release);
}

void Fr10MotionController::healthCallback(
    std_msgs::msg::Float64MultiArray::ConstSharedPtr message) {
  if (message->data.size() < 20 || message->data[0] != 2.0 ||
      !exactCounter(message->data[16]) ||
      !exactCounter(message->data[17]) ||
      !std::isfinite(message->data[18]) ||
      !std::isfinite(message->data[19])) {
    return;
  }
  driver_write_sequence_.store(
      static_cast<uint64_t>(message->data[16]), std::memory_order_release);
  driver_success_sequence_.store(
      static_cast<uint64_t>(message->data[17]), std::memory_order_release);
  driver_gap_snapshot_.store(message->data[18], std::memory_order_release);
  driver_error_snapshot_.store(
      static_cast<int32_t>(message->data[19]), std::memory_order_release);
}

void Fr10MotionController::holdMeasuredPosition() {
  if (command_interfaces_.size() != fr10_motion::kJointCount ||
      state_interfaces_.size() < fr10_motion::kJointCount) {
    return;
  }
  for (std::size_t joint = 0; joint < fr10_motion::kJointCount; ++joint) {
    const double measured =
        state_interfaces_[kPositionStateOffset + joint].get_value();
    if (std::isfinite(measured)) {
      command_interfaces_[joint].set_value(measured);
    }
  }
}

controller_interface::return_type Fr10MotionController::update(
    const rclcpp::Time &time, const rclcpp::Duration &period) {
  if (!executor_ || state_interfaces_.size() != kStateInterfaceCount ||
      command_interfaces_.size() != fr10_motion::kJointCount) {
    return controller_interface::return_type::ERROR;
  }

  fr10_motion::RobotState feedback;
  feedback.timestamp_s = time.seconds();
  feedback.received_steady_s = feedback.timestamp_s;
  feedback.feedback_sequence = ++feedback_sequence_;
  feedback.valid = true;
  const double actual_period_s = period.seconds();
  for (std::size_t joint = 0; joint < fr10_motion::kJointCount; ++joint) {
    feedback.joint_position_rad[joint] =
        state_interfaces_[kPositionStateOffset + joint].get_value();
    feedback.joint_velocity_rad_s[joint] =
        state_interfaces_[kVelocityStateOffset + joint].get_value();
    feedback.joint_acceleration_rad_s2[joint] =
        have_previous_measured_velocity_ && finitePositive(actual_period_s)
            ? (feedback.joint_velocity_rad_s[joint] -
               previous_measured_velocity_[joint]) /
                  actual_period_s
            : 0.0;
    if (!std::isfinite(feedback.joint_position_rad[joint]) ||
        !std::isfinite(feedback.joint_velocity_rad_s[joint]) ||
        !std::isfinite(feedback.joint_acceleration_rad_s2[joint])) {
      holdMeasuredPosition();
      detail_snapshot_.store(DetailCode::EXECUTOR_FAULT,
                             std::memory_order_release);
      return controller_interface::return_type::ERROR;
    }
    previous_measured_velocity_[joint] = feedback.joint_velocity_rad_s[joint];
  }
  have_previous_measured_velocity_ = true;

  const bool timing_violation =
      !finitePositive(actual_period_s) ||
      actual_period_s > maximum_cycle_gap_s_;
  if (timing_violation) {
    stop_requested_.store(true, std::memory_order_release);
    detail_snapshot_.store(DetailCode::TIMING_VIOLATION,
                           std::memory_order_release);
  }
  if (stop_requested_.exchange(false, std::memory_order_acq_rel)) {
    executor_->stop();
  }

  if (executor_->initialized()) {
    const auto *pending = pending_buffer_.readFromRT();
    if (pending && *pending &&
        (*pending)->generation > consumed_generation_) {
      const auto activation = executor_->activatePreparedPoseTrajectory(
          (*pending)->prepared);
      consumed_generation_ = (*pending)->generation;
      if (activation == fr10_motion::PoseActivationResult::STALE) {
        executor_->stop();
        detail_snapshot_.store(DetailCode::STALE_TRAJECTORY,
                               std::memory_order_release);
      } else if (activation ==
                 fr10_motion::PoseActivationResult::REJECTED) {
        detail_snapshot_.store(DetailCode::EXECUTOR_FAULT,
                               std::memory_order_release);
      }
    }
  }

  const double executor_period_s =
      finitePositive(actual_period_s) ? actual_period_s : nominal_period_s_;
  const auto status = executor_->update(feedback, executor_period_s);
  const auto command = executor_->command();
  if (command.type == fr10_motion::CommandType::JOINT_POSITION) {
    for (std::size_t joint = 0; joint < fr10_motion::kJointCount; ++joint) {
      command_interfaces_[joint].set_value(command.joint_position_rad[joint]);
      reference_position_snapshot_[joint].store(
          command.joint_position_rad[joint], std::memory_order_release);
      reference_velocity_snapshot_[joint].store(
          command.joint_velocity_rad_s[joint], std::memory_order_release);
      reference_acceleration_snapshot_[joint].store(
          command.joint_acceleration_rad_s2[joint],
          std::memory_order_release);
    }
  } else {
    holdMeasuredPosition();
  }

  uint8_t public_state =
      fr10_motion_interfaces::msg::ControllerState::IDLE;
  DetailCode detail = DetailCode::READY;
  switch (status.state) {
  case fr10_motion::MotionState::TRACKING:
    public_state =
        fr10_motion_interfaces::msg::ControllerState::TRACKING;
    detail = DetailCode::TRACKING;
    break;
  case fr10_motion::MotionState::STOPPING:
    public_state =
        fr10_motion_interfaces::msg::ControllerState::STOPPING;
    detail = timing_violation ? DetailCode::TIMING_VIOLATION
                              : DetailCode::CONTROLLED_STOP;
    break;
  case fr10_motion::MotionState::HOLDING:
  case fr10_motion::MotionState::STOPPED:
    public_state =
        fr10_motion_interfaces::msg::ControllerState::HOLDING;
    detail = status.outcome == fr10_motion::MotionOutcome::GOAL_REACHED
                 ? DetailCode::GOAL_REACHED
                 : DetailCode::CONTROLLED_STOP;
    break;
  case fr10_motion::MotionState::FAULT:
    public_state =
        fr10_motion_interfaces::msg::ControllerState::FAULT;
    detail = DetailCode::EXECUTOR_FAULT;
    break;
  case fr10_motion::MotionState::IDLE:
    break;
  }
  if (driver_error_snapshot_.load(std::memory_order_acquire) != 0) {
    public_state = fr10_motion_interfaces::msg::ControllerState::FAULT;
    detail = DetailCode::DRIVER_ERROR;
  }

  state_snapshot_.store(public_state, std::memory_order_release);
  generation_snapshot_.store(
      status.active_generation != 0 ? status.active_generation
                                    : status.active_revision,
      std::memory_order_release);
  command_sequence_snapshot_.store(command.sequence,
                                   std::memory_order_release);
  progress_snapshot_.store(status.cartesian_path_parameter,
                           std::memory_order_release);
  reference_error_snapshot_.store(status.max_reference_actual_error_rad,
                                  std::memory_order_release);
  cycle_period_snapshot_.store(actual_period_s, std::memory_order_release);
  cycle_jitter_snapshot_.store(
      std::isfinite(actual_period_s)
          ? std::abs(actual_period_s - nominal_period_s_)
          : std::numeric_limits<double>::infinity(),
      std::memory_order_release);
  detail_snapshot_.store(detail, std::memory_order_release);
  return controller_interface::return_type::OK;
}

const char *Fr10MotionController::detailText(DetailCode code) {
  switch (code) {
  case DetailCode::READY:
    return "ready";
  case DetailCode::TRACKING:
    return "tracking";
  case DetailCode::GOAL_REACHED:
    return "goal reached and holding";
  case DetailCode::CONTROLLED_STOP:
    return "controlled stop";
  case DetailCode::TIMING_VIOLATION:
    return "controller cycle exceeded configured maximum";
  case DetailCode::STALE_TRAJECTORY:
    return "trajectory seed did not match the active reference";
  case DetailCode::EXECUTOR_FAULT:
    return "motion executor fault";
  case DetailCode::DRIVER_ERROR:
    return "FAIRINO ServoJ reported an error";
  }
  return "unknown";
}

void Fr10MotionController::publishState() {
  if (!state_pub_) {
    return;
  }
  fr10_motion_interfaces::msg::ControllerState message;
  message.header.stamp = get_node()->now();
  message.state = state_snapshot_.load(std::memory_order_acquire);
  message.generation = generation_snapshot_.load(std::memory_order_acquire);
  message.command_sequence =
      command_sequence_snapshot_.load(std::memory_order_acquire);
  message.driver_write_sequence =
      driver_write_sequence_.load(std::memory_order_acquire);
  message.driver_success_sequence =
      driver_success_sequence_.load(std::memory_order_acquire);
  message.progress = progress_snapshot_.load(std::memory_order_acquire);
  for (std::size_t joint = 0; joint < fr10_motion::kJointCount; ++joint) {
    message.reference_joint_position[joint] =
        reference_position_snapshot_[joint].load(std::memory_order_acquire);
    message.reference_joint_velocity[joint] =
        reference_velocity_snapshot_[joint].load(std::memory_order_acquire);
    message.reference_joint_acceleration[joint] =
        reference_acceleration_snapshot_[joint].load(
            std::memory_order_acquire);
  }
  message.max_reference_error_rad =
      reference_error_snapshot_.load(std::memory_order_acquire);
  message.cycle_period_s =
      cycle_period_snapshot_.load(std::memory_order_acquire);
  message.cycle_jitter_s =
      cycle_jitter_snapshot_.load(std::memory_order_acquire);
  message.driver_write_gap_s =
      driver_gap_snapshot_.load(std::memory_order_acquire);
  message.driver_error_code =
      driver_error_snapshot_.load(std::memory_order_acquire);
  message.detail =
      detailText(detail_snapshot_.load(std::memory_order_acquire));
  state_pub_->publish(message);
}

} // namespace fr10_motion_controller

PLUGINLIB_EXPORT_CLASS(fr10_motion_controller::Fr10MotionController,
                       controller_interface::ControllerInterface)
