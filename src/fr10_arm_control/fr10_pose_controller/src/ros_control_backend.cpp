#include "fr10_pose_controller/ros_control_backend.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>
#include <stdexcept>

namespace fr10_motion {

namespace {
double steadySeconds() {
  return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool parseExactUint64(double value, uint64_t &parsed) {
  // Float64MultiArray can represent every integer only through 2^53. Reject
  // negative, fractional, or lossy counters before any integer cast.
  constexpr double kMaximumExactInteger = 9007199254740991.0;
  if (!std::isfinite(value) || value < 0.0 ||
      value > kMaximumExactInteger || std::trunc(value) != value) {
    return false;
  }
  parsed = static_cast<uint64_t>(value);
  return true;
}

bool isExactInt(double value) {
  return std::isfinite(value) &&
         value >= static_cast<double>(std::numeric_limits<int>::min()) &&
         value <= static_cast<double>(std::numeric_limits<int>::max()) &&
         std::trunc(value) == value;
}
}

RosControlBackend::RosControlBackend(
    const rclcpp::Node::SharedPtr &node,
    const std::vector<std::string> &joint_names,
    const std::string &command_topic, const std::string &joint_state_topic,
    const rclcpp::CallbackGroup::SharedPtr &callback_group,
    double feedback_timeout_s, bool require_driver_health)
    : node_(node), joint_names_(joint_names), command_topic_(command_topic),
      joint_state_topic_(joint_state_topic), feedback_timeout_s_(feedback_timeout_s),
      require_driver_health_(require_driver_health) {
  if (!std::isfinite(feedback_timeout_s_) || feedback_timeout_s_ <= 0.0) {
    throw std::invalid_argument("feedback timeout must be positive and finite");
  }
  command_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      command_topic_, rclcpp::SystemDefaultsQoS());

  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = callback_group;
  health_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/fairino_driver/health", rclcpp::QoS(1).reliable(),
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr message) {
        const bool v1 = message->data.size() == 16 && message->data[0] == 1;
        const bool v2 = message->data.size() >= 20 && message->data[0] == 2;
        if ((!v1 && !v2) ||
            !std::all_of(message->data.begin(), message->data.end(),
                         [](double value) { return std::isfinite(value); })) { return; }
        uint64_t sequence = 0;
        uint64_t read_errors = 0;
        uint64_t write_errors = 0;
        if (!parseExactUint64(message->data[1], sequence) ||
            !parseExactUint64(message->data[14], read_errors) ||
            !parseExactUint64(message->data[15], write_errors) ||
            !isExactInt(message->data[4]) ||
            !isExactInt(message->data[5]) ||
            !isExactInt(message->data[7]) ||
            !isExactInt(message->data[10]) ||
            !isExactInt(message->data[11]) ||
            (message->data[6] != 0.0 && message->data[6] != 1.0) ||
            (message->data[8] != 0.0 && message->data[8] != 1.0) ||
            (message->data[9] != 0.0 && message->data[9] != 1.0) ||
            (message->data[12] != 0.0 && message->data[12] != 1.0) ||
            (message->data[13] != 0.0 && message->data[13] != 1.0)) {
          return;
        }
        (void)read_errors;
        (void)write_errors;
        DriverHealth health;
        // Errors must propagate even when a stale source sequence repeats.
        if (sequence <= last_health_sequence_ && message->data[4] == 0 && message->data[5] == 0) { return; }
        const std::size_t health_fields = v2 ? health.data.size() : 16U;
        std::copy_n(message->data.begin(), health_fields, health.data.begin());
        health.received_s = steadySeconds();
        if (health_buffer_.write(health)) { last_health_sequence_ = sequence; }
      }, subscription_options);
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      // joint_state_broadcaster publishes with sensor-data (best-effort) QoS.
      // A reliable subscriber is incompatible with that publisher and leaves
      // the controller permanently waiting for its first feedback sample.
      joint_state_topic_, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&RosControlBackend::jointStateCallback, this,
                std::placeholders::_1),
      subscription_options);

  command_message_.data.resize(kJointCount);
}

bool RosControlBackend::initialize() {
  ready_.store(false, std::memory_order_release);
  active_.store(true, std::memory_order_release);
  return true;
}

bool RosControlBackend::shutdown() {
  // Close the callback re-arm gate before observing state or clearing ready_.
  // A callback already in flight may still refresh the state buffer, but it
  // cannot make the backend ready again after this point.
  active_.store(false, std::memory_order_release);
  ready_.store(false, std::memory_order_release);
  // The runner must finish its braking stream before shutdown. Publishing
  // measured q here would jump backwards from the last commanded reference.
  return true;
}

bool RosControlBackend::isReady() const {
  DriverHealth health;
  if (require_driver_health_ && (!health_buffer_.read(health) ||
      steadySeconds() - health.received_s > feedback_timeout_s_ ||
      health.data[4] != 0 || health.data[5] != 0 || health.data[6] == 0 ||
      health.data[7] != 0 || health.data[8] != 0 || health.data[9] != 0 ||
      health.data[10] != 0 || health.data[11] != 0 || health.data[12] != 0 || health.data[13] != 0)) { return false; }
  RobotState state;
  if (!state_buffer_.read(state) || !state.valid ||
      steadySeconds() - state.received_steady_s > feedback_timeout_s_) {
    return false;
  }
  return active_.load(std::memory_order_acquire) &&
         ready_.load(std::memory_order_acquire) && state_buffer_.valid() &&
         command_pub_ &&
         (command_pub_->get_subscription_count() +
          command_pub_->get_intra_process_subscription_count()) > 0;
}

void RosControlBackend::jointStateCallback(
    const sensor_msgs::msg::JointState::ConstSharedPtr &message) {
  if (!active_.load(std::memory_order_acquire)) {
    return;
  }
  if (message->name.empty() || message->position.size() < kJointCount) {
    rejected_feedback_samples_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  const double next_stamp =
      message->header.stamp.sec + message->header.stamp.nanosec * 1e-9;
  if (!std::isfinite(next_stamp) || next_stamp <= 0.0 ||
      (have_joint_state_ && next_stamp <= writer_state_.timestamp_s)) {
    rejected_feedback_stamps_.fetch_add(1, std::memory_order_relaxed);
    return; // Replayed/out-of-order data must not refresh the watchdog.
  }
  const double received_s = steadySeconds();
  if (have_joint_state_) {
    const double source_gap_s = next_stamp - writer_state_.timestamp_s;
    const double receive_gap_s = received_s - writer_state_.received_steady_s;
    if (source_gap_s > max_source_gap_s_.load(std::memory_order_relaxed)) {
      max_source_gap_s_.store(source_gap_s, std::memory_order_relaxed);
    }
    if (receive_gap_s > max_receive_gap_s_.load(std::memory_order_relaxed)) {
      max_receive_gap_s_.store(receive_gap_s, std::memory_order_relaxed);
    }
  }
  const double dt =
      have_joint_state_ ? next_stamp - writer_state_.timestamp_s : 0.0;
  const auto previous_position = writer_state_.joint_position_rad;

  if (!joint_state_index_map_valid_ || message->name != joint_state_layout_) {
    std::vector<bool> used(message->name.size(), false);
    for (std::size_t expected = 0; expected < kJointCount; ++expected) {
      bool found = false;
      for (std::size_t source = 0; source < message->name.size(); ++source) {
        if (!used[source] && message->name[source] == joint_names_[expected]) {
          joint_state_index_map_[expected] = source;
          used[source] = true;
          found = true;
          break;
        }
      }
      if (!found) {
        joint_state_index_map_valid_ = false;
        rejected_feedback_samples_.fetch_add(1, std::memory_order_relaxed);
        return;
      }
    }
    joint_state_layout_ = message->name;
    joint_state_index_map_valid_ = true;
  }

  for (std::size_t i = 0; i < kJointCount; ++i) {
    const std::size_t src_idx = joint_state_index_map_[i];
    if (src_idx >= message->position.size() ||
        !std::isfinite(message->position[src_idx])) {
      rejected_feedback_samples_.fetch_add(1, std::memory_order_relaxed);
      return;
    }
  }

  // JointState positions are radians; the cached map makes steady-state O(1).
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const std::size_t src_idx = joint_state_index_map_[i];
    writer_state_.joint_position_rad[i] = message->position[src_idx];
    // ros2_control represents an unavailable state interface as NaN.  That is
    // not an invalid position sample: derive velocity from two unique,
    // timestamped positions instead.
    if (message->velocity.size() > src_idx &&
        std::isfinite(message->velocity[src_idx])) {
      writer_state_.joint_velocity_rad_s[i] = message->velocity[src_idx];
    } else if (have_joint_state_ && dt > 1e-6 && std::isfinite(dt)) {
      writer_state_.joint_velocity_rad_s[i] =
          (writer_state_.joint_position_rad[i] - previous_position[i]) / dt;
    } else {
      writer_state_.joint_velocity_rad_s[i] = 0.0;
    }
    writer_state_.joint_acceleration_rad_s2[i] = 0.0;
    if (message->effort.size() > src_idx &&
        std::isfinite(message->effort[src_idx])) {
      writer_state_.joint_effort_Nm[i] = message->effort[src_idx];
    } else {
      writer_state_.joint_effort_Nm[i] = 0.0;
    }
  }

  writer_state_.timestamp_s = next_stamp;
  writer_state_.received_steady_s = received_s;
  ++writer_state_.feedback_sequence;
  writer_state_.valid = true;
  have_joint_state_ = true;
  const bool published = state_buffer_.write(writer_state_);
  accepted_feedback_callbacks_.fetch_add(1, std::memory_order_relaxed);
  if (published && active_.load(std::memory_order_acquire)) {
    ready_.store(true, std::memory_order_release);
  }
}

bool RosControlBackend::writeCommand(const MotionCommand &command) {
  if (!isReady() ||
      command.type == CommandType::NONE) {
    return command.type == CommandType::NONE;
  }
  if (command.type != CommandType::JOINT_POSITION) {
    return false;
  }

  // JointGroupPositionController is a setpoint bridge. MotionExecutor owns all
  // interpolation and emits the dense jerk-limited position stream.
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (!std::isfinite(command.joint_position_rad[i]) ||
        !std::isfinite(command.joint_velocity_rad_s[i]) ||
        !std::isfinite(command.joint_acceleration_rad_s2[i])) { return false; }
    command_message_.data[i] = command.joint_position_rad[i];
  }
  try { command_pub_->publish(command_message_); }
  catch (const std::exception &) { return false; }
  return true;
}

RosControlBackend::FeedbackDiagnostics
RosControlBackend::feedbackDiagnostics() const {
  FeedbackDiagnostics diagnostics;
  diagnostics.accepted_callbacks =
      accepted_feedback_callbacks_.load(std::memory_order_relaxed);
  diagnostics.rejected_stamps =
      rejected_feedback_stamps_.load(std::memory_order_relaxed);
  diagnostics.rejected_samples =
      rejected_feedback_samples_.load(std::memory_order_relaxed);
  diagnostics.dropped_buffer_writes = state_buffer_.droppedWrites();
  diagnostics.max_source_gap_s =
      max_source_gap_s_.load(std::memory_order_relaxed);
  diagnostics.max_receive_gap_s =
      max_receive_gap_s_.load(std::memory_order_relaxed);
  return diagnostics;
}

bool RosControlBackend::readState(RobotState &state) {
  if (!state_buffer_.read(state)) {
    state.valid = false;
    return false;
  }
  state.feedback_age_s = steadySeconds() - state.received_steady_s;
  DriverHealth health;
  state.safety_known = health_buffer_.read(health) &&
      steadySeconds() - health.received_s <= feedback_timeout_s_;
  if (require_driver_health_ && !state.safety_known) { state.valid = false; return false; }
  if (state.safety_known) {
    state.emergency_stop = health.data[8] != 0 || health.data[12] != 0 || health.data[13] != 0;
    state.collision_detected = health.data[9] != 0;
    state.robot_mode = static_cast<int>(health.data[7]);
    state.main_error_code = static_cast<int>(health.data[10]);
    state.sub_error_code = static_cast<int>(health.data[11]);
    state.sdk_read_errors = static_cast<uint64_t>(health.data[14]);
    state.sdk_write_errors = static_cast<uint64_t>(health.data[15]);
    if (health.data[4] != 0 || health.data[5] != 0 || health.data[6] == 0 || health.data[7] != 0) {
      state.valid = false; return false;
    }
  }
  if (!active_.load(std::memory_order_acquire) ||
      state.feedback_age_s > feedback_timeout_s_ || state.feedback_age_s < 0.0) {
    state.valid = false;
    return false;
  }
  return state.valid;
}

} // namespace fr10_motion
