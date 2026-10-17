#include "fr10_insertion_manager/parking_ros_adapter.hpp"
#include <chrono>

namespace fr10_insertion {
namespace {
double steadyNow() { return std::chrono::duration<double>(
  std::chrono::steady_clock::now().time_since_epoch()).count(); }
}

ParkingRosAdapter::ParkingRosAdapter(rclcpp::Node *node, const std::string &odom_topic,
                                     const std::string &feedback_frame, bool twist_from_pose, double pose_window_s)
    : node_(node), pose_velocity_(pose_window_s) {
  feedback_group_=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
  rclcpp::SubscriptionOptions feedback_options;
  feedback_options.callback_group=feedback_group_;
  client_owner_id_ = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
  publisher_ = node->create_publisher<fr10_motion_interfaces::msg::VehicleParkingLease>(
      "/vehicle/parking_lease", rclcpp::QoS(10).reliable());
  status_sub_ = node->create_subscription<fr10_motion_interfaces::msg::VehicleParkingState>(
      "/vehicle/parking_state", rclcpp::QoS(10).reliable(),
      [this](fr10_motion_interfaces::msg::VehicleParkingState::ConstSharedPtr message) {
        status_.write(Status{message->owner_id, message->sequence, message->state, steadyNow()});
      },feedback_options);
  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS().keep_last(1),
      [this, feedback_frame, twist_from_pose](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        const auto stamp = rclcpp::Time(message->header.stamp).nanoseconds();
        if (stamp == last_odom_source_ns_) return;
        BaseFeedback sample;
        sample.source_ns = stamp;
        sample.receipt_s = steadyNow();
        sample.valid = stamp > last_odom_source_ns_ &&
          message->header.frame_id == feedback_frame && message->child_frame_id == "base_footprint";
        last_odom_source_ns_ = stamp;
        const auto &pose = message->pose.pose;
        const auto &twist = message->twist.twist;
        sample.position = {pose.position.x, pose.position.y, pose.position.z};
        sample.orientation = {pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w};
        sample.twist = {twist.linear.x, twist.linear.y, twist.linear.z,
                       twist.angular.x, twist.angular.y, twist.angular.z};
        if (twist_from_pose) sample=pose_velocity_.update(sample);
        feedback_.write(sample);
      },feedback_options);
  // Continue receipt-based renewal while physics is paused. It never renews
  // odometry freshness or advances the phase/integration source clock.
  timer_ = node->create_wall_timer(std::chrono::milliseconds(50),
                                  [this] { heartbeat(); },feedback_group_);
  feedback_executor_.add_callback_group(feedback_group_,node->get_node_base_interface());
  feedback_thread_=std::thread([this] {feedback_executor_.spin();});
}
ParkingRosAdapter::~ParkingRosAdapter() {
  feedback_executor_.cancel();
  if(feedback_thread_.joinable()) feedback_thread_.join();
}

void ParkingRosAdapter::acquire() {
  monitor_.reset();
  owner_id_.store(client_owner_id_, std::memory_order_release);
}
void ParkingRosAdapter::release() { owner_id_.store(0, std::memory_order_release); }

void ParkingRosAdapter::heartbeat() {
  const auto owner = owner_id_.load(std::memory_order_acquire);
  if (owner == 0 && published_owner_ == 0) return;
  fr10_motion_interfaces::msg::VehicleParkingLease message;
  message.owner_id = owner ? owner : published_owner_;
  message.sequence = ++sequence_;
  message.release = owner == 0;
  message.lease_duration.nanosec = 300000000;
  publisher_->publish(message);
  if (owner) published_owner_ = owner;
  // Repeat release until acknowledged, surviving a dropped DDS packet.
  if (!owner) {
    Status status;
    if (status_.read(status) && status.owner == 0 && status.sequence >= message.sequence-1)
      published_owner_ = 0;
  }
}

BaseHoldResult ParkingRosAdapter::update() {
  Status status;
  BaseFeedback feedback;
  const bool have_status=status_.read(status);
  feedback_.read(feedback);
  // Capture time AFTER both immutable observations. A callback may publish
  // between an earlier caller timestamp and these reads; that is a new sample,
  // not a future timestamp, lost lease or stale source.
  const double now_s=steadyNow();
  const bool lease_valid=have_status && status.owner==owner_id_.load() &&
    status.owner!=0 && status.sequence!=0 && status.state==2 &&
    now_s>=status.receipt_s && now_s-status.receipt_s<=.050;
  const auto result=monitor_.update(feedback,now_s,lease_valid);
  if(result.fault!=BaseHoldFault::NONE && status.owner!=0)
    RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,
      "[PARKING OBSERVATION] fault=%d state=%u status_age_ms=%.3f base_age_ms=%.3f owner=%lu sequence=%lu",
      static_cast<int>(result.fault),status.state,1000.*(now_s-status.receipt_s),
      1000.*(now_s-feedback.receipt_s),status.owner,status.sequence);
  return result;
}
bool ParkingRosAdapter::freshFeedback() const {
  BaseFeedback sample;
  const bool have_feedback=feedback_.read(sample);
  const double now_s=steadyNow();
  return have_feedback && sample.valid && now_s>=sample.receipt_s &&
    now_s-sample.receipt_s<=.050;
}
} // namespace fr10_insertion
