#include "fr10_insertion_manager/insertion_ros_adapter.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

#include "fr10_force_control/wrench_transform.hpp"
#include "fr10_motion_core/math_utils.hpp"
#include "fr10_insertion_manager/insertion_geometry_monitor.hpp"

namespace fr10_insertion {
namespace {

fr10_force::CartesianPose toCorePose(const geometry_msgs::msg::Pose &pose) {
  return {{pose.position.x, pose.position.y, pose.position.z},
          {pose.orientation.x, pose.orientation.y, pose.orientation.z,
           pose.orientation.w}};
}

geometry_msgs::msg::Pose toRosPose(const fr10_force::CartesianPose &pose) {
  geometry_msgs::msg::Pose result;
  result.position.x = pose.position_m[0];
  result.position.y = pose.position_m[1];
  result.position.z = pose.position_m[2];
  result.orientation.x = pose.orientation_xyzw[0];
  result.orientation.y = pose.orientation_xyzw[1];
  result.orientation.z = pose.orientation_xyzw[2];
  result.orientation.w = pose.orientation_xyzw[3];
  return result;
}

fr10_force::Wrench toCoreWrench(const geometry_msgs::msg::Wrench &wrench) {
  return {{wrench.force.x, wrench.force.y, wrench.force.z},
          {wrench.torque.x, wrench.torque.y, wrench.torque.z}};
}

geometry_msgs::msg::Wrench toRosWrench(const fr10_force::Wrench &wrench) {
  geometry_msgs::msg::Wrench result;
  result.force.x = wrench.force_n[0];
  result.force.y = wrench.force_n[1];
  result.force.z = wrench.force_n[2];
  result.torque.x = wrench.torque_nm[0];
  result.torque.y = wrench.torque_nm[1];
  result.torque.z = wrench.torque_nm[2];
  return result;
}

const char *stateName(InsertionState state) {
  switch (state) {
  case InsertionState::IDLE: return "IDLE";
  case InsertionState::MOVE_PREINSERT: return "MOVE_PREINSERT";
  case InsertionState::HOLD_PREINSERT: return "HOLD_PREINSERT";
  case InsertionState::FT_ZERO: return "FT_ZERO";
  case InsertionState::APPROACH: return "APPROACH";
  case InsertionState::CONTACT_DETECTED: return "CONTACT_DETECTED";
  case InsertionState::FORCE_INSERTION: return "FORCE_INSERTION";
  case InsertionState::VERIFY_INSERTION: return "VERIFY_INSERTION";
  case InsertionState::SUCCESS: return "SUCCESS";
  case InsertionState::HOLD: return "HOLD";
  case InsertionState::RETRACT: return "RETRACT";
  case InsertionState::STOPPING: return "STOPPING";
  case InsertionState::FAILED: return "FAILED";
  case InsertionState::CONTACT_ALIGN: return "CONTACT_ALIGN";
  case InsertionState::INSERT_COMPLIANT: return "INSERT_COMPLIANT";
  case InsertionState::SEATING: return "SEATING";
  case InsertionState::APPROACH_COMPLIANT: return "APPROACH_COMPLIANT";
  }
  return "UNKNOWN";
}

} // namespace

InsertionRosAdapter::InsertionRosAdapter(
    rclcpp::Node *node, ForceSessionConfig base_config,
    std::string base_frame, WrenchAdapterConfig wrench_adapter,
    std::string profile_id, bool enabled, AcceptGuard accept_guard)
    : node_(node), base_config_(std::move(base_config)),
      base_frame_(std::move(base_frame)),
      wrench_source_frame_(std::move(wrench_adapter.source_frame)),
      wrench_frame_(std::move(wrench_adapter.target_frame)),
      wrench_target_from_source_(wrench_adapter.target_from_source),
      lateral_tolerance_m_(wrench_adapter.lateral_tolerance_m),
      orientation_tolerance_rad_(wrench_adapter.orientation_tolerance_rad),
      mating_clearance_m_(wrench_adapter.mating_clearance_m),
      mating_feature_lever_m_(wrench_adapter.mating_feature_lever_m),
      profile_id_(std::move(profile_id)), enabled_(enabled),
      accept_guard_(std::move(accept_guard)),
      compensator_(wrench_adapter.compensation),
      filter_(wrench_adapter.filter),
      tare_(wrench_adapter.tare),
      tare_max_joint_velocity_rad_s_(wrench_adapter.tare_max_joint_velocity_rad_s),
      tare_pose_tolerance_m_(wrench_adapter.tare_pose_tolerance_m),
      tare_orientation_tolerance_rad_(wrench_adapter.tare_orientation_tolerance_rad) {
  require_source_status_ = wrench_adapter.require_source_status;
  for (double value : {tare_max_joint_velocity_rad_s_, tare_pose_tolerance_m_,
                       tare_orientation_tolerance_rad_})
    if (!std::isfinite(value) || value <= 0.)
      throw std::invalid_argument("invalid stationary tare limits");
  insertion_tf_pub_ = node_->create_publisher<tf2_msgs::msg::TFMessage>("/tf",100);
  insertion_tf_message_.transforms.resize(1);
  insertion_tf_message_.transforms[0].header.frame_id = base_frame_;
  simulation_time_ = node_->has_parameter("use_sim_time") && node_->get_parameter("use_sim_time").as_bool();
  if (wrench_source_frame_.empty() || wrench_frame_.empty() ||
      !fr10_force::WrenchTransform::valid(wrench_target_from_source_) ||
      (base_config_.state_machine.process_mode == InsertionProcessMode::MATCHED_MATING &&
       (!std::isfinite(mating_clearance_m_) || mating_clearance_m_ <= 0. ||
        !std::isfinite(mating_feature_lever_m_) || mating_feature_lever_m_ <= 0.))) {
    throw std::invalid_argument("wrench adapter configuration is invalid");
  }
  result_pub_ = node_->create_publisher<Insert::Result>(
      "/fr10_force/result", rclcpp::QoS(1).reliable().transient_local());
  feedback_pub_ = node_->create_publisher<Insert::Feedback>(
      "/fr10_force/feedback", rclcpp::SensorDataQoS());
  action_server_ = rclcpp_action::create_server<Insert>(
      node_, "/fr10_force/insert",
      std::bind(&InsertionRosAdapter::handleGoal, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&InsertionRosAdapter::handleCancel, this,
                std::placeholders::_1),
      std::bind(&InsertionRosAdapter::handleAccepted, this,
                std::placeholders::_1));
  wrench_sub_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
      wrench_adapter.wrench_topic, rclcpp::SensorDataQoS(),
      std::bind(&InsertionRosAdapter::wrenchCallback, this,
                std::placeholders::_1));
  driver_status_sub_ = node_->create_subscription<fr10_motion_interfaces::msg::WrenchSourceStatus>(
      wrench_adapter.source_status_topic, 1,
      [this](const fr10_motion_interfaces::msg::WrenchSourceStatus::ConstSharedPtr &status) {
        driver_status_seen_.store(true);
        // Any overload is fail-closed, including a duplicate status packet.
        if (status->overload) driver_overload_.store(true);
        // An offline report is immediately authoritative even when it repeats
        // the last transport sequence; online duplicates cannot refresh it.
        if (!status->online) driver_online_.store(false);
        if (status->sequence <= driver_status_sequence_) return;
        driver_status_sequence_ = status->sequence;
        driver_online_.store(status->online);
        driver_overload_.store(status->overload);
        driver_status_receipt_s_.store(steadyNow());
      });
  tare_service_ =
      node_->create_service<fr10_motion_interfaces::srv::TareWrench>(
          "/ft_sensor/tare",
          std::bind(&InsertionRosAdapter::tareCallback, this,
                    std::placeholders::_1, std::placeholders::_2));
  filtered_wrench_pub_ =
      node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
          "/fr10_force/filtered_wrench", rclcpp::SensorDataQoS());
  force_error_pub_ =
      node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
          "/fr10_force/force_error", rclcpp::SensorDataQoS());
  nominal_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/fr10_force/nominal_pose", rclcpp::SensorDataQoS());
  corrected_pose_pub_ =
      node_->create_publisher<geometry_msgs::msg::PoseStamped>(
          "/fr10_force/corrected_pose", rclcpp::SensorDataQoS());
  correction_pub_ =
      node_->create_publisher<geometry_msgs::msg::TwistStamped>(
          "/fr10_force/correction", rclcpp::SensorDataQoS());
  contact_state_pub_ = node_->create_publisher<std_msgs::msg::UInt8>(
      "/fr10_force/contact_state", rclcpp::QoS(1).reliable());
  state_pub_ = node_->create_publisher<std_msgs::msg::UInt8>(
      "/fr10_force/state", rclcpp::QoS(1).reliable());
  source_status_pub_ =
      node_->create_publisher<
          fr10_motion_interfaces::msg::WrenchSourceStatus>(
          "/ft_sensor/status", rclcpp::QoS(1).reliable().transient_local());
}

bool InsertionRosAdapter::busy() const {
  std::lock_guard<std::mutex> lock(goal_mutex_);
  return goal_handle_ != nullptr || internal_goal_ || admission_reserved_ || hold_owned_.load();
}

const char *InsertionRosAdapter::diagnosticPhase() const {
  std::lock_guard<std::mutex> lock(goal_mutex_);
  if (!goal_handle_ && !internal_goal_ && !admission_reserved_ && !hold_owned_.load())
    return nullptr;
  const auto state = published_state_.load(std::memory_order_acquire);
  if (pending_available_ || admission_reserved_ || state == InsertionState::IDLE)
    return "WAITING_ADMISSION";
  return stateName(state);
}

bool InsertionRosAdapter::freshUniqueWrench() const {
  if ((require_source_status_ && !driver_status_seen_.load()) ||
      driver_overload_.load() || (driver_status_seen_.load() &&
      (!driver_online_.load() || steadyNow()-driver_status_receipt_s_.load() >
       base_config_.safety.stale_timeout_s))) return false;
  std::lock_guard<std::mutex> lock(wrench_mutex_);
  return wrench_writer_.sample.sequence > consumed_wrench_sequence_ &&
      wrench_writer_.sample.valid &&
      steadyNow()-wrench_writer_.sample.receipt_steady_s <= base_config_.safety.stale_timeout_s;
}

bool InsertionRosAdapter::requiresFreshWrenchForResume() const {
  return session_ && session_->stateMachine().requiresFreshWrenchForResume();
}

rclcpp_action::GoalResponse InsertionRosAdapter::handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Insert::Goal> goal) {
  if (!enabled_ || busy() || (accept_guard_ && !accept_guard_())) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  std::size_t selected_axis = 0;
  for (std::size_t axis = 0; axis < goal->selected_axes.size(); ++axis) {
    if (goal->selected_axes[axis]) {
      selected_axis = axis;
    }
  }
  const auto pose = toCorePose(goal->target_pose.pose);
  const auto desired = toCoreWrench(goal->desired_wrench);
  const double timeout_s = static_cast<double>(goal->timeout.sec) +
      1e-9 * static_cast<double>(goal->timeout.nanosec);
  const bool limits_valid = std::all_of(
      goal->max_force.begin(), goal->max_force.end(),
      [](double value) { return std::isfinite(value) && value > 0.0; }) &&
      std::all_of(goal->max_torque.begin(), goal->max_torque.end(),
      [](double value) { return std::isfinite(value) && value > 0.0; });
  const bool valid = goal->target_pose.header.frame_id == base_frame_ &&
      fr10_force::finite(pose) && fr10_motion::math::quaternionNormSquared(
          pose.orientation_xyzw) > 1e-12 && fr10_force::finite(desired) &&
      std::isfinite(goal->preinsert_distance) &&
      goal->preinsert_distance >= 0.0 &&
      std::isfinite(goal->insertion_depth) && goal->insertion_depth > 0.0 &&
      std::isfinite(goal->insertion_speed) && goal->insertion_speed > 0.0 &&
      std::isfinite(timeout_s) && timeout_s > 0.0 && limits_valid &&
      validInsertionAxes(goal->selected_axes,
          base_config_.state_machine.process_mode == InsertionProcessMode::MATCHED_MATING ? 2 : selected_axis,
          base_config_.state_machine.process_mode) &&
      goal->profile_id == profile_id_ &&
      (goal->completion_behavior == Insert::Goal::HOLD ||
       goal->completion_behavior == Insert::Goal::RETRACT);
  if (!valid) return rclcpp_action::GoalResponse::REJECT;
  std::lock_guard<std::mutex> lock(goal_mutex_);
  if (goal_handle_ || internal_goal_ || admission_reserved_ || hold_owned_.load())
    return rclcpp_action::GoalResponse::REJECT;
  admission_reserved_ = true;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse InsertionRosAdapter::handleCancel(
    const std::shared_ptr<GoalHandle>) {
  cancel_requested_.store(true, std::memory_order_release);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void InsertionRosAdapter::handleAccepted(
    const std::shared_ptr<GoalHandle> goal_handle) {
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    goal_handle_ = goal_handle;
    admission_reserved_ = false;
  }
  enqueueGoal(*goal_handle->get_goal());
}

bool InsertionRosAdapter::submitPoseGoal(const Insert::Goal &goal) {
  if (handleGoal({}, std::make_shared<const Insert::Goal>(goal)) !=
      rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE) return false;
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (goal_handle_ || internal_goal_) return false;
    internal_goal_ = true;
    admission_reserved_ = false;
  }
  enqueueGoal(goal);
  return true;
}

void InsertionRosAdapter::enqueueGoal(const Insert::Goal &request) {
  const auto *goal = &request;
  PendingGoal pending;
  pending.final_pose = toCorePose(goal->target_pose.pose);
  pending.measurement_ns = rclcpp::Time(goal->target_pose.header.stamp).nanoseconds();
  pending.desired_wrench = toCoreWrench(goal->desired_wrench);
  pending.goal.preinsert_distance_m = goal->preinsert_distance;
  pending.goal.insertion_depth_m = goal->insertion_depth;
  pending.goal.insertion_speed_m_s = goal->insertion_speed;
  pending.goal.selected_axes = goal->selected_axes;
  pending.goal.insertion_axis = 2;
  if (base_config_.state_machine.process_mode == InsertionProcessMode::AXIAL_CONTACT)
    for (std::size_t axis = 0; axis < 3; ++axis)
      if (goal->selected_axes[axis]) pending.goal.insertion_axis = axis;
  const auto desired = pending.desired_wrench.vector();
  pending.goal.desired_axis_force_n =
      desired[pending.goal.insertion_axis];
  pending.goal.timeout_s = static_cast<double>(goal->timeout.sec) +
      1e-9 * static_cast<double>(goal->timeout.nanosec);
  pending.goal.completion_behavior =
      goal->completion_behavior == Insert::Goal::RETRACT
          ? CompletionBehavior::RETRACT : CompletionBehavior::HOLD;
  pending.max_force_n = goal->max_force;
  pending.max_torque_nm = goal->max_torque;
  pending.profile_id = goal->profile_id;
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    pending_goal_ = pending;
    pending_available_ = true;
  }
  cancel_requested_.store(false, std::memory_order_release);
}

std::optional<InsertionRosAdapter::PendingGoal>
InsertionRosAdapter::takePendingGoal() {
  std::lock_guard<std::mutex> lock(goal_mutex_);
  if (!pending_available_) return std::nullopt;
  pending_available_ = false;
  return pending_goal_;
}

bool InsertionRosAdapter::startSession(
    const PendingGoal &goal, const fr10_force::Vector3 &axis_base,
    uint64_t generation, uint64_t scene_revision, double source_time_s) {
  if (!simulation_time_) source_time_s = steadyNow();
  ForceSessionConfig config = base_config_;
  config.admittance.selected_axes = goal.goal.selected_axes;
  config.safety.insertion_axis = goal.goal.insertion_axis;
  config.safety.max_force_n = goal.max_force_n;
  config.safety.max_torque_nm = goal.max_torque_nm;
  config.safety.max_depth_m = goal.goal.insertion_depth_m +
      config.state_machine.depth_tolerance_m;
  if (!fr10_motion::math::quaternionToRotationMatrix(goal.final_pose.orientation_xyzw,
      config.rotation_base_from_insertion)) return false;
  auto session = std::make_unique<ForceSession>(config);
  if (!session->start(goal.goal, goal.final_pose, axis_base,
                      goal.desired_wrench, generation, scene_revision,
                      source_time_s)) {
    return false;
  }
  active_goal_ = goal;
  axis_base_ = axis_base;
  generation_ = generation;
  insertion_axes_frame_ = "fr10_force/insertion_axes_at_tcp_"+std::to_string(generation);
  insertion_tf_message_.transforms[0].child_frame_id = insertion_axes_frame_;
  auto frame_pose = goal.final_pose;
  const double inverse_norm = 1.0 / std::sqrt(
      fr10_motion::math::quaternionNormSquared(frame_pose.orientation_xyzw));
  for (double &value : frame_pose.orientation_xyzw) value *= inverse_norm;
  insertion_tf_message_.transforms[0].transform.rotation =
      toRosPose(frame_pose).orientation;
  scene_revision_ = scene_revision;
  session_ = std::move(session);
  completion_reported_ = false;
  hold_retract_requested_.store(false);
  contact_pose_ = goal.final_pose;
  preinsert_pose_ = goal.final_pose;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    contact_pose_.position_m[axis] -=
        axis_base_[axis] * goal.goal.insertion_depth_m;
    preinsert_pose_.position_m[axis] = contact_pose_.position_m[axis] -
        axis_base_[axis] * goal.goal.preinsert_distance_m;
  }
  retract_pose_ = preinsert_pose_;
  retract_corridor_.reset();
  // Keep the previous calibrated bias until a qualified replacement exists.
  tare_.reset(compensator_.bias());
  filter_.reset();
  have_conditioned_wrench_ = false;
  consumed_wrench_sequence_ = 0;
  last_conditioned_source_time_s_ = 0.0;
  tare_complete_ = false;
  ++tare_generation_;
  last_source_time_s_ = source_time_s;
  peak_force_n_ = 0.0;
  peak_torque_nm_ = 0.0;
  previous_command_accepted_ = true;
  {
    std::lock_guard<std::mutex> lock(wrench_mutex_);
    wrench_envelope_.reset();
    wrench_batch_size_=0;
  }
  return true;
}

bool InsertionRosAdapter::reframePreinsert(const PendingGoal &goal, uint64_t scene_revision) {
  fr10_force::RotationMatrix rotation;
  if (!fr10_motion::math::quaternionToRotationMatrix(goal.final_pose.orientation_xyzw, rotation)) return false;
  fr10_force::Vector3 local_axis{};local_axis[goal.goal.insertion_axis]=1.;
  const auto axis=fr10_motion::math::rotateVector(goal.final_pose.orientation_xyzw,local_axis);
  if (!session_ || !session_->reframePreinsert(goal.final_pose,axis,rotation,scene_revision)) return false;
  active_goal_=goal;axis_base_=axis;scene_revision_=scene_revision;
  contact_pose_=preinsert_pose_=goal.final_pose;
  for(unsigned i=0;i<3;++i) {
    contact_pose_.position_m[i]-=axis[i]*goal.goal.insertion_depth_m;
    preinsert_pose_.position_m[i]=contact_pose_.position_m[i]-axis[i]*goal.goal.preinsert_distance_m;
  }
  insertion_tf_message_.transforms[0].transform.rotation=toRosPose(goal.final_pose).orientation;
  retract_pose_ = preinsert_pose_;
  return true;
}

void InsertionRosAdapter::rejectPending(uint16_t code,
                                        const std::string &message) {
  std::shared_ptr<GoalHandle> handle;
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    handle = goal_handle_;
    goal_handle_.reset();
    pending_available_ = false;
    internal_goal_ = false;
    admission_reserved_ = false;
  }
  auto result = std::make_shared<Insert::Result>();
  result->success = false;
  result->code = code;
  result->message = message;
  result_pub_->publish(*result);
  if (handle && handle->is_active()) {
    if (code == Insert::Result::CANCELED && handle->is_canceling()) handle->canceled(result);
    else handle->abort(result);
  }
  session_.reset();
}

void InsertionRosAdapter::wrenchCallback(
    const geometry_msgs::msg::WrenchStamped::ConstSharedPtr &message) {
  WrenchBuffer next;
  next.frame_id = message->header.frame_id;
  next.sample.wrench = toCoreWrench(message->wrench);
  next.sample.receipt_steady_s = steadyNow();
  const int64_t stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
  const bool duplicate = stamp_ns != 0 && stamp_ns == last_wrench_stamp_ns_;
  if (!duplicate) {
    ++wrench_sequence_;
    last_wrench_stamp_ns_ = stamp_ns;
  }
  next.sample.sequence = wrench_sequence_;
  next.sample.source_timestamp_s = 1e-9 * static_cast<double>(stamp_ns);
  next.sample.valid = stamp_ns > 0 && message->header.frame_id == wrench_source_frame_ &&
      fr10_force::finite(next.sample.wrench);
  std::lock_guard<std::mutex> lock(wrench_mutex_);
  wrench_envelope_.observe(next.sample.wrench,next.sample.valid);
  wrench_writer_ = next;
  if(!duplicate) {
    if(wrench_batch_size_<wrench_batch_.size()) wrench_batch_[wrench_batch_size_++]=next.sample;
    else wrench_envelope_.valid=false; // Overflow is lost evidence, never silently decimated.
  }
}

void InsertionRosAdapter::tareCallback(
    const std::shared_ptr<
        fr10_motion_interfaces::srv::TareWrench::Request> request,
    std::shared_ptr<fr10_motion_interfaces::srv::TareWrench::Response>
        response) {
  const auto state = published_state_.load(std::memory_order_acquire);
  response->accepted = enabled_ && (state == InsertionState::IDLE ||
      state == InsertionState::HOLD_PREINSERT || state == InsertionState::FT_ZERO);
  if (response->accepted) {
    tare_reset_bias_.store(request->reset_existing, std::memory_order_release);
    response->generation = tare_request_generation_.fetch_add(1) + 1;
    tare_requested_.store(true, std::memory_order_release);
  }
  response->message = response->accepted ? "tare queued for control-owner FT_ZERO phase"
                                       : "tare rejected while disabled or in motion/contact";
}

ForceSessionOutput InsertionRosAdapter::update(
    const RuntimeInput &input, fr10_motion::IKProvider &provider) {
  ForceSessionOutput output;
  if (!session_) return output;
  const double command_source_time_s = node_->now().seconds();
  const double source_time_s = simulation_time_ ? command_source_time_s : steadyNow();
  const double source_dt_s = source_time_s - last_source_time_s_;
  if (source_dt_s == 0.0) {
    return output;
  }
  last_source_time_s_ = source_time_s;

  WrenchBuffer wrench;
  fr10_force::RawWrenchEnvelope raw_envelope;
  std::array<fr10_force::WrenchSample,kWrenchBatchCapacity> batch;
  std::size_t batch_size=0;
  bool conditioned_reference_changed = false;
  if (tare_requested_.exchange(false, std::memory_order_acq_rel)) {
    const auto reference = tare_reset_bias_.exchange(false)
        ? fr10_force::Wrench{} : compensator_.bias();
    tare_.reset(reference);
    tare_complete_ = false;
    ++tare_generation_;
  }
  const auto geometry = observeInsertionGeometry(
      contact_pose_, axis_base_, input.actual_pose,
      lateral_tolerance_m_, orientation_tolerance_rad_);
  using namespace fr10_motion::math;
  fr10_force::Vector6 measured_correction{};
  if (geometry.valid) {
    fr10_force::Vector3 lateral{};
    for (std::size_t i = 0; i < 3; ++i)
      lateral[i] = input.actual_pose.position_m[i] - contact_pose_.position_m[i] - geometry.depth_m * axis_base_[i];
    const auto translation = rotateVector(quaternionConjugate(contact_pose_.orientation_xyzw), lateral);
    const auto rotation = quaternionToRotationVector(quaternionMultiply(
        quaternionConjugate(contact_pose_.orientation_xyzw), input.actual_pose.orientation_xyzw));
    for (std::size_t i = 0; i < 3; ++i) {
      measured_correction[i] = translation[i];
      measured_correction[i + 3] = rotation[i];
    }
  }
  fr10_force::RotationMatrix base_from_tcp{};
  const bool rotation_valid = fr10_motion::math::quaternionToRotationMatrix(
      input.actual_pose.orientation_xyzw, base_from_tcp);
  fr10_force::RotationMatrix source_from_base{};
  // R_BS = R_BT R_TS; gravity is configured in the base/reference frame.
  for (std::size_t i=0; i<3; ++i)
    for (std::size_t j=0; j<3; ++j)
      for (std::size_t k=0; k<3; ++k)
        source_from_base[i*3+j] +=
            wrench_target_from_source_.rotation_target_from_source[k*3+i] * base_from_tcp[j*3+k];
  {
    std::lock_guard<std::mutex> lock(wrench_mutex_);
    wrench = wrench_writer_;
    raw_envelope=wrench_envelope_;
    wrench_envelope_.reset();
    batch_size=wrench_batch_size_;
    std::copy_n(wrench_batch_.begin(),batch_size,batch.begin());
    wrench_batch_size_=0;
  }
  const bool source_seen = driver_status_seen_.load();
  const bool source_healthy = (!require_source_status_ || source_seen) &&
      (!source_seen || (driver_online_.load() && !driver_overload_.load() &&
          steadyNow()-driver_status_receipt_s_.load() <= base_config_.safety.stale_timeout_s));
  wrench.sample.valid = wrench.sample.valid && source_healthy;
  wrench.sample.overload = driver_overload_.load();
  last_raw_sample_ = wrench.sample;
  if (session_->stateMachine().state() == InsertionState::FT_ZERO &&
      !tare_complete_) {
    // Tare must not alias alternating physics/sensor substeps into DC bias.
    // The same stationary control qualification covers every raw batch sample;
    // mean AND variance retain all unique source evidence, before filtering.
    if (batch_size==0) batch[batch_size++]=wrench.sample;
    std::array<fr10_force::Wrench,kWrenchBatchCapacity> gravity_removed;
    const auto prior_bias = compensator_.bias().vector();
    for (std::size_t sample_index=0; sample_index<batch_size; ++sample_index) {
      auto value = compensator_.apply(batch[sample_index].wrench, source_from_base).vector();
      for (std::size_t i=0; i<6; ++i) value[i] += prior_bias[i];
      gravity_removed[sample_index] = fr10_force::Wrench::fromVector(value);
    }
    const bool stationary = input.measured_joint_velocity_rad_s &&
        fr10_force::finite(*input.measured_joint_velocity_rad_s) &&
        std::all_of(input.measured_joint_velocity_rad_s->begin(), input.measured_joint_velocity_rad_s->end(),
            [this](double velocity) { return std::abs(velocity) <= tare_max_joint_velocity_rad_s_; });
    // Clearance is qualified by pose/scene independently of force. A tool
    // carrying its normal payload can tare here after gravity removal.
    const bool no_contact = input.actual_pose_valid && geometry.valid &&
        geometry.depth_m < -tare_pose_tolerance_m_ && input.collision_free &&
        input.current_scene_revision == scene_revision_ && input.base_stationary &&
        fr10_motion::math::distance3D(input.actual_pose.position_m,
                                     preinsert_pose_.position_m) <= tare_pose_tolerance_m_ &&
        fr10_motion::math::quaternionAngularDistance(input.actual_pose.orientation_xyzw,
            preinsert_pose_.orientation_xyzw) <= tare_orientation_tolerance_rad_;
    const auto bias = tare_.observeBatch(batch.data(), gravity_removed.data(), batch_size,
        stationary && no_contact && source_healthy &&
            (!raw_envelope.populated || raw_envelope.valid));
    if (bias) {
      tare_complete_ = compensator_.setBias(*bias);
      const auto value = bias->vector();
      RCLCPP_INFO(node_->get_logger(),
          "[TARE QUALIFIED] raw_samples=%zu bias=[%.6f,%.6f,%.6f,%.6f,%.6f,%.6f]",
          tare_.sampleCount(),value[0],value[1],value[2],value[3],value[4],value[5]);
      conditioned_reference_changed = tare_complete_;
      filter_.reset();
      last_conditioned_source_time_s_ = 0.0;
      have_conditioned_wrench_ = false;
    }
  }
  const bool new_wrench = wrench.sample.sequence > consumed_wrench_sequence_;
  if (new_wrench) {
    consumed_wrench_sequence_ = wrench.sample.sequence;
    if (wrench.sample.valid) {
      if(batch_size==0) batch[batch_size++]=wrench.sample;
      for(std::size_t index=0;index<batch_size;++index) {
        const auto &sensor_sample=batch[index];
        if(!sensor_sample.valid) continue;
        const auto compensated = compensator_.apply(sensor_sample.wrench, source_from_base);
        const auto at_tcp = fr10_force::WrenchTransform::apply(
            compensated, wrench_target_from_source_);
        // The TCP rotates with the measured arm; the insertion axes stay fixed
        // for the session. Moments are already shifted to TCP by the first map.
        const auto transformed = fr10_force::WrenchTransform::apply(
            at_tcp, geometry.insertion_from_tcp);
        fr10_force::WrenchSample conditioned_sample = sensor_sample;
        conditioned_sample.wrench = transformed;
        conditioned_sample.valid = conditioned_sample.valid &&
            input.actual_pose_valid && geometry.valid && rotation_valid &&
            fr10_force::finite(transformed);
        // Condition every unique received source sample before decimation.
        // Reuse this control observation's frame map for the bounded batch;
        // no planning/network call or additional integration occurs per sample.
        const double filter_dt_s = last_conditioned_source_time_s_ > 0.0
            ? sensor_sample.source_timestamp_s - last_conditioned_source_time_s_
            : 0.008;
        const auto filtered = filter_.update(conditioned_sample, filter_dt_s);
        if (filtered.accepted) {
          last_conditioned_tcp_pose_ = input.actual_pose;
          last_conditioned_source_time_s_ = sensor_sample.source_timestamp_s;
          last_conditioned_wrench_ = filtered.sample.wrench;
          have_conditioned_wrench_ = true;
        }
      }
    }
  }

  ForceSessionInput session_input;
  session_input.raw_sample = last_raw_sample_;
  if(raw_envelope.populated) {
    session_input.raw_sample.wrench=raw_envelope.peak;
    session_input.raw_sample.valid=session_input.raw_sample.valid && raw_envelope.valid;
  }
  if (!have_conditioned_wrench_) session_input.raw_sample.valid = false;
  session_input.conditioned_wrench = last_conditioned_wrench_;
  session_input.joint_seed = input.joint_seed;
  session_input.source_time_s = source_time_s;
  session_input.command_source_time_s = command_source_time_s;
  session_input.now_steady_s = steadyNow();
  session_input.dt_s = std::max(0.0, source_dt_s);
  if (input.actual_pose_valid && geometry.valid) session_input.actual_correction = measured_correction;
  session_input.allow_motion_proposal = input.allow_motion_proposal;
  session_input.preinsert_reached = input.preinsert_reached;
  session_input.planning_failed = input.planning_failed;
  session_input.tare_complete = tare_complete_;
  session_input.conditioned_reference_changed = conditioned_reference_changed;
  session_input.mating_valid = base_config_.state_machine.process_mode != InsertionProcessMode::MATCHED_MATING ||
      (input.actual_pose_valid && withinMatingClearance(geometry, mating_clearance_m_, mating_feature_lever_m_));
  if (session_->contactReference().enabled()) {
    const auto &reference = session_->contactReference().state();
    // This is a tracking envelope about a bounded contact-derived anchor, not
    // knowledge of the true port pose. Physics fit is evaluated independently.
    auto estimated_contact = contact_pose_;
    const auto equilibrium = contactEquilibriumAtDepth(reference, 0.);
    const auto shift = rotateVector(contact_pose_.orientation_xyzw,
        {equilibrium[0], equilibrium[1], 0.});
    for (std::size_t i = 0; i < 3; ++i) estimated_contact.position_m[i] += shift[i];
    estimated_contact.orientation_xyzw = quaternionMultiply(contact_pose_.orientation_xyzw,
        rotationVectorToQuaternion({reference.equilibrium[3], reference.equilibrium[4], reference.equilibrium[5]}));
    const auto estimated_axis = rotateVector(estimated_contact.orientation_xyzw, {0., 0., 1.});
    const auto alignment = observeInsertionGeometry(estimated_contact, estimated_axis, input.actual_pose,
        lateral_tolerance_m_, orientation_tolerance_rad_);
    session_input.mating_valid = reference.mode == ContactReferenceMode::ACQUIRED &&
        withinMatingClearance(alignment, mating_clearance_m_, mating_feature_lever_m_);
  }
  // Command target, depth, lateral/orientation tracking and mating checks must
  // refer to the same accepted withdrawal corridor. The immutable input frame
  // remains the authority for FT conditioning, not a second withdrawal axis.
  const auto tracking_geometry = retract_corridor_ ? observeInsertionGeometry(
      *retract_corridor_,input.actual_pose,
      lateral_tolerance_m_,orientation_tolerance_rad_) : geometry;
  session_input.actual_depth_m = tracking_geometry.depth_m;
  session_input.lateral_valid = input.actual_pose_valid && tracking_geometry.valid && tracking_geometry.lateral_valid;
  session_input.orientation_valid = input.actual_pose_valid && tracking_geometry.valid && tracking_geometry.orientation_valid;
  if(retract_corridor_)session_input.mating_valid=withinMatingClearance(
      tracking_geometry,mating_clearance_m_,mating_feature_lever_m_);
  session_input.tracking_valid = input.tracking_valid && input.actual_pose_valid && tracking_geometry.valid;
  session_input.robot_ok = input.robot_ok && input.actual_pose_valid && tracking_geometry.valid;
  session_input.base_stationary = input.base_stationary;
  session_input.collision_free = input.collision_free;
  session_input.scene_unchanged =
      input.current_scene_revision == scene_revision_;
  session_input.command_accepted = previous_command_accepted_;
  session_input.stop_complete = input.stop_complete;
  session_input.retract_complete = input.retract_complete;
  session_input.retract_requested = hold_retract_requested_.exchange(false);
  session_input.canceled = cancel_requested_.load(std::memory_order_acquire);
  const auto previous_state=session_->stateMachine().state();
  output = session_->update(session_input, provider);
  if (output.decision.state == InsertionState::RETRACT && previous_state != InsertionState::RETRACT &&
      session_->contactReference().enabled() && input.actual_pose_valid && geometry.valid) {
    // Withdraw on the accepted physical insertion orientation without pulling
    // sideways toward the uncertain original input while pins remain engaged.
    retract_corridor_=InsertionCorridor{input.actual_pose,
        rotateVector(input.actual_pose.orientation_xyzw,{0.,0.,1.})};
    retract_pose_ = input.actual_pose;
    for (std::size_t i = 0; i < 3; ++i) {
      retract_corridor_->mouth_pose.position_m[i] -= retract_corridor_->axis_base[i] * geometry.depth_m;
      retract_pose_.position_m[i] -= retract_corridor_->axis_base[i] *
          (geometry.depth_m + active_goal_.goal.preinsert_distance_m);
    }
  }
  if(output.decision.state==InsertionState::VERIFY_INSERTION)
    RCLCPP_INFO_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,
        "[VERIFY OBSERVATION] force_error=%.4f immediate_rate=%.4f verification_rate=%.4f depth_error_mm=%.4f LOMT=%d%d%d%d",
        output.force_error[2],output.safety.force_rate_n_s[2],
        output.verification_force_rate_n_s?(*output.verification_force_rate_n_s)[2]:std::numeric_limits<double>::quiet_NaN(),
        1000.*(geometry.depth_m-active_goal_.goal.insertion_depth_m),
        session_input.lateral_valid,session_input.orientation_valid,session_input.mating_valid,session_input.tracking_valid);
  // An already acknowledged HOLD can transition straight to FAILED in this
  // tick. Its previous success report must not suppress that new fault event.
  if (output.decision.state == InsertionState::RETRACT ||
      output.decision.state == InsertionState::STOPPING ||
      (previous_state==InsertionState::HOLD && output.decision.state!=InsertionState::HOLD))
    completion_reported_ = false;
  previous_command_accepted_ = true;
  peak_force_n_ = std::max(peak_force_n_,
                           fr10_force::norm(last_conditioned_wrench_.force_n));
  peak_torque_nm_ = std::max(
      peak_torque_nm_, fr10_force::norm(last_conditioned_wrench_.torque_nm));
  publish(output, std::max(0.0, steadyNow() -
                                   last_raw_sample_.receipt_steady_s));
  if (output.decision.terminal && !completion_reported_) complete(output, input.actual_pose);
  return output;
}

bool InsertionRosAdapter::commitOnlineTarget(
    uint64_t sequence, bool accepted_and_published) {
  previous_command_accepted_ = accepted_and_published;
  return session_ &&
      session_->commitOnlineTarget(sequence, accepted_and_published);
}

bool InsertionRosAdapter::canCommitOnlineTarget(uint64_t sequence) const {
  return session_ && session_->canCommitOnlineTarget(sequence);
}

void InsertionRosAdapter::discardOnlineTarget(uint64_t sequence) {
  // The FSM withdrew a proposal before actuator submission. Roll it back;
  // do not report a backend rejection or acknowledge an unpublished command.
  if (session_) session_->commitOnlineTarget(sequence, false);
}

void InsertionRosAdapter::requestCancel() {
  cancel_requested_.store(true, std::memory_order_release);
}

bool InsertionRosAdapter::requestHoldRetract() {
  const auto state = published_state_.load();
  if (!hold_owned_.load() || (state != InsertionState::HOLD && state != InsertionState::FAILED))
    return false;
  hold_retract_requested_.store(true);
  return true;
}

InsertionState InsertionRosAdapter::state() const {
  return session_ ? session_->stateMachine().state() : InsertionState::IDLE;
}

bool InsertionRosAdapter::retractRequested() const {
  return state() == InsertionState::RETRACT;
}

void InsertionRosAdapter::publish(const ForceSessionOutput &output,
                                  double source_age_s) {
  published_state_.store(output.decision.state, std::memory_order_release);
  const auto stamp = node_->now();
  geometry_msgs::msg::WrenchStamped filtered;
  filtered.header.stamp = stamp;
  if (last_conditioned_source_time_s_ > 0.0) {
    filtered.header.stamp = rclcpp::Time(static_cast<int64_t>(last_conditioned_source_time_s_*1e9));
  }
  filtered.header.frame_id = insertion_axes_frame_;
  auto &frame = insertion_tf_message_.transforms[0];
  frame.header.stamp = filtered.header.stamp;
  const auto &p = last_conditioned_tcp_pose_.position_m;
  frame.transform.translation.x=p[0]; frame.transform.translation.y=p[1]; frame.transform.translation.z=p[2];
  if (have_conditioned_wrench_) insertion_tf_pub_->publish(insertion_tf_message_);
  filtered.wrench = toRosWrench(last_conditioned_wrench_);
  filtered_wrench_pub_->publish(filtered);
  geometry_msgs::msg::WrenchStamped error = filtered;
  error.wrench = toRosWrench(
      fr10_force::Wrench::fromVector(output.force_error));
  force_error_pub_->publish(error);
  geometry_msgs::msg::PoseStamped nominal;
  nominal.header.stamp = stamp;
  nominal.header.frame_id = base_frame_;
  nominal.pose = toRosPose(output.nominal_pose);
  nominal_pose_pub_->publish(nominal);
  auto corrected = nominal;
  corrected.pose = toRosPose(output.corrected_pose);
  corrected_pose_pub_->publish(corrected);
  geometry_msgs::msg::TwistStamped correction;
  correction.header = nominal.header;
  correction.header.frame_id = insertion_axes_frame_;
  correction.twist.linear.x = output.correction.position[0];
  correction.twist.linear.y = output.correction.position[1];
  correction.twist.linear.z = output.correction.position[2];
  correction.twist.angular.x = output.correction.position[3];
  correction.twist.angular.y = output.correction.position[4];
  correction.twist.angular.z = output.correction.position[5];
  correction_pub_->publish(correction);
  std_msgs::msg::UInt8 contact_state;
  contact_state.data = static_cast<uint8_t>(output.contact_state);
  contact_state_pub_->publish(contact_state);
  std_msgs::msg::UInt8 state;
  state.data = static_cast<uint8_t>(output.decision.state);
  state_pub_->publish(state);

  fr10_motion_interfaces::msg::WrenchSourceStatus status;
  status.header.stamp = stamp;
  status.header.frame_id = wrench_frame_;
  status.online = last_raw_sample_.valid;
  status.calibrated = tare_complete_;
  status.overload = last_raw_sample_.overload;
  status.sequence = last_raw_sample_.sequence;
  status.receipt_age_s = source_age_s;
  status.source = "/ft_sensor/wrench";
  status.detail = stateName(output.decision.state);
  source_status_pub_->publish(status);

  std::shared_ptr<GoalHandle> handle;
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    handle = goal_handle_;
  }
  auto feedback = std::make_shared<Insert::Feedback>();
  feedback->state = static_cast<uint8_t>(output.decision.state);
  feedback->progress = output.decision.progress;
  feedback->depth = output.actual_depth_m;
  feedback->measured_wrench = toRosWrench(last_conditioned_wrench_);
  feedback->force_error = error.wrench;
  feedback->nominal_pose = nominal.pose;
  feedback->corrected_pose = corrected.pose;
  feedback->correction = output.correction.position;
  feedback->contact_state = static_cast<uint8_t>(output.contact_state);
  feedback->source_age = source_age_s;
  feedback->generation = generation_;
  feedback->detail = stateName(output.decision.state);
  if (session_->contactReference().enabled())
    feedback->detail += "; contact_reference=" + std::to_string(static_cast<int>(output.reference_mode));
  if (session_->contactReference().yawProbe().enabled())
    feedback->detail += "; yaw_probe=" + std::to_string(static_cast<int>(output.yaw_probe_mode));
  if (session_->contactReference().lateralProbe(0).enabled())
    feedback->detail += "; xy_probe=" + std::to_string(static_cast<int>(output.lateral_probe_modes[0])) +
        "," + std::to_string(static_cast<int>(output.lateral_probe_modes[1]));
  feedback->detail += "; contact_lateral=" + std::to_string(static_cast<int>(output.contact_evidence.lateral)) +
      "; contact_tilt=" + std::to_string(static_cast<int>(output.contact_evidence.tilt)) +
      "; contact_torsion=" + std::to_string(static_cast<int>(output.contact_evidence.torsion));
  feedback_pub_->publish(*feedback);
  if (handle && handle->is_active()) handle->publish_feedback(feedback);
}

void InsertionRosAdapter::complete(
    const ForceSessionOutput &output,
    const fr10_force::CartesianPose &actual_pose) {
  std::shared_ptr<GoalHandle> handle;
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    handle = goal_handle_;
    const bool seated_hold = output.decision.success &&
        session_->stateMachine().goal().completion_behavior == CompletionBehavior::HOLD;
    if (seated_hold) hold_owned_.store(true);
    if (!output.decision.success && output.actual_depth_m >= 0. &&
        peak_force_n_ >= base_config_.contact.enter_force_n) hold_owned_.store(true);
    goal_handle_.reset();
    internal_goal_ = false;
  }
  auto result = std::make_shared<Insert::Result>();
  result->success = output.decision.success;
  result->code = resultCode(output.decision);
  const bool retracted = output.decision.success && session_->stateMachine().goal().completion_behavior == CompletionBehavior::RETRACT;
  result->message = output.decision.success ? (retracted ? "RETRACT: verified stationary preinsert boundary" : "HOLD: depth, CAD mating clearance, load and dwell verified") :
      std::string(insertionFaultName(output.decision.fault)) +
      "; safety_fault=" + std::to_string(static_cast<int>(output.decision.safety_fault)) +
      "; controlled stop completed; no automatic retract";
  if (output.decision.success && !retracted && session_->contactReference().enabled())
    result->message = "HOLD: mechanically seated estimate from FT/q and bounded contact anchor; electrical/latch state unconfirmed";
  result->final_depth = output.actual_depth_m;
  result->final_pose = toRosPose(actual_pose);
  result->peak_force = peak_force_n_;
  result->peak_torque = peak_torque_nm_;
  result->controlled_stop_completed =
      output.decision.state == InsertionState::FAILED ||
      (output.decision.success &&
       active_goal_.goal.completion_behavior == CompletionBehavior::HOLD);
  result_pub_->publish(*result);
  RCLCPP_INFO(node_->get_logger(), "[INSERT RESULT] success=%s code=%u depth=%.6f: %s",
              result->success ? "true" : "false", result->code, result->final_depth,
              result->message.c_str());
  if (handle && handle->is_active() && output.decision.success) {
    handle->succeed(result);
  } else if (handle && output.decision.fault == InsertionFault::CANCELED && handle->is_canceling()) {
    handle->canceled(result);
  } else if (handle && handle->is_active()) {
    handle->abort(result);
  }
  completion_reported_ = true;
  if (retracted) hold_owned_.store(false);
  if (!hold_owned_.load()) session_.reset();
  cancel_requested_.store(false, std::memory_order_release);
}

uint16_t InsertionRosAdapter::resultCode(
    const InsertionDecision &decision) {
  if (decision.success) return Insert::Result::SUCCESS;
  if (decision.fault == InsertionFault::INVALID_GOAL)
    return Insert::Result::INVALID_GOAL;
  if (decision.fault == InsertionFault::TIMEOUT)
    return Insert::Result::TIMEOUT;
  if (decision.fault == InsertionFault::CANCELED)
    return Insert::Result::CANCELED;
  if (decision.fault == InsertionFault::CLOCK_JUMP)
    return Insert::Result::CLOCK_JUMP;
  if (decision.fault == InsertionFault::NO_PROGRESS)
    return Insert::Result::NO_PROGRESS;
  if (decision.fault == InsertionFault::PLANNING_FAILED)
    return Insert::Result::PLANNING_FAILED;
  switch (decision.safety_fault) {
  case fr10_force::ForceSafetyFault::STALE_WRENCH:
    return Insert::Result::STALE_WRENCH;
  case fr10_force::ForceSafetyFault::FORCE_LIMIT:
  case fr10_force::ForceSafetyFault::SENSOR_OVERLOAD:
    return Insert::Result::FORCE_LIMIT;
  case fr10_force::ForceSafetyFault::TORQUE_LIMIT:
    return Insert::Result::TORQUE_LIMIT;
  case fr10_force::ForceSafetyFault::FORCE_RATE_LIMIT:
    return Insert::Result::FORCE_RATE_LIMIT;
  case fr10_force::ForceSafetyFault::CORRECTION_LIMIT:
    return Insert::Result::CORRECTION_LIMIT;
  case fr10_force::ForceSafetyFault::DEPTH_LIMIT:
    return Insert::Result::DEPTH_LIMIT;
  case fr10_force::ForceSafetyFault::LOST_CONTACT:
    return Insert::Result::LOST_CONTACT;
  case fr10_force::ForceSafetyFault::IK_FAILURE:
    return Insert::Result::IK_FAILURE;
  case fr10_force::ForceSafetyFault::COLLISION:
    return Insert::Result::COLLISION;
  case fr10_force::ForceSafetyFault::SCENE_CHANGED:
    return Insert::Result::SCENE_CHANGED;
  case fr10_force::ForceSafetyFault::TRACKING_FAILURE:
    return Insert::Result::TRACKING_ERROR;
  case fr10_force::ForceSafetyFault::ROBOT_FAULT:
    return Insert::Result::ROBOT_FAULT;
  case fr10_force::ForceSafetyFault::COMMAND_REJECTED:
    return Insert::Result::COMMAND_REJECTED;
  case fr10_force::ForceSafetyFault::INVALID_WRENCH:
    return Insert::Result::SENSOR_UNAVAILABLE;
  case fr10_force::ForceSafetyFault::CLOCK_JUMP:
    return Insert::Result::CLOCK_JUMP;
  case fr10_force::ForceSafetyFault::NONE:
    break;
  }
  return Insert::Result::SENSOR_UNAVAILABLE;
}

double InsertionRosAdapter::steadyNow() {
  return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

} // namespace fr10_insertion
