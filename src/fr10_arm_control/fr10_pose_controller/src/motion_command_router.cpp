#include "motion_adapter_internal.hpp"

namespace fr10_pose_controller::detail {

void MotionAdapter::controlledStopCallback(
      const std_msgs::msg::Empty::ConstSharedPtr & /*message*/) {
    if (stop_requested_.load() || !motion_executor_ ||
        !motion_executor_->initialized()) {
      return;
    }
    RCLCPP_WARN(get_logger(),
                "[CONTROLLED STOP REQUEST] explicit software stop requested");
    if (force_adapter_ && force_adapter_->busy()) {
      coordinator_->reject(fr10_motion::ControlMode::JOINT, fr10_motion::CommandReason::BUSY);
      force_adapter_->requestCancel();
    }
    stop_latch_.store(StopReason::CONTROLLED_STOP);
  }

void MotionAdapter::jointTargetCallback(
      const sensor_msgs::msg::JointState::ConstSharedPtr &message) {
    if (force_adapter_ && force_adapter_->busy()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Joint target rejected: force insertion is BUSY");
      return;
    }
    if (stop_requested_.load() || !motion_executor_->initialized() ||
        !backend_ || !backend_->isReady() ||
        message->name.size() != fr10_motion::kJointCount ||
        message->position.size() != fr10_motion::kJointCount) {
      coordinator_->reject(fr10_motion::ControlMode::JOINT, fr10_motion::CommandReason::NOT_READY);
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Joint target rejected: ros2_control not ready, feedback "
          "not synchronized, or six names/positions not provided");
      return;
    }

    const auto decoded = fr10_motion::decodeJointTarget(
        message->name, message->position, params_.joint_names,
        params_.joint_input_unit);
    if (decoded.error != fr10_motion::JointTargetDecodeError::NONE) {
      coordinator_->reject(fr10_motion::ControlMode::JOINT, fr10_motion::CommandReason::INVALID_INPUT);
      RCLCPP_WARN(get_logger(), "Joint target rejected: %s",
                  decoded.error == fr10_motion::JointTargetDecodeError::NONFINITE
                      ? "joint values must be finite"
                      : "names must contain j1..j6 exactly once");
      return;
    }

    fr10_motion::JointTarget target;
    target.position_rad = decoded.position_rad;
    target.velocity_scale = params_.joint_velocity_scale;
    target.acceleration_scale = params_.joint_acceleration_scale;
    target.jerk_scale = params_.joint_jerk_scale;
    target.timestamp_s = now().seconds();

    fr10_motion::RobotState seed;
    if (!feedbackSnapshot(seed)) {
      coordinator_->reject(fr10_motion::ControlMode::JOINT, fr10_motion::CommandReason::NOT_READY);
      RCLCPP_WARN(get_logger(),
                  "Joint target rejected: no valid feedback seed");
      return;
    }

    KinematicsTask task;
    task.control_mode = fr10_motion::ControlMode::JOINT;
    task.joint_target = target;
    task.seed = seed;
    fr10_motion::MotionCommand reference;
    if (latest_command_buffer_.read(reference) &&
        reference.type == fr10_motion::CommandType::JOINT_POSITION) {
      task.seed.joint_position_rad = reference.joint_position_rad;
      task.seed.joint_velocity_rad_s = reference.joint_velocity_rad_s;
      task.seed.joint_acceleration_rad_s2 = reference.joint_acceleration_rad_s2;
    }
    fr10_motion::CommandIntent intent;
    intent.mode = task.control_mode; intent.joint = target;
    const auto decision = coordinator_->request(intent, scene_revision_.load(), reference.sequence);
    if (!decision.accepted) return;
    task.command_id = decision.report.id;
    task.joint_target.revision = decision.report.generation;
    task.sequence = kinematics_task_sequence_.fetch_add(1) + 1;
    queueKinematicsTask(task);
  }

void MotionAdapter::freePoseTargetCallback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr &message) {
  using namespace fr10_motion;
  if (force_adapter_ && force_adapter_->busy()) {
    coordinator_->reject(ControlMode::POSE, CommandReason::BUSY); return;
  }
  if (shutdown_in_progress_.load() || stop_latch_.load() != StopReason::NONE ||
      motion_executor_->status().state == MotionState::STOPPING) {
    coordinator_->reject(ControlMode::POSE, CommandReason::NOT_READY); return;
  }
  Pose pose;
  pose.position_m = {message->pose.position.x, message->pose.position.y, message->pose.position.z};
  pose.orientation_quat = {message->pose.orientation.x, message->pose.orientation.y,
                          message->pose.orientation.z, message->pose.orientation.w};
  const double stamp = rclcpp::Time(message->header.stamp, get_clock()->get_clock_type()).seconds();
  const double age = now().seconds() - stamp;
  if (message->header.frame_id != params_.base_frame ||
      !std::all_of(pose.position_m.begin(), pose.position_m.end(),
                   [](double value) { return std::isfinite(value); }) ||
      !math::normalizeQuaternion(pose.orientation_quat) ||
      (stamp != 0.0 && (!std::isfinite(age) || age < -0.10 || age > params_.max_measurement_age_s))) {
    coordinator_->reject(ControlMode::POSE, CommandReason::INVALID_INPUT); return;
  }
  insertion_sequence_state_.clear();
  if (queuePoseTarget(pose, PoseExecutionPolicy::FREE_SPACE, true))
    display_pose_goal_buffer_.write(DisplayPoseGoalSnapshot{pose, true});
}

void MotionAdapter::poseTargetCallback(
      const geometry_msgs::msg::PoseStamped::ConstSharedPtr &message) {
    if (force_adapter_ && force_adapter_->busy()) {
      coordinator_->reject(fr10_motion::ControlMode::POSE, fr10_motion::CommandReason::BUSY);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Pose target rejected: force insertion is BUSY");
      return;
    }
    if (shutdown_in_progress_.load() || stop_latch_.load() != StopReason::NONE ||
        motion_executor_->status().state == fr10_motion::MotionState::STOPPING) { return; }
    if (message->header.frame_id != params_.base_frame) {
      RCLCPP_WARN(get_logger(),
                  "Insertion target rejected: frame '%s' must be '%s'",
                  message->header.frame_id.c_str(), params_.base_frame.c_str());
      return;
    }
    const rclcpp::Time receive_time = now();
    const rclcpp::Time measurement_time(message->header.stamp,
                                        get_clock()->get_clock_type());
    if (measurement_time.nanoseconds() != 0) {
      const double age_s = (receive_time - measurement_time).seconds();
      if (!std::isfinite(age_s) || age_s < -0.10 ||
          age_s > params_.max_measurement_age_s) {
        RCLCPP_WARN(get_logger(),
                    "Pose target rejected: stale/future timestamp age=%.3f s",
                    age_s);
        return;
      }
    }
    fr10_motion::Pose target_pose;
    target_pose.position_m = {message->pose.position.x,
                              message->pose.position.y,
                              message->pose.position.z};
    target_pose.orientation_quat = {
        message->pose.orientation.x, message->pose.orientation.y,
        message->pose.orientation.z, message->pose.orientation.w};
    if (!std::all_of(target_pose.position_m.begin(),
                     target_pose.position_m.end(),
                     [](double value) { return std::isfinite(value); }) ||
        !fr10_motion::math::normalizeQuaternion(target_pose.orientation_quat)) {
      RCLCPP_WARN(get_logger(),
                  "Insertion target rejected: invalid pose/quaternion");
      return;
    }
    if (force_runtime_config_.automatic_pose) {
      using Insert = fr10_insertion::InsertionRosAdapter::Insert;
      if (!force_adapter_ || !force_runtime_config_.enabled) {
        coordinator_->reject(fr10_motion::ControlMode::POSE, fr10_motion::CommandReason::NOT_READY);
        RCLCPP_ERROR(get_logger(), "Socket pose rejected: force session is unavailable");
        return;
      }
      Insert::Goal goal;
      goal.target_pose = *message;
      goal.target_pose.pose.orientation.x = target_pose.orientation_quat[0];
      goal.target_pose.pose.orientation.y = target_pose.orientation_quat[1];
      goal.target_pose.pose.orientation.z = target_pose.orientation_quat[2];
      goal.target_pose.pose.orientation.w = target_pose.orientation_quat[3];
      const auto axis = fr10_motion::math::rotateVector(
          target_pose.orientation_quat, {0.0, 0.0, 1.0});
      goal.target_pose.pose.position.x += axis[0] * force_runtime_config_.insertion_depth_m;
      goal.target_pose.pose.position.y += axis[1] * force_runtime_config_.insertion_depth_m;
      goal.target_pose.pose.position.z += axis[2] * force_runtime_config_.insertion_depth_m;
      goal.preinsert_distance = force_runtime_config_.approach_distance_m;
      goal.insertion_depth = force_runtime_config_.insertion_depth_m;
      goal.insertion_speed = force_runtime_config_.insertion_speed_m_s;
      goal.desired_wrench.force.z = force_runtime_config_.seating_force_n;
      goal.selected_axes = force_runtime_config_.selected_axes;
      goal.max_force = force_runtime_config_.max_force_n;
      goal.max_torque = force_runtime_config_.max_torque_nm;
      goal.timeout = static_cast<builtin_interfaces::msg::Duration>(rclcpp::Duration::from_seconds(force_runtime_config_.timeout_s));
      goal.profile_id = force_runtime_config_.profile_id;
      goal.completion_behavior = Insert::Goal::HOLD;
      if (!force_adapter_->submitPoseGoal(goal))
        coordinator_->reject(fr10_motion::ControlMode::POSE, fr10_motion::CommandReason::BUSY);
      return;
    }
    const auto staging =
        fr10_motion::PreInsertPlanner::plan(target_pose, params_.pre_insert_config);
    const auto insertion =
        staging.success
            ? fr10_motion::LinearInsertPlanner::plan(
                  staging.pre_insert_pose, target_pose, params_.linear_insert_config)
            : fr10_motion::LinearInsertPlan{};
    if (!staging.success || !insertion.success) {
      RCLCPP_ERROR(get_logger(), "[INSERTION REJECTED: GEOMETRY] %s",
                   staging.success ? insertion.message.c_str()
                                   : staging.message.c_str());
      return;
    }

    fr10_motion::CartesianTarget insertion_target =
        fr10_motion::CartesianTarget::fromPose(target_pose);
    const auto &policy_config = params_.path_time_policies[policyIndex(
        fr10_motion::PoseExecutionPolicy::INSERTION)];
    insertion_target.linear_speed_scale = policy_config.linear_scale;
    insertion_target.angular_speed_scale = policy_config.angular_scale;
    insertion_target.timestamp_s = now().seconds();

    fr10_motion::RobotState live_seed;
    fr10_motion::Pose live_tcp;
    if (feedbackSnapshot(live_seed)) {
      live_tcp = {live_seed.tcp_position_m, live_seed.tcp_orientation_quat};
      double pre_insert_error_squared = 0.0;
      for (std::size_t axis = 0; axis < 3; ++axis) {
        const double error = live_tcp.position_m[axis] -
                             staging.pre_insert_pose.position_m[axis];
        pre_insert_error_squared += error * error;
      }
      const double orientation_error =
          fr10_motion::math::quaternionAngularDistance(
              live_tcp.orientation_quat,
              staging.pre_insert_pose.orientation_quat);
      if (std::sqrt(pre_insert_error_squared) <=
              params_.linear_insert_config.position_tolerance_m &&
          orientation_error <=
              params_.linear_insert_config.orientation_tolerance_rad) {
        fr10_motion::MotionCommand live_reference;
        latest_command_buffer_.read(live_reference);
        if (queueStrictInsertionTarget(insertion_target,
                                       staging.pre_insert_pose, target_pose,
                                       live_seed, live_reference)) {
          display_pose_goal_buffer_.write(
              DisplayPoseGoalSnapshot{target_pose, true});
          insertion_sequence_state_.markDirect(insertion_target);
          RCLCPP_INFO(get_logger(),
                      "[MODE TRANSITION] already at PRE_INSERT; entering "
                      "INSERTION policy directly");
        }
        return;
      }
    }

    if (insertion_sequence_state_.isDuplicate(
            insertion_target, params_.pose_position_change_tolerance_m,
            params_.pose_angular_change_tolerance_rad)) {
      return;
    }

    const auto staging_generation = queuePoseTarget(
        staging.pre_insert_pose, fr10_motion::PoseExecutionPolicy::FREE_SPACE, true);
    if (!staging_generation) {
      RCLCPP_WARN(get_logger(), "Insertion staging target was not queued");
      return;
    }
    display_pose_goal_buffer_.write(DisplayPoseGoalSnapshot{target_pose, true});
    insertion_sequence_state_.arm(insertion_target, staging.pre_insert_pose,
                                  *staging_generation);
    RCLCPP_INFO(
        get_logger(),
        "[POSE POLICY TRANSITION ARMED] FREE_SPACE -> PRE_INSERT (%.1f mm) "
        "-> INSERTION",
        staging.distance_m * 1000.0);
  }

std::optional<uint64_t> MotionAdapter::queuePoseTarget(
      const fr10_motion::Pose &pose,
      fr10_motion::PoseExecutionPolicy policy,
      bool preserve_orientation,
      std::optional<fr10_insertion::InsertionRosAdapter::PendingGoal> force_goal) {
    if (stop_requested_.load() || !motion_executor_->initialized() ||
        !backend_ || !backend_->isReady()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Cartesian target rejected: ros2_control command "
                           "consumer is not ready");
      return std::nullopt;
    }
    // The sole caller has validated the incoming PoseStamped once.
    const rclcpp::Time receive_time = now();
    fr10_motion::CartesianTarget target =
        fr10_motion::CartesianTarget::fromPose(pose);

    fr10_motion::RobotState seed;
    if (!feedbackSnapshot(seed)) {
      RCLCPP_WARN(get_logger(),
                  "Cartesian target rejected: no valid feedback seed");
      return std::nullopt;
    }

    const auto &active_policy_config =
        params_.path_time_policies[policyIndex(policy)];
    target.linear_speed_scale = active_policy_config.linear_scale;
    target.angular_speed_scale = active_policy_config.angular_scale;
    target.timestamp_s = receive_time.seconds();

    fr10_motion::Pose tcp_feedback;
    tcp_feedback = {seed.tcp_position_m, seed.tcp_orientation_quat};
    {
      const double orientation_deviation =
          fr10_motion::math::quaternionAngularDistance(
              target.orientation_quat, tcp_feedback.orientation_quat);
      if (params_.lock_orientation && !preserve_orientation) {
        target.orientation_quat = tcp_feedback.orientation_quat;
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Cartesian orientation locked at current TCP orientation "
            "(requested deviation %.1f deg)",
            orientation_deviation * fr10_motion::kRadToDeg);
      } else if (orientation_deviation > params_.max_angular_deviation_rad) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[LARGE ORIENTATION] Planning %.1f deg with dense quaternion "
            "SLERP; orientation is not locked",
            orientation_deviation * fr10_motion::kRadToDeg);
      }
    }

    KinematicsTask task;
    task.control_mode = fr10_motion::ControlMode::POSE;
    task.force_goal = std::move(force_goal);
    task.pose_target = target;
    task.policy = policy;
    task.seed = seed;
    fr10_motion::MotionCommand live_reference;
    latest_command_buffer_.read(live_reference);
    if (live_reference.type == fr10_motion::CommandType::JOINT_POSITION) {
      task.seed.joint_position_rad = live_reference.joint_position_rad;
      task.seed.joint_velocity_rad_s = live_reference.joint_velocity_rad_s;
      task.seed.joint_acceleration_rad_s2 =
          live_reference.joint_acceleration_rad_s2;
    }
    fr10_motion::CommandIntent intent;
    intent.mode = task.control_mode; intent.pose = target; intent.policy = policy;
    intent.session_id = task.force_goal ? force_adapter_->generation() : 0;
    const auto decision = coordinator_->request(intent, scene_revision_.load(), live_reference.sequence);
    if (!decision.accepted) return std::nullopt;
    task.command_id = decision.report.id;
    target.generation = decision.report.generation;
    task.pose_target = target;
    task.sequence = kinematics_task_sequence_.fetch_add(1) + 1;
    queueKinematicsTask(task);
    return target.generation;
  }

bool
  MotionAdapter::queueStrictInsertionTarget(fr10_motion::CartesianTarget target,
                             const fr10_motion::Pose &expected_path_start,
                             const fr10_motion::Pose &expected_path_goal,
                             const fr10_motion::RobotState &seed,
                             const fr10_motion::MotionCommand &live_reference) {
    fr10_motion::Pose current_pose;
    current_pose.position_m = seed.tcp_position_m;
    current_pose.orientation_quat = seed.tcp_orientation_quat;
    fr10_motion::KinematicState measured_joints;
    measured_joints.position_rad = seed.joint_position_rad;
    if (!analytical_ik_provider_->forward(measured_joints, current_pose)) { return false; }
    const fr10_motion::Pose target_pose = target.toPose();
    const auto axis_validation = fr10_motion::LinearInsertPlanner::checkAxis(
        current_pose, target_pose, expected_path_start, expected_path_goal,
        params_.linear_insert_config);
    if (!axis_validation.success) {
      RCLCPP_ERROR(get_logger(),
                   "[INSERTION REJECTED: AXIS_INVARIANT] deviation=%.3f mm "
                   "forward_projection=%.6g reason=%s; reposition PRE_INSERT",
                   axis_validation.axis_deviation_m * 1000.0,
                   axis_validation.forward_projection,
                   axis_validation.message.c_str());
      return false;
    }

    target.timestamp_s = now().seconds();
    const auto &policy_config = params_.path_time_policies[policyIndex(
        fr10_motion::PoseExecutionPolicy::INSERTION)];
    target.linear_speed_scale = policy_config.linear_scale;
    target.angular_speed_scale = policy_config.angular_scale;
    KinematicsTask task;
    task.control_mode = fr10_motion::ControlMode::POSE;
    task.pose_target = target;
    task.policy = fr10_motion::PoseExecutionPolicy::INSERTION;
    task.seed = seed;
    if (live_reference.type == fr10_motion::CommandType::JOINT_POSITION) {
      task.seed.joint_position_rad = live_reference.joint_position_rad;
      task.seed.joint_velocity_rad_s = live_reference.joint_velocity_rad_s;
      task.seed.joint_acceleration_rad_s2 =
          live_reference.joint_acceleration_rad_s2;
    }
    task.sequence = kinematics_task_sequence_.fetch_add(1) + 1;
    fr10_motion::CommandIntent intent;
    intent.mode = task.control_mode; intent.pose = target; intent.policy = task.policy;
    const auto decision = coordinator_->request(intent, scene_revision_.load(), live_reference.sequence);
    if (!decision.accepted) return false;
    task.command_id = decision.report.id;
    target.generation = decision.report.generation;
    task.pose_target = target;
    queueKinematicsTask(task);
    RCLCPP_INFO(
        get_logger(),
        "[POSE POLICY QUEUED] INSERTION generation=%lu distance=%.1f mm "
        "axis_deviation=%.3f mm",
        target.generation,
        axis_validation.distance_m * 1000.0,
        axis_validation.axis_deviation_m * 1000.0);
    return true;
  }

void
  MotionAdapter::queueInsertionAfterStagingIfReady(const fr10_motion::MotionStatus &status,
                                    const fr10_motion::RobotState &feedback,
                                    const fr10_motion::MotionCommand &command) {
    if (status.state != fr10_motion::MotionState::HOLDING ||
        status.outcome != fr10_motion::MotionOutcome::GOAL_REACHED ||
        !status.in_goal_tolerance) {
      return;
    }
    if (command.type == fr10_motion::CommandType::JOINT_POSITION) {
      constexpr double kBoundaryVelocityToleranceRadS = 1e-5;
      constexpr double kBoundaryAccelerationToleranceRadS2 = 1e-4;
      for (std::size_t joint = 0; joint < fr10_motion::kJointCount; ++joint) {
        if (std::abs(command.joint_velocity_rad_s[joint]) >
                kBoundaryVelocityToleranceRadS ||
            std::abs(command.joint_acceleration_rad_s2[joint]) >
                kBoundaryAccelerationToleranceRadS2) {
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "PRE_INSERT HOLDING boundary is not settled: waiting for "
              "qdot/qddot -> 0 before INSERTION");
          return;
        }
      }
    }
    const auto pending = insertion_sequence_state_.pendingForGeneration(
        status.active_generation);
    if (!pending) {
      return;
    }

    // A completed reference is not enough to authorize INSERTION. The
    // measured robot must be at the PRE_INSERT boundary on this fresh
    // feedback sample; otherwise HOLDING is allowed to let the servo settle.
    fr10_motion::KinematicState measured_joints;
    measured_joints.position_rad = feedback.joint_position_rad;
    fr10_motion::Pose measured_tcp;
    if (!feedback.valid || !analytical_ik_provider_ ||
        !analytical_ik_provider_->forward(measured_joints, measured_tcp)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "PRE_INSERT boundary waiting for valid measured TCP feedback");
      return;
    }
    double boundary_error_squared = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const double error = measured_tcp.position_m[axis] -
                           pending->pre_insert_pose.position_m[axis];
      boundary_error_squared += error * error;
    }
    const double boundary_position_error = std::sqrt(boundary_error_squared);
    const double boundary_orientation_error =
        fr10_motion::math::quaternionAngularDistance(
            measured_tcp.orientation_quat,
            pending->pre_insert_pose.orientation_quat);
    if (boundary_position_error >
            params_.linear_insert_config.position_tolerance_m ||
        boundary_orientation_error >
            params_.linear_insert_config.orientation_tolerance_rad) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "PRE_INSERT measured boundary not reached: position=%.3f mm "
          "orientation=%.3f deg",
          boundary_position_error * 1000.0,
          boundary_orientation_error * fr10_motion::kRadToDeg);
      return;
    }
    if (!insertion_sequence_state_.takeIfGeneration(status.active_generation)) {
      return;
    }
    const fr10_motion::Pose target_pose =
        pending->insertion_target.toPose();
    if (queueStrictInsertionTarget(pending->insertion_target,
                                   pending->pre_insert_pose, target_pose,
                                   feedback, command)) {
      insertion_sequence_state_.retain(pending->insertion_target);
      stop_reason_.store(StopReason::NONE);
      RCLCPP_INFO(get_logger(), "[POSE POLICY TRANSITION] PRE_INSERT reached; "
                                "FREE_SPACE -> INSERTION");
    }
  }

void
  MotionAdapter::retractCallback(const std_msgs::msg::Empty::ConstSharedPtr & /*message*/) {
    if (force_adapter_ && force_adapter_->requestHoldRetract()) return;
    if (force_adapter_ && force_adapter_->busy()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Retract topic rejected: force insertion is BUSY");
      return;
    }
    const auto socket_target = insertion_sequence_state_.beginRetract();
    if (!socket_target) {
      RCLCPP_WARN(get_logger(),
                  "Retract rejected: no validated insertion target retained");
      return;
    }
    const fr10_motion::Pose socket_pose = socket_target->toPose();
    const auto staging =
        fr10_motion::PreInsertPlanner::plan(socket_pose, params_.pre_insert_config);
    fr10_motion::RobotState seed;
    if (!staging.success || !feedbackSnapshot(seed)) {
      RCLCPP_ERROR(get_logger(), "Retract rejected: %s",
                   staging.success ? "feedback unavailable"
                                   : staging.message.c_str());
      return;
    }
    fr10_motion::CartesianTarget retract_target =
        fr10_motion::CartesianTarget::fromPose(staging.pre_insert_pose);
    fr10_motion::MotionCommand live_reference;
    latest_command_buffer_.read(live_reference);
    if (queueStrictInsertionTarget(retract_target, socket_pose,
                                   staging.pre_insert_pose, seed,
                                   live_reference)) {
      display_pose_goal_buffer_.write(
          DisplayPoseGoalSnapshot{staging.pre_insert_pose, true});
      RCLCPP_INFO(get_logger(),
                  "[PURE LINEAR RETRACT] INSERTION policy target=PRE_INSERT");
    }
  }

} // namespace fr10_pose_controller::detail
