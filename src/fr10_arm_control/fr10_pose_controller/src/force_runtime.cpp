#include "motion_adapter_internal.hpp"

namespace fr10_pose_controller::detail {

bool MotionAdapter::forceGoalCanStart() {
    if (!force_runtime_config_.enabled || stop_requested_.load() ||
        shutdown_in_progress_.load() || !motion_executor_ ||
        !motion_executor_->initialized() || !backend_ ||
        !backend_->isReady()) {
      return false;
    }
    const auto state = motion_executor_->status().state;
    if (state != fr10_motion::MotionState::IDLE &&
        state != fr10_motion::MotionState::HOLDING) {
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(kinematics_mutex_);
      if (pending_kinematics_task_) return false;
    }
    {
      std::lock_guard<std::mutex> lock(reversal_mutex_);
      if (post_stop_replan_task_) return false;
    }
    return true;
  }

void MotionAdapter::consumePendingForceGoal() {
    if (!force_adapter_) return;
    if (!force_admission_goal_) {
      force_admission_goal_ = force_adapter_->takePendingGoal();
      if (force_admission_goal_) {
        force_admission_source_s_ = now().seconds();
        if (force_parking_) force_parking_->acquire();
      }
    }
    if (!force_admission_goal_) {
      // Preserve parking while decelerating. Faults never release a moving base
      // or a moving arm simply because the action handler has gone away.
      if (force_parking_ && !force_adapter_->busy() &&
          motion_executor_->status().state == fr10_motion::MotionState::HOLDING)
        force_parking_->release();
      return;
    }
    using Result = fr10_motion_interfaces::action::Insert::Result;
    const bool canceled = force_adapter_->cancelRequested() || stop_requested_.load();
    const double admission_age = now().seconds()-force_admission_source_s_;
    if (canceled || admission_age < 0 || admission_age > 5.) {
      force_adapter_->rejectPending(canceled ? Result::CANCELED :
          (admission_age < 0 ? Result::CLOCK_JUMP : Result::TRACKING_ERROR),
          "parking admission canceled, clock changed or fresh stationary feedback unavailable");
      force_admission_goal_.reset();
      if (force_parking_) force_parking_->release(); // Arm has not been moved.
      return;
    }
    if (force_parking_) {
      if (!force_parking_->update().ready) return;
    }
    const auto pending = std::move(force_admission_goal_);
    force_admission_goal_.reset();

    fr10_motion::PreInsertPlannerConfig staging_config =
        params_.pre_insert_config;
    staging_config.local_axis = {0.0, 0.0, 0.0};
    staging_config.local_axis[pending->goal.insertion_axis] = 1.0;
    staging_config.distance_m = pending->goal.preinsert_distance_m +
        pending->goal.insertion_depth_m;
    // Insert.action carries its own explicit preinsert and depth. The legacy
    // POSE profile's 50 mm minimum is not a force-control safety limit and
    // must not reject a calibrated shallow simulation/commissioning goal.
    staging_config.min_distance_m = 0.0;
    fr10_motion::Pose final_pose{
        pending->final_pose.position_m,
        pending->final_pose.orientation_xyzw};
    const auto staging =
        fr10_motion::PreInsertPlanner::plan(final_pose, staging_config);
    fr10_motion::Pose contact_pose = final_pose;
    if (staging.success) {
      for (std::size_t axis = 0; axis < 3; ++axis) {
        contact_pose.position_m[axis] -= staging.base_axis[axis] *
            pending->goal.insertion_depth_m;
      }
    }
    auto corridor_config = params_.linear_insert_config;
    corridor_config.max_insertion_distance_m = std::max(
        corridor_config.max_insertion_distance_m,
        pending->goal.insertion_depth_m);
    const auto corridor = staging.success
        ? fr10_motion::LinearInsertPlanner::plan(
              contact_pose, final_pose, corridor_config)
        : fr10_motion::LinearInsertPlan{};
    if (!staging.success || !corridor.success) {
      force_adapter_->rejectPending(
          fr10_motion_interfaces::action::Insert::Result::PLANNING_FAILED,
          staging.success ? corridor.message : staging.message);
      return;
    }

    const uint64_t generation = force_generation_.fetch_add(1) + 1;
    const uint64_t scene_revision =
        scene_revision_.load(std::memory_order_acquire);
    if (!force_adapter_->startSession(*pending, staging.base_axis, generation,
                                      scene_revision, now().seconds())) {
      force_adapter_->rejectPending(
          fr10_motion_interfaces::action::Insert::Result::INVALID_GOAL,
          "force session rejected the validated goal");
      return;
    }
    const fr10_motion::Pose preinsert_pose{
        force_adapter_->preinsertPose().position_m,
        force_adapter_->preinsertPose().orientation_xyzw};
    const auto staging_generation = queuePoseTarget(
        preinsert_pose, fr10_motion::PoseExecutionPolicy::FREE_SPACE, true, *pending);
    if (!staging_generation) {
      force_adapter_->rejectPending(
          fr10_motion_interfaces::action::Insert::Result::PLANNING_FAILED,
          "failed to queue force preinsert trajectory");
      return;
    }
    insertion_sequence_state_.clear();
    force_staging_generation_ = *staging_generation;
    force_retract_generation_ = 0;
    force_preinsert_reached_ = false;
    force_alignment_queued_ = false;
    force_corridor_validated_ = false;
    force_validation_pending_ = false;
    pending_force_validation_sequence_ = 0;
    consumed_force_validation_sequence_ = 0;
    // The planner is the sole buffer writer. Generation/sequence admission
    // rejects old results; a control-thread reset would race that writer.
    force_retract_queued_ = false;
    force_retract_requested_last_tick_ = false;
    display_pose_goal_buffer_.write(
        DisplayPoseGoalSnapshot{final_pose, true});
    RCLCPP_INFO(get_logger(),
                "[FORCE INSERT] generation=%lu staging_generation=%lu "
                "preinsert=%.1f mm insertion=%.1f mm profile=%s",
                generation, force_staging_generation_,
                pending->goal.preinsert_distance_m * 1000.0,
                pending->goal.insertion_depth_m * 1000.0,
                pending->profile_id.c_str());
  }

bool MotionAdapter::forcePreinsertReached(const fr10_motion::MotionStatus &status,
                             const fr10_motion::RobotState &feedback,
                             const fr10_motion::MotionCommand &command) {
    if (!force_corridor_validated_) return false;
    if (force_preinsert_reached_) return true;
    if (force_staging_generation_ == 0 ||
        status.active_generation != force_staging_generation_ ||
        status.state != fr10_motion::MotionState::HOLDING ||
        status.outcome != fr10_motion::MotionOutcome::GOAL_REACHED ||
        !status.in_goal_tolerance) {
      return false;
    }
    if (command.type == fr10_motion::CommandType::JOINT_POSITION) {
      for (std::size_t joint = 0; joint < fr10_motion::kJointCount; ++joint) {
        if (std::abs(command.joint_velocity_rad_s[joint]) > 1e-5 ||
            std::abs(command.joint_acceleration_rad_s2[joint]) > 1e-4) {
          return false;
        }
      }
    }
    fr10_motion::KinematicState measured;
    measured.position_rad = feedback.joint_position_rad;
    fr10_motion::Pose actual;
    if (!analytical_ik_provider_ ||
        !analytical_ik_provider_->forward(measured, actual)) {
      return false;
    }
    const auto &expected = force_adapter_->preinsertPose();
    double squared_error = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const double error = actual.position_m[axis] - expected.position_m[axis];
      squared_error += error * error;
    }
    const double orientation_error =
        fr10_motion::math::quaternionAngularDistance(
            actual.orientation_quat, expected.orientation_xyzw);
    force_preinsert_reached_ =
        std::sqrt(squared_error) <=
            params_.linear_insert_config.position_tolerance_m &&
        orientation_error <=
            params_.linear_insert_config.orientation_tolerance_rad;
    if (force_preinsert_reached_ && !force_runtime_config_.task_frame.empty() && !force_alignment_queued_) {
      // One measured stationary boundary refinement. Worker resolves the same
      // immutable task pose and geometry together; no TF/planning in this tick.
      const fr10_motion::Pose target{expected.position_m,expected.orientation_xyzw};
      const auto generation=queuePoseTarget(target,fr10_motion::PoseExecutionPolicy::INSERTION,
                                           true,force_adapter_->activeGoal());
      force_alignment_queued_=true;
      force_preinsert_reached_=false;
      force_corridor_validated_=false;
      if(generation) force_staging_generation_=*generation;
      else force_adapter_->requestCancel();
      RCLCPP_INFO(get_logger(),"[FORCE TASK FRAME] stationary preinsert refinement queued");
    }
    return force_preinsert_reached_;
  }

MotionAdapter::ForceTickSubmission MotionAdapter::updateForceRuntime(
      const fr10_motion::RobotState &feedback,
      const fr10_motion::MotionStatus &status,
      const fr10_motion::MotionCommand &command) {
    ForceTickSubmission submission;
    if (!force_adapter_ || !force_adapter_->busy() ||
        force_adapter_->state() == fr10_insertion::InsertionState::IDLE) {
      return submission;
    }
    fr10_motion::KinematicState seed;
    seed.position_rad = feedback.joint_position_rad;
    seed.velocity_rad_s = feedback.joint_velocity_rad_s;
    seed.acceleration_rad_s2 = feedback.joint_acceleration_rad_s2;
    if (command.type == fr10_motion::CommandType::JOINT_POSITION) {
      seed.position_rad = command.joint_position_rad;
      seed.velocity_rad_s = command.joint_velocity_rad_s;
      seed.acceleration_rad_s2 = command.joint_acceleration_rad_s2;
    }
    // The IK seed follows the acknowledged command for continuity.  Physical
    // depth/completion must instead use measured joints, independently of it.
    fr10_motion::KinematicState measured_state;
    measured_state.position_rad = feedback.joint_position_rad;
    measured_state.velocity_rad_s = feedback.joint_velocity_rad_s;
    measured_state.acceleration_rad_s2 = feedback.joint_acceleration_rad_s2;
    fr10_motion::Pose actual_motion;
    const bool measured_fk_valid = analytical_ik_provider_ &&
        analytical_ik_provider_->forward(measured_state, actual_motion);
    fr10_insertion::InsertionRosAdapter::RuntimeInput input;
    input.joint_seed = seed;
    input.measured_joint_velocity_rad_s = feedback.joint_velocity_rad_s;
    input.actual_pose_valid = measured_fk_valid;
    input.actual_pose = {actual_motion.position_m,
                         actual_motion.orientation_quat};
    input.current_scene_revision =
        scene_revision_.load(std::memory_order_acquire);
    input.preinsert_reached =
        forcePreinsertReached(status, feedback, command);
    const auto failed_generation = failed_planning_generation_.load(std::memory_order_acquire);
    const bool retracting = force_adapter_->state() == fr10_insertion::InsertionState::RETRACT;
    input.planning_failed = failed_generation != 0 &&
        (retracting ? failed_generation == force_retract_generation_ :
                     !input.preinsert_reached && failed_generation == force_staging_generation_);
    input.stop_complete =
        status.state == fr10_motion::MotionState::HOLDING &&
        status.outcome == fr10_motion::MotionOutcome::STOPPED_BEFORE_GOAL;
    input.retract_complete = force_retract_generation_ != 0 &&
        status.active_generation == force_retract_generation_ &&
        status.state == fr10_motion::MotionState::HOLDING &&
        status.outcome == fr10_motion::MotionOutcome::GOAL_REACHED;
    input.collision_free = !feedback.collision_detected;
    input.tracking_valid = status.state != fr10_motion::MotionState::FAULT &&
        !(fr10_motion::invalidatesForceTracking(stop_reason_.load()) &&
          force_adapter_->state() != fr10_insertion::InsertionState::FAILED);
    input.robot_ok = feedback.valid && !feedback.emergency_stop &&
        feedback.main_error_code == 0 && feedback.sub_error_code == 0;
    input.base_stationary = true;
    if(force_parking_) {
      const auto base_hold=force_parking_->update();
      input.base_stationary=base_hold.ready;
      if(!base_hold.ready)
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),1000,
            "[FORCE BASE INTERLOCK] fault=%d phase=%d; parking lease and measured stationary boundary required",
            static_cast<int>(base_hold.fault),static_cast<int>(force_adapter_->state()));
    }
    input.allow_motion_proposal = !force_validation_pending_;
    auto output = force_adapter_->update(input, *analytical_ik_provider_);
    if (output.decision.request_controlled_stop &&
        status.state != fr10_motion::MotionState::STOPPING) {
      requestControlledStop(StopReason::CONTROLLED_STOP);
    }
    const bool retract_requested = force_adapter_->retractRequested();
    if (retract_requested && !force_retract_requested_last_tick_) {
      force_retract_queued_ = false;
      force_retract_generation_ = 0;
    }
    force_retract_requested_last_tick_ = retract_requested;
    if (retract_requested && !force_retract_queued_ && !output.decision.request_controlled_stop) {
      const fr10_motion::Pose preinsert{
          force_adapter_->retractPose().position_m,
          force_adapter_->retractPose().orientation_xyzw};
      auto retract_goal=force_adapter_->activeGoal();
      retract_goal.preinsert_alignment=false;
      const auto generation = queuePoseTarget(
          preinsert, fr10_motion::PoseExecutionPolicy::INSERTION, true,
          retract_goal);
      if (generation) {
        force_retract_generation_ = *generation;
        force_retract_queued_ = true;
      } else {
        force_adapter_->requestCancel();
      }
    }
    if (output.online_target_valid) {
      {
        std::lock_guard<std::mutex> lock(kinematics_mutex_);
        pending_force_validation_ = ForceValidationRequest{output.online_target, seed};
      }
      force_validation_pending_ = true;
      pending_force_validation_sequence_ = output.online_target.sequence;
      pending_force_validation_deadline_s_ = output.online_target.valid_until_s;
      kinematics_cv_.notify_one();
    }
    if (!output.decision.request_online_motion || output.decision.request_controlled_stop) {
      if (force_validation_pending_) {
        force_adapter_->discardOnlineTarget(pending_force_validation_sequence_);
        force_validation_pending_ = false;
        pending_force_validation_sequence_ = 0;
      }
      return submission;
    }
    if (force_validation_pending_ && now().seconds() > pending_force_validation_deadline_s_) {
      RCLCPP_ERROR(get_logger(), "[FORCE CHECK EXPIRED] sequence=%lu", pending_force_validation_sequence_);
      force_adapter_->commitOnlineTarget(pending_force_validation_sequence_, false);
      force_validation_pending_ = false;
      pending_force_validation_sequence_ = 0;
      requestControlledStop(StopReason::CONTROLLED_STOP);
      return submission;
    }
    ForceValidationResult checked;
    if (force_validation_pending_ && force_validation_result_.read(checked) &&
        checked.target.generation == force_adapter_->generation() &&
        checked.target.sequence == pending_force_validation_sequence_ &&
        checked.target.sequence > consumed_force_validation_sequence_) {
      consumed_force_validation_sequence_ = checked.target.sequence;
      force_validation_pending_ = false;
      pending_force_validation_sequence_ = 0;
      // A loaded-axis observation window can break while collision checking
      // is pending. Discard that unissued estimate before actuator admission;
      // keep the existing target/lease and the same bounded session deadline.
      if (!force_adapter_->canCommitOnlineTarget(checked.target.sequence)) {
        force_adapter_->discardOnlineTarget(checked.target.sequence);
        return submission;
      }
      const double admission_source_s = std::max(now().seconds(),feedback.timestamp_s);
      const bool fresh = checked.target.generation == force_adapter_->generation() &&
          checked.target.scene_revision == scene_revision_.load() &&
          admission_source_s <= checked.target.valid_until_s;
      submission.online_sequence = checked.target.sequence;
      // A proposal's validation deadline and an accepted command's lease
      // serve different boundaries. Spending most of the deadline in the
      // worker must not expire the previous command before its replacement.
      // Wrench/robot safety still runs independently on every control tick.
      checked.target.command_lease_s = force_runtime_config_.command_lease_s;
      submission.accepted = fresh && checked.valid &&
          !output.decision.request_controlled_stop &&
          motion_executor_->submitOnlineJointTarget(checked.target,admission_source_s);
      if (!submission.accepted) {
        RCLCPP_ERROR(get_logger(),
            "[FORCE TARGET REJECTED] sequence=%lu fresh=%s checked=%s stop=%s age_ms=%.3f deadline_ms=%.3f feedback_age_ms=%.3f state=%d outcome=%d",
            checked.target.sequence, fresh ? "true" : "false", checked.valid ? "true" : "false",
            output.decision.request_controlled_stop ? "true" : "false",
            1000. * (now().seconds()-checked.target.timestamp_s),
            1000. * (checked.target.valid_until_s-checked.target.timestamp_s),
            1000. * (feedback.timestamp_s-checked.target.timestamp_s),
            static_cast<int>(motion_executor_->status().state),
            static_cast<int>(motion_executor_->status().outcome));
        force_adapter_->commitOnlineTarget(submission.online_sequence, false);
      } else {
        active_scene_revision_.store(checked.target.scene_revision, std::memory_order_release);
      }
    }
    return submission;
  }

} // namespace fr10_pose_controller::detail
