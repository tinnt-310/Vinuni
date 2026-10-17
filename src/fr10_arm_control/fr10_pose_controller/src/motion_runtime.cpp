#include "motion_adapter_internal.hpp"

namespace fr10_pose_controller::detail {

void MotionAdapter::requestStopAll(StopReason reason) {
    bool expected = false;
    if (!stop_requested_.compare_exchange_strong(expected, true)) {
      return;
    }
    stop_reason_.store(reason);
    if (control_timer_) {
      control_timer_->cancel();
    }
    if (diagnostics_timer_) {
      diagnostics_timer_->cancel();
    }
    if (path_monitor_timer_) {
      path_monitor_timer_->cancel();
    }
    if (status_log_timer_) {
      status_log_timer_->cancel();
    }
    kinematics_cv_.notify_all();
    if (motion_executor_) {
      motion_executor_->stop();
    }
    RCLCPP_INFO(get_logger(), "[SHUTDOWN: %s] 125 Hz control timers stopped.",
                stopReasonName(reason));
  }

void MotionAdapter::cleanupWorkers() {
    kinematics_cv_.notify_all();
    if (kinematics_worker_.joinable()) {
      kinematics_worker_.join();
    }
  }

bool MotionAdapter::stopRequested() const { return stop_requested_.load(); }

void MotionAdapter::beginGracefulShutdown() {
    shutdown_in_progress_.store(true);
    if (!motion_executor_ || !motion_executor_->initialized()) {
      // There is no accepted feedback and therefore no command trajectory to
      // brake. Waiting for HOLDING here can only time out.
      requestStopAll(StopReason::USER_INTERRUPT);
      return;
    }
    stop_latch_.store(StopReason::USER_INTERRUPT);
  }

double MotionAdapter::shutdownTimeoutS() const {
    double duration = 0.0;
    const auto &limits = params_.controller_config.limits;
    for (std::size_t i = 0; i < fr10_motion::kJointCount; ++i) {
      duration = std::max(duration,
          limits.max_joint_velocity_rad_s[i] / limits.max_joint_acceleration_rad_s2[i] +
          2.0 * limits.max_joint_acceleration_rad_s2[i] / limits.max_joint_jerk_rad_s3[i]);
    }
    return duration + 0.5;
  }

bool MotionAdapter::shutdownBackend() {
    if (backend_shutdown_.exchange(true) || !backend_) {
      return true;
    }
    const std::string backend_name = backend_->name();
    const bool clean = backend_->shutdown();
    backend_.reset();
    RCLCPP_INFO(
        get_logger(),
        "[SHUTDOWN: %s] %s shutdown %s; this does not confirm physical standstill.",
        stopReasonName(stop_reason_.load()), backend_name.c_str(),
        clean ? "completed" : "reported an error");
    return clean;
  }

void MotionAdapter::requestControlledStop(StopReason reason) {
    // Invalidate worker output before starting the braking profile so a stale
    // result cannot replace the stop trajectory.
    kinematics_task_sequence_.fetch_add(1);
    {
      std::lock_guard<std::mutex> lock(kinematics_mutex_);
      pending_kinematics_task_.reset();
    }
    const bool retain_replan =
        reason == StopReason::RETARGET_TIMING ||
        reason == StopReason::STALE_TRAJECTORY;
    if (!retain_replan) {
      std::lock_guard<std::mutex> lock(reversal_mutex_);
      post_stop_replan_task_.reset();
    }
    if (!retain_replan) {
      insertion_sequence_state_.clear();
    }
    coordinator_->stop(retain_replan ? fr10_motion::CommandReason::STALE_BOUNDARY :
        fr10_motion::CommandReason::USER_STOP, retain_replan);
    stop_reason_.store(reason);
    if (motion_executor_) {
      motion_executor_->stop();
    }
    pending_event_.store(reason);
  }

bool MotionAdapter::feedbackSnapshot(fr10_motion::RobotState &snapshot) const {
    if (!feedback_buffer_.read(snapshot) || !snapshot.valid) { return false; }
    const double age = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count() - snapshot.received_steady_s;
    if (age > params_.feedback_timeout_s || age < 0.0) { return false; }
    fr10_motion::KinematicState joints;
    joints.position_rad = snapshot.joint_position_rad;
    fr10_motion::Pose tcp;
    if (!analytical_ik_provider_ || !analytical_ik_provider_->forward(joints, tcp)) { return false; }
    snapshot.tcp_position_m = tcp.position_m;
    snapshot.tcp_orientation_quat = tcp.orientation_quat;
    return true;
  }

void MotionAdapter::controlTick() {
    if (stop_requested_.load()) {
      return;
    }
    const auto tick_start = std::chrono::steady_clock::now();
    const double steady_clock_s = std::chrono::duration<double>(tick_start.time_since_epoch()).count();
    const bool simulation = has_parameter("use_sim_time") && get_parameter("use_sim_time").as_bool();
    const bool sim_force = simulation && force_adapter_ && force_adapter_->busy();
    const double source_clock_s = simulation ? now().seconds() : steady_clock_s;
    const auto tick = tick_clock_.advance(source_clock_s, simulation);
    if (tick == fr10_motion::TickDecision::INVALID) {
      requestStopAll(StopReason::COMM_FAILURE); return;
    }
    if (tick == fr10_motion::TickDecision::BACKWARD_JUMP) {
      if (sim_force) force_adapter_->rejectPending(fr10_motion_interfaces::action::Insert::Result::CLOCK_JUMP,
          "backward simulation clock invalidated session; controlled stop requested");
      requestControlledStop(StopReason::CONTROLLED_STOP);
      last_logic_source_s_ = source_clock_s;
      return;
    }
    if (tick == fr10_motion::TickDecision::WAIT) {
      if (simulation && source_clock_s == last_force_tick_clock_s_) {
        simulation_paused_ = true;
        force_resume_steady_s_ = 0.0;
      }
      last_force_tick_clock_s_ = source_clock_s;
      return; // All modes hold on pause; no reference/integrator advancement.
    }
    const double measured_period_s = simulation && last_logic_source_s_ > 0.0
        ? source_clock_s - last_logic_source_s_
        : std::chrono::duration<double>(tick_start - last_tick_).count();
    last_logic_source_s_ = source_clock_s;
    last_tick_ = tick_start;
    last_force_tick_clock_s_ = source_clock_s;

    fr10_motion::RobotState feedback;
    const bool feedback_ok = backend_ && backend_->readState(feedback);
    // DDS clock discovery can precede the first JointState. This is startup,
    // not a resumed, initialized robot whose measured feedback has expired.
    if(!feedback_ok && params_.backend_name == "ros_control" && !motion_executor_->initialized()) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),1000,
          "Waiting for the first complete ros_control /joint_states sample");
      force_resume_steady_s_=0.;
      return;
    }
    if (simulation && simulation_paused_) {
      if (force_resume_steady_s_ == 0.0) force_resume_steady_s_ = steady_clock_s;
      const bool observations_ready = !sim_force || !force_adapter_->requiresFreshWrenchForResume() ||
          (force_adapter_->freshUniqueWrench() &&
          (!force_parking_ || force_parking_->freshFeedback()));
      switch (fr10_motion::resumeReadiness(feedback_ok, observations_ready,
          steady_clock_s-force_resume_steady_s_, params_.feedback_timeout_s)) {
      case fr10_motion::ResumeDecision::WAIT:
        return; // No reference, integrator or watchdog renewal while waiting.
      case fr10_motion::ResumeDecision::FEEDBACK_EXPIRED:
        RCLCPP_ERROR(get_logger(),"Simulation resumed without fresh initialized robot feedback");
        if (sim_force) force_adapter_->rejectPending(fr10_motion_interfaces::action::Insert::Result::STALE_WRENCH,
            "simulation resumed without fresh encoder feedback; stop outcome unknown");
        requestStopAll(StopReason::COMM_FAILURE);
        return;
      case fr10_motion::ResumeDecision::OBSERVATION_EXPIRED:
        // FT/parking safety runs below with the actual expired observation.
        // Fresh encoders retain the sole executor for braking and fault HOLD.
        [[fallthrough]];
      case fr10_motion::ResumeDecision::READY:
        simulation_paused_ = false;
        break;
      }
    }
    if (!feedback_ok) {
      ++consecutive_feedback_failures_;
      fr10_motion::RosControlBackend::FeedbackDiagnostics feedback_diag{};
      if (backend_) {
        feedback_diag = static_cast<fr10_motion::RosControlBackend *>(
            backend_.get())->feedbackDiagnostics();
      }
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Robot feedback read failed (%d consecutive): source_seq=%lu "
          "age_ms=%.3f safety_known=%s valid=%s callbacks=%lu "
          "source_gap_ms=%.3f receive_gap_ms=%.3f buffer_drops=%lu "
          "rejected_stamps=%lu rejected_samples=%lu",
          consecutive_feedback_failures_, feedback.feedback_sequence,
          feedback.feedback_age_s * 1000.0,
          feedback.safety_known ? "true" : "false",
          feedback.valid ? "true" : "false",
          feedback_diag.accepted_callbacks,
          feedback_diag.max_source_gap_s * 1000.0,
          feedback_diag.max_receive_gap_s * 1000.0,
          feedback_diag.dropped_buffer_writes,
          feedback_diag.rejected_stamps,
          feedback_diag.rejected_samples);
      if (consecutive_feedback_failures_ >= 1) {
        requestStopAll(StopReason::COMM_FAILURE);
      }
      return;
    }
    consecutive_feedback_failures_ = 0;
    if (backend_->isReady() && !backend_ready_announced_.exchange(true)) {
      RCLCPP_INFO(
          get_logger(),
          "[READY] fr10_pose_controller ready at %.1f Hz; waiting for a target",
          params_.controller_config.control_rate_hz);
    }
    feedback_buffer_.write(feedback);
    consumePendingForceGoal();
    const auto pre_update_status = motion_executor_->status();
    if (params_.collision_checking_enabled &&
        pre_update_status.state == fr10_motion::MotionState::TRACKING &&
        active_scene_revision_.load(std::memory_order_acquire) !=
            scene_revision_.load(std::memory_order_acquire)) {
      StopReason expected = StopReason::NONE;
      if (stop_latch_.compare_exchange_strong(expected,
                                               StopReason::SCENE_CHANGED)) {
        RCLCPP_WARN(get_logger(),
                    "[SCENE CHANGED] active=%lu current=%lu; requesting "
                    "controlled stop",
                    active_scene_revision_.load(std::memory_order_acquire),
                    scene_revision_.load(std::memory_order_acquire));
      }
    }
    const auto stop_request = stop_latch_.exchange(StopReason::NONE);
    ForceTickSubmission force_submission;
    if (stop_request != StopReason::NONE) {
      if (stop_request != StopReason::PATH_DEVIATION ||
          path_stop_generation_.exchange(0) ==
              motion_executor_->status().active_generation) {
        requestControlledStop(stop_request);
      }
    } else if (!shutdown_in_progress_.load()) {
      consumeKinematicsResult();
      force_submission = updateForceRuntime(
          feedback, motion_executor_->status(), motion_executor_->command());
    }

    auto status = motion_executor_->update(feedback, measured_period_s);
    if(pre_update_status.owner == fr10_motion::MotionOwner::FORCE_ONLINE &&
       pre_update_status.state == fr10_motion::MotionState::TRACKING &&
       status.state == fr10_motion::MotionState::STOPPING) {
      RCLCPP_WARN(get_logger(),
          "[ONLINE CONTROLLED STOP] active_sequence=%lu feedback_source=%.12f lease_until=%.12f source_gap_ms=%.3f reference_lag=%.9f",
          pre_update_status.active_online_sequence,feedback.timestamp_s,
          pre_update_status.online_valid_until_s,1000.*measured_period_s,
          status.lag_event.error_rad);
    }
    const auto command = motion_executor_->command();
    if (status.lag_event.sequence > handled_lag_event_) {
      handled_lag_event_ = status.lag_event.sequence;
      // Invalidates staging/reversal/worker results as well as the active goal.
      // Executor stop() is idempotent and cannot restart the braking profile.
      requestControlledStop(StopReason::REFERENCE_LAG);
    }
    if (status.state == fr10_motion::MotionState::FAULT) {
      const StopReason fault_reason =
          stop_reason_.load() == StopReason::PATH_DEVIATION
              ? StopReason::PATH_DEVIATION
              : StopReason::HARDWARE_ALARM;
      RCLCPP_ERROR(
          get_logger(),
          "MotionExecutor FAULT reason=%s: feedback_valid=%s emergency=%s "
          "collision=%s robot_mode=%d main_error=%d sub_error=%d",
          stopReasonName(fault_reason), feedback.valid ? "true" : "false",
          feedback.emergency_stop ? "true" : "false",
          feedback.collision_detected ? "true" : "false", feedback.robot_mode,
          feedback.main_error_code, feedback.sub_error_code);
      requestStopAll(fault_reason);
      return;
    }

    if (command.type != fr10_motion::CommandType::NONE) {
      const bool command_ok = backend_ && backend_->writeCommand(command);
      if (!command_ok) {
        if (force_submission.accepted && force_adapter_) {
          force_adapter_->commitOnlineTarget(
              force_submission.online_sequence, false);
        }
        ++command_write_errors_;
        if (force_adapter_ && force_adapter_->busy()) {
          force_adapter_->rejectPending(fr10_motion_interfaces::action::Insert::Result::COMMAND_REJECTED,
              "backend publish failed; no acknowledged controlled stop is available");
        }
        RCLCPP_ERROR(
            get_logger(),
            "Backend command write failed: type=%s sequence=%lu errors=%lu",
            fr10_motion::commandTypeName(command.type), command.sequence,
            command_write_errors_);
        requestStopAll(StopReason::COMM_FAILURE);
        return;
      }
      updateContinuityMetrics(command);
      motion_executor_->acknowledgeCommand(command.sequence);
      if (force_submission.accepted && force_adapter_) {
        force_adapter_->commitOnlineTarget(
            force_submission.online_sequence, true);
      }
      latest_command_buffer_.write(command);
      if (force_joint_trace_pub_ && force_adapter_ && force_adapter_->busy() &&
          command.type == fr10_motion::CommandType::JOINT_POSITION) {
        for (std::size_t i=0; i<6; ++i) {
          force_joint_trace_message_.data[i] = command.joint_position_rad[i];
          force_joint_trace_message_.data[6+i] = command.joint_velocity_rad_s[i];
          force_joint_trace_message_.data[12+i] = command.joint_acceleration_rad_s2[i];
        }
        force_joint_trace_message_.layout.data_offset = static_cast<uint32_t>(command.sequence);
        force_joint_trace_pub_->publish(force_joint_trace_message_);
      }
    } else if (force_submission.accepted && force_adapter_) {
      force_adapter_->commitOnlineTarget(force_submission.online_sequence,
                                         false);
    }

    // Terminal state and every handover consumer observe the publication
    // commit, not an unacknowledged candidate from update().
    status = motion_executor_->status();
    const auto owned = coordinator_->current();
    const uint64_t executing_generation = status.owner == fr10_motion::MotionOwner::JOINT_OTG
        ? status.active_revision : status.active_generation;
    if (owned && owned->phase == fr10_motion::CommandPhase::EXECUTING &&
        owned->generation == executing_generation &&
        status.state == fr10_motion::MotionState::HOLDING &&
        status.outcome == fr10_motion::MotionOutcome::GOAL_REACHED) {
      coordinator_->transition(owned->id, fr10_motion::CommandPhase::SUCCEEDED);
    }
    queuePostStopReplanIfReady(status, feedback, command);
    queueInsertionAfterStagingIfReady(status, feedback, command);

    ++loop_samples_;
    loop_elapsed_s_ += measured_period_s;
    const double absolute_jitter_ms =
        std::abs(measured_period_s - params_.nominal_period_s) * 1000.0;
    loop_abs_jitter_sum_ms_ += absolute_jitter_ms;
    max_abs_jitter_ms_ = std::max(max_abs_jitter_ms_, absolute_jitter_ms);
    max_cycle_period_ms_ =
        std::max(max_cycle_period_ms_, measured_period_s * 1000.0);
    const double execution_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - tick_start).count();
    max_tick_execution_ms_ = std::max(max_tick_execution_ms_, execution_ms);
    max_feedback_age_ms_ = std::max(max_feedback_age_ms_,
                                    feedback.feedback_age_s * 1000.0);
    const auto bin = [](double milliseconds) {
      return std::min<std::size_t>(127, static_cast<std::size_t>(
          std::max(0.0, milliseconds) * 2.0));
    };
    ++period_histogram_[bin(measured_period_s * 1000.0)];
    ++execution_histogram_[bin(execution_ms)];
    if (execution_ms > params_.nominal_period_s * 1000.0) {
      ++tick_deadline_misses_;
      ++consecutive_tick_misses_;
      max_consecutive_tick_misses_ =
          std::max(max_consecutive_tick_misses_, consecutive_tick_misses_);
    } else {
      consecutive_tick_misses_ = 0;
    }
    if (measured_period_s > 1.5 * params_.nominal_period_s) {
      ++scheduler_gaps_;
    }
    fr10_motion::recordGoalReachedStopReason(
        stop_reason_, stop_requested_, stop_latch_,
        status.state == fr10_motion::MotionState::HOLDING &&
            status.outcome == fr10_motion::MotionOutcome::GOAL_REACHED);
    DiagnosticsSnapshot snapshot;
    snapshot.status = status;
    snapshot.feedback = feedback;
    snapshot.command = command;
    snapshot.loop.samples = loop_samples_;
    snapshot.loop.elapsed_s = loop_elapsed_s_;
    snapshot.loop.mean_abs_jitter_ms =
        loop_abs_jitter_sum_ms_ / static_cast<double>(loop_samples_);
    snapshot.loop.max_abs_jitter_ms = max_abs_jitter_ms_;
    snapshot.loop.max_cycle_period_ms = max_cycle_period_ms_;
    snapshot.loop.max_tick_execution_ms = max_tick_execution_ms_;
    snapshot.loop.max_feedback_age_ms = max_feedback_age_ms_;
    snapshot.loop.tick_deadline_misses = tick_deadline_misses_;
    snapshot.loop.scheduler_gaps = scheduler_gaps_;
    snapshot.loop.max_consecutive_tick_misses = max_consecutive_tick_misses_;
    snapshot.loop.period_histogram = period_histogram_;
    snapshot.loop.execution_histogram = execution_histogram_;
    snapshot.loop.max_delta_joint_velocity_rad_s =
        max_delta_joint_velocity_rad_s_;
    snapshot.loop.max_delta_linear_velocity_m_s =
        max_delta_linear_velocity_m_s_;
    snapshot.loop.command_write_errors = command_write_errors_;
    diagnostics_buffer_.write(snapshot);
    if (shutdown_in_progress_.load() &&
        (status.state == fr10_motion::MotionState::HOLDING ||
         status.state == fr10_motion::MotionState::IDLE)) {
      requestStopAll(StopReason::USER_INTERRUPT);
    }
  }

void MotionAdapter::pathMonitorTick() {
    const auto tick = std::chrono::steady_clock::now();
    const double dt =
        std::chrono::duration<double>(tick - path_monitor_last_tick_).count();
    path_monitor_last_tick_ = tick;

    DiagnosticsSnapshot snapshot;
    if (!diagnostics_buffer_.read(snapshot) || !analytical_ik_provider_ ||
        snapshot.status.owner != fr10_motion::MotionOwner::POSE_PATH ||
        !snapshot.status.cartesian_reference_valid ||
        (snapshot.status.state != fr10_motion::MotionState::TRACKING &&
         snapshot.status.state != fr10_motion::MotionState::HOLDING)) {
      path_fault_monitor_.reset();
      return;
    }

    const auto active_policy = active_pose_policy_.load();
    const auto &monitor_config =
        params_.cartesian_fault_monitor_configs[policyIndex(active_policy)];
    if (!path_monitor_policy_initialized_ ||
        monitored_pose_policy_ != active_policy) {
      path_fault_monitor_.setConfig(monitor_config);
      monitored_pose_policy_ = active_policy;
      path_monitor_policy_initialized_ = true;
      RCLCPP_INFO(
          get_logger(),
          "[PATH MONITOR PROFILE] policy=%s along_limit=%.1f mm "
          "persistence=%.1f ms perpendicular_limit=%.1f mm",
          fr10_motion::poseExecutionPolicyName(active_policy),
          monitor_config.tracking_lag_tolerance_m * 1000.0,
          monitor_config.persistence_s * 1000.0,
          monitor_config.perpendicular_tolerance_m * 1000.0);
    }

    fr10_motion::KinematicState joint_state;
    joint_state.position_rad = snapshot.feedback.joint_position_rad;
    joint_state.velocity_rad_s = snapshot.feedback.joint_velocity_rad_s;
    joint_state.acceleration_rad_s2 =
        snapshot.feedback.joint_acceleration_rad_s2;
    fr10_motion::Pose actual_pose;
    std::string error;
    fr10_motion::CartesianTrackingErrors errors;
    if (!analytical_ik_provider_->forward(joint_state, actual_pose, &error)) {
      const double invalid = std::numeric_limits<double>::infinity();
      errors = {invalid, invalid, invalid, invalid};
    } else {
      errors = fr10_motion::cartesianTrackingErrors(
          snapshot.status.desired_cartesian_pose,
          snapshot.status.desired_cartesian_translation_tangent, actual_pose);
    }

    if (active_policy == fr10_motion::PoseExecutionPolicy::FREE_SPACE) {
      // FREE_SPACE is intentionally joint-space/free-form.  Its sampled
      // Cartesian reference is telemetry, not an invariant path: applying
      // orientation/perpendicular thresholds here silently converts Mode 1
      // back into Cartesian tracking.  Kinematic/collision checks cover every
      // planned sample and the independent joint-reference lead monitor (with
      // the FREE_SPACE persistence window) remains the runtime stop guard.
      path_fault_monitor_.reset();
      motion_executor_->reportCartesianTrackingError(
          errors.position_error_m, errors.orientation_error_rad,
          errors.perpendicular_error_m, false);
      return;
    }

    const auto previous = path_fault_monitor_.state();
    const auto state = path_fault_monitor_.update(
        errors, snapshot.status.state == fr10_motion::MotionState::TRACKING,
        dt);
    const bool persistent =
        state == fr10_motion::CartesianFaultState::PERSISTENT_FAULT;
    // The monitor thread publishes telemetry only. It latches a
    // generation-qualified request for the control runner, which remains the
    // sole writer of executor state.
    motion_executor_->reportCartesianTrackingError(
        errors.position_error_m, errors.orientation_error_rad,
        errors.perpendicular_error_m, false);
    if (state == fr10_motion::CartesianFaultState::WARNING &&
        previous == fr10_motion::CartesianFaultState::NORMAL) {
      RCLCPP_WARN(get_logger(),
                  "[PATH MONITOR WARNING] along=%.6f m perpendicular=%.6f m "
                  "orientation=%.6f rad; persistence threshold=%.3f ms",
                  errors.along_track_error_m, errors.perpendicular_error_m,
                  errors.orientation_error_rad,
                  monitor_config.persistence_s * 1000.0);
    }
    if (persistent &&
        previous != fr10_motion::CartesianFaultState::PERSISTENT_FAULT) {
      RCLCPP_ERROR(get_logger(),
                   "[PATH MONITOR PERSISTENT] duration=%.3f ms position=%.6f m "
                   "along=%.6f m perpendicular=%.6f m orientation=%.6f rad; "
                   "requesting CONTROLLED STOP",
                   path_fault_monitor_.violationDuration() * 1000.0,
                   errors.position_error_m, errors.along_track_error_m,
                   errors.perpendicular_error_m, errors.orientation_error_rad);
      path_stop_generation_.store(snapshot.status.active_generation);
      stop_latch_.store(StopReason::PATH_DEVIATION);
    }
  }

} // namespace fr10_pose_controller::detail
