#include "motion_adapter_internal.hpp"

namespace fr10_pose_controller::detail {

void MotionAdapter::publishCommandEvents() {
  for (std::size_t i = 0; i < 64; ++i) {
    const auto event = coordinator_->nextEvent();
    if (!event) break;
    fr10_motion_interfaces::msg::MotionCommandStatus message;
    message.header.stamp = now();
    message.header.frame_id = params_.base_frame;
    message.command_id = event->id;
    message.generation = event->generation;
    message.scene_revision = event->scene_revision;
    message.boundary_sequence = event->boundary_sequence;
    message.mode = static_cast<uint8_t>(event->mode);
    message.phase = static_cast<uint8_t>(event->phase);
    message.reason = static_cast<uint16_t>(event->reason);
    message.dropped_events = coordinator_->droppedEvents();
    command_status_pub_->publish(message);
  }
}

void MotionAdapter::initializeDiagnosticFrames() {
    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();
    state.update();
    const Eigen::Isometry3d flange_to_tcp =
        state.getGlobalLinkTransform(params_.diagnostic_flange_frame).inverse() *
        state.getGlobalLinkTransform(params_.kinematics_tip_link);
    if (!flange_to_tcp.matrix().allFinite()) {
      throw std::runtime_error(
          "Diagnostic FLANGE-to-TCP transform is non-finite");
    }
    const Eigen::Vector3d translation = flange_to_tcp.translation();
    const Eigen::Quaterniond orientation(flange_to_tcp.rotation());
    flange_to_tcp_.position_m = {translation.x(), translation.y(),
                                 translation.z()};
    flange_to_tcp_.orientation_quat = {orientation.x(), orientation.y(),
                                       orientation.z(), orientation.w()};
    if (!fr10_motion::math::normalizeQuaternion(
            flange_to_tcp_.orientation_quat)) {
      throw std::runtime_error(
          "Diagnostic FLANGE-to-TCP orientation is invalid");
    }

    const std::array<double, 3> specified_translation{0.060, 0.0, 0.175};
    double translation_mismatch_squared = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const double delta =
          flange_to_tcp_.position_m[axis] - specified_translation[axis];
      translation_mismatch_squared += delta * delta;
    }
    const double translation_mismatch_m =
        std::sqrt(translation_mismatch_squared);
    const double rotation_from_identity_rad =
        fr10_motion::math::quaternionAngularDistance(
            flange_to_tcp_.orientation_quat, {0.0, 0.0, 0.0, 1.0});
    const auto rpy = fr10_motion::math::quaternionToFixedXyzRpy(
        flange_to_tcp_.orientation_quat);

    RCLCPP_INFO(get_logger(),
                "[TOOL MODEL] FLANGE=%s TCP=%s model_translation=[%.6f, "
                "%.6f, %.6f] m model_fixed_XYZ_RPY=[%.6f, %.6f, %.6f] deg "
                "source=robot_description/URDF",
                params_.diagnostic_flange_frame.c_str(), params_.kinematics_tip_link.c_str(),
                flange_to_tcp_.position_m[0], flange_to_tcp_.position_m[1],
                flange_to_tcp_.position_m[2], rpy[0] * fr10_motion::kRadToDeg,
                rpy[1] * fr10_motion::kRadToDeg,
                rpy[2] * fr10_motion::kRadToDeg);
    if (translation_mismatch_m > 1e-6) {
      RCLCPP_WARN(get_logger(),
                  "[TOOL MODEL MISMATCH] specified cylinder L=0.150 m D=0.060 "
                  "m R=0.030 m center_flange=[0.060, 0.000, 0.100] m "
                  "tcp_flange=[0.060, 0.000, 0.175] m; current %s->%s "
                  "translation differs by %.3f mm. Model was not modified.",
                  params_.diagnostic_flange_frame.c_str(),
                  params_.kinematics_tip_link.c_str(),
                  translation_mismatch_m * 1000.0);
    } else {
      RCLCPP_INFO(get_logger(), "[TOOL MODEL VERIFIED] specified and modeled "
                                "FLANGE-to-TCP translations match");
    }
    RCLCPP_INFO(
        get_logger(),
        "[TOOL ROTATION VERIFIED] fixed rotation source=robot_description/URDF "
        "identity=%s angular_distance_from_identity=%.9f deg",
        rotation_from_identity_rad <= 1e-9 ? "true" : "false",
        rotation_from_identity_rad * fr10_motion::kRadToDeg);
  }

bool MotionAdapter::populateCartesianFeedback(fr10_motion::RobotState &feedback,
                                 fr10_motion::Pose &actual_flange) const {
    if (!moveit_ik_provider_) {
      return false;
    }
    fr10_motion::KinematicState state;
    state.position_rad = feedback.joint_position_rad;
    state.velocity_rad_s = feedback.joint_velocity_rad_s;
    state.acceleration_rad_s2 = feedback.joint_acceleration_rad_s2;
    if (!moveit_ik_provider_->forwardLink(state, params_.diagnostic_flange_frame,
                                          actual_flange)) {
      return false;
    }
    const fr10_motion::Pose actual_tcp =
        fr10_motion::diagnostics::composePose(actual_flange, flange_to_tcp_);
    feedback.tcp_position_m = actual_tcp.position_m;
    feedback.tcp_orientation_quat = actual_tcp.orientation_quat;
    return true;
  }

void MotionAdapter::updateContinuityMetrics(const fr10_motion::MotionCommand &command) {
    if (have_previous_command_ && command.type == previous_command_.type) {
      if (command.type == fr10_motion::CommandType::JOINT_POSITION) {
        for (std::size_t i = 0; i < fr10_motion::kJointCount; ++i) {
          max_delta_joint_velocity_rad_s_ =
              std::max(max_delta_joint_velocity_rad_s_,
                       std::abs(command.joint_velocity_rad_s[i] -
                                previous_command_.joint_velocity_rad_s[i]));
        }
      }
    }
    previous_command_ = command;
    have_previous_command_ = true;
  }

void MotionAdapter::publishTcpPose(const fr10_motion::RobotState &feedback) {
    // This pose is FK of one measured JointState, not a measurement at the
    // diagnostics callback's wall time. Preserve the source sample stamp.
    if (!feedback.valid || !std::isfinite(feedback.timestamp_s) || feedback.timestamp_s <= 0.0) {
      return;
    }
    tcp_pose_message_.header.stamp = rclcpp::Time(
        static_cast<int64_t>(std::llround(feedback.timestamp_s * 1e9)),
        get_clock()->get_clock_type());
    tcp_pose_message_.pose.position.x = feedback.tcp_position_m[0];
    tcp_pose_message_.pose.position.y = feedback.tcp_position_m[1];
    tcp_pose_message_.pose.position.z = feedback.tcp_position_m[2];
    tcp_pose_message_.pose.orientation.x = feedback.tcp_orientation_quat[0];
    tcp_pose_message_.pose.orientation.y = feedback.tcp_orientation_quat[1];
    tcp_pose_message_.pose.orientation.z = feedback.tcp_orientation_quat[2];
    tcp_pose_message_.pose.orientation.w = feedback.tcp_orientation_quat[3];
    tcp_pose_pub_->publish(tcp_pose_message_);
  }

void MotionAdapter::diagnosticsTick() {
    publishCommandEvents();
    DiagnosticsSnapshot snapshot;
    if (!diagnostics_buffer_.read(snapshot)) {
      return;
    }
    auto status = snapshot.status;
    auto feedback = snapshot.feedback;
    fr10_motion::Pose actual_flange;
    const bool actual_flange_valid =
        populateCartesianFeedback(feedback, actual_flange);
    if (actual_flange_valid) {
      fr10_motion::Pose tcp_feedback;
      tcp_feedback.position_m = feedback.tcp_position_m;
      tcp_feedback.orientation_quat = feedback.tcp_orientation_quat;
      tcp_feedback_buffer_.write(tcp_feedback);
      publishTcpPose(feedback);
    }

    double along_track_error_m = 0.0;
    bool tracking = false;
    if (status.owner == fr10_motion::MotionOwner::POSE_PATH &&
        status.cartesian_reference_valid) {
      fr10_motion::Pose actual_pose;
      actual_pose.position_m = feedback.tcp_position_m;
      actual_pose.orientation_quat = feedback.tcp_orientation_quat;
      const auto errors = fr10_motion::cartesianTrackingErrors(
          status.desired_cartesian_pose,
          status.desired_cartesian_translation_tangent, actual_pose);
      status.position_error_m = errors.position_error_m;
      status.angular_error_rad = errors.orientation_error_rad;
      status.cartesian_path_deviation_m = errors.perpendicular_error_m;
      along_track_error_m = errors.along_track_error_m;
      tracking = status.state == fr10_motion::MotionState::TRACKING;
    }

    const StopReason event = pending_event_.exchange(StopReason::NONE);
    if (event == StopReason::SINGULARITY) {
      RCLCPP_ERROR(get_logger(),
                   "[TARGET REJECTED: SINGULARITY] Target violates the "
                   "configured kinematic safety barrier.");
    }
    if (event == StopReason::PATH_DEVIATION) {
      const auto &monitor_config = params_.cartesian_fault_monitor_configs[policyIndex(
          active_pose_policy_.load())];
      RCLCPP_ERROR(
          get_logger(),
          "[PATH MONITOR CONTROLLED STOP] state=%s position=%.6f m "
          "along=%.6f m "
          "along_limit=%.6f m orientation=%.6f rad perpendicular=%.6f m "
          "perpendicular_limit=%.6f m",
          tracking ? "TRACKING" : fr10_motion::motionStateName(status.state),
          status.position_error_m, along_track_error_m,
          tracking ? monitor_config.tracking_lag_tolerance_m
                   : params_.cartesian_position_tolerance_m,
          status.angular_error_rad, status.cartesian_path_deviation_m,
          params_.cartesian_path_deviation_tolerance_m);
    }
    if (status.lag_event.sequence > reported_lag_event_) {
      reported_lag_event_ = status.lag_event.sequence;
      const auto &lag = status.lag_event;
      RCLCPP_ERROR(
          get_logger(),
          "[REFERENCE LAG CONTROLLED STOP] event=%lu joint=%zu feedback_seq=%lu "
          "q_cmd=%.6f q_actual=%.6f v_cmd=%.6f age_ms=%.3f "
          "error=%.6f rad limit=%.6f rad",
          lag.sequence, lag.joint + 1, lag.feedback_sequence,
          lag.sent_reference.joint_position_rad[lag.joint],
          lag.feedback.joint_position_rad[lag.joint],
          lag.sent_reference.joint_velocity_rad_s[lag.joint],
          lag.feedback_age_s * 1000.0, lag.error_rad,
          params_.controller_config.max_joint_reference_lead_rad);
    }
    if (event == StopReason::CONTROLLED_STOP) {
      RCLCPP_WARN(get_logger(),
                  "[CONTROLLED STOP] jerk-limited braking profile active");
    }
    if (event == StopReason::RETARGET_TIMING) {
      RCLCPP_WARN(get_logger(),
                  "[RETARGET TIMING STOP] jerk-limited braking profile "
                  "active; target retained for zero-velocity replan");
    }
    if (event == StopReason::STALE_TRAJECTORY) {
      RCLCPP_WARN(get_logger(),
                  "[STALE TRAJECTORY CONTROLLED STOP] target retained for "
                  "one zero-velocity replan");
    }
    StatusLogSnapshot status_log_snapshot;
    status_log_snapshot.status = status;
    status_log_snapshot.feedback = feedback;
    status_log_snapshot.command = snapshot.command;
    status_log_snapshot.actual_flange = actual_flange;
    status_log_snapshot.loop = snapshot.loop;
    status_log_snapshot.actual_flange_valid = actual_flange_valid;
    status_log_snapshot.force_phase =
        force_adapter_ ? force_adapter_->diagnosticPhase() : nullptr;
    if (status.owner == fr10_motion::MotionOwner::POSE_PATH) {
      DisplayPoseGoalSnapshot display_goal;
      if (display_pose_goal_buffer_.read(display_goal) && display_goal.valid) {
        status_log_snapshot.final_pose_goal = display_goal.pose;
        status_log_snapshot.final_pose_goal_valid = true;
      } else if (status.cartesian_reference_valid) {
        status_log_snapshot.final_pose_goal = status.cartesian_path_goal;
        status_log_snapshot.final_pose_goal_valid = true;
      }
    }
    status_log_buffer_.write(status_log_snapshot);
    publishStatus(status, feedback, snapshot.command, snapshot.loop);
  }

void MotionAdapter::publishStatus(const fr10_motion::MotionStatus &status,
                     const fr10_motion::RobotState &feedback,
                     const fr10_motion::MotionCommand &command,
                     const LoopMetrics &loop) {
    const double mean_rate_hz =
        loop.elapsed_s > 0.0 ? loop.samples / loop.elapsed_s : 0.0;
    const auto percentile_ms = [](const std::array<uint32_t, 128> &bins,
                                  double percentile) {
      uint64_t total = 0;
      for (const auto count : bins) { total += count; }
      if (total == 0) { return 0.0; }
      const uint64_t rank = static_cast<uint64_t>(
          std::ceil(percentile * static_cast<double>(total)));
      uint64_t cumulative = 0;
      for (std::size_t index = 0; index < bins.size(); ++index) {
        cumulative += bins[index];
        if (cumulative >= rank) { return (static_cast<double>(index) + 0.5) * 0.5; }
      }
      return 63.75;
    };
    const uint64_t sdk_errors = feedback.sdk_read_errors + feedback.sdk_write_errors;
    const auto ik_statistics = analytical_ik_provider_
                                   ? analytical_ik_provider_->statistics()
                                   : fr10_motion::Fr10AnalyticalIKStatistics{};
    double position_error_m = status.position_error_m;
    double angular_error_rad = status.angular_error_rad;
    if (status.owner == fr10_motion::MotionOwner::POSE_PATH &&
        !status.cartesian_reference_valid) {
      fr10_motion::CartesianTarget target;
      bool have_target = false;
      AcceptedTargetSnapshot accepted_target;
      if (accepted_target_buffer_.read(accepted_target)) {
        have_target = accepted_target.have_cartesian_target;
        target = accepted_target.cartesian_target;
      }
      if (have_target) {
        double squared_error = 0.0;
        for (std::size_t axis = 0; axis < 3; ++axis) {
          const double delta =
              target.position_m[axis] - feedback.tcp_position_m[axis];
          squared_error += delta * delta;
        }
        position_error_m = std::sqrt(squared_error);
        angular_error_rad = fr10_motion::math::quaternionAngularDistance(
            target.orientation_quat, feedback.tcp_orientation_quat);
      }
    }

    std::array<char, 2048> buffer{};
    const int length = std::snprintf(
        buffer.data(), buffer.size(),
        "state=%s outcome=%s mode=%s pose_policy=%s owner=%s type=%s "
        "revision=%lu generation=%lu scene_revision=%lu "
        "active_scene_revision=%lu command_seq=%lu feedback_seq=%lu "
        "feedback_age_ms=%.3f safety_known=%s "
        "backend_ready=%s loop_samples=%lu loop_hz=%.3f "
        "mean_cycle_period_ms=%.6f "
        "mean_abs_jitter_ms=%.6f max_abs_jitter_ms=%.6f "
        "max_cycle_period_ms=%.6f "
        "period_p50_ms=%.3f period_p95_ms=%.3f period_p99_ms=%.3f "
        "execution_p50_ms=%.3f execution_p95_ms=%.3f execution_p99_ms=%.3f "
        "max_tick_execution_ms=%.3f tick_deadline_misses=%lu "
        "scheduler_gaps=%lu max_consecutive_tick_misses=%lu "
        "max_feedback_age_ms=%.3f "
        "ik_mean_us=%.3f ik_max_us=%.3f "
        "position_error_m=%.6f angular_error_rad=%.6f path_deviation_m=%.6f "
        "reference_actual_rad=%.6f candidate_reference_actual_rad=%.6f "
        "reference_lagging=%s "
        "reference_lag_samples=%u "
        "max_delta_qdot_rad_s=%.6f max_delta_v_m_s=%.6f "
        "sdk_read_errors=%lu sdk_write_errors=%lu command_write_errors=%lu "
        "sdk_errors=%lu "
        "reason=%s",
        fr10_motion::motionStateName(status.state),
        fr10_motion::motionOutcomeName(status.outcome),
        fr10_motion::controlModeName(active_control_mode_.load()),
        fr10_motion::poseExecutionPolicyName(active_pose_policy_.load()),
        fr10_motion::motionOwnerName(status.owner),
        fr10_motion::commandTypeName(status.active_type),
        status.active_revision, status.active_generation,
        scene_revision_.load(std::memory_order_acquire),
        active_scene_revision_.load(std::memory_order_acquire),
        command.sequence,
        feedback.feedback_sequence, feedback.feedback_age_s * 1000.0,
        feedback.safety_known ? "true" : "false",
        backend_ready_announced_.load() ? "true" : "false", loop.samples,
        mean_rate_hz,
        loop.samples > 0
            ? (loop.elapsed_s / static_cast<double>(loop.samples)) * 1000.0
            : 0.0,
        loop.mean_abs_jitter_ms, loop.max_abs_jitter_ms,
        loop.max_cycle_period_ms,
        percentile_ms(loop.period_histogram, 0.50),
        percentile_ms(loop.period_histogram, 0.95),
        percentile_ms(loop.period_histogram, 0.99),
        percentile_ms(loop.execution_histogram, 0.50),
        percentile_ms(loop.execution_histogram, 0.95),
        percentile_ms(loop.execution_histogram, 0.99),
        loop.max_tick_execution_ms, loop.tick_deadline_misses,
        loop.scheduler_gaps, loop.max_consecutive_tick_misses,
        loop.max_feedback_age_ms, ik_statistics.mean_solve_time_us,
        ik_statistics.max_solve_time_us, position_error_m, angular_error_rad,
        status.cartesian_path_deviation_m,
        status.max_reference_actual_error_rad,
        status.candidate_reference_error_rad,
        status.reference_lagging ? "true" : "false",
        status.reference_lag_samples,
        loop.max_delta_joint_velocity_rad_s, loop.max_delta_linear_velocity_m_s,
        feedback.sdk_read_errors, feedback.sdk_write_errors,
        loop.command_write_errors, sdk_errors,
        stopReasonName(stop_reason_.load()));
    if (length > 0) {
      status_message_.data.assign(
          buffer.data(), static_cast<std::size_t>(std::min<int>(
                             length, static_cast<int>(buffer.size() - 1))));
      status_pub_->publish(status_message_);
      tracking_diagnostics_pub_->publish(status_message_);
    }

    // Goal Reached event: 1-time log when transitioning from TRACKING ->
    // HOLDING
    if (previous_reported_state_ == fr10_motion::MotionState::STOPPING &&
        status.state == fr10_motion::MotionState::HOLDING) {
      RCLCPP_INFO(get_logger(),
                  "[CONTROLLED STOP COMPLETE] qdot reference reached zero; "
                  "robot command is HOLDING");
    } else if (status.outcome == fr10_motion::MotionOutcome::GOAL_REACHED &&
               previous_reported_state_ != fr10_motion::MotionState::HOLDING &&
               status.state == fr10_motion::MotionState::HOLDING) {
      if (status.owner == fr10_motion::MotionOwner::POSE_PATH) {
        stop_reason_.store(StopReason::GOAL_REACHED);
        RCLCPP_INFO(get_logger(),
                    "[STOP: GOAL_REACHED] Position error: %.1f mm / angular "
                    "error: %.1f deg. Stable HOLD.",
                    position_error_m * 1000.0,
                    angular_error_rad * fr10_motion::kRadToDeg);
      } else if (status.active_type ==
                 fr10_motion::CommandType::JOINT_POSITION) {
        double max_joint_err_deg = 0.0;
        for (std::size_t i = 0; i < fr10_motion::kJointCount; ++i) {
          max_joint_err_deg =
              std::max(max_joint_err_deg, std::abs(status.joint_error_rad[i]) *
                                              fr10_motion::kRadToDeg);
        }
        stop_reason_.store(StopReason::GOAL_REACHED);
        RCLCPP_INFO(get_logger(),
                    "[STOP: GOAL_REACHED] Maximum joint error: %.2f deg. "
                    "Stable HOLD.",
                    max_joint_err_deg);
      } else {
        stop_reason_.store(StopReason::GOAL_REACHED);
        RCLCPP_INFO(get_logger(),
                    "[STOP: GOAL_REACHED] Robot is in stable HOLD.");
      }
    }

    previous_reported_state_ = status.state;
  }

void MotionAdapter::statusLogTick() {
    StatusLogSnapshot snapshot;
    if (!status_log_buffer_.read(snapshot) || !snapshot.actual_flange_valid) {
      return;
    }

    std::array<double, fr10_motion::kJointCount> target_joint_rad =
        snapshot.feedback.joint_position_rad;
    if (snapshot.status.owner == fr10_motion::MotionOwner::JOINT_OTG) {
      AcceptedTargetSnapshot accepted_target;
      if (accepted_target_buffer_.read(accepted_target) &&
          accepted_target.have_joint_target) {
        target_joint_rad = accepted_target.joint_target.position_rad;
      }
    } else if (snapshot.command.type ==
               fr10_motion::CommandType::JOINT_POSITION) {
      target_joint_rad = snapshot.command.joint_position_rad;
    }

    fr10_motion::Pose target_tcp;
    if (snapshot.status.owner == fr10_motion::MotionOwner::POSE_PATH &&
        snapshot.final_pose_goal_valid) {
      target_tcp = snapshot.final_pose_goal;
    } else {
      fr10_motion::KinematicState target_state;
      target_state.position_rad = target_joint_rad;
      std::string fk_error;
      if (!moveit_ik_provider_->forward(target_state, target_tcp, &fk_error)) {
        RCLCPP_WARN(get_logger(), "Unified diagnostic target FK failed: %s",
                    fk_error.c_str());
        return;
      }
    }

    const auto diagnostic =
        fr10_motion::diagnostics::makeUnifiedTrackingDiagnostic(
            target_tcp, snapshot.actual_flange, flange_to_tcp_, target_joint_rad,
            snapshot.feedback.joint_position_rad);
    const double mean_rate_hz =
        snapshot.loop.elapsed_s > 0.0
            ? snapshot.loop.samples / snapshot.loop.elapsed_s
            : params_.controller_config.control_rate_hz;
    const char *mode =
        snapshot.status.owner == fr10_motion::MotionOwner::POSE_PATH
            ? "POSE"
            : (snapshot.command.type == fr10_motion::CommandType::JOINT_POSITION
                   ? "JOINT"
                   : "IDLE");
    const std::string block =
        fr10_motion::diagnostics::formatUnifiedTrackingBlock(
            fr10_motion::motionStateName(snapshot.status.state), mode,
            mean_rate_hz, diagnostic, snapshot.force_phase);
    if (block.empty()) {
      return;
    }
    const std::string log_key = fr10_motion::diagnostics::unifiedTrackingLogKey(
        fr10_motion::motionStateName(snapshot.status.state), mode, diagnostic,
        snapshot.force_phase);
    if (!fr10_motion::diagnostics::shouldEmitChangedTrackingLog(
            log_key, last_status_log_key_)) {
      return;
    }
    RCLCPP_INFO(get_logger(), "%s", block.c_str());
  }

} // namespace fr10_pose_controller::detail
