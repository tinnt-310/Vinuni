#include "motion_adapter_internal.hpp"
#include "fr10_motion_planning/joint_route_planner.hpp"

namespace fr10_pose_controller::detail {

void MotionAdapter::initializeKinematics() {
    robot_model_loader_ =
        std::make_shared<robot_model_loader::RobotModelLoader>(
            shared_from_this(), "robot_description", true);
    robot_model_ = robot_model_loader_->getModel();
    if (!robot_model_) {
      throw std::runtime_error("Failed to load MoveIt robot model");
    }
    joint_model_group_ = robot_model_->getJointModelGroup(params_.kinematics_group);
    if (!joint_model_group_ ||
        joint_model_group_->getVariableCount() != fr10_motion::kJointCount) {
      throw std::runtime_error("Invalid six-axis MoveIt group: " +
                               params_.kinematics_group);
    }
    if (!robot_model_->hasLinkModel(params_.kinematics_tip_link) ||
        !robot_model_->hasLinkModel(params_.kinematics_safety_link) ||
        !robot_model_->hasLinkModel(params_.diagnostic_flange_frame)) {
      throw std::runtime_error(
          "MoveIt model is missing configured tip/safety/diagnostic link");
    }
    const auto &model_joint_names =
        joint_model_group_->getActiveJointModelNames();
    if (model_joint_names != params_.joint_names) {
      throw std::runtime_error(
          "MoveIt group joint order must match controller.joint_names");
    }

    if (params_.collision_checking_enabled) {
      planning_scene_monitor_ =
          std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
              shared_from_this(), robot_model_loader_,
              "fr10_pose_controller_scene_monitor");
      if (!planning_scene_monitor_->getPlanningScene()) {
        throw std::runtime_error(
            "Failed to initialize MoveIt PlanningScene collision monitor");
      }
      planning_scene_monitor_->startSceneMonitor(params_.planning_scene_topic);
      planning_scene_monitor_->startWorldGeometryMonitor(
          params_.collision_object_topic, params_.planning_scene_world_topic, false);
      planning_scene_monitor_->startStateMonitor(params_.joint_state_topic);
      if (!force_runtime_config_.task_frame.empty()) {
        task_geometry_sub_=create_subscription<moveit_msgs::msg::CollisionObject>(
          params_.collision_object_topic,rclcpp::QoS(10).reliable(),
          [this](moveit_msgs::msg::CollisionObject::ConstSharedPtr message) {
            std::lock_guard<std::mutex> lock(task_geometry_mutex_);
            if(message->operation==moveit_msgs::msg::CollisionObject::ADD &&
               message->header.frame_id==force_runtime_config_.task_frame) task_geometry_[message->id]=*message;
            else task_geometry_.erase(message->id); // Never reuse geometry after a partial/frame-changing update.
            scene_revision_.fetch_add(1,std::memory_order_acq_rel);
          });
      }
      // Read-only scene evidence/inspection; served outside the actuation tick.
      planning_scene_monitor_->providePlanningSceneService("/get_planning_scene");
      planning_scene_monitor_->addUpdateCallback(
          [this](planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType
                     update_type) {
            const int update = static_cast<int>(update_type);
            const int collision_relevant =
                static_cast<int>(planning_scene_monitor::PlanningSceneMonitor::
                                     UPDATE_TRANSFORMS) |
                static_cast<int>(planning_scene_monitor::PlanningSceneMonitor::
                                     UPDATE_GEOMETRY);
            if ((update & collision_relevant) != 0) {
              scene_revision_.fetch_add(1, std::memory_order_acq_rel);
            }
          });
      RCLCPP_INFO(get_logger(),
                  "PlanningScene collision validation enabled: scene=%s "
                  "objects=%s world=%s",
                  params_.planning_scene_topic.c_str(),
                  params_.collision_object_topic.c_str(),
                  params_.planning_scene_world_topic.c_str());
    }

    fr10_motion_planning::MoveItIKProviderConfig provider_config;
    provider_config.group_name = params_.kinematics_group;
    provider_config.tip_link = params_.kinematics_tip_link;
    provider_config.base_link = params_.base_frame;
    provider_config.safety_link = params_.kinematics_safety_link;
    provider_config.timeout_s = params_.ik_timeout_s;
    provider_config.wrist_min_abs_rad = params_.wrist_singularity_min_rad;
    provider_config.elbow_min_abs_rad = params_.elbow_singularity_min_rad;
    provider_config.max_wrist_reach_m = params_.max_wrist_reach_m;
    provider_config.min_wrist_radius_m = params_.min_wrist_radius_m;
    provider_config.max_jacobian_condition = params_.max_jacobian_condition;
    provider_config.collision_checking_enabled = params_.collision_checking_enabled;
    provider_config.planning_scene_monitor = planning_scene_monitor_;
    provider_config.limits = params_.controller_config.limits;
    moveit_ik_provider_ =
        std::make_unique<fr10_motion_planning::MoveItIKProvider>(
            robot_model_, std::move(provider_config));
    path_ik_provider_ = moveit_ik_provider_.get();
    initializeDiagnosticFrames();

    if (params_.analytical_ik_enabled) {
      if (params_.analytical_ik_use_model_tool_transform) {
        if (!robot_model_->hasLinkModel("wrist3_link")) {
          throw std::runtime_error("MoveIt model lacks wrist3_link");
        }
        moveit::core::RobotState model_state(robot_model_);
        model_state.setToDefaultValues();
        model_state.update();
        const Eigen::Isometry3d wrist_to_tip =
            model_state.getGlobalLinkTransform("wrist3_link").inverse() *
            model_state.getGlobalLinkTransform(params_.kinematics_tip_link);
        if (!wrist_to_tip.matrix().allFinite()) {
          throw std::runtime_error("Nonfinite URDF wrist3-to-TCP transform");
        }
        const Eigen::Quaterniond rotation(wrist_to_tip.rotation());
        const auto &position = wrist_to_tip.translation();
        params_.analytical_ik_config.wrist3_to_tip.position_m =
            {position.x(), position.y(), position.z()};
        params_.analytical_ik_config.wrist3_to_tip.orientation_quat =
            {rotation.x(), rotation.y(), rotation.z(), rotation.w()};
        RCLCPP_INFO(get_logger(),
                    "Analytical IK wrist3-to-%s fixed transform derived from URDF",
                    params_.kinematics_tip_link.c_str());
      }
      analytical_ik_provider_ =
          std::make_unique<fr10_motion::Fr10AnalyticalIKProvider>(
              params_.analytical_ik_config);
      const std::array<std::array<double, 6>, 3> validation_states{
          {{{0.9281, -1.9512, 1.9526, -1.6002, -1.2993, 0.0}},
           {{-0.75, -1.35, 1.10, -1.20, 1.05, 0.35}},
           {{0.40, -1.10, -1.25, -1.00, -1.15, -0.40}}}};
      for (const auto &positions : validation_states) {
        fr10_motion::KinematicState state;
        state.position_rad = positions;
        fr10_motion::Pose moveit_pose;
        fr10_motion::Pose analytical_pose;
        if (!moveit_ik_provider_->forward(state, moveit_pose) ||
            !analytical_ik_provider_->forward(state, analytical_pose)) {
          throw std::runtime_error(
              "Analytical/MoveIt FK startup cross-check failed");
        }
        double squared_error = 0.0;
        for (std::size_t axis = 0; axis < 3; ++axis) {
          const double delta =
              moveit_pose.position_m[axis] - analytical_pose.position_m[axis];
          squared_error += delta * delta;
        }
        const double position_error = std::sqrt(squared_error);
        const double orientation_error =
            fr10_motion::math::quaternionAngularDistance(
                moveit_pose.orientation_quat, analytical_pose.orientation_quat);
        if (position_error > params_.analytical_ik_model_position_tolerance_m ||
            orientation_error >
                params_.analytical_ik_model_orientation_tolerance_rad) {
          throw std::runtime_error(
              "Analytical IK geometry does not match the loaded MoveIt model");
        }
      }
      collision_validated_ik_provider_ =
          std::make_unique<fr10_motion_planning::CollisionValidatedIKProvider>(
              *analytical_ik_provider_, *moveit_ik_provider_);
      path_ik_provider_ = collision_validated_ik_provider_.get();
      RCLCPP_INFO(get_logger(),
                  "Closed-form FR10 analytical IK enabled and cross-checked "
                  "against MoveIt FK");
    }
  }

void MotionAdapter::seedPlanningBoundary(KinematicsTask &task) {
  task.boundary = motion_executor_->planningBoundary(planning_horizon_cycles_.load());
  if (!task.boundary.valid) return;
  task.seed.joint_position_rad = task.boundary.reference.position_rad;
  task.seed.joint_velocity_rad_s = task.boundary.reference.velocity_rad_s;
  task.seed.joint_acceleration_rad_s2 = task.boundary.reference.acceleration_rad_s2;
}

void MotionAdapter::queueKinematicsTask(const KinematicsTask &incoming) {
    KinematicsTask task = incoming;
    if (shutdown_in_progress_.load() || stop_latch_.load() != StopReason::NONE ||
        motion_executor_->status().state == fr10_motion::MotionState::STOPPING) {
      coordinator_->transition(task.command_id, fr10_motion::CommandPhase::REJECTED,
                               fr10_motion::CommandReason::NOT_READY); return;
    }
    seedPlanningBoundary(task);
    coordinator_->transition(task.command_id, fr10_motion::CommandPhase::PLANNING);
    {
      std::lock_guard<std::mutex> lock(kinematics_mutex_);
      pending_kinematics_task_ = task;
    }
    kinematics_cv_.notify_one();
  }

void MotionAdapter::clearRejectedStagingGeneration(uint64_t generation) {
    insertion_sequence_state_.rejectStaging(generation);
  }

void MotionAdapter::logTimeParameterDiagnostics(
      uint64_t generation,
      const fr10_motion::PathTimeParameterizationResult &result) const {
    const auto &diagnostics = result.diagnostics;
    RCLCPP_INFO(
        get_logger(),
        "[TIME_PARAM_DIAGNOSTIC] generation=%lu policy=%s\n"
        "Trajectory Distance     : %.3f m | Rotation: %.1f deg | Duration: "
        "%.3f s\n"
        "Max Joint Vel Ratio     : %.3f (Joint %zu)\n"
        "Max Joint Acc Ratio     : %.3f (Joint %zu)\n"
        "Max Joint Jerk Ratio    : %.3f (Joint %zu)\n"
        "Max Linear Vel Ratio    : %.3f | Acc: %.3f | Jerk: %.3f\n"
        "Max Angular Vel Ratio   : %.3f | Acc: %.3f | Jerk: %.3f\n"
        "INITIAL PATH VEL CAP    : %s\n"
        "LIMITING CONSTRAINT     : %s | scaling iterations: %zu",
        generation,
        fr10_motion::poseExecutionPolicyName(diagnostics.policy),
        diagnostics.trajectory_distance_m,
        diagnostics.trajectory_rotation_rad * fr10_motion::kRadToDeg,
        diagnostics.duration_s, diagnostics.max_joint_velocity_ratio,
        diagnostics.max_joint_velocity_index + 1,
        diagnostics.max_joint_acceleration_ratio,
        diagnostics.max_joint_acceleration_index + 1,
        diagnostics.max_joint_jerk_ratio, diagnostics.max_joint_jerk_index + 1,
        diagnostics.linear_velocity_ratio,
        diagnostics.linear_acceleration_ratio, diagnostics.linear_jerk_ratio,
        diagnostics.angular_velocity_ratio,
        diagnostics.angular_acceleration_ratio, diagnostics.angular_jerk_ratio,
        fr10_motion::timeParameterConstraintName(
            diagnostics.initial_velocity_constraint),
        fr10_motion::timeParameterConstraintName(
            diagnostics.limiting_constraint),
        diagnostics.scaling_iterations);
  }

bool MotionAdapter::publishKinematicsResult(const KinematicsResult &result) {
    retained_trajectories_.erase(std::remove_if(retained_trajectories_.begin(),
        retained_trajectories_.end(), [](const auto &trajectory) {
          return trajectory.use_count() == 1 && trajectory->trajectoryUseCount() == 1;
        }), retained_trajectories_.end());
    if (result.pose_trajectory) { retained_trajectories_.push_back(result.pose_trajectory); }
    while (!stop_requested_.load() && result.sequence == kinematics_task_sequence_.load()) {
      if (kinematics_result_buffer_.write(result)) { return true; }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
  }

void MotionAdapter::kinematicsWorkerLoop() {
    while (!stop_requested_.load()) {
      KinematicsTask task;
      std::optional<ForceValidationRequest> force_check;
      {
        std::unique_lock<std::mutex> lock(kinematics_mutex_);
        kinematics_cv_.wait(lock, [this]() {
          return stop_requested_.load() || pending_kinematics_task_.has_value() ||
                 pending_force_validation_.has_value();
        });
        if (stop_requested_.load()) return;
        if (pending_kinematics_task_) {
          task = *pending_kinematics_task_;
          pending_kinematics_task_.reset();
        } else {
          force_check = pending_force_validation_;
          pending_force_validation_.reset();
        }
      }
      if (force_check) {
        validateOnlineForceTarget(*force_check);
        continue;
      }

      KinematicsResult result;
      result.control_mode = task.control_mode;
      result.sequence = task.sequence;
      result.command_id = task.command_id;
      result.boundary = task.boundary;
      result.replan_attempt = task.replan_attempt;
      result.timing_stop_attempts = task.timing_stop_attempts;
      result.policy = task.policy;
      result.pose_target = task.pose_target;
      result.joint_target = task.joint_target;
      result.force_goal = task.force_goal;
      bool delivered = false;
      struct ForceFailureDelivery {
        std::function<void()> callback;
        ~ForceFailureDelivery() { callback(); }
      } force_failure_delivery{[this, &task, &result, &delivered]() {
        if (!delivered && !result.requires_controlled_stop_replan &&
            task.sequence == kinematics_task_sequence_.load()) {
          failed_planning_generation_.store(task.pose_target.generation, std::memory_order_release);
          coordinator_->transition(task.command_id, fr10_motion::CommandPhase::REJECTED,
                                   fr10_motion::CommandReason::PLANNING_FAILED);
        }
        if (!delivered && task.force_goal && !stop_requested_.load() &&
            task.sequence == kinematics_task_sequence_.load()) {
          result.pose_trajectory.reset();
          result.force_planning_failed = true;
          publishKinematicsResult(result);
        }
      }};
      planning_scene::PlanningSceneConstPtr planning_scene_snapshot;
      result.scene_revision = scene_revision_.load(std::memory_order_acquire);
      if (params_.collision_checking_enabled && planning_scene_monitor_) {
        bool captured = false;
        for (unsigned attempt = 0; attempt < 3 && !captured; ++attempt) {
          const uint64_t revision_before =
              scene_revision_.load(std::memory_order_acquire);
          {
            planning_scene_monitor::LockedPlanningSceneRO scene(
                planning_scene_monitor_);
            if (!scene) {
              break;
            }
            planning_scene_snapshot = planning_scene::PlanningScene::clone(
                static_cast<const planning_scene::PlanningSceneConstPtr &>(
                    scene));
          }
          const uint64_t revision_after =
              scene_revision_.load(std::memory_order_acquire);
          captured = planning_scene_snapshot &&
                     revision_before == revision_after;
          if (captured) {
            result.scene_revision = revision_after;
          }
        }
        if (!captured) {
          fr10_motion::recordWorkerStopReason(stop_reason_, stop_requested_, StopReason::IK_FAILURE);
          RCLCPP_ERROR(get_logger(),
                       "[TARGET REJECTED: SCENE_UNAVAILABLE] sequence=%lu",
                       task.sequence);
          continue;
        }
      }
      if (!resolveForceTaskFrame(task,planning_scene_snapshot)) continue;
      result.pose_target=task.pose_target;
      result.force_goal=task.force_goal;
      moveit_ik_provider_->setPlanningSceneSnapshot(planning_scene_snapshot);
      struct SceneSnapshotReset {
        fr10_motion_planning::MoveItIKProvider *provider;
        ~SceneSnapshotReset() { provider->setPlanningSceneSnapshot({}); }
      } scene_snapshot_reset{moveit_ik_provider_.get()};
      // Success RETRACT is the same validated axial corridor in reverse.
      // Permit only its intentional connector/fixture contact, never any
      // other pair, including during seed and time-parameter validation.
      if (task.force_goal && !task.force_goal->preinsert_alignment &&
          task.policy == fr10_motion::PoseExecutionPolicy::INSERTION) {
        if (!planning_scene_snapshot || force_runtime_config_.contact_object_id.empty() ||
            !planning_scene_snapshot->getWorld()->hasObject(force_runtime_config_.contact_object_id)) continue;
        auto contact_scene = planning_scene::PlanningScene::clone(planning_scene_snapshot);
        for (const auto &link : force_runtime_config_.contact_links)
          contact_scene->getAllowedCollisionMatrixNonConst().setEntry(
              link, force_runtime_config_.contact_object_id, true);
        planning_scene_snapshot = contact_scene;
        moveit_ik_provider_->setPlanningSceneSnapshot(planning_scene_snapshot);
      }
      fr10_motion::KinematicState seed;
      seed.position_rad = task.seed.joint_position_rad;
      seed.velocity_rad_s = task.seed.joint_velocity_rad_s;
      seed.acceleration_rad_s2 = task.seed.joint_acceleration_rad_s2;

      if (task.control_mode == fr10_motion::ControlMode::POSE) {
        fr10_motion::Pose start_pose;
        std::string rejection_reason;
        if (!moveit_ik_provider_->forward(seed, start_pose,
                                          &rejection_reason)) {
          clearRejectedStagingGeneration(task.pose_target.generation);
          fr10_motion::recordWorkerStopReason(stop_reason_, stop_requested_, StopReason::IK_FAILURE);
          RCLCPP_ERROR(get_logger(),
                       "[TARGET REJECTED: IK_FAILURE] FK seed failed: %s",
                       rejection_reason.c_str());
          continue;
        }
        fr10_motion::CartesianTarget effective_target = task.pose_target;
        if (params_.lock_orientation) {
          effective_target.orientation_quat = start_pose.orientation_quat;
        }
        result.pose_target = effective_target;
        const fr10_motion::Pose goal_pose = effective_target.toPose();
        auto route_config = params_.cartesian_route_config;
        route_config.mode =
            task.policy == fr10_motion::PoseExecutionPolicy::FREE_SPACE
                ? fr10_motion::CartesianRouteMode::FREE_SPACE_JOINT
                : fr10_motion::CartesianRouteMode::CARTESIAN_LINEAR;
        if (task.policy == fr10_motion::PoseExecutionPolicy::INSERTION) {
          // Precision mating/unmating never bends around a singular segment.
          route_config.singularity_avoidance_enabled = false;
        }
        const auto canceled = [this, sequence = task.sequence]() {
          return stop_requested_.load() || sequence != kinematics_task_sequence_.load();
        };
        // An approach branch is eligible only if the entire subsequent force
        // segment is valid. Staging itself keeps the original collision policy.
        auto endpoint_guard = [&](const fr10_motion::KinematicState &endpoint,
                                  const fr10_motion::CartesianRoutePlanner::CancelCheck &budget_cancel) {
          if (!task.force_goal) return true;
          if (!planning_scene_snapshot ||
              !planning_scene_snapshot->getWorld()->hasObject(force_runtime_config_.contact_object_id))
            return false;
          auto contact_scene = planning_scene::PlanningScene::clone(planning_scene_snapshot);
          for (const auto &link : force_runtime_config_.contact_links)
            contact_scene->getAllowedCollisionMatrixNonConst().setEntry(
                link, force_runtime_config_.contact_object_id, true);
          moveit_ik_provider_->setPlanningSceneSnapshot(contact_scene);
          const bool valid = validateForceCorridor(task, endpoint, budget_cancel);
          moveit_ik_provider_->setPlanningSceneSnapshot(planning_scene_snapshot);
          return valid;
        };
        auto route = task.policy == fr10_motion::PoseExecutionPolicy::FREE_SPACE
            ? fr10_motion_planning::JointRoutePlanner::plan(
                  goal_pose, seed, *path_ik_provider_, route_config, endpoint_guard, canceled)
            : fr10_motion::CartesianRoutePlanner::plan(
                  start_pose, goal_pose, seed, *path_ik_provider_, route_config, canceled);
        if (!route.success) {
          if (task.sequence != kinematics_task_sequence_.load()) {
            continue;
          }
          const auto seed_check = moveit_ik_provider_->validate(seed, seed);
          std::array<fr10_motion::IKResult,
                     fr10_motion::IKProvider::kMaxCandidates> candidates{};
          std::string endpoint_error;
          const auto endpoint_count = path_ik_provider_->solveCandidates(
              goal_pose, seed, candidates, &endpoint_error);
          RCLCPP_ERROR(get_logger(),
                       "[ROUTE DIAGNOSTIC] seed_valid=%s seed_reason=%s "
                       "endpoint_count=%zu endpoint_reason=%s",
                       seed_check.success ? "true" : "false",
                       seed_check.message.c_str(), endpoint_count,
                       endpoint_error.c_str());
          if (planning_scene_snapshot && !seed_check.success) {
            moveit::core::RobotState contact_state(
                planning_scene_snapshot->getCurrentState());
            contact_state.setJointGroupPositions(
                joint_model_group_, seed.position_rad.data());
            contact_state.update();
            collision_detection::CollisionRequest request;
            request.group_name = params_.kinematics_group;
            request.contacts = true;
            request.max_contacts = 12;
            request.max_contacts_per_pair = 1;
            collision_detection::CollisionResult collision;
            planning_scene_snapshot->checkCollision(request, collision,
                                                     contact_state);
            for (const auto &entry : collision.contacts) {
              if (!entry.second.empty()) {
                RCLCPP_ERROR(get_logger(),
                             "[ROUTE COLLISION] first=%s second=%s depth=%.6f m",
                             entry.first.first.c_str(),
                             entry.first.second.c_str(),
                             entry.second.front().depth);
              }
            }
          }
          fr10_motion::recordWorkerStopReason(stop_reason_, stop_requested_, StopReason::IK_FAILURE);
          clearRejectedStagingGeneration(task.pose_target.generation);
          RCLCPP_ERROR(get_logger(),
                       "[TARGET REJECTED: IK_FAILURE] generation=%lu "
                       "direct_waypoint=%zu detour_attempts=%zu reason=%s",
                       task.pose_target.generation, route.direct_failure_index,
                       route.detour_attempts, route.message.c_str());
          continue;
        }
        if (route.used_detour) {
          RCLCPP_WARN(
              get_logger(),
              "[KINEMATIC DETOUR] generation=%lu kind=%s direct_waypoint=%zu "
              "direct_reason=%s attempts=%zu clearance=%.3f m direction=[%.3f, "
              "%.3f, %.3f]; PlanningScene validation is applied",
              task.pose_target.generation,
              route.used_alternate_orientation
                  ? "ALTERNATE_ORIENTATION"
                  : (route.used_joint_space_detour ? "JOINT_SPACE_DETOUR"
                                                   : "POSITIONAL_DETOUR"),
              route.direct_failure_index, route.direct_failure_message.c_str(),
              route.detour_attempts, route.detour_clearance_m,
              route.detour_direction[0], route.detour_direction[1],
              route.detour_direction[2]);
        }
        auto &cartesian_path = route.cartesian_path;
        auto &continuous = route.continuous_ik;
        if (task.force_goal) {
          if (!planning_scene_snapshot || force_runtime_config_.contact_object_id.empty() ||
              !planning_scene_snapshot->getWorld()->hasObject(force_runtime_config_.contact_object_id)) {
            RCLCPP_ERROR(get_logger(), "[FORCE CORRIDOR REJECTED] contact object absent from pinned scene");
            continue;
          }
          auto contact_scene = planning_scene::PlanningScene::clone(planning_scene_snapshot);
          // Scoped to contact corridor only. Staging and all other pairs
          // retain their original collision policy.
          for (const auto &link : force_runtime_config_.contact_links)
            contact_scene->getAllowedCollisionMatrixNonConst().setEntry(
                link, force_runtime_config_.contact_object_id, true);
          moveit_ik_provider_->setPlanningSceneSnapshot(contact_scene);
          result.force_corridor_validated = validateForceCorridor(
              task, continuous.joint_path.back());
          moveit_ik_provider_->setPlanningSceneSnapshot(planning_scene_snapshot);
          if (!result.force_corridor_validated) continue;
          force_validation_scene_ = contact_scene;
        }
        result.used_kinematic_detour = route.used_detour;
        result.used_alternate_orientation = route.used_alternate_orientation;
        result.used_joint_space_detour = route.used_joint_space_detour;
        result.detour_clearance_m = route.detour_clearance_m;
        result.manipulability = continuous.min_manipulability;
        result.condition_number = continuous.max_condition_number;
        result.ik_waypoint_count = continuous.joint_path.size();
        auto timing_config =
            params_.path_time_policies[policyIndex(task.policy)];
        if (task.force_goal && !task.force_goal->preinsert_alignment &&
            task.policy == fr10_motion::PoseExecutionPolicy::INSERTION) {
          timing_config.initial_linear_velocity_m_s = force_runtime_config_.retract_initial_velocity_m_s;
          timing_config.initial_linear_distance_m = force_runtime_config_.retract_initial_distance_m;
        }
        fr10_motion::PathTimeParameterizer::PoseSampler smooth_fk;
        if(route.free_space_joint_route) smooth_fk=[this](const auto &q,auto &pose) {
          return analytical_ik_provider_ && analytical_ik_provider_->forward(q,pose);
        };
        auto parameterized = fr10_motion::PathTimeParameterizer::parameterize(
            cartesian_path, continuous.joint_path, seed, timing_config,
            task.pose_target.generation, [this, sequence = task.sequence]() {
              return stop_requested_.load() ||
                     sequence != kinematics_task_sequence_.load();
            },smooth_fk);
        if (parameterized.success || parameterized.duration_s > 0.0) {
          logTimeParameterDiagnostics(task.pose_target.generation,
                                      parameterized);
        }
        if (!parameterized.success) {
          if (task.sequence != kinematics_task_sequence_.load()) {
            continue;
          }
          if (parameterized.status == fr10_motion::TimeParameterizationStatus::
                                          REQUIRES_CONTROLLED_STOP &&
              fr10_motion::timingStopRetryAllowed(
                  task.timing_stop_attempts)) {
            result.requires_controlled_stop_replan = true;
            result.timing_diagnostics = parameterized.diagnostics;
            if (!publishKinematicsResult(result)) {
              RCLCPP_WARN(get_logger(),
                          "Retarget-stop mailbox busy; dropping sequence=%lu",
                          result.sequence);
            }
            delivered = true;
            RCLCPP_WARN(get_logger(),
                        "[RETARGET TIMING STOP] generation=%lu policy=%s; %s",
                        task.pose_target.generation,
                        fr10_motion::poseExecutionPolicyName(task.policy),
                        parameterized.message.c_str());
            continue;
          }
          fr10_motion::recordWorkerStopReason(stop_reason_, stop_requested_, StopReason::IK_FAILURE);
          clearRejectedStagingGeneration(task.pose_target.generation);
          RCLCPP_ERROR(get_logger(),
                       "[TARGET REJECTED: TIME_PARAMETERIZATION] "
                       "generation=%lu reason=%s",
                       task.pose_target.generation,
                       parameterized.message.c_str());
          continue;
        }
        result.timing_diagnostics = parameterized.diagnostics;

        const fr10_motion::TrajectoryTraceLimits trace_limits{
            task.policy == fr10_motion::PoseExecutionPolicy::INSERTION,
            params_.cartesian_position_tolerance_m,
            params_.cartesian_angular_tolerance_rad,
            params_.cartesian_path_deviation_tolerance_m};
        const auto trace = fr10_motion::TrajectoryTraceValidator::validate(
            parameterized.trajectory, seed, *moveit_ik_provider_,
            trace_limits, [this, sequence = task.sequence]() {
              return stop_requested_.load() ||
                     sequence != kinematics_task_sequence_.load();
            });
        result.planned_max_position_error_m = trace.max_position_error_m;
        result.planned_max_orientation_error_rad =
            trace.max_orientation_error_rad;
        result.planned_max_path_deviation_m = trace.max_path_deviation_m;
        rejection_reason = trace.message;
        if (!trace.success) {
          if (task.sequence != kinematics_task_sequence_.load()) {
            continue;
          }
          fr10_motion::recordWorkerStopReason(stop_reason_, stop_requested_, StopReason::PATH_DEVIATION);
          clearRejectedStagingGeneration(task.pose_target.generation);
          RCLCPP_ERROR(
              get_logger(),
              "[TARGET REJECTED: PATH_DEVIATION] generation=%lu reason=%s "
              "position=%.6f orientation=%.6f perpendicular=%.6f",
              task.pose_target.generation, rejection_reason.c_str(),
              result.planned_max_position_error_m,
              result.planned_max_orientation_error_rad,
              result.planned_max_path_deviation_m);
          continue;
        }
        result.trajectory_duration_s = parameterized.duration_s;
        auto raw_trajectory =
            std::make_shared<fr10_motion::CartesianJointTrajectory>(
                std::move(parameterized.trajectory));
        raw_trajectory->planning_timestamp_s = now().seconds();
        raw_trajectory->seed_joint_state = seed;
        result.pose_trajectory =
            motion_executor_->preparePoseTrajectory(std::move(raw_trajectory));
        if (!result.pose_trajectory) {
          fr10_motion::recordWorkerStopReason(stop_reason_, stop_requested_, StopReason::PATH_DEVIATION);
          clearRejectedStagingGeneration(task.pose_target.generation);
          RCLCPP_ERROR(
              get_logger(),
              "[TARGET REJECTED: TRAJECTORY_VALIDATION] generation=%lu",
              task.pose_target.generation);
          continue;
        }
        result.joint_target.position_rad =
            continuous.joint_path.back().position_rad;
        result.joint_target.timestamp_s = task.pose_target.timestamp_s;
        const auto &active_mode_config =
            params_.path_time_policies[policyIndex(task.policy)];
        result.joint_target.velocity_scale =
            std::min({active_mode_config.joint_velocity_scale,
                      active_mode_config.linear_scale,
                      active_mode_config.angular_scale});
        result.joint_target.acceleration_scale =
            active_mode_config.joint_acceleration_scale;
        result.joint_target.jerk_scale = active_mode_config.joint_jerk_scale;
      } else {
        fr10_motion::KinematicState target;
        target.position_rad = task.joint_target.position_rad;
        const fr10_motion::IKResult validation =
            path_ik_provider_->validate(target, seed);
        if (!validation.success) {
          fr10_motion::recordWorkerStopReason(stop_reason_, stop_requested_, StopReason::SINGULARITY);
          RCLCPP_ERROR(get_logger(),
                       "[TARGET REJECTED: SINGULARITY] sequence=%lu reason=%s",
                       task.sequence, validation.message.c_str());
          continue;
        }
        result.manipulability = validation.manipulability;
        result.condition_number = validation.condition_number;
        // Validate the complete Ruckig stream and execute this exact buffer;
        // validating only the endpoint misses intermediate collisions.
        auto trajectory = std::make_shared<fr10_motion::CartesianJointTrajectory>();
        trajectory->generation = task.sequence;
        trajectory->seed_joint_state = seed;
        trajectory->planning_timestamp_s = now().seconds();
        fr10_motion::JointOtg planner(params_.nominal_period_s,
            params_.controller_config.limits,
            params_.controller_config.max_joint_reference_lead_rad);
        planner.reset(seed.position_rad, seed.velocity_rad_s, seed.acceleration_rad_s2);
        planner.submitTarget(task.joint_target);
        fr10_motion::CartesianTrajectoryPoint point;
        point.joint_state = seed;
        trajectory->points.push_back(point);
        bool path_valid = true;
        auto previous = seed;
        constexpr int kMaximumJointPlanSamples = 15000;
        for (int sample = 1; sample <= kMaximumJointPlanSamples; ++sample) {
          if (task.sequence != kinematics_task_sequence_.load() || stop_requested_.load() ||
              !planner.step(task.seed, point.joint_state.position_rad,
                            point.joint_state.velocity_rad_s, point.joint_state.acceleration_rad_s2)) {
            path_valid = false;
            break;
          }
          const auto checked = path_ik_provider_->validate(point.joint_state, previous);
          if (!checked.success) {
            RCLCPP_ERROR(get_logger(), "[JOINT PATH REJECTED] sequence=%lu sample=%d reason=%s",
                        task.sequence, sample, checked.message.c_str());
            path_valid = false;
            break;
          }
          previous = point.joint_state;
          point.time_from_start_s = sample * params_.nominal_period_s;
          trajectory->points.push_back(point);
          if (planner.isFinished()) { break; }
        }
        if (!path_valid || !planner.isFinished()) { continue; }
        for (std::size_t i = 0; i < trajectory->points.size(); ++i) {
          trajectory->points[i].path_parameter = double(i) / double(trajectory->points.size() - 1);
        }
        result.trajectory_duration_s = trajectory->points.back().time_from_start_s;
        result.pose_trajectory = motion_executor_->prepareJointTrajectory(trajectory, result.joint_target);
        if (!result.pose_trajectory) { continue; }
      }

      if (task.sequence != kinematics_task_sequence_.load()) {
        continue;
      }
      if (!publishKinematicsResult(result)) {
        RCLCPP_WARN(get_logger(),
                    "Kinematics result mailbox busy; dropping sequence=%lu",
                    result.sequence);
        continue;
      }
      delivered = true;

      if (task.control_mode == fr10_motion::ControlMode::POSE) {
        RCLCPP_INFO(
            get_logger(),
            "[NEW_GOAL] Pose gen=%lu policy=%s validated with %zu IK "
            "waypoints and %zu "
            "timed samples (%.3f s); condition=%.2f path_error=%.6f m route=%s "
            "clearance=%.3f m",
            task.pose_target.generation,
            fr10_motion::poseExecutionPolicyName(task.policy),
            result.ik_waypoint_count,
            result.pose_trajectory
                ? result.pose_trajectory->trajectory().points.size()
                : 0,
            result.trajectory_duration_s, result.condition_number,
            result.planned_max_path_deviation_m,
            task.policy == fr10_motion::PoseExecutionPolicy::FREE_SPACE
                ? "FREE_SPACE_JOINT"
                : (result.used_alternate_orientation
                       ? "ALTERNATE_ORIENTATION"
                       : (result.used_joint_space_detour
                              ? "JOINT_SPACE_DETOUR"
                              : (result.used_kinematic_detour
                                     ? "KINEMATIC_DETOUR"
                                     : "DIRECT"))),
            result.detour_clearance_m);
      } else {
        RCLCPP_INFO(get_logger(),
                    "[NEW_GOAL] Joint rev=%lu validated; manipulability=%.6g "
                    "condition=%.2f",
                    result.joint_target.revision, result.manipulability,
                    result.condition_number);
      }
    }
  }

void MotionAdapter::consumeKinematicsResult() {
    KinematicsResult result;
    if (!kinematics_result_buffer_.read(result) ||
        result.sequence == consumed_kinematics_result_sequence_ ||
        result.sequence != kinematics_task_sequence_.load()) {
      return;
    }
    if (!coordinator_->owns(result.command_id)) return;
    fr10_motion::MotionCommand boundary_reference;
    latest_command_buffer_.read(boundary_reference);
    if (result.boundary.sequence != 0 && boundary_reference.sequence < result.boundary.sequence)
      return; // Worker is ready; keep old valid path until the scheduled boundary.
    consumed_kinematics_result_sequence_ = result.sequence;
    if (result.force_planning_failed) {
      // Preserve ownership until the FSM acknowledges a controlled stop.
      // This also applies to a recovery retract rejected by the worker.
      failed_planning_generation_.store(result.pose_target.generation, std::memory_order_release);
      force_corridor_validated_ = false;
      return;
    }

    const uint64_t current_scene_revision =
        scene_revision_.load(std::memory_order_acquire);
    if (params_.collision_checking_enabled &&
        result.scene_revision != current_scene_revision) {
      const auto recovery = fr10_motion::staleActivationRecovery(
          result.control_mode, result.replan_attempt);
      if (recovery == fr10_motion::StaleActivationRecovery::REJECT) {
        coordinator_->transition(result.command_id, fr10_motion::CommandPhase::REJECTED,
                                 fr10_motion::CommandReason::SCENE_CHANGED);
        RCLCPP_ERROR(get_logger(), "[SCENE REJECTED] bounded planning recovery exhausted for command=%lu",
                     result.command_id);
        return;
      }
      if (recovery == fr10_motion::StaleActivationRecovery::CONTROLLED_STOP_REPLAN) {
        KinematicsTask retry;
        retry.command_id = result.command_id;
        retry.control_mode = result.control_mode;
        retry.joint_target = result.joint_target;
        retry.pose_target = result.pose_target;
        retry.policy = result.policy;
        retry.force_goal = result.force_goal;
        retry.replan_attempt = result.replan_attempt+1;
        retry.timing_stop_attempts = result.timing_stop_attempts;
        {
          std::lock_guard<std::mutex> lock(reversal_mutex_);
          post_stop_replan_task_ = retry;
        }
        requestControlledStop(StopReason::STALE_TRAJECTORY);
        return;
      }
      fr10_motion::RobotState live_seed;
      if (!feedbackSnapshot(live_seed)) {
        RCLCPP_WARN(get_logger(),
                    "[STALE SCENE] feedback unavailable; result sequence=%lu "
                    "discarded",
                    result.sequence);
        return;
      }
      fr10_motion::MotionCommand live_reference;
      latest_command_buffer_.read(live_reference);
      if (live_reference.type ==
          fr10_motion::CommandType::JOINT_POSITION) {
        live_seed.joint_position_rad = live_reference.joint_position_rad;
        live_seed.joint_velocity_rad_s = live_reference.joint_velocity_rad_s;
        live_seed.joint_acceleration_rad_s2 =
            live_reference.joint_acceleration_rad_s2;
      }
      KinematicsTask retry;
      retry.control_mode = result.control_mode;
      retry.command_id = result.command_id;
      retry.force_goal = result.force_goal;
      retry.joint_target = result.joint_target;
      retry.pose_target = result.pose_target;
      retry.policy = result.policy;
      retry.seed = live_seed;
      retry.replan_attempt = result.replan_attempt + 1;
      retry.timing_stop_attempts = result.timing_stop_attempts;
      retry.sequence = kinematics_task_sequence_.fetch_add(1) + 1;
      RCLCPP_WARN(get_logger(),
                  "[STALE SCENE] planned=%lu current=%lu sequence=%lu; "
                  "replanning latest target",
                  result.scene_revision, current_scene_revision,
                  result.sequence);
      queueKinematicsTask(retry);
      return;
    }

    if (result.requires_controlled_stop_replan) {
      KinematicsTask retry;
      retry.control_mode = result.control_mode;
      retry.command_id = result.command_id;
      retry.joint_target = result.joint_target;
      retry.force_goal = result.force_goal;
      retry.pose_target = result.pose_target;
      retry.policy = result.policy;
      retry.replan_attempt = result.replan_attempt + 1;
      retry.timing_stop_attempts = result.timing_stop_attempts + 1;
      {
        std::lock_guard<std::mutex> lock(reversal_mutex_);
        post_stop_replan_task_ = retry;
      }
      RCLCPP_WARN(get_logger(),
                  "[RETARGET TIMING STOP] generation=%lu requesting jerk-limited "
                  "controlled stop before zero-velocity replan",
                  result.pose_target.generation);
      requestControlledStop(StopReason::RETARGET_TIMING);
      return;
    }

    fr10_motion::PoseActivationResult pose_activation =
        fr10_motion::PoseActivationResult::REJECTED;
    const bool missed_boundary = result.boundary.sequence != 0 &&
        (boundary_reference.sequence != result.boundary.sequence ||
         motion_executor_->status().active_generation != result.boundary.anchor_generation);
    const bool accepted =
        (pose_activation = missed_boundary ? fr10_motion::PoseActivationResult::STALE :
                   motion_executor_->activatePreparedPoseTrajectory(
                       result.pose_trajectory)) ==
                  fr10_motion::PoseActivationResult::ACCEPTED;
    if (!accepted) {
      if (pose_activation == fr10_motion::PoseActivationResult::STALE &&
          fr10_motion::staleActivationRecovery(result.control_mode,
                                               result.replan_attempt) ==
              fr10_motion::StaleActivationRecovery::REPLAN_LIVE) {
        fr10_motion::RobotState live_seed;
        if (!feedbackSnapshot(live_seed)) {
          RCLCPP_WARN(get_logger(),
                      "[STALE TRAJECTORY] feedback unavailable; retaining "
                      "previous valid motion state");
          return;
        }
        fr10_motion::MotionCommand live_reference;
        latest_command_buffer_.read(live_reference);
        if (live_reference.type == fr10_motion::CommandType::JOINT_POSITION) {
          live_seed.joint_position_rad = live_reference.joint_position_rad;
          live_seed.joint_velocity_rad_s = live_reference.joint_velocity_rad_s;
          live_seed.joint_acceleration_rad_s2 =
              live_reference.joint_acceleration_rad_s2;
        }
        KinematicsTask retry;
        retry.control_mode = result.control_mode;
        retry.command_id = result.command_id;
        retry.force_goal = result.force_goal;
        retry.joint_target = result.joint_target;
        retry.pose_target = result.pose_target;
        retry.policy = result.policy;
        retry.seed = live_seed;
        retry.replan_attempt = result.replan_attempt + 1;
        retry.timing_stop_attempts = result.timing_stop_attempts;
        retry.sequence = kinematics_task_sequence_.fetch_add(1) + 1;
        RCLCPP_WARN(
            get_logger(),
            "[STALE TRAJECTORY] generation=%lu plan_stamp=%.6f "
            "activation seed diverged from live state; replan=%u/3",
            result.pose_target.generation,
            result.pose_trajectory
                ? result.pose_trajectory->trajectory().planning_timestamp_s
                : 0.0,
            retry.replan_attempt);
        queueKinematicsTask(retry);
        return;
      }
      if (pose_activation == fr10_motion::PoseActivationResult::STALE &&
          fr10_motion::staleActivationRecovery(result.control_mode,
                                               result.replan_attempt) ==
              fr10_motion::StaleActivationRecovery::CONTROLLED_STOP_REPLAN) {
        KinematicsTask retry;
        retry.control_mode = result.control_mode;
        retry.command_id = result.command_id;
        retry.joint_target = result.joint_target;
        retry.force_goal = result.force_goal;
        retry.pose_target = result.pose_target;
        retry.policy = result.policy;
        retry.replan_attempt = result.replan_attempt + 1;
        retry.timing_stop_attempts = result.timing_stop_attempts;
        {
          std::lock_guard<std::mutex> lock(reversal_mutex_);
          post_stop_replan_task_ = retry;
        }
        RCLCPP_WARN(get_logger(),
                    "[STALE TRAJECTORY STOP] generation=%lu exhausted three "
                    "live reseeds; controlled stop before one final replan",
                    result.pose_target.generation);
        requestControlledStop(StopReason::STALE_TRAJECTORY);
        return;
      }
      if (pose_activation == fr10_motion::PoseActivationResult::STALE) {
        stop_reason_.store(StopReason::STALE_TRAJECTORY);
        coordinator_->transition(result.command_id, fr10_motion::CommandPhase::REJECTED,
                                 fr10_motion::CommandReason::STALE_BOUNDARY);
        RCLCPP_ERROR(get_logger(),
                    "[STALE TRAJECTORY REJECTED] generation=%lu attempts=%u; "
                    "retaining safe current motion state",
                    result.pose_target.generation, result.replan_attempt);
        return;
      }
      stop_reason_.store(StopReason::SINGULARITY);
      pending_event_.store(StopReason::SINGULARITY);
      return;
    }

    coordinator_->transition(result.command_id, fr10_motion::CommandPhase::EXECUTING);

    if (result.control_mode == fr10_motion::ControlMode::JOINT) {
      accepted_target_writer_.joint_target = result.joint_target;
      accepted_target_writer_.have_joint_target = true;
    } else {
      accepted_target_writer_.cartesian_target = result.pose_target;
      accepted_target_writer_.have_cartesian_target = true;
    }
    accepted_target_buffer_.write(accepted_target_writer_);
    active_control_mode_.store(result.control_mode);
    if (result.control_mode == fr10_motion::ControlMode::POSE) {
      active_pose_policy_.store(result.policy);
    }
    stop_reason_.store(StopReason::NONE);
    active_scene_revision_.store(result.scene_revision,
                                 std::memory_order_release);
    if (result.force_goal) {
      if(result.force_goal->preinsert_alignment &&
         !force_adapter_->reframePreinsert(*result.force_goal,result.scene_revision)) {
        force_adapter_->requestCancel();
        requestControlledStop(StopReason::CONTROLLED_STOP);
        return;
      }
      force_corridor_validated_=result.force_corridor_validated;
      if(result.force_goal->preinsert_alignment)
        display_pose_goal_buffer_.write(DisplayPoseGoalSnapshot{
            {result.force_goal->final_pose.position_m,result.force_goal->final_pose.orientation_xyzw},true});
    }
  }

void MotionAdapter::queuePostStopReplanIfReady(const fr10_motion::MotionStatus &status,
                                  const fr10_motion::RobotState &feedback,
                                  const fr10_motion::MotionCommand &command) {
    if (status.state != fr10_motion::MotionState::HOLDING) {
      return;
    }
    std::optional<KinematicsTask> retry;
    {
      std::lock_guard<std::mutex> lock(reversal_mutex_);
      if (!post_stop_replan_task_) {
        return;
      }
      retry = std::move(post_stop_replan_task_);
      post_stop_replan_task_.reset();
    }
    retry->seed = feedback;
    if (command.type == fr10_motion::CommandType::JOINT_POSITION) {
      retry->seed.joint_position_rad = command.joint_position_rad;
      retry->seed.joint_velocity_rad_s = command.joint_velocity_rad_s;
      retry->seed.joint_acceleration_rad_s2 = command.joint_acceleration_rad_s2;
    } else {
      retry->seed.joint_velocity_rad_s.fill(0.0);
      retry->seed.joint_acceleration_rad_s2.fill(0.0);
    }
    retry->sequence = kinematics_task_sequence_.fetch_add(1) + 1;
    stop_reason_.store(StopReason::NONE);
    RCLCPP_INFO(get_logger(),
                "[RETARGET REPLAN] generation=%lu policy=%s from "
                "controlled-stop "
                "boundary",
                retry->pose_target.generation,
                fr10_motion::poseExecutionPolicyName(retry->policy));
    queueKinematicsTask(*retry);
  }

} // namespace fr10_pose_controller::detail
