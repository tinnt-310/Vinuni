#include "motion_adapter_internal.hpp"
#include "fr10_motion_planning/online_motion_validator.hpp"

namespace fr10_pose_controller::detail {

// Managed worker only: one measured transform for goal and fixed fixtures.
bool MotionAdapter::resolveForceTaskFrame(KinematicsTask &task,
    planning_scene::PlanningSceneConstPtr &planning_scene_snapshot) {
  if(!task.force_goal || force_runtime_config_.task_frame.empty()) return true;
  try {
    using namespace fr10_motion::math;
    if(!planning_scene_monitor_ || !planning_scene_snapshot)
      throw std::runtime_error("No pinned collision scene for anchored task");
    const auto tf=planning_scene_monitor_->getTFClient();
    if(!tf) throw std::runtime_error("No task-frame transform buffer");
    const auto current=tf->lookupTransform(params_.base_frame,force_runtime_config_.task_frame,
                                           rclcpp::Time(0,0,get_clock()->get_clock_type()));
    const double age=(now()-rclcpp::Time(current.header.stamp,get_clock()->get_clock_type())).seconds();
    if(!std::isfinite(age) || age<0. || age>.050) throw std::runtime_error("Task transform is stale/future");
    auto coreTransform=[](const geometry_msgs::msg::Transform &t) {
      fr10_motion::Pose p{{t.translation.x,t.translation.y,t.translation.z},
                         {t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w}};
      if(!normalizeQuaternion(p.orientation_quat)) throw std::runtime_error("Invalid task transform");
      for(double v:p.position_m) if(!std::isfinite(v)) throw std::runtime_error("Invalid task translation");
      return p;
    };
    const auto base_from_task=coreTransform(current.transform);
    auto &goal=*task.force_goal;
    if(goal.preinsert_alignment) {
      if(!goal.task_final_pose) {
        if(goal.measurement_ns<=0) throw std::runtime_error("Anchored task needs a measurement timestamp");
        const auto measured=tf->lookupTransform(force_runtime_config_.task_frame,params_.base_frame,
           rclcpp::Time(goal.measurement_ns,get_clock()->get_clock_type()));
        goal.task_final_pose=composePose(coreTransform(measured.transform),
           {goal.final_pose.position_m,goal.final_pose.orientation_xyzw});
      }
      const auto final=composePose(base_from_task,*goal.task_final_pose);
      goal.final_pose={final.position_m,final.orientation_quat};
      std::array<double,3> local_axis{};local_axis[goal.goal.insertion_axis]=1.;
      const auto axis=rotateVector(final.orientation_quat,local_axis);
      task.pose_target.position_m=final.position_m;
      task.pose_target.orientation_quat=final.orientation_quat;
      for(unsigned i=0;i<3;++i) task.pose_target.position_m[i]-=
          axis[i]*(goal.goal.preinsert_distance_m+goal.goal.insertion_depth_m);
    }
    auto scene=planning_scene::PlanningScene::clone(planning_scene_snapshot);
    std::vector<moveit_msgs::msg::CollisionObject> objects;
    {
      std::lock_guard<std::mutex> lock(task_geometry_mutex_);
      for(const auto &entry:task_geometry_) objects.push_back(entry.second);
    }
    bool contact_present=false;
    for(auto &object:objects) {
      contact_present |= object.id==force_runtime_config_.contact_object_id;
      // Transform the CollisionObject origin; CAD local geometry/subframes
      // stay identical. Every fixture uses the same measured transform.
      const auto &p=object.pose;
      const auto transformed=composePose(base_from_task,{{p.position.x,p.position.y,p.position.z},
          {p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w}});
      object.header.frame_id=params_.base_frame;
      object.pose.position.x=transformed.position_m[0];object.pose.position.y=transformed.position_m[1];
      object.pose.position.z=transformed.position_m[2];
      object.pose.orientation.x=transformed.orientation_quat[0];object.pose.orientation.y=transformed.orientation_quat[1];
      object.pose.orientation.z=transformed.orientation_quat[2];object.pose.orientation.w=transformed.orientation_quat[3];
      if(!scene->processCollisionObjectMsg(object)) throw std::runtime_error("Task fixture reframe rejected");
    }
    if(!contact_present) throw std::runtime_error("No task-frame contact fixture received");
    planning_scene_snapshot=scene;
    RCLCPP_INFO(get_logger(),"[FORCE TASK FRAME] goal/fixture snapshot frame=%s stamp=%.6f objects=%zu",
       force_runtime_config_.task_frame.c_str(),rclcpp::Time(current.header.stamp).seconds(),objects.size());
  } catch(const std::exception &error) {
    RCLCPP_ERROR(get_logger(),"[FORCE TASK FRAME REJECTED] %s",error.what());return false;
  }
  return true;
}


bool MotionAdapter::validateForceCorridor(
    const KinematicsTask &task,
    const fr10_motion::KinematicState &preinsert_seed,
    const fr10_motion::CartesianRoutePlanner::CancelCheck &cancel) {
  if (!task.force_goal || !path_ik_provider_ || !analytical_ik_provider_ ||
      !params_.collision_checking_enabled || force_runtime_config_.contact_object_id.empty()) return false;
  const auto &goal = *task.force_goal;
  // Nominal/braking screening is complemented by exact candidate/stop checks
  // on the managed worker throughout the force session. It is not a proof of
  // every point in a six-dimensional correction volume.
  fr10_motion::Pose contact{goal.final_pose.position_m,
                           goal.final_pose.orientation_xyzw};
  std::array<double, 3> local_axis{};
  local_axis[goal.goal.insertion_axis] = 1.0;
  const auto axis = fr10_motion::math::rotateVector(contact.orientation_quat, local_axis);
  for (std::size_t i=0; i<3; ++i)
    contact.position_m[i] -= axis[i] * goal.goal.insertion_depth_m;
  fr10_motion_planning::ForceCorridorValidationConfig config;
  config.preinsert_distance_m = goal.goal.preinsert_distance_m;
  config.insertion_depth_m = goal.goal.insertion_depth_m;
  config.axial_correction_m = force_runtime_config_.session.admittance.max_correction[goal.goal.insertion_axis];
  config.depth_tolerance_m = force_runtime_config_.session.state_machine.depth_tolerance_m;
  config.sample_step_m = std::min(0.001, params_.cartesian_route_config.path.max_translation_step_m);
  config.max_waypoints = params_.cartesian_route_config.path.max_waypoints;
  config.continuous_ik = force_runtime_config_.session.continuous_ik;
  config.axial_speed_m_s = goal.goal.insertion_speed_m_s +
      force_runtime_config_.session.admittance.max_velocity[goal.goal.insertion_axis];
  config.limits = params_.controller_config.limits;
  config.period_s = params_.nominal_period_s;
  config.max_reference_lead_rad = params_.controller_config.max_joint_reference_lead_rad;
  const auto contract = force_runtime_config_.session.contact_reference.search_radius_m > 0. ?
      fr10_motion_planning::ForceCorridorContract::CONTACT_ADAPTIVE :
      fr10_motion_planning::ForceCorridorContract::FIXED_NOMINAL;
  const auto result = fr10_motion_planning::validateForceAdmissionCorridor(
      contact, axis, preinsert_seed, *analytical_ik_provider_, *path_ik_provider_, config, contract,
      [&] { return (cancel && cancel()) || stop_requested_.load() ||
          task.sequence != kinematics_task_sequence_.load(); });
  if (!result.valid) {
    RCLCPP_ERROR(get_logger(), "[FORCE CORRIDOR REJECTED] depth=%.6f detail=%s",
        result.failure_depth_m, result.detail.c_str());
    return false;
  }
  RCLCPP_INFO(get_logger(), "[FORCE CORRIDOR VALIDATED] generation=%lu pose_samples=%zu braking_samples=%zu "
              "collision_end_depth=%.6f contract=%s resolution=%.6f scene=%lu",
              task.pose_target.generation, result.pose_samples, result.braking_samples,
              result.collision_end_depth_m,
              contract == fr10_motion_planning::ForceCorridorContract::CONTACT_ADAPTIVE ?
                  "CONTACT_ADAPTIVE" : "FIXED_NOMINAL", config.sample_step_m, scene_revision_.load());
  return true;
}

void MotionAdapter::validateOnlineForceTarget(const ForceValidationRequest &request) {
  const auto wall_start = std::chrono::steady_clock::now();
  const double source_start = now().seconds();
  ForceValidationResult result{request.target, false};
  fr10_motion_planning::OnlineMotionValidationStatistics statistics;
  if (force_validation_scene_ && moveit_ik_provider_ && path_ik_provider_ &&
      request.target.scene_revision == scene_revision_.load()) {
    moveit_ik_provider_->setPlanningSceneSnapshot(force_validation_scene_);
    result.valid = fr10_motion_planning::validateOnlineMotion(request.target, request.seed,
        *path_ik_provider_, params_.controller_config.limits, params_.nominal_period_s,
        params_.controller_config.max_joint_reference_lead_rad, [&] {
          return stop_requested_.load() || request.target.scene_revision != scene_revision_.load() ||
              request.target.generation != force_generation_.load() ||
              now().seconds() > request.target.valid_until_s;
        },&statistics);
  }
  const double wall_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now()-wall_start).count();
  if (!result.valid)
    RCLCPP_WARN(get_logger(), "[FORCE CHECK TIMING] sequence=%lu valid=%s wall_ms=%.3f "
        "queued_source_ms=%.3f finish_source_ms=%.3f transition=%u braking=%u",
        request.target.sequence, result.valid ? "true" : "false", wall_ms,
        1000.*(source_start-request.target.timestamp_s),
        1000.*(now().seconds()-request.target.timestamp_s),
        statistics.transition_samples,statistics.braking_samples);
  else if(wall_ms>8.)
    RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),1000,
        "[FORCE CHECK TIMING] sequence=%lu valid=true wall_ms=%.3f transition=%u braking=%u",
        request.target.sequence,wall_ms,statistics.transition_samples,statistics.braking_samples);
  // The control owner consumes this immutable result, never MoveIt or network.
  while (!force_validation_result_.write(result) && !stop_requested_.load())
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

} // namespace fr10_pose_controller::detail
