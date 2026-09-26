#include "fr10_pose_controller/node_parameters.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

namespace {

template <std::size_t N>
std::array<double, N> vectorToArray(const std::vector<double> &values,
                                    const std::string &parameter_name) {
  if (values.size() != N ||
      !std::all_of(values.begin(), values.end(),
                   [](double value) { return std::isfinite(value); })) {
    throw std::runtime_error(parameter_name + " must contain exactly " +
                             std::to_string(N) + " finite values");
  }
  std::array<double, N> result{};
  std::copy(values.begin(), values.end(), result.begin());
  return result;
}

std::size_t policyIndex(fr10_motion::PoseExecutionPolicy policy) {
  return static_cast<std::size_t>(policy);
}

} // namespace

namespace fr10_pose_controller {

NodeParameters NodeParameters::declare(rclcpp::Node *node) {
  if (node == nullptr) {
    throw std::invalid_argument("NodeParameters::declare requires a node");
  }
  NodeParameters params;
  params.controller_config.require_command_ack = true;
  params.feedback_timeout_s = node->declare_parameter<double>(
      "controller.feedback_timeout_s", 0.050);
  if (!std::isfinite(params.feedback_timeout_s) || params.feedback_timeout_s <= 0.0) {
    throw std::runtime_error("controller.feedback_timeout_s must be positive and finite");
  }
  params.backend_name =
      node->declare_parameter<std::string>("controller.backend", "ros_control");
  params.robot_ip = node->declare_parameter<std::string>(
      "controller.robot_ip", "192.168.58.2");
  params.joint_input_topic = node->declare_parameter<std::string>(
      "controller.input_joint_topic", "/joint_command");
  params.joint_input_unit = node->declare_parameter<std::string>(
      "controller.joint_input_unit", "degrees");
  if (params.joint_input_unit != "degrees" &&
      params.joint_input_unit != "radians") {
    throw std::runtime_error(
        "controller.joint_input_unit must be 'degrees' or 'radians'");
  }
  params.pose_input_topic = node->declare_parameter<std::string>(
      "controller.input_pose_topic", "/charging_port/raw_pose");
  params.retract_topic = node->declare_parameter<std::string>(
      "controller.retract_topic", "/fr10_motion/retract");
  params.controlled_stop_topic = node->declare_parameter<std::string>(
      "controller.controlled_stop_topic", "/fr10_motion/controlled_stop");
  params.joint_state_topic = node->declare_parameter<std::string>(
      "controller.joint_state_topic", "/joint_states");
  params.command_topic = node->declare_parameter<std::string>(
      "controller.command_topic", "/fairino10_controller/commands");
  params.base_frame = node->declare_parameter<std::string>(
      "controller.base_frame", "base_link");
  params.diagnostic_flange_frame = node->declare_parameter<std::string>(
      "controller.diagnostics.flange_frame", "flange_link");
  params.diagnostic_log_rate_hz = node->declare_parameter<double>(
      "controller.diagnostics.status_log_rate_hz", 1.0);
  if (!(params.diagnostic_log_rate_hz > 0.0) ||
      !std::isfinite(params.diagnostic_log_rate_hz)) {
    throw std::runtime_error(
        "controller.diagnostics.status_log_rate_hz must be finite and "
        "positive");
  }
  params.joint_names = node->declare_parameter<std::vector<std::string>>(
      "controller.joint_names", {"j1", "j2", "j3", "j4", "j5", "j6"});
  if (params.joint_names.size() != fr10_motion::kJointCount ||
      std::unordered_set<std::string>(params.joint_names.begin(),
                                      params.joint_names.end())
              .size() != fr10_motion::kJointCount) {
    throw std::runtime_error(
        "controller.joint_names must contain six unique names");
  }

  params.controller_config.control_rate_hz =
      node->declare_parameter<double>("controller.control_rate_hz", 125.0);
  if (!std::isfinite(params.controller_config.control_rate_hz) ||
      std::abs(params.controller_config.control_rate_hz - 125.0) > 1e-9) {
    throw std::runtime_error(
        "FAIRINO runtime requires controller.control_rate_hz=125.0");
  }
  params.nominal_period_s = 1.0 / params.controller_config.control_rate_hz;

  params.controller_config.limits.min_joint_position_rad = vectorToArray<6>(
      node->declare_parameter<std::vector<double>>(
          "controller.limits.min_joint_position_rad",
          {-3.0543, -4.6251, -2.8274, -4.6251, -3.0543, -3.0543}),
      "controller.limits.min_joint_position_rad");
  params.controller_config.limits.max_joint_position_rad = vectorToArray<6>(
      node->declare_parameter<std::vector<double>>(
          "controller.limits.max_joint_position_rad",
          {3.0543, 1.4835, 2.8274, 1.4835, 3.0543, 3.0543}),
      "controller.limits.max_joint_position_rad");
  params.controller_config.limits.max_joint_velocity_rad_s = vectorToArray<6>(
      node->declare_parameter<std::vector<double>>(
          "controller.limits.max_joint_velocity_rad_s",
          {0.30, 0.30, 0.30, 0.30, 0.30, 0.30}),
      "controller.limits.max_joint_velocity_rad_s");
  params.controller_config.limits.max_joint_acceleration_rad_s2 =
      vectorToArray<6>(node->declare_parameter<std::vector<double>>(
                           "controller.limits.max_joint_acceleration_rad_s2",
                           {0.80, 0.80, 0.80, 0.80, 0.80, 0.80}),
                       "controller.limits.max_joint_acceleration_rad_s2");
  params.controller_config.limits.max_joint_jerk_rad_s3 = vectorToArray<6>(
      node->declare_parameter<std::vector<double>>(
          "controller.limits.max_joint_jerk_rad_s3",
          {4.0, 4.0, 4.0, 4.0, 4.0, 4.0}),
      "controller.limits.max_joint_jerk_rad_s3");
  params.controller_config.limits.max_linear_velocity_m_s =
      node->declare_parameter<double>(
          "controller.limits.max_linear_velocity_m_s", 0.20);
  params.controller_config.limits.max_linear_acceleration_m_s2 =
      node->declare_parameter<double>(
          "controller.limits.max_linear_acceleration_m_s2", 0.80);
  params.controller_config.limits.max_linear_jerk_m_s3 =
      node->declare_parameter<double>(
          "controller.limits.max_linear_jerk_m_s3", 4.0);
  params.controller_config.limits.max_angular_velocity_rad_s =
      node->declare_parameter<double>(
          "controller.limits.max_angular_velocity_rad_s", 0.40);
  params.controller_config.limits.max_angular_acceleration_rad_s2 =
      node->declare_parameter<double>(
          "controller.limits.max_angular_acceleration_rad_s2", 1.20);
  params.controller_config.limits.max_angular_jerk_rad_s3 =
      node->declare_parameter<double>(
          "controller.limits.max_angular_jerk_rad_s3", 6.0);

  params.controller_config.joint_goal_tolerance_rad =
      node->declare_parameter<double>(
          "controller.joint_goal_tolerance_rad", 0.002);
  params.cartesian_position_tolerance_m = node->declare_parameter<double>(
      "controller.cartesian_position_tolerance_m", 0.002);
  params.cartesian_angular_tolerance_rad = node->declare_parameter<double>(
      "controller.cartesian_angular_tolerance_rad", fr10_motion::kDegToRad);
  auto &free_space_monitor = params.cartesian_fault_monitor_configs[policyIndex(
      fr10_motion::PoseExecutionPolicy::FREE_SPACE)];
  free_space_monitor.tracking_lag_tolerance_m = node->declare_parameter<double>(
      "controller.cartesian_tracking_lag_tolerance_m", 0.050);
  free_space_monitor.persistence_s = node->declare_parameter<double>(
      "controller.cartesian_fault_persistence_s", 0.150);
  auto &insertion_monitor = params.cartesian_fault_monitor_configs[policyIndex(
      fr10_motion::PoseExecutionPolicy::INSERTION)];
  insertion_monitor.tracking_lag_tolerance_m = node->declare_parameter<double>(
      "controller.pose_policies.insertion.cartesian_tracking_lag_tolerance_m",
      0.030);
  insertion_monitor.persistence_s = node->declare_parameter<double>(
      "controller.pose_policies.insertion.cartesian_fault_persistence_s",
      0.024);
  params.controller_config.max_joint_reference_lead_rad =
      node->declare_parameter<double>(
          "controller.max_joint_reference_lead_rad", fr10_motion::kDegToRad);
  params.controller_config.reference_lag_persistence_samples =
      node->declare_parameter<int>(
          "controller.reference_lag_persistence_samples", 19);
  params.controller_config.goal_stable_samples =
      node->declare_parameter<int>("controller.goal_stable_samples", 5);

  params.joint_velocity_scale = node->declare_parameter<double>(
      "controller.joint_velocity_scale", 1.0);
  params.joint_acceleration_scale = node->declare_parameter<double>(
      "controller.joint_acceleration_scale", 1.0);
  params.joint_jerk_scale = node->declare_parameter<double>(
      "controller.joint_jerk_scale", 1.0);
  params.cartesian_linear_scale = node->declare_parameter<double>(
      "controller.cartesian_linear_scale", 1.0);
  params.cartesian_angular_scale = node->declare_parameter<double>(
      "controller.cartesian_angular_scale", 1.0);
  params.max_angular_deviation_rad = node->declare_parameter<double>(
      "controller.max_angular_deviation_rad", 0.35);
  params.lock_orientation =
      node->declare_parameter<bool>("controller.lock_orientation", false);

  params.kinematics_group = node->declare_parameter<std::string>(
      "controller.kinematics_group", "fairino10_v6_group");
  params.kinematics_tip_link = node->declare_parameter<std::string>(
      "controller.kinematics_tip_link", "tool0");
  params.kinematics_safety_link = node->declare_parameter<std::string>(
      "controller.kinematics_safety_link", "wrist3_link");
  params.ik_timeout_s =
      node->declare_parameter<double>("controller.ik_timeout_s", 0.05);
  params.collision_checking_enabled = node->declare_parameter<bool>(
      "controller.collision_checking.enabled", true);
  params.planning_scene_topic = node->declare_parameter<std::string>(
      "controller.collision_checking.planning_scene_topic",
      "/monitored_planning_scene");
  params.collision_object_topic = node->declare_parameter<std::string>(
      "controller.collision_checking.collision_object_topic",
      "/collision_object");
  params.planning_scene_world_topic = node->declare_parameter<std::string>(
      "controller.collision_checking.planning_scene_world_topic",
      "/planning_scene_world");
  params.cartesian_path_config.max_translation_step_m =
      node->declare_parameter<double>(
          "controller.cartesian_path_resolution_m", 0.005);
  params.cartesian_path_config.max_rotation_step_rad =
      node->declare_parameter<double>(
          "controller.cartesian_path_resolution_rad", 0.02);
  const int max_cartesian_waypoints = node->declare_parameter<int>(
      "controller.cartesian_path_max_waypoints", 2000);
  if (max_cartesian_waypoints < 2) {
    throw std::runtime_error(
        "controller.cartesian_path_max_waypoints must be at least 2");
  }
  params.cartesian_path_config.max_waypoints =
      static_cast<std::size_t>(max_cartesian_waypoints);
  params.continuous_ik_config.max_joint_step_rad =
      node->declare_parameter<double>(
          "controller.continuous_ik_max_joint_step_rad", 0.35);
  params.continuous_ik_config.max_position_error_m =
      node->declare_parameter<double>(
          "controller.ik_fk_position_tolerance_m", 0.001);
  params.continuous_ik_config.max_orientation_error_rad =
      node->declare_parameter<double>(
          "controller.ik_fk_orientation_tolerance_rad", 0.01);
  params.cartesian_route_config.path = params.cartesian_path_config;
  params.cartesian_route_config.continuous_ik = params.continuous_ik_config;
  params.cartesian_route_config.singularity_avoidance_enabled =
      node->declare_parameter<bool>(
          "controller.singularity_avoidance.enabled", true);
  params.cartesian_route_config.detour_clearances_m =
      node->declare_parameter<std::vector<double>>(
          "controller.singularity_avoidance.detour_clearances_m",
          {0.05, 0.10, 0.15});
  params.cartesian_route_config.joint_space_step_rad =
      node->declare_parameter<double>(
          "controller.singularity_avoidance.joint_space_step_rad", 0.03);
  params.cartesian_route_config.cylindrical_detour_radius_m =
      node->declare_parameter<double>(
          "controller.singularity_avoidance.cylindrical_detour_radius_m",
          0.350);
  params.cartesian_route_config.limits = params.controller_config.limits;
  params.path_time_config.sample_period_s = params.nominal_period_s;
  params.path_time_config.limits = params.controller_config.limits;
  params.path_time_config.joint_velocity_scale = params.joint_velocity_scale;
  params.path_time_config.joint_acceleration_scale =
      params.joint_acceleration_scale;
  params.path_time_config.joint_jerk_scale = params.joint_jerk_scale;
  params.path_time_config.linear_scale = params.cartesian_linear_scale;
  params.path_time_config.angular_scale = params.cartesian_angular_scale;
  const int trajectory_max_samples = node->declare_parameter<int>(
      "controller.cartesian_trajectory_max_samples", 250000);
  if (trajectory_max_samples < 2) {
    throw std::runtime_error(
        "controller.cartesian_trajectory_max_samples must be at least 2");
  }
  params.path_time_config.max_samples =
      static_cast<std::size_t>(trajectory_max_samples);
  params.path_time_config.minimum_ramp_time_s =
      node->declare_parameter<double>("controller.minimum_ramp_time_s", 0.20);

  const auto load_pose_policy =
      [node, &params](fr10_motion::PoseExecutionPolicy policy,
                      const std::string &key, double joint_velocity,
                      double joint_acceleration, double joint_jerk,
                      double linear, double angular) {
        auto config = params.path_time_config;
        config.policy = policy;
        const std::string prefix = "controller.pose_policies." + key + ".";
        config.joint_velocity_scale = node->declare_parameter<double>(
            prefix + "joint_velocity_scale", joint_velocity);
        config.joint_acceleration_scale = node->declare_parameter<double>(
            prefix + "joint_acceleration_scale", joint_acceleration);
        config.joint_jerk_scale = node->declare_parameter<double>(
            prefix + "joint_jerk_scale", joint_jerk);
        config.linear_scale = node->declare_parameter<double>(
            prefix + "linear_scale", linear);
        config.angular_scale = node->declare_parameter<double>(
            prefix + "angular_scale", angular);
        params.path_time_policies[policyIndex(policy)] = config;
      };
  load_pose_policy(fr10_motion::PoseExecutionPolicy::FREE_SPACE, "free_space",
                   params.joint_velocity_scale,
                   params.joint_acceleration_scale, params.joint_jerk_scale,
                   params.cartesian_linear_scale,
                   params.cartesian_angular_scale);
  load_pose_policy(fr10_motion::PoseExecutionPolicy::INSERTION, "insertion",
                   0.25, 0.50, 0.25, 0.075, 0.25);

  params.pre_insert_config.distance_m = node->declare_parameter<double>(
      "controller.insertion.pre_insert_distance_m", 0.200);
  params.pre_insert_config.local_axis = vectorToArray<3>(
      node->declare_parameter<std::vector<double>>(
          "controller.insertion.local_axis", {0.0, 0.0, 1.0}),
      "controller.insertion.local_axis");
  params.pre_insert_config.min_distance_m = node->declare_parameter<double>(
      "controller.insertion.min_pre_insert_distance_m", 0.050);
  params.pre_insert_config.max_distance_m = node->declare_parameter<double>(
      "controller.insertion.max_pre_insert_distance_m", 0.400);
  params.linear_insert_config.path = params.cartesian_path_config;
  params.linear_insert_config.max_insertion_distance_m =
      node->declare_parameter<double>(
          "controller.insertion.max_insertion_distance_m", 0.250);
  params.linear_insert_config.perpendicular_tolerance_m =
      node->declare_parameter<double>(
          "controller.insertion.perpendicular_tolerance_m", 0.002);
  params.linear_insert_config.position_tolerance_m =
      params.linear_insert_config.perpendicular_tolerance_m;
  params.linear_insert_config.max_orientation_change_rad =
      node->declare_parameter<double>(
          "controller.insertion.max_orientation_change_rad",
          fr10_motion::kDegToRad);
  params.linear_insert_config.orientation_tolerance_rad =
      params.linear_insert_config.max_orientation_change_rad;
  const auto &insertion_policy = params.path_time_policies[policyIndex(
      fr10_motion::PoseExecutionPolicy::INSERTION)];
  params.linear_insert_config.linear_speed_scale =
      insertion_policy.linear_scale;
  params.linear_insert_config.angular_speed_scale =
      insertion_policy.angular_scale;
  params.cartesian_path_deviation_tolerance_m =
      node->declare_parameter<double>(
          "controller.cartesian_path_deviation_tolerance_m", 0.002);
  params.max_measurement_age_s = node->declare_parameter<double>(
      "controller.max_measurement_age_s", 0.40);
  params.joint_change_tolerance_rad = node->declare_parameter<double>(
      "controller.joint_change_tolerance_rad", 5e-4);
  params.pose_position_change_tolerance_m = node->declare_parameter<double>(
      "controller.pose_position_change_tolerance_m", 5e-5);
  params.pose_angular_change_tolerance_rad = node->declare_parameter<double>(
      "controller.pose_angular_change_tolerance_rad", 2e-3);
  params.wrist_singularity_min_rad = node->declare_parameter<double>(
      "controller.singularity.wrist_min_abs_rad",
      8.0 * fr10_motion::kDegToRad);
  params.elbow_singularity_min_rad = node->declare_parameter<double>(
      "controller.singularity.elbow_min_abs_rad",
      15.0 * fr10_motion::kDegToRad);
  params.max_wrist_reach_m = node->declare_parameter<double>(
      "controller.singularity.max_wrist_reach_m", 1.30);
  params.min_wrist_radius_m = node->declare_parameter<double>(
      "controller.singularity.min_wrist_radius_m", 0.200);
  params.cartesian_route_config.cylindrical_barrier_radius_m =
      params.min_wrist_radius_m;
  params.max_jacobian_condition = node->declare_parameter<double>(
      "controller.singularity.max_jacobian_condition", 30.0);

  params.analytical_ik_enabled = node->declare_parameter<bool>(
      "controller.analytical_ik.enabled", true);
  params.analytical_ik_config.shoulder_height_m =
      node->declare_parameter<double>(
          "controller.analytical_ik.shoulder_height_m", 0.180);
  params.analytical_ik_config.upper_arm_length_m =
      node->declare_parameter<double>(
          "controller.analytical_ik.upper_arm_length_m", 0.700);
  params.analytical_ik_config.forearm_length_m =
      node->declare_parameter<double>(
          "controller.analytical_ik.forearm_length_m", 0.586);
  params.analytical_ik_config.wrist_1_offset_m =
      node->declare_parameter<double>(
          "controller.analytical_ik.wrist_1_offset_m", 0.159);
  params.analytical_ik_config.wrist_2_offset_m =
      node->declare_parameter<double>(
          "controller.analytical_ik.wrist_2_offset_m", 0.114);
  params.analytical_ik_config.wrist3_to_tip.position_m = vectorToArray<3>(
      node->declare_parameter<std::vector<double>>(
          "controller.analytical_ik.wrist3_to_tip_translation_m",
          {0.060, 0.0, 0.268}),
      "controller.analytical_ik.wrist3_to_tip_translation_m");
  params.analytical_ik_config.wrist3_to_tip.orientation_quat =
      vectorToArray<4>(node->declare_parameter<std::vector<double>>(
                           "controller.analytical_ik."
                           "wrist3_to_tip_quaternion_xyzw",
                           {0.0, 0.0, 0.0, 1.0}),
                       "controller.analytical_ik."
                       "wrist3_to_tip_quaternion_xyzw");
  params.analytical_ik_config.min_target_reach_m =
      node->declare_parameter<double>(
          "controller.analytical_ik.min_target_reach_m", 0.060);
  params.analytical_ik_config.max_target_reach_m =
      node->declare_parameter<double>(
          "controller.analytical_ik.max_target_reach_m", 1.750);
  params.analytical_ik_config.seed_weights = vectorToArray<6>(
      node->declare_parameter<std::vector<double>>(
          "controller.analytical_ik.seed_weights",
          {1.0, 1.0, 1.0, 0.5, 0.5, 0.25}),
      "controller.analytical_ik.seed_weights");
  params.analytical_ik_model_position_tolerance_m =
      node->declare_parameter<double>(
          "controller.analytical_ik.model_position_tolerance_m", 0.00005);
  params.analytical_ik_model_orientation_tolerance_rad =
      node->declare_parameter<double>(
          "controller.analytical_ik.model_orientation_tolerance_rad",
          0.00005);
  params.analytical_ik_config.wrist_min_abs_rad =
      params.wrist_singularity_min_rad;
  params.analytical_ik_config.elbow_min_abs_rad =
      params.elbow_singularity_min_rad;
  params.analytical_ik_config.max_wrist_reach_m = params.max_wrist_reach_m;
  params.analytical_ik_config.min_wrist_radius_m = params.min_wrist_radius_m;
  params.analytical_ik_config.max_jacobian_condition =
      params.max_jacobian_condition;
  params.analytical_ik_config.limits = params.controller_config.limits;

  const auto positive = [](double value, const char *name) {
    if (!(value > 0.0) || !std::isfinite(value)) {
      throw std::runtime_error(std::string(name) +
                               " must be finite and positive");
    }
  };
  const auto unit_scale = [](double value, const std::string &name) {
    if (!(value > 0.0) || value > 1.0 || !std::isfinite(value)) {
      throw std::runtime_error(name + " must be finite in (0, 1]");
    }
  };
  positive(params.ik_timeout_s, "controller.ik_timeout_s");
  positive(params.cartesian_position_tolerance_m,
           "controller.cartesian_position_tolerance_m");
  positive(params.cartesian_angular_tolerance_rad,
           "controller.cartesian_angular_tolerance_rad");
  for (const auto &monitor : params.cartesian_fault_monitor_configs) {
    positive(monitor.tracking_lag_tolerance_m,
             "controller Cartesian tracking lag tolerance");
    positive(monitor.persistence_s,
             "controller Cartesian fault persistence");
  }
  if (!std::isfinite(params.path_time_config.minimum_ramp_time_s) ||
      params.path_time_config.minimum_ramp_time_s < 0.20) {
    throw std::runtime_error(
        "controller.minimum_ramp_time_s must be finite and >= 0.20");
  }
  positive(params.cartesian_path_config.max_translation_step_m,
           "controller.cartesian_path_resolution_m");
  positive(params.cartesian_path_config.max_rotation_step_rad,
           "controller.cartesian_path_resolution_rad");
  positive(params.continuous_ik_config.max_joint_step_rad,
           "controller.continuous_ik_max_joint_step_rad");
  positive(params.continuous_ik_config.max_position_error_m,
           "controller.ik_fk_position_tolerance_m");
  positive(params.continuous_ik_config.max_orientation_error_rad,
           "controller.ik_fk_orientation_tolerance_rad");
  positive(params.cartesian_path_deviation_tolerance_m,
           "controller.cartesian_path_deviation_tolerance_m");
  positive(params.max_measurement_age_s,
           "controller.max_measurement_age_s");
  positive(params.joint_change_tolerance_rad,
           "controller.joint_change_tolerance_rad");
  positive(params.pose_position_change_tolerance_m,
           "controller.pose_position_change_tolerance_m");
  positive(params.pose_angular_change_tolerance_rad,
           "controller.pose_angular_change_tolerance_rad");
  positive(params.wrist_singularity_min_rad,
           "controller.singularity.wrist_min_abs_rad");
  positive(params.elbow_singularity_min_rad,
           "controller.singularity.elbow_min_abs_rad");
  positive(params.max_wrist_reach_m,
           "controller.singularity.max_wrist_reach_m");
  positive(params.min_wrist_radius_m,
           "controller.singularity.min_wrist_radius_m");
  positive(params.max_jacobian_condition,
           "controller.singularity.max_jacobian_condition");
  positive(params.analytical_ik_config.upper_arm_length_m,
           "controller.analytical_ik.upper_arm_length_m");
  positive(params.analytical_ik_config.forearm_length_m,
           "controller.analytical_ik.forearm_length_m");
  positive(params.analytical_ik_config.wrist_1_offset_m,
           "controller.analytical_ik.wrist_1_offset_m");
  positive(params.analytical_ik_config.wrist_2_offset_m,
           "controller.analytical_ik.wrist_2_offset_m");
  positive(params.analytical_ik_config.max_target_reach_m,
           "controller.analytical_ik.max_target_reach_m");
  positive(params.analytical_ik_model_position_tolerance_m,
           "controller.analytical_ik.model_position_tolerance_m");
  positive(params.analytical_ik_model_orientation_tolerance_rad,
           "controller.analytical_ik.model_orientation_tolerance_rad");
  positive(params.controller_config.limits.max_linear_velocity_m_s,
           "controller.limits.max_linear_velocity_m_s");
  positive(params.controller_config.limits.max_linear_acceleration_m_s2,
           "controller.limits.max_linear_acceleration_m_s2");
  positive(params.controller_config.limits.max_linear_jerk_m_s3,
           "controller.limits.max_linear_jerk_m_s3");
  positive(params.controller_config.limits.max_angular_velocity_rad_s,
           "controller.limits.max_angular_velocity_rad_s");
  positive(params.controller_config.limits.max_angular_acceleration_rad_s2,
           "controller.limits.max_angular_acceleration_rad_s2");
  positive(params.controller_config.limits.max_angular_jerk_rad_s3,
           "controller.limits.max_angular_jerk_rad_s3");
  positive(params.cartesian_linear_scale,
           "controller.cartesian_linear_scale");
  positive(params.cartesian_angular_scale,
           "controller.cartesian_angular_scale");
  unit_scale(params.joint_velocity_scale,
             "controller.joint_velocity_scale");
  unit_scale(params.joint_acceleration_scale,
             "controller.joint_acceleration_scale");
  unit_scale(params.joint_jerk_scale, "controller.joint_jerk_scale");
  unit_scale(params.cartesian_linear_scale,
             "controller.cartesian_linear_scale");
  unit_scale(params.cartesian_angular_scale,
             "controller.cartesian_angular_scale");
  const std::array<std::string, 2> policy_keys{"free_space", "insertion"};
  for (std::size_t index = 0; index < params.path_time_policies.size();
       ++index) {
    const auto &policy = params.path_time_policies[index];
    const std::string prefix =
        "controller.pose_policies." + policy_keys[index] + ".";
    unit_scale(policy.joint_velocity_scale,
               prefix + "joint_velocity_scale");
    unit_scale(policy.joint_acceleration_scale,
               prefix + "joint_acceleration_scale");
    unit_scale(policy.joint_jerk_scale, prefix + "joint_jerk_scale");
    unit_scale(policy.linear_scale, prefix + "linear_scale");
    unit_scale(policy.angular_scale, prefix + "angular_scale");
  }

  positive(params.pre_insert_config.distance_m,
           "controller.insertion.pre_insert_distance_m");
  positive(params.pre_insert_config.min_distance_m,
           "controller.insertion.min_pre_insert_distance_m");
  positive(params.pre_insert_config.max_distance_m,
           "controller.insertion.max_pre_insert_distance_m");
  if (params.pre_insert_config.min_distance_m >
          params.pre_insert_config.distance_m ||
      params.pre_insert_config.distance_m >
          params.pre_insert_config.max_distance_m) {
    throw std::runtime_error(
        "controller.insertion.pre_insert_distance_m (" +
        std::to_string(params.pre_insert_config.distance_m * 1000.0) +
        " mm) must be within the configured [" +
        std::to_string(params.pre_insert_config.min_distance_m * 1000.0) +
        ", " +
        std::to_string(params.pre_insert_config.max_distance_m * 1000.0) +
        "] mm staging range");
  }
  positive(params.linear_insert_config.max_insertion_distance_m,
           "controller.insertion.max_insertion_distance_m");
  positive(params.linear_insert_config.perpendicular_tolerance_m,
           "controller.insertion.perpendicular_tolerance_m");
  positive(params.linear_insert_config.max_orientation_change_rad,
           "controller.insertion.max_orientation_change_rad");
  positive(params.max_angular_deviation_rad,
           "controller.max_angular_deviation_rad");
  if (params.cartesian_route_config.singularity_avoidance_enabled &&
      params.cartesian_route_config.detour_clearances_m.empty()) {
    throw std::runtime_error(
        "controller.singularity_avoidance.detour_clearances_m must not be "
        "empty when avoidance is enabled");
  }
  for (const double clearance :
       params.cartesian_route_config.detour_clearances_m) {
    positive(clearance,
             "controller.singularity_avoidance.detour_clearances_m");
  }
  positive(params.cartesian_route_config.joint_space_step_rad,
           "controller.singularity_avoidance.joint_space_step_rad");
  positive(params.cartesian_route_config.cylindrical_detour_radius_m,
           "controller.singularity_avoidance.cylindrical_detour_radius_m");
  if (params.cartesian_route_config.cylindrical_detour_radius_m <
      params.cartesian_route_config.cylindrical_barrier_radius_m) {
    throw std::runtime_error(
        "controller.singularity_avoidance.cylindrical_detour_radius_m must "
        "be at least controller.singularity.min_wrist_radius_m");
  }

  for (auto &monitor : params.cartesian_fault_monitor_configs) {
    monitor.endpoint_position_tolerance_m =
        params.cartesian_position_tolerance_m;
    monitor.perpendicular_tolerance_m =
        params.cartesian_path_deviation_tolerance_m;
    monitor.orientation_tolerance_rad =
        params.cartesian_angular_tolerance_rad;
  }
  return params;
}

} // namespace fr10_pose_controller
