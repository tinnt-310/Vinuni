#pragma once

#include <array>
#include <string>
#include <vector>

#include <rclcpp/node.hpp>

#include "fr10_motion_core/cartesian_route_planner.hpp"
#include "fr10_motion_core/cartesian_trajectory.hpp"
#include "fr10_motion_core/fr10_analytical_ik.hpp"
#include "fr10_motion_core/linear_insert_planner.hpp"
#include "fr10_motion_core/motion_controller.hpp"
#include "fr10_motion_core/pre_insert_planner.hpp"

namespace fr10_pose_controller {

// Immutable-after-startup ROS adapter configuration. Declaration, conversion,
// cross-field validation, and construction of policy-specific configs live in
// node_parameters.cpp so the motion adapter only consumes typed values.
struct NodeParameters {
  static NodeParameters declare(rclcpp::Node *node);

  fr10_motion::MotionExecutorConfig controller_config;
  std::string backend_name;
  std::string robot_ip;
  std::string joint_input_topic;
  std::string joint_input_unit{"degrees"};
  std::string pose_input_topic;
  std::string retract_topic{"/fr10_motion/retract"};
  std::string controlled_stop_topic{"/fr10_motion/controlled_stop"};
  std::string joint_state_topic;
  std::string command_topic;
  std::string base_frame;
  std::string diagnostic_flange_frame{"plate_frame"};
  std::vector<std::string> joint_names;
  double nominal_period_s{0.008};
  double feedback_timeout_s{0.050};
  bool require_driver_health{true};
  double diagnostic_log_rate_hz{1.0};
  double joint_velocity_scale{1.0};
  double joint_acceleration_scale{1.0};
  double joint_jerk_scale{1.0};
  double cartesian_linear_scale{1.0};
  double cartesian_angular_scale{1.0};
  double max_angular_deviation_rad{0.35};
  double cartesian_position_tolerance_m{0.002};
  double cartesian_angular_tolerance_rad{0.02};
  std::array<fr10_motion::CartesianFaultMonitorConfig, 2>
      cartesian_fault_monitor_configs{};
  fr10_motion::CartesianPathConfig cartesian_path_config;
  fr10_motion::ContinuousIKConfig continuous_ik_config;
  fr10_motion::CartesianRoutePlannerConfig cartesian_route_config;
  fr10_motion::PathTimeParameterizerConfig path_time_config;
  std::array<fr10_motion::PathTimeParameterizerConfig, 2> path_time_policies{};
  fr10_motion::PreInsertPlannerConfig pre_insert_config;
  fr10_motion::LinearInsertPlannerConfig linear_insert_config;
  double cartesian_path_deviation_tolerance_m{0.002};
  bool lock_orientation{false};
  std::string kinematics_group{"fairino10_v6_group"};
  std::string kinematics_tip_link{"charging_tcp"};
  std::string kinematics_safety_link{"wrist3_link"};
  bool collision_checking_enabled{true};
  std::string planning_scene_topic{"/monitored_planning_scene"};
  std::string collision_object_topic{"/collision_object"};
  std::string planning_scene_world_topic{"/planning_scene_world"};
  double ik_timeout_s{0.05};
  double max_measurement_age_s{0.40};
  double joint_change_tolerance_rad{5e-4};
  double pose_position_change_tolerance_m{5e-5};
  double pose_angular_change_tolerance_rad{2e-3};
  double wrist_singularity_min_rad{8.0 * fr10_motion::kDegToRad};
  double elbow_singularity_min_rad{15.0 * fr10_motion::kDegToRad};
  double max_wrist_reach_m{1.30};
  double min_wrist_radius_m{0.200};
  double max_jacobian_condition{30.0};
  bool analytical_ik_enabled{true};
  bool analytical_ik_use_model_tool_transform{true};
  fr10_motion::Fr10AnalyticalIKConfig analytical_ik_config;
  double analytical_ik_model_position_tolerance_m{0.00005};
  double analytical_ik_model_orientation_tolerance_rad{0.00005};
};

} // namespace fr10_pose_controller
