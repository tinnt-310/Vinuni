#pragma once

#include <string>
#include <vector>

#include <rclcpp/node.hpp>

#include "fr10_insertion_manager/force_session.hpp"
#include "fr10_insertion_manager/insertion_ros_adapter.hpp"

namespace fr10_pose_controller {

struct ForceRuntimeConfig {
  bool enabled{false};
  bool automatic_pose{false};
  std::string task_frame; // Empty retains the fixed arm-base contract.
  double command_lease_s{0.048};
  bool native_simulator_verified{false};
  std::string native_simulator_image_digest;
  std::string native_simulator_endpoint;
  double approach_distance_m{0.03};
  double insertion_depth_m{0.015};
  double insertion_speed_m_s{0.002};
  double seating_force_n{5.0};
  double timeout_s{150.0};
  double retract_initial_velocity_m_s{0.0};
  double retract_initial_distance_m{0.0};
  std::array<bool, 6> selected_axes{{true, true, true, true, true, true}};
  std::array<double, 3> max_force_n{{50.0, 50.0, 50.0}};
  std::array<double, 3> max_torque_nm{{10.0, 10.0, 10.0}};
  std::string profile_id{"simulation_v1"};
  std::string contact_object_id;
  std::vector<std::string> contact_links;
  fr10_insertion::WrenchAdapterConfig wrench_adapter;
  fr10_insertion::ForceSessionConfig session;
};

ForceRuntimeConfig declareForceRuntimeConfig(rclcpp::Node *node,
                                             const std::string &default_frame);

} // namespace fr10_pose_controller
