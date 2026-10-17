#include "fr10_pose_controller/force_runtime_config.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace fr10_pose_controller {
namespace {

template <std::size_t N>
std::array<double, N> arrayParameter(rclcpp::Node *node,
                                    const std::string &name,
                                    const std::array<double, N> &defaults) {
  const std::vector<double> default_vector(defaults.begin(), defaults.end());
  const auto value = node->declare_parameter<std::vector<double>>(
      name, default_vector);
  if (value.size() != N) {
    throw std::invalid_argument(name + " must contain " +
                                std::to_string(N) + " values");
  }
  std::array<double, N> result{};
  std::copy(value.begin(), value.end(), result.begin());
  return result;
}

} // namespace

ForceRuntimeConfig declareForceRuntimeConfig(
    rclcpp::Node *node, const std::string &default_frame) {
  ForceRuntimeConfig config;
  config.native_simulator_verified = node->declare_parameter<bool>("force.native_simulator.verified", false);
  config.native_simulator_image_digest = node->declare_parameter<std::string>("force.native_simulator.image_digest", "");
  config.native_simulator_endpoint = node->declare_parameter<std::string>("force.native_simulator.endpoint", "");
  config.enabled = node->declare_parameter<bool>("force.enabled", false);
  config.automatic_pose = node->declare_parameter<bool>("force.automatic_pose", false);
  config.task_frame = node->declare_parameter<std::string>("force.task_frame", "");
  config.approach_distance_m = node->declare_parameter<double>("force.goal.approach_distance_m", .03);
  config.insertion_depth_m = node->declare_parameter<double>("force.goal.insertion_depth_m", .015);
  config.insertion_speed_m_s = node->declare_parameter<double>("force.goal.insertion_speed_m_s", .002);
  config.seating_force_n = node->declare_parameter<double>("force.goal.seating_force_n", 5.0);
  config.timeout_s = node->declare_parameter<double>("force.goal.timeout_s", 150.0);
  config.retract_initial_velocity_m_s = node->declare_parameter<double>("force.retract_initial_velocity_m_s", 0.0);
  config.retract_initial_distance_m = node->declare_parameter<double>("force.retract_initial_distance_m", 0.0);
  if (!std::isfinite(config.retract_initial_velocity_m_s) ||
      !std::isfinite(config.retract_initial_distance_m) ||
      config.retract_initial_velocity_m_s < 0. || config.retract_initial_distance_m < 0. ||
      ((config.retract_initial_velocity_m_s > 0.) != (config.retract_initial_distance_m > 0.)))
    throw std::invalid_argument("loaded retract speed/distance must both be positive or disabled");
  config.max_force_n = arrayParameter<3>(node, "force.goal.max_force_n", {50., 50., 50.});
  config.max_torque_nm = arrayParameter<3>(node, "force.goal.max_torque_nm", {10., 10., 10.});
  const auto selected = node->declare_parameter<std::vector<bool>>(
      "force.goal.selected_axes", {true, true, true, true, true, true});
  if (selected.size() != 6) throw std::invalid_argument("force.goal.selected_axes needs six flags");
  std::copy(selected.begin(), selected.end(), config.selected_axes.begin());
  if (!(config.approach_distance_m > 0.0) || !(config.insertion_depth_m > 0.0) ||
      !(config.insertion_speed_m_s > 0.0) || !(config.seating_force_n > 0.0) ||
      !std::isfinite(config.timeout_s) || config.timeout_s <= 0.0)
    throw std::invalid_argument("automatic insertion goal configuration is invalid");
  config.profile_id = node->declare_parameter<std::string>(
      "force.profile_id", "simulation_v1");
  const auto process = node->declare_parameter<std::string>("force.process_mode", "axial_contact");
  if (process != "axial_contact" && process != "matched_mating")
    throw std::invalid_argument("unknown force.process_mode");
  config.session.state_machine.process_mode = process == "matched_mating"
      ? fr10_insertion::InsertionProcessMode::MATCHED_MATING
      : fr10_insertion::InsertionProcessMode::AXIAL_CONTACT;
  config.contact_object_id = node->declare_parameter<std::string>(
      "force.contact_object_id", "");
  config.contact_links = node->declare_parameter<std::vector<std::string>>(
      "force.contact_links", {"charging_connector_link"});
  if (config.contact_links.empty()) throw std::invalid_argument("force.contact_links is empty");
  config.wrench_adapter.lateral_tolerance_m = node->declare_parameter<double>(
      "force.geometry.capture_lateral_tolerance_m", .002);
  config.wrench_adapter.orientation_tolerance_rad = node->declare_parameter<double>(
      "force.geometry.capture_orientation_tolerance_rad", .03490658503988659);
  config.wrench_adapter.source_frame = node->declare_parameter<std::string>(
      "force.wrench_source_frame", default_frame);
  config.wrench_adapter.wrench_topic = node->declare_parameter<std::string>(
      "force.wrench_topic", "/ft_sensor/wrench");
  config.wrench_adapter.source_status_topic = node->declare_parameter<std::string>(
      "force.source_status_topic", "/ft_sensor/source_status");
  config.wrench_adapter.require_source_status = node->declare_parameter<bool>(
      "force.require_source_status", false);
  config.wrench_adapter.mating_clearance_m = node->declare_parameter<double>(
      "force.geometry.mating_clearance_m", 0.0);
  config.wrench_adapter.mating_feature_lever_m = node->declare_parameter<double>(
      "force.geometry.mating_feature_lever_m", 0.0);
  config.wrench_adapter.target_frame = node->declare_parameter<std::string>(
      "force.wrench_target_frame", default_frame);
  config.wrench_adapter.target_from_source.rotation_target_from_source =
      arrayParameter<9>(node, "force.wrench_transform.rotation_target_from_source",
                        {1.0, 0.0, 0.0,
                         0.0, 1.0, 0.0,
                         0.0, 0.0, 1.0});
  config.wrench_adapter.target_from_source.target_to_source_m =
      arrayParameter<3>(node, "force.wrench_transform.target_to_source_m", {});
  config.wrench_adapter.compensation.source_sign =
      node->declare_parameter<double>("force.wrench_source_sign", 1.0);
  const auto gravity_owner = node->declare_parameter<std::string>("force.gravity_compensation.owner", "application");
  if (gravity_owner != "application" && gravity_owner != "source")
    throw std::invalid_argument("gravity owner must be application or source");
  config.wrench_adapter.compensation.gravity_owner = gravity_owner == "application"
      ? fr10_force::GravityOwner::APPLICATION : fr10_force::GravityOwner::SOURCE;
  config.wrench_adapter.tare_max_joint_velocity_rad_s = node->declare_parameter<double>(
      "force.tare.max_joint_velocity_rad_s", 0.005);
  config.wrench_adapter.tare_pose_tolerance_m = node->declare_parameter<double>(
      "force.tare.pose_tolerance_m", 0.0005);
  config.wrench_adapter.tare_orientation_tolerance_rad = node->declare_parameter<double>(
      "force.tare.orientation_tolerance_rad", 0.002);
  config.wrench_adapter.tare.maximum_force_residual_n = node->declare_parameter<double>(
      "force.tare.max_force_residual_n", 5.0);
  config.wrench_adapter.tare.maximum_torque_residual_nm = node->declare_parameter<double>(
      "force.tare.max_torque_residual_nm", 0.5);
  config.wrench_adapter.tare.maximum_force_stddev_n = node->declare_parameter<double>(
      "force.tare.max_force_stddev_n", 0.25);
  config.wrench_adapter.tare.maximum_torque_stddev_nm = node->declare_parameter<double>(
      "force.tare.max_torque_stddev_nm", 0.03);
  config.wrench_adapter.compensation.gravity_enabled =
      node->declare_parameter<bool>("force.gravity_compensation.enabled", false);
  config.wrench_adapter.compensation.gravity_calibrated =
      node->declare_parameter<bool>("force.gravity_compensation.calibrated", false);
  config.wrench_adapter.compensation.tool_mass_kg =
      node->declare_parameter<double>("force.gravity_compensation.tool_mass_kg", 0.0);
  config.wrench_adapter.compensation.tool_cog_m = arrayParameter<3>(
      node, "force.gravity_compensation.tool_cog_m", {});
  config.wrench_adapter.compensation.gravity_m_s2 = arrayParameter<3>(
      node, "force.gravity_compensation.gravity_m_s2",
      {0.0, 0.0, -9.80665});
  config.wrench_adapter.filter.cutoff_hz =
      node->declare_parameter<double>("force.filter_cutoff_hz", 20.0);
  config.session.target_validity_s = node->declare_parameter<double>(
      "force.target_validity_s", 0.032);
  config.command_lease_s = node->declare_parameter<double>(
      "force.command_lease_s", config.session.target_validity_s + .016);
  if (!std::isfinite(config.command_lease_s) || config.command_lease_s <= 0. ||
      config.command_lease_s > 2. * config.session.target_validity_s)
    throw std::invalid_argument("force command lease must be positive and at most twice the proposal TTL");
  config.session.max_nominal_lead_m = node->declare_parameter<double>(
      "force.max_nominal_lead_m",0.0001);
  auto &reference = config.session.contact_reference;
  reference.search_radius_m = node->declare_parameter<double>("force.search.radius_m", 0.0);
  reference.spiral_enabled = node->declare_parameter<bool>("force.search.spiral_enabled", true);
  reference.spiral_pitch_m = node->declare_parameter<double>("force.search.pitch_m", 0.00025);
  reference.search_speed_m_s = node->declare_parameter<double>("force.search.speed_m_s", 0.0005);
  reference.stall_time_s = node->declare_parameter<double>("force.search.stall_time_s", 0.5);
  reference.search_budget_s = node->declare_parameter<double>("force.search.budget_s", 180.0);
  reference.progress_step_m = node->declare_parameter<double>("force.search.progress_step_m", 0.0001);
  reference.acquisition_depth_m = node->declare_parameter<double>("force.search.acquisition_depth_m", 0.004);
  reference.acquisition_hold_s = node->declare_parameter<double>("force.search.acquisition_hold_s", 0.08);
  reference.maximum_observation_gap_s = node->declare_parameter<double>("force.search.maximum_observation_gap_s", 0.05);
  reference.contact_force_n = node->declare_parameter<double>("force.search.contact_force_n", 0.5);
  reference.search_force_n = node->declare_parameter<double>("force.search.force_n", 1.0);
  reference.max_lateral_force_n = node->declare_parameter<double>("force.search.max_lateral_force_n", 1.0);
  reference.max_torque_nm = node->declare_parameter<double>("force.search.max_torque_nm", 0.10);
  reference.max_rotation_rad = node->declare_parameter<double>("force.search.max_rotation_rad", 0.03490658503988659);
  reference.anchor_position_tolerance_m = node->declare_parameter<double>("force.search.anchor_position_tolerance_m", 0.00005);
  reference.anchor_rotation_tolerance_rad = node->declare_parameter<double>("force.search.anchor_rotation_tolerance_rad", 0.00005);
  reference.keyed_yaw.drive_torque_nm = node->declare_parameter<double>("force.search.yaw_probe_torque_nm", 0.0);
  reference.keyed_yaw.contact_torque_nm = node->declare_parameter<double>("force.search.yaw_contact_torque_nm", 0.10);
  reference.keyed_yaw.center_gain_nm_rad = node->declare_parameter<double>("force.search.yaw_center_gain_nm_rad", 25.0);
  reference.keyed_yaw.contact_free_half_span_rad = node->declare_parameter<double>(
      "force.search.yaw_free_half_span_rad", std::numeric_limits<double>::infinity());
  reference.lateral_probe.drive_load = node->declare_parameter<double>("force.search.xy_probe_force_n",0.0);
  reference.lateral_probe.contact_load = node->declare_parameter<double>("force.search.xy_contact_force_n",1.0);
  reference.lateral_probe.center_gain = node->declare_parameter<double>("force.search.xy_center_gain_n_m",1000.0);
  reference.lateral_probe.max_coordinate = node->declare_parameter<double>(
      "force.search.xy_max_coordinate_m", reference.lateral_probe.max_coordinate);
  reference.lateral_probe.contact_free_half_span = node->declare_parameter<double>(
      "force.search.xy_free_half_span_m",std::numeric_limits<double>::infinity());
  config.session.velocity_scale = node->declare_parameter<double>(
      "force.joint_velocity_scale", 0.10);
  config.session.acceleration_scale = node->declare_parameter<double>(
      "force.joint_acceleration_scale", 0.10);
  config.session.jerk_scale = node->declare_parameter<double>(
      "force.joint_jerk_scale", 0.10);
  config.session.admittance.mass = arrayParameter<6>(
      node, "force.admittance.mass",
      {1.0, 1.0, 1.0, 0.1, 0.1, 0.1});
  config.session.admittance.damping = arrayParameter<6>(
      node, "force.admittance.damping",
      {50.0, 50.0, 50.0, 5.0, 5.0, 5.0});
  config.session.admittance.stiffness = arrayParameter<6>(
      node, "force.admittance.stiffness", {});
  config.session.admittance.drive_deadband = arrayParameter<6>(
      node, "force.admittance.drive_deadband", {});
  config.session.admittance.max_correction = arrayParameter<6>(
      node, "force.admittance.max_correction",
      {0.02, 0.02, 0.02, 0.10, 0.10, 0.10});
  config.session.admittance.max_velocity = arrayParameter<6>(
      node, "force.admittance.max_velocity",
      {0.02, 0.02, 0.02, 0.10, 0.10, 0.10});
  config.session.admittance.max_acceleration = arrayParameter<6>(
      node, "force.admittance.max_acceleration",
      {0.20, 0.20, 0.20, 1.0, 1.0, 1.0});
  config.session.contact.enter_force_n = node->declare_parameter<double>(
      "force.contact.enter_force_n", 5.0);
  config.session.contact.exit_force_n = node->declare_parameter<double>(
      "force.contact.exit_force_n", 3.0);
  config.session.contact.confirm_time_s = node->declare_parameter<double>(
      "force.contact.confirm_time_s", 0.040);
  config.session.contact.release_time_s = node->declare_parameter<double>(
      "force.contact.release_time_s", 0.040);
  config.session.contact.lateral_enter_force_n = node->declare_parameter<double>(
      "force.contact.lateral_enter_force_n", 1.0);
  config.session.contact.lateral_exit_force_n = node->declare_parameter<double>(
      "force.contact.lateral_exit_force_n", 0.5);
  config.session.contact.tilt_enter_torque_nm = node->declare_parameter<double>(
      "force.contact.tilt_enter_torque_nm", 0.02);
  config.session.contact.tilt_exit_torque_nm = node->declare_parameter<double>(
      "force.contact.tilt_exit_torque_nm", 0.01);
  config.session.contact.torsion_enter_torque_nm = node->declare_parameter<double>(
      "force.contact.torsion_enter_torque_nm", 0.02);
  config.session.contact.torsion_exit_torque_nm = node->declare_parameter<double>(
      "force.contact.torsion_exit_torque_nm", 0.01);
  config.session.state_machine.preinsert_hold_s =
      node->declare_parameter<double>("force.preinsert_hold_s", 0.10);
  config.session.state_machine.depth_tolerance_m =
      node->declare_parameter<double>("force.depth_tolerance_m", 0.0005);
  config.session.state_machine.force_error_tolerance_n =
      node->declare_parameter<double>("force.force_error_tolerance_n", 1.0);
  config.session.state_machine.pre_stop_force_error_tolerance_n = node->declare_parameter<double>(
      "force.pre_stop_force_error_tolerance_n", config.session.state_machine.force_error_tolerance_n);
  config.session.state_machine.force_rate_tolerance_n_s =
      node->declare_parameter<double>("force.force_rate_tolerance_n_s", 5.0);
  config.session.state_machine.minimum_alignment_progress_fraction = node->declare_parameter<double>(
      "force.minimum_alignment_progress_fraction", .05);
  config.session.state_machine.completion_hold_s =
      node->declare_parameter<double>("force.completion_hold_s", 0.20);
  config.session.state_machine.alignment_hold_s = node->declare_parameter<double>("force.alignment_hold_s", .08);
  config.session.state_machine.alignment_force_tolerance_n = node->declare_parameter<double>("force.alignment_force_tolerance_n", 1.0);
  config.session.state_machine.alignment_torque_tolerance_nm = node->declare_parameter<double>("force.alignment_torque_tolerance_nm", .10);
  config.session.state_machine.no_progress_timeout_s = node->declare_parameter<double>("force.no_progress_timeout_s", 2.0);
  config.session.state_machine.minimum_progress_m = node->declare_parameter<double>("force.minimum_progress_m", 0.0001);
  config.session.safety.max_force_rate_n_s = arrayParameter<3>(
      node, "force.safety.max_force_rate_n_s",
      {1000.0, 1000.0, 1000.0});
  config.session.safety.max_correction = arrayParameter<6>(
      node, "force.safety.max_correction",
      config.session.admittance.max_correction);
  config.session.safety.max_lateral_correction_norm_m = node->declare_parameter<double>(
      "force.safety.max_lateral_correction_norm_m", 1.0);
  config.session.safety.max_rotation_correction_norm_rad = node->declare_parameter<double>(
      "force.safety.max_rotation_correction_norm_rad", 3.141592653589793);
  config.session.safety.stale_timeout_s = node->declare_parameter<double>(
      "force.safety.stale_timeout_s", 0.050);
  config.session.continuous_ik.max_joint_step_rad =
      node->declare_parameter<double>("force.ik.max_joint_step_rad", 0.10);
  config.session.continuous_ik.max_position_error_m =
      node->declare_parameter<double>("force.ik.max_position_error_m", 0.001);
  config.session.continuous_ik.max_orientation_error_rad =
      node->declare_parameter<double>("force.ik.max_orientation_error_rad",
                                      0.01);
  return config;
}

} // namespace fr10_pose_controller
