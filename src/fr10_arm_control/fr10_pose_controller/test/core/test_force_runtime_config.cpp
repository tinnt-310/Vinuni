#include <cmath>
#include <iostream>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include "fr10_pose_controller/force_runtime_config.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  bool ok = true;
  const auto check = [&](bool value, const char *name) {
    if (!value) { std::cerr << "FAIL " << name << '\n'; ok = false; }
  };
  {
    rclcpp::Node node("force_config_defaults");
    const auto config = fr10_pose_controller::declareForceRuntimeConfig(&node, "charging_tcp");
    check(config.session.contact_reference.lateral_probe.max_coordinate == .0025,
          "DEFAULT_XY_PROBE_LIMIT_REMAINS_2_5_MM");
    check(config.session.contact_reference.spiral_enabled, "DEFAULT_SPIRAL_POLICY_UNCHANGED");
  }
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("force.search.radius_m", .0075),
        rclcpp::Parameter("force.search.spiral_enabled", false),
        rclcpp::Parameter("force.search.xy_max_coordinate_m", .0075)});
    rclcpp::Node node("force_config_extended", options);
    const auto config = fr10_pose_controller::declareForceRuntimeConfig(&node, "charging_tcp");
    check(config.session.contact_reference.lateral_probe.max_coordinate == .0075,
          "EXTENDED_XY_PROBE_LIMIT_REACHES_THE_REFERENCE_CONFIG");
    check(!config.session.contact_reference.spiral_enabled, "PASSIVE_MODE_REACHES_THE_REFERENCE_CONFIG");
    fr10_insertion::ContactReference reference(config.session.contact_reference);
    check(reference.config().lateral_probe.max_coordinate == .0075,
          "EXTENDED_XY_PROBE_LIMIT_IS_NOT_REPLACED_BY_THE_OLD_DEFAULT");
  }
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("force.search.xy_max_coordinate_m",
        std::numeric_limits<double>::quiet_NaN())});
    rclcpp::Node node("force_config_invalid", options);
    const auto config = fr10_pose_controller::declareForceRuntimeConfig(&node, "charging_tcp");
    bool rejected = false;
    try { fr10_insertion::ContactReference reference(config.session.contact_reference); }
    catch (const std::invalid_argument &) { rejected = true; }
    check(rejected, "NONFINITE_XY_PROBE_LIMIT_REJECTED_BEFORE_MOTION");
  }
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
