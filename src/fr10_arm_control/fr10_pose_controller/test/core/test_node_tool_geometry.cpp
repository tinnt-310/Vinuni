#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "fr10_pose_controller/node_parameters.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  bool ok = true;
  {
    rclcpp::Node node("model_tool_default");
    ok = fr10_pose_controller::NodeParameters::declare(&node)
             .analytical_ik_use_model_tool_transform;
  }
  for (const auto &override : {
      rclcpp::Parameter("controller.analytical_ik.use_model_tool_transform", false),
      rclcpp::Parameter("controller.analytical_ik.wrist3_to_tip_translation_m",
                        std::vector<double>{.01, .02, .03}),
      rclcpp::Parameter("controller.analytical_ik.wrist3_to_tip_quaternion_xyzw",
                        std::vector<double>{0., 0., 0., 1.})}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({override});
    rclcpp::Node node("manual_tool_rejected", options);
    bool rejected = false;
    try { (void)fr10_pose_controller::NodeParameters::declare(&node); }
    catch (const std::invalid_argument &) { rejected = true; }
    if (!rejected) {
      std::cerr << "Manual tool geometry accepted: " << override.get_name() << '\n';
      ok = false;
    }
  }
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
