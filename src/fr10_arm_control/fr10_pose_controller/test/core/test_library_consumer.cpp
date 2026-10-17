#include "fr10_motion_core/motion_controller.hpp"

#include <memory>
#include <string>

namespace {
std::shared_ptr<fr10_motion::CartesianJointTrajectory> poseTrajectory() {
  auto trajectory =
      std::make_shared<fr10_motion::CartesianJointTrajectory>();
  trajectory->generation = 1;
  for (std::size_t index = 0; index < 2; ++index) {
    fr10_motion::CartesianTrajectoryPoint point;
    point.joint_state.position_rad[0] = 0.01 * static_cast<double>(index);
    point.time_from_start_s = 0.008 * static_cast<double>(index);
    point.path_parameter = static_cast<double>(index);
    trajectory->points.push_back(point);
  }
  return trajectory;
}

bool finishStop(fr10_motion::MotionController &controller,
                fr10_motion::RobotState &feedback) {
  for (int cycle = 0;
       cycle < 1000 &&
       controller.status().state == fr10_motion::MotionState::STOPPING;
       ++cycle) {
    controller.update(feedback, 0.008);
    const auto command = controller.command();
    feedback.joint_position_rad = command.joint_position_rad;
    feedback.joint_velocity_rad_s = command.joint_velocity_rad_s;
    feedback.joint_acceleration_rad_s2 = command.joint_acceleration_rad_s2;
  }
  return controller.status().state == fr10_motion::MotionState::HOLDING;
}
} // namespace

int main() {
  if (std::string(fr10_motion::controlModeName(
          fr10_motion::ControlMode::JOINT)) != "JOINT" ||
      std::string(fr10_motion::controlModeName(
          fr10_motion::ControlMode::POSE)) != "POSE") {
    return 1;
  }
  fr10_motion::MotionControllerConfig config;
  config.control_rate_hz = 125.0;
  config.limits.min_joint_position_rad.fill(-3.0);
  config.limits.max_joint_position_rad.fill(3.0);
  config.limits.max_joint_velocity_rad_s.fill(0.3);
  config.limits.max_joint_acceleration_rad_s2.fill(0.8);
  config.limits.max_joint_jerk_rad_s3.fill(4.0);
  fr10_motion::MotionController controller(config);
  fr10_motion::RobotState feedback;
  feedback.valid = true;
  controller.update(feedback, 0.008);
  fr10_motion::JointTarget a;
  a.position_rad[0] = 0.5;
  a.revision = 1;
  fr10_motion::JointTarget b = a;
  b.position_rad[0] = -0.25;
  b.revision = 2;
  if (!controller.submitJointTarget(a) || !controller.submitJointTarget(b) ||
      !controller.submitPoseTrajectory(poseTrajectory())) {
    return 1;
  }
  (void)controller.status();
  controller.stop();
  if (!finishStop(controller, feedback)) {
    return 1;
  }
  b.revision = 3;
  if (!controller.submitJointTarget(b)) {
    return 1;
  }
  controller.update(feedback, 0.008);
  if (controller.status().state != fr10_motion::MotionState::TRACKING) {
    return 1;
  }
  controller.stop();
  return finishStop(controller, feedback) ? 0 : 1;
}
