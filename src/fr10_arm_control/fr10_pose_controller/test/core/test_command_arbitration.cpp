#include "fr10_pose_controller/motion_controller.hpp"
#include "test_support.hpp"

namespace {
std::shared_ptr<fr10_motion::CartesianJointTrajectory>
poseTrajectory(const std::array<double, 6> &start,
               const std::array<double, 6> &goal, uint64_t generation) {
  auto trajectory =
      std::make_shared<fr10_motion::CartesianJointTrajectory>();
  trajectory->generation = generation;
  for (std::size_t index = 0; index < 2; ++index) {
    fr10_motion::CartesianTrajectoryPoint point;
    point.joint_state.position_rad = index == 0 ? start : goal;
    point.time_from_start_s = 0.008 * static_cast<double>(index);
    point.path_parameter = static_cast<double>(index);
    trajectory->points.push_back(point);
  }
  return trajectory;
}
} // namespace

int main() {
  fr10_motion_test::Check check;
  fr10_motion::MotionControllerConfig config;
  config.control_rate_hz = 125.0;
  config.limits = fr10_motion_test::limits();
  fr10_motion::MotionController controller(config);
  fr10_motion::RobotState feedback;
  feedback.valid = true;
  feedback.tcp_position_m = {-0.4, -0.1, 0.6};
  feedback.tcp_orientation_quat = {0, 0, 0, 1};
  controller.update(feedback, 0.008);

  fr10_motion::JointTarget joint;
  joint.position_rad[0] = 0.5;
  joint.revision = 1;
  check.require(controller.submitJointTarget(joint), "OWNER_JOINT_ACCEPT",
                "joint target rejected");
  auto status = controller.update(feedback, 0.008);
  check.require(status.owner == fr10_motion::MotionOwner::JOINT_OTG &&
                    controller.command().type ==
                        fr10_motion::CommandType::JOINT_POSITION,
                "OWNER_JOINT_EXCLUSIVE", "joint owner/command mismatch");

  auto pose_goal = feedback.joint_position_rad;
  pose_goal[0] = 0.1;
  check.require(controller.submitPoseTrajectory(
                    poseTrajectory(feedback.joint_position_rad, pose_goal, 1)),
                "OWNER_POSE_TARGET_TAKEOVER", "Pose target takeover rejected");
  status = controller.update(feedback, 0.008);
  check.require(status.owner == fr10_motion::MotionOwner::POSE_PATH &&
                    controller.command().type ==
                        fr10_motion::CommandType::JOINT_POSITION,
                "OWNER_POSE_PATH_EXCLUSIVE", "more than one logical owner");

  joint.position_rad[0] = -0.5;
  joint.revision = 2;
  check.require(controller.submitJointTarget(joint), "OWNER_JOINT_TAKEBACK",
                "joint takeover rejected");
  status = controller.update(feedback, 0.008);
  check.require(status.owner == fr10_motion::MotionOwner::JOINT_OTG &&
                    controller.command().type ==
                        fr10_motion::CommandType::JOINT_POSITION,
                "OWNER_FINAL_EXCLUSIVE", "owner arbitration not deterministic");
  return check.result();
}
