#include <cmath>
#include <limits>

#include "fr10_pose_controller/motion_controller.hpp"
#include "test_support.hpp"

namespace {
std::shared_ptr<fr10_motion::CartesianJointTrajectory>
poseTrajectory(uint64_t generation) {
  auto trajectory =
      std::make_shared<fr10_motion::CartesianJointTrajectory>();
  trajectory->generation = generation;
  for (std::size_t index = 0; index < 2; ++index) {
    fr10_motion::CartesianTrajectoryPoint point;
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
  feedback.tcp_orientation_quat = {0, 0, 0, 1};
  controller.update(feedback, 0.008);

  fr10_motion::JointTarget joint;
  joint.revision = 1;
  auto invalid_joint = joint;
  invalid_joint.position_rad[0] = std::numeric_limits<double>::quiet_NaN();
  check.require(!controller.submitJointTarget(invalid_joint), "INVALID_JOINT_NAN",
                "NaN joint target accepted");
  invalid_joint = joint;
  invalid_joint.position_rad[0] = std::numeric_limits<double>::infinity();
  check.require(!controller.submitJointTarget(invalid_joint), "INVALID_JOINT_INF",
                "Inf joint target accepted");
  invalid_joint = joint;
  invalid_joint.position_rad[0] = 4.0;
  check.require(!controller.submitJointTarget(invalid_joint), "INVALID_JOINT_LIMIT",
                "out-of-bounds joint target accepted");
  invalid_joint = joint;
  invalid_joint.velocity_scale = std::numeric_limits<double>::quiet_NaN();
  check.require(!controller.submitJointTarget(invalid_joint), "INVALID_JOINT_SCALE",
                "NaN joint scale accepted");

  auto invalid_pose = poseTrajectory(1);
  invalid_pose->points.back().joint_state.position_rad[0] =
      std::numeric_limits<double>::quiet_NaN();
  check.require(!controller.submitPoseTrajectory(invalid_pose),
                "INVALID_POSE_JOINT_NAN", "NaN Pose IK target accepted");
  invalid_pose = poseTrajectory(0);
  check.require(!controller.submitPoseTrajectory(invalid_pose),
                "INVALID_POSE_GENERATION", "zero Pose generation accepted");
  invalid_pose = poseTrajectory(2);
  invalid_pose->points.back().joint_state.velocity_rad_s[0] =
      std::numeric_limits<double>::infinity();
  check.require(!controller.submitPoseTrajectory(invalid_pose),
                "INVALID_POSE_VELOCITY", "invalid Pose velocity accepted");
  invalid_pose = poseTrajectory(3);
  invalid_pose->points.back().time_from_start_s = 1e-8;
  check.require(!controller.submitPoseTrajectory(invalid_pose),
                "INVALID_POSE_SUB_MICROSECOND_SAMPLE",
                "numerically ill-conditioned trajectory interval accepted");
  return check.result();
}
