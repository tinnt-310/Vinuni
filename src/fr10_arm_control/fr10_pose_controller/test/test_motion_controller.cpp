#include <cmath>
#include <iostream>
#include <utility>

#include "fr10_pose_controller/motion_controller.hpp"
#include "test_support.hpp"

namespace {
std::shared_ptr<fr10_motion::CartesianJointTrajectory>
poseTrajectory(const std::array<double, 6> &start,
               const std::array<double, 6> &goal, uint64_t generation,
               std::size_t samples = 101) {
  auto trajectory =
      std::make_shared<fr10_motion::CartesianJointTrajectory>();
  trajectory->generation = generation;
  for (std::size_t index = 0; index < samples; ++index) {
    const double fraction =
        static_cast<double>(index) / static_cast<double>(samples - 1);
    fr10_motion::CartesianTrajectoryPoint point;
    for (std::size_t joint = 0; joint < 6; ++joint) {
      point.joint_state.position_rad[joint] =
          start[joint] + fraction * (goal[joint] - start[joint]);
      point.joint_state.velocity_rad_s[joint] =
          (goal[joint] - start[joint]) /
          (0.008 * static_cast<double>(samples - 1));
    }
    if (index + 1 == samples) {
      point.joint_state.velocity_rad_s.fill(0.0);
    }
    point.time_from_start_s = 0.008 * static_cast<double>(index);
    point.path_parameter = fraction;
    point.desired_pose.position_m[0] = fraction;
    trajectory->points.push_back(point);
  }
  trajectory->path_start = trajectory->points.front().desired_pose;
  trajectory->path_goal = trajectory->points.back().desired_pose;
  return trajectory;
}
} // namespace

int main() {
  fr10_motion_test::Check check;
  fr10_motion::MotionExecutorConfig config;
  config.control_rate_hz = 125.0;
  config.joint_goal_tolerance_rad = 0.002;
  config.goal_stable_samples = 2;
  config.max_joint_reference_lead_rad = 1.0;
  config.limits.min_joint_position_rad.fill(-3.0);
  config.limits.max_joint_position_rad.fill(3.0);
  config.limits.max_joint_velocity_rad_s.fill(0.5);
  config.limits.max_joint_acceleration_rad_s2.fill(1.0);
  config.limits.max_joint_jerk_rad_s3.fill(5.0);

  fr10_motion::MotionExecutor executor(config);
  fr10_motion::RobotState feedback;
  feedback.valid = true;
  feedback.joint_position_rad = {0.0, -1.0, 1.0, 0.0, 1.0, 0.0};
  constexpr double dt = 0.008;
  executor.update(feedback, dt);

  fr10_motion::JointTarget first_target;
  first_target.position_rad = feedback.joint_position_rad;
  first_target.position_rad[0] = 0.25;
  first_target.revision = 1;
  check.require(executor.submitJointTarget(first_target),
                "EXECUTOR_JOINT_ACCEPT", "first Joint target was rejected");
  for (int cycle = 0; cycle < 20; ++cycle) {
    executor.update(feedback, dt);
    const auto command = executor.command();
    feedback.joint_position_rad = command.joint_position_rad;
    feedback.joint_velocity_rad_s = command.joint_velocity_rad_s;
    feedback.joint_acceleration_rad_s2 = command.joint_acceleration_rad_s2;
  }
  const double velocity_before_retarget = feedback.joint_velocity_rad_s[0];
  check.require(velocity_before_retarget > 0.0, "EXECUTOR_JOINT_MOVING",
                "Joint OTG did not accelerate before retargeting");

  fr10_motion::JointTarget second_target = first_target;
  second_target.position_rad[0] = -0.15;
  second_target.revision = 2;
  check.require(executor.submitJointTarget(second_target),
                "EXECUTOR_JOINT_RETARGET",
                "online Joint retarget was rejected");
  executor.update(feedback, dt);
  check.require(std::isfinite(executor.command().joint_velocity_rad_s[0]),
                "EXECUTOR_RETARGET_FINITE",
                "retarget generated a non-finite velocity");

  int cycles = 0;
  while (executor.status().state != fr10_motion::MotionState::HOLDING &&
         cycles++ < 1000) {
    executor.update(feedback, dt);
    const auto command = executor.command();
    feedback.joint_position_rad = command.joint_position_rad;
    feedback.joint_velocity_rad_s = command.joint_velocity_rad_s;
    feedback.joint_acceleration_rad_s2 = command.joint_acceleration_rad_s2;
  }
  check.require(executor.status().state == fr10_motion::MotionState::HOLDING,
                "EXECUTOR_JOINT_HOLD", "Joint target did not reach HOLDING");

  auto pose_target = feedback.joint_position_rad;
  pose_target[1] += 0.10;
  check.require(executor.submitPoseTrajectory(
                    poseTrajectory(feedback.joint_position_rad, pose_target, 7)),
                "EXECUTOR_POSE_TARGET_ACCEPT", "Pose target was rejected");
  check.require(executor.status().state == fr10_motion::MotionState::TRACKING,
                "EXECUTOR_POSE_TARGET_TRACKING",
                "accepted Pose target did not enter TRACKING");

  for (int cycle = 0; cycle < 20; ++cycle) {
    executor.update(feedback, dt);
    const auto command = executor.command();
    feedback.joint_position_rad = command.joint_position_rad;
    feedback.joint_velocity_rad_s = command.joint_velocity_rad_s;
    feedback.joint_acceleration_rad_s2 = command.joint_acceleration_rad_s2;
  }
  const double pose_velocity_before_retarget =
      feedback.joint_velocity_rad_s[1];
  auto pose_retarget = feedback.joint_position_rad;
  pose_retarget[1] -= 0.15;
  check.require(executor.submitPoseTrajectory(
                    poseTrajectory(feedback.joint_position_rad,
                                   pose_retarget, 8, 151)),
                "EXECUTOR_POSE_RETARGET_ACCEPT",
                "mid-flight Pose retarget was rejected");
  executor.update(feedback, dt);
  const auto first_retarget_command = executor.command();
  check.require(std::abs(pose_velocity_before_retarget) > 0.0 &&
                    std::isfinite(first_retarget_command.joint_velocity_rad_s[1]),
                "EXECUTOR_POSE_RETARGET_CONTINUOUS",
                "Pose retarget did not preserve a finite moving state");
  feedback.joint_position_rad = first_retarget_command.joint_position_rad;
  feedback.joint_velocity_rad_s = first_retarget_command.joint_velocity_rad_s;
  feedback.joint_acceleration_rad_s2 =
      first_retarget_command.joint_acceleration_rad_s2;

  cycles = 0;
  bool joint_only_output = true;
  while (executor.status().state != fr10_motion::MotionState::HOLDING &&
         cycles++ < 300) {
    executor.update(feedback, dt);
    const auto command = executor.command();
    joint_only_output = joint_only_output &&
                        command.type ==
                            fr10_motion::CommandType::JOINT_POSITION;
    feedback.joint_position_rad = command.joint_position_rad;
    feedback.joint_velocity_rad_s = command.joint_velocity_rad_s;
    feedback.joint_acceleration_rad_s2 = command.joint_acceleration_rad_s2;
  }
  check.require(joint_only_output, "EXECUTOR_JOINT_ONLY_OUTPUT",
                "Pose path emitted a non-joint command");
  check.require(executor.status().owner == fr10_motion::MotionOwner::POSE_PATH,
                "EXECUTOR_POSE_OWNERSHIP", "Pose ownership was lost");
  check.require(executor.status().active_generation == 8,
                "EXECUTOR_POSE_GENERATION", "Pose generation changed");
  check.require(executor.status().state == fr10_motion::MotionState::HOLDING,
                "EXECUTOR_POSE_HOLD", "Pose path did not reach HOLDING");
  check.require(std::abs(feedback.joint_position_rad[1] -
                         pose_retarget[1]) < 0.002,
                "EXECUTOR_POSE_ENDPOINT", "Pose path endpoint was not reached");

  executor.reportCartesianTrackingError(0.003, 0.01, 0.003, true);
  executor.update(feedback, dt);
  check.require(executor.status().state == fr10_motion::MotionState::STOPPING ||
                    executor.status().state == fr10_motion::MotionState::HOLDING,
                "EXECUTOR_PATH_MONITOR_CONTROLLED_STOP",
                "Cartesian path monitor did not request controlled stopping");

  fr10_motion::MotionExecutorConfig lag_config = config;
  lag_config.max_joint_reference_lead_rad = 1e-5;
  lag_config.reference_lag_persistence_samples = 3;
  fr10_motion::MotionExecutor lag_executor(lag_config);
  fr10_motion::RobotState lag_feedback;
  lag_feedback.valid = true;
  lag_executor.update(lag_feedback, dt);
  fr10_motion::JointTarget lag_target;
  lag_target.position_rad[0] = 0.25;
  lag_target.revision = 101;
  check.require(lag_executor.submitJointTarget(lag_target),
                "EXECUTOR_REFERENCE_LAG_TARGET_ACCEPT",
                "reference-lag test target was rejected");
  fr10_motion::MotionStatus lag_status;
  for (int cycle = 0; cycle < 20; ++cycle) {
    lag_status = lag_executor.update(lag_feedback, dt);
    if (lag_status.state == fr10_motion::MotionState::STOPPING) {
      break;
    }
  }
  check.require(lag_status.reference_lagging &&
                    lag_status.reference_lag_samples == 1 &&
                    lag_status.state == fr10_motion::MotionState::STOPPING,
                "EXECUTOR_REFERENCE_LAG_IMMEDIATE_STOP",
                "fresh reference lag did not initiate controlled stop");

  return check.result();
}
