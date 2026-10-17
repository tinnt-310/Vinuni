#include <gtest/gtest.h>

#include <array>
#include <string>

#include "fr10_motion_controller/trajectory_decoder.hpp"

namespace {

fr10_motion_interfaces::msg::JointTrajectoryCommand validCommand() {
  fr10_motion_interfaces::msg::JointTrajectoryCommand command;
  command.generation = 7;
  command.mode =
      fr10_motion_interfaces::msg::JointTrajectoryCommand::CARTESIAN_LINEAR;
  command.path_start.orientation.w = 1.0;
  command.path_goal.orientation.w = 1.0;
  command.path_goal.position.x = 0.1;
  command.trajectory.joint_names = {"j3", "j1", "j2", "j6", "j4", "j5"};
  trajectory_msgs::msg::JointTrajectoryPoint first;
  first.positions = {3.0, 1.0, 2.0, 6.0, 4.0, 5.0};
  first.velocities.assign(6, 0.0);
  first.accelerations.assign(6, 0.0);
  trajectory_msgs::msg::JointTrajectoryPoint second = first;
  second.positions[1] = 1.001;
  second.time_from_start.nanosec = 8000000;
  command.trajectory.points = {first, second};
  command.planned_duration_s = 0.008;
  return command;
}

TEST(TrajectoryDecoder, MapsJointNamesAndPreservesBoundaryState) {
  const auto decoded =
      fr10_motion_controller::decodeTrajectory(validCommand());
  ASSERT_TRUE(decoded) << decoded.error;
  ASSERT_EQ(decoded.trajectory->points.size(), 2U);
  const std::array<double, 6> expected{1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  EXPECT_EQ(decoded.trajectory->points.front().joint_state.position_rad,
            expected);
  EXPECT_DOUBLE_EQ(decoded.trajectory->points.front().time_from_start_s, 0.0);
  EXPECT_DOUBLE_EQ(decoded.trajectory->points.back().path_parameter, 1.0);
  EXPECT_DOUBLE_EQ(decoded.trajectory->path_goal.position_m[0], 0.1);
}

TEST(TrajectoryDecoder, RejectsMissingDynamicState) {
  auto command = validCommand();
  command.trajectory.points.back().accelerations.clear();
  const auto decoded =
      fr10_motion_controller::decodeTrajectory(command);
  EXPECT_FALSE(decoded);
  EXPECT_NE(decoded.error.find("velocities and accelerations"),
            std::string::npos);
}

TEST(TrajectoryDecoder, RejectsDurationMismatch) {
  auto command = validCommand();
  command.planned_duration_s = 1.0;
  const auto decoded =
      fr10_motion_controller::decodeTrajectory(command);
  EXPECT_FALSE(decoded);
  EXPECT_NE(decoded.error.find("duration"), std::string::npos);
}

TEST(TrajectoryDecoder, RejectsReplayedGenerationZero) {
  auto command = validCommand();
  command.generation = 0;
  const auto decoded =
      fr10_motion_controller::decodeTrajectory(command);
  EXPECT_FALSE(decoded);
  EXPECT_NE(decoded.error.find("generation"), std::string::npos);
}

} // namespace
