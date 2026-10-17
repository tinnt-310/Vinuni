#include "fr10_motion_controller/trajectory_decoder.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "fr10_motion_core/cartesian_path.hpp"
#include "fr10_motion_core/math_utils.hpp"

namespace fr10_motion_controller {
namespace {

constexpr std::array<const char *, fr10_motion::kJointCount> kJointNames{
    "j1", "j2", "j3", "j4", "j5", "j6"};

double seconds(const builtin_interfaces::msg::Duration &duration) {
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) * 1e-9;
}

fr10_motion::Pose toCorePose(const geometry_msgs::msg::Pose &pose) {
  fr10_motion::Pose result;
  result.position_m = {pose.position.x, pose.position.y, pose.position.z};
  result.orientation_quat = {pose.orientation.x, pose.orientation.y,
                             pose.orientation.z, pose.orientation.w};
  return result;
}

bool finitePose(const fr10_motion::Pose &pose) {
  return std::all_of(pose.position_m.begin(), pose.position_m.end(),
                     [](double value) { return std::isfinite(value); }) &&
         std::all_of(pose.orientation_quat.begin(),
                     pose.orientation_quat.end(),
                     [](double value) { return std::isfinite(value); }) &&
         fr10_motion::math::quaternionNormSquared(pose.orientation_quat) >
             1e-12;
}

} // namespace

DecodedTrajectory decodeTrajectory(
    const fr10_motion_interfaces::msg::JointTrajectoryCommand &message) {
  DecodedTrajectory result;
  result.mode = message.mode;
  if (message.generation == 0) {
    result.error = "generation must be non-zero";
    return result;
  }
  if (message.mode >
      fr10_motion_interfaces::msg::JointTrajectoryCommand::CARTESIAN_LINEAR) {
    result.error = "unknown motion mode";
    return result;
  }
  const auto &input = message.trajectory;
  if (input.points.size() < 2 ||
      input.joint_names.size() != fr10_motion::kJointCount) {
    result.error = "trajectory requires six joints and at least two points";
    return result;
  }

  std::array<std::size_t, fr10_motion::kJointCount> source_index{};
  std::array<bool, fr10_motion::kJointCount> used{};
  for (std::size_t expected = 0; expected < kJointNames.size(); ++expected) {
    const auto found = std::find(input.joint_names.begin(),
                                 input.joint_names.end(),
                                 kJointNames[expected]);
    if (found == input.joint_names.end()) {
      result.error = std::string("missing joint ") + kJointNames[expected];
      return result;
    }
    const auto index = static_cast<std::size_t>(
        std::distance(input.joint_names.begin(), found));
    if (used[index]) {
      result.error = "duplicate joint name";
      return result;
    }
    used[index] = true;
    source_index[expected] = index;
  }

  auto output = std::make_shared<fr10_motion::CartesianJointTrajectory>();
  output->generation = message.generation;
  output->planning_timestamp_s =
      static_cast<double>(message.header.stamp.sec) +
      static_cast<double>(message.header.stamp.nanosec) * 1e-9;
  output->path_start = toCorePose(message.path_start);
  output->path_goal = toCorePose(message.path_goal);
  if (!finitePose(output->path_start) || !finitePose(output->path_goal)) {
    result.error = "path endpoints must have finite positions and quaternions";
    return result;
  }
  output->path_start.orientation_quat =
      fr10_motion::math::normalizedQuaternion(output->path_start.orientation_quat);
  output->path_goal.orientation_quat =
      fr10_motion::math::normalizedQuaternion(output->path_goal.orientation_quat);

  const double duration_s = seconds(input.points.back().time_from_start);
  if (!std::isfinite(duration_s) || duration_s <= 0.0 ||
      !std::isfinite(message.planned_duration_s) ||
      std::abs(message.planned_duration_s - duration_s) > 1e-6) {
    result.error = "planned duration must equal the final positive point time";
    return result;
  }

  std::array<double, 3> tangent{};
  double tangent_norm = 0.0;
  for (std::size_t axis = 0; axis < tangent.size(); ++axis) {
    tangent[axis] = output->path_goal.position_m[axis] -
                    output->path_start.position_m[axis];
    tangent_norm += tangent[axis] * tangent[axis];
  }
  tangent_norm = std::sqrt(tangent_norm);
  if (tangent_norm > 1e-12) {
    for (double &value : tangent) {
      value /= tangent_norm;
    }
  }

  output->points.reserve(input.points.size());
  double previous_time_s = -1.0;
  for (const auto &input_point : input.points) {
    if (input_point.positions.size() != fr10_motion::kJointCount ||
        input_point.velocities.size() != fr10_motion::kJointCount ||
        input_point.accelerations.size() != fr10_motion::kJointCount) {
      result.error = "every point requires six positions, velocities and accelerations";
      return result;
    }
    const double time_s = seconds(input_point.time_from_start);
    if (!std::isfinite(time_s) || time_s <= previous_time_s) {
      result.error = "point times must be finite and strictly increasing";
      return result;
    }
    fr10_motion::CartesianTrajectoryPoint output_point;
    for (std::size_t joint = 0; joint < fr10_motion::kJointCount; ++joint) {
      const auto source = source_index[joint];
      const double position = input_point.positions[source];
      const double velocity = input_point.velocities[source];
      const double acceleration = input_point.accelerations[source];
      if (!std::isfinite(position) || !std::isfinite(velocity) ||
          !std::isfinite(acceleration)) {
        result.error = "trajectory contains a non-finite joint sample";
        return result;
      }
      output_point.joint_state.position_rad[joint] = position;
      output_point.joint_state.velocity_rad_s[joint] = velocity;
      output_point.joint_state.acceleration_rad_s2[joint] = acceleration;
    }
    output_point.time_from_start_s = time_s;
    output_point.path_parameter = std::clamp(time_s / duration_s, 0.0, 1.0);
    output_point.desired_pose = fr10_motion::CartesianPath::interpolate(
        output->path_start, output->path_goal, output_point.path_parameter);
    output_point.desired_translation_tangent = tangent;
    output->points.push_back(output_point);
    previous_time_s = time_s;
  }
  output->seed_joint_state = output->points.front().joint_state;
  result.trajectory = std::move(output);
  return result;
}

} // namespace fr10_motion_controller
