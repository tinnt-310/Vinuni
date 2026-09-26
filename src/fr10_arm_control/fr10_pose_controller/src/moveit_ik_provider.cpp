#include "fr10_pose_controller/moveit_ik_provider.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <geometry_msgs/msg/pose.hpp>

namespace fr10_pose_controller {

MoveItIKProvider::MoveItIKProvider(
    const moveit::core::RobotModelPtr &robot_model,
    MoveItIKProviderConfig config)
    : robot_model_(robot_model), config_(std::move(config)) {
  if (!robot_model_) {
    throw std::invalid_argument("MoveItIKProvider requires a robot model");
  }
  joint_model_group_ = robot_model_->getJointModelGroup(config_.group_name);
  if (joint_model_group_ == nullptr ||
      joint_model_group_->getVariableCount() != fr10_motion::kJointCount ||
      !robot_model_->hasLinkModel(config_.tip_link) ||
      !robot_model_->hasLinkModel(config_.safety_link)) {
    throw std::invalid_argument("MoveItIKProvider model/group/link mismatch");
  }
}

fr10_motion::IKResult
MoveItIKProvider::solve(const fr10_motion::Pose &target,
                        const fr10_motion::KinematicState &seed) {
  fr10_motion::IKResult result;
  moveit::core::RobotState state(robot_model_);
  state.setToDefaultValues();
  state.setJointGroupPositions(joint_model_group_, seed.position_rad.data());
  state.update();

  geometry_msgs::msg::Pose pose;
  pose.position.x = target.position_m[0];
  pose.position.y = target.position_m[1];
  pose.position.z = target.position_m[2];
  pose.orientation.x = target.orientation_quat[0];
  pose.orientation.y = target.orientation_quat[1];
  pose.orientation.z = target.orientation_quat[2];
  pose.orientation.w = target.orientation_quat[3];
  if (!state.setFromIK(joint_model_group_, pose, config_.tip_link,
                       config_.timeout_s)) {
    result.message = "MoveIt IK failed";
    return result;
  }
  state.copyJointGroupPositions(joint_model_group_,
                                result.state.position_rad.data());
  return validate(result.state, seed);
}

fr10_motion::IKResult
MoveItIKProvider::validate(const fr10_motion::KinematicState &target,
                           const fr10_motion::KinematicState &seed) {
  fr10_motion::IKResult result;
  result.state = target;
  moveit::core::RobotState state(robot_model_);
  state.setToDefaultValues();
  state.setJointGroupPositions(joint_model_group_, target.position_rad.data());
  state.update();
  if (!validateState(state, seed, target, result.manipulability,
                     result.condition_number, result.message)) {
    return result;
  }
  result.success = true;
  result.message = "success";
  return result;
}

void MoveItIKProvider::setPlanningSceneSnapshot(
    planning_scene::PlanningSceneConstPtr planning_scene) {
  planning_scene_snapshot_ = std::move(planning_scene);
}

bool MoveItIKProvider::validateState(moveit::core::RobotState &state,
                                     const fr10_motion::KinematicState &seed,
                                     const fr10_motion::KinematicState &target,
                                     double &manipulability,
                                     double &condition_number,
                                     std::string &error) const {
  for (std::size_t joint = 0; joint < fr10_motion::kJointCount; ++joint) {
    if (!std::isfinite(target.position_rad[joint]) ||
        target.position_rad[joint] <
            config_.limits.min_joint_position_rad[joint] ||
        target.position_rad[joint] >
            config_.limits.max_joint_position_rad[joint]) {
      error = "joint limit violation";
      return false;
    }
  }

  const auto barrier_safe = [](double seed_value, double target_value,
                               double minimum_abs) {
    if (minimum_abs <= 0.0) {
      return true;
    }
    if (std::abs(target_value) < minimum_abs) {
      return false;
    }
    if (std::abs(seed_value) < minimum_abs) {
      return true;
    }
    return std::signbit(seed_value) == std::signbit(target_value);
  };
  if (!barrier_safe(seed.position_rad[4], target.position_rad[4],
                    config_.wrist_min_abs_rad)) {
    error = "wrist singularity barrier violated";
    return false;
  }
  if (!barrier_safe(seed.position_rad[2], target.position_rad[2],
                    config_.elbow_min_abs_rad)) {
    error = "elbow singularity barrier violated";
    return false;
  }

  state.setJointGroupPositions(joint_model_group_, target.position_rad.data());
  state.update();
  if (config_.collision_checking_enabled) {
    bool colliding = false;
    if (planning_scene_snapshot_) {
      colliding = planning_scene_snapshot_->isStateColliding(
          state, config_.group_name, false);
    } else if (config_.planning_scene_monitor) {
      planning_scene_monitor::LockedPlanningSceneRO scene(
          config_.planning_scene_monitor);
      if (!scene) {
        error = "PlanningScene collision scene unavailable";
        return false;
      }
      colliding =
          scene->isStateColliding(state, config_.group_name, false);
    } else if (config_.planning_scene) {
      colliding = config_.planning_scene->isStateColliding(
          state, config_.group_name, false);
    } else {
      error = "PlanningScene collision monitor unavailable";
      return false;
    }
    if (colliding) {
      error = "PlanningScene self/world collision detected";
      return false;
    }
  }
  const Eigen::Vector3d safety_position =
      state.getGlobalLinkTransform(config_.safety_link).translation();
  if (!safety_position.allFinite() ||
      safety_position.norm() > config_.max_wrist_reach_m ||
      std::hypot(safety_position.x(), safety_position.y()) <
          config_.min_wrist_radius_m) {
    error = "workspace/singularity safety-link barrier violated";
    return false;
  }

  const Eigen::MatrixXd jacobian = state.getJacobian(joint_model_group_);
  if (jacobian.rows() == 0 || jacobian.cols() == 0 || !jacobian.allFinite()) {
    error = "invalid Jacobian";
    return false;
  }
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
  const auto singular_values = svd.singularValues();
  if (singular_values.size() == 0 ||
      singular_values(singular_values.size() - 1) <=
          std::numeric_limits<double>::epsilon()) {
    error = "singular Jacobian";
    return false;
  }
  condition_number =
      singular_values(0) / singular_values(singular_values.size() - 1);
  manipulability =
      std::sqrt(std::max(0.0, (jacobian * jacobian.transpose()).determinant()));
  if (!std::isfinite(condition_number) ||
      condition_number > config_.max_jacobian_condition) {
    error = "Jacobian condition-number limit exceeded";
    return false;
  }
  return true;
}

bool MoveItIKProvider::forward(const fr10_motion::KinematicState &input,
                               fr10_motion::Pose &pose,
                               std::string *error) const {
  return forwardLink(input, config_.tip_link, pose, error);
}

bool MoveItIKProvider::forwardLink(const fr10_motion::KinematicState &input,
                                   const std::string &link_name,
                                   fr10_motion::Pose &pose,
                                   std::string *error) const {
  if (!robot_model_->hasLinkModel(link_name)) {
    if (error != nullptr) {
      *error = "MoveIt FK link does not exist: " + link_name;
    }
    return false;
  }
  moveit::core::RobotState state(robot_model_);
  state.setToDefaultValues();
  state.setJointGroupPositions(joint_model_group_, input.position_rad.data());
  state.update();
  const Eigen::Isometry3d &transform = state.getGlobalLinkTransform(link_name);
  if (!transform.matrix().allFinite()) {
    if (error != nullptr) {
      *error = "MoveIt FK produced a non-finite transform";
    }
    return false;
  }
  const Eigen::Vector3d translation = transform.translation();
  const Eigen::Quaterniond orientation(transform.rotation());
  pose.position_m = {translation.x(), translation.y(), translation.z()};
  pose.orientation_quat = {orientation.x(), orientation.y(), orientation.z(),
                           orientation.w()};
  return true;
}

} // namespace fr10_pose_controller
