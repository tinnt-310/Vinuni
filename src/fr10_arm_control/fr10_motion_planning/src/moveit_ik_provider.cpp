#include "fr10_motion_planning/moveit_ik_provider.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <geometry_msgs/msg/pose.hpp>

namespace fr10_motion_planning {

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
      !robot_model_->hasLinkModel(config_.base_link) ||
      !robot_model_->hasLinkModel(config_.safety_link)) {
    throw std::invalid_argument("MoveItIKProvider model/group/link mismatch");
  }
}

fr10_motion::IKResult
MoveItIKProvider::solve(const fr10_motion::Pose &target,
                        const fr10_motion::KinematicState &seed) {
  fr10_motion::IKResult result;
  moveit::core::RobotState state = stateFromScene();
  state.setJointGroupPositions(joint_model_group_, seed.position_rad.data());
  state.update();

  Eigen::Quaterniond orientation(target.orientation_quat[3],
                                 target.orientation_quat[0],
                                 target.orientation_quat[1],
                                 target.orientation_quat[2]);
  if (!orientation.coeffs().allFinite() || orientation.norm() < 1e-9 ||
      !std::all_of(target.position_m.begin(), target.position_m.end(),
                   [](double value) { return std::isfinite(value); })) {
    result.message = "invalid target pose";
    return result;
  }
  orientation.normalize();
  Eigen::Isometry3d base_target = Eigen::Isometry3d::Identity();
  base_target.translation() = Eigen::Vector3d(target.position_m[0],
                                               target.position_m[1],
                                               target.position_m[2]);
  base_target.linear() = orientation.toRotationMatrix();
  const Eigen::Isometry3d global_target =
      state.getGlobalLinkTransform(config_.base_link) * base_target;
  const Eigen::Quaterniond global_orientation(global_target.rotation());
  geometry_msgs::msg::Pose pose;
  pose.position.x = global_target.translation().x();
  pose.position.y = global_target.translation().y();
  pose.position.z = global_target.translation().z();
  pose.orientation.x = global_orientation.x();
  pose.orientation.y = global_orientation.y();
  pose.orientation.z = global_orientation.z();
  pose.orientation.w = global_orientation.w();
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
  if (!validation_state_)
    validation_state_ = std::make_unique<moveit::core::RobotState>(stateFromScene());
  else if (!planning_scene_snapshot_)
    *validation_state_ = stateFromScene();
  auto &state = *validation_state_;
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
  if (planning_scene == planning_scene_snapshot_) return;
  planning_scene_snapshot_ = std::move(planning_scene);
  validation_state_.reset();
  snapshot_acm_.reset();
  if (planning_scene_snapshot_) {
    initializeInvariantClearPairs(*planning_scene_snapshot_);
    snapshot_acm_ = std::make_unique<collision_detection::AllowedCollisionMatrix>(
        planning_scene_snapshot_->getAllowedCollisionMatrix());
    for (const auto &pair : invariant_clear_pairs_)
      snapshot_acm_->setEntry(pair.first, pair.second, true);
  }
}

void MoveItIKProvider::initializeInvariantClearPairs(
    const planning_scene::PlanningScene &scene) {
  if (invariant_pairs_initialized_) return;
  // Only immutable RobotModel geometry is cached. Attached objects and world
  // geometry still use the current scene's complete collision matrix.
  moveit::core::RobotState reference(robot_model_);
  reference.setToDefaultValues();
  reference.update();
  const auto &links = robot_model_->getLinkModelsWithCollisionGeometry();
  const auto rigid_root = [](const moveit::core::LinkModel *link) {
    while (link->getParentJointModel() &&
           link->getParentJointModel()->getType() == moveit::core::JointModel::FIXED &&
           link->getParentLinkModel()) link = link->getParentLinkModel();
    return link;
  };
  collision_detection::AllowedCollisionMatrix isolated(robot_model_->getLinkModelNames(), true);
  for (std::size_t i = 0; i < links.size(); ++i) {
    for (std::size_t j = i + 1; j < links.size(); ++j) {
      if (rigid_root(links[i]) != rigid_root(links[j])) continue;
      isolated.setEntry(links[i]->getName(), links[j]->getName(), false);
      collision_detection::DistanceRequest request;
      request.acm = &isolated;
      request.distance_threshold = 0.001; // Conservative 1 mm proof margin.
      collision_detection::DistanceResult result;
      scene.getCollisionEnvUnpadded()->distanceSelf(request, result, reference);
      isolated.setEntry(links[i]->getName(), links[j]->getName(), true);
      if (!result.collision && result.minimum_distance.distance >= request.distance_threshold)
        invariant_clear_pairs_.emplace_back(links[i]->getName(), links[j]->getName());
    }
  }
  invariant_pairs_initialized_ = true;
}

moveit::core::RobotState MoveItIKProvider::stateFromScene() const {
  moveit::core::RobotState state(robot_model_);
  state.setToDefaultValues();
  // Arm candidates are expressed relative to the arm base, but collision
  // geometry outside the arm group belongs to the same robot model. Preserve
  // the scene's measured vehicle-joint state while replacing only j1-j6.
  if (planning_scene_snapshot_) {
    state = planning_scene_snapshot_->getCurrentState();
  } else if (config_.planning_scene_monitor) {
    planning_scene_monitor::LockedPlanningSceneRO scene(
        config_.planning_scene_monitor);
    if (scene) {
      state = scene->getCurrentState();
    }
  } else if (config_.planning_scene) {
    state = config_.planning_scene->getCurrentState();
  }
  return state;
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
      collision_detection::CollisionRequest request;
      request.group_name = config_.group_name;
      collision_detection::CollisionResult result;
      planning_scene_snapshot_->checkCollision(request, result, state, *snapshot_acm_);
      colliding = result.collision;
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
      // The planning worker supplies an immutable scene snapshot. Ask it for
      // one contact only on the rejection path so diagnostics identify which
      // model pair blocked this candidate without work in the control tick.
      if (planning_scene_snapshot_) {
        collision_detection::CollisionRequest request;
        request.group_name = config_.group_name;
        request.contacts = true;
        request.max_contacts = 1;
        request.max_contacts_per_pair = 1;
        collision_detection::CollisionResult details;
        planning_scene_snapshot_->checkCollision(request, details, state, *snapshot_acm_);
        if (!details.contacts.empty()) {
          const auto &contact = *details.contacts.begin();
          error += ": " + contact.first.first + " / " +
                   contact.first.second;
          if (!contact.second.empty()) {
            error += " depth=" +
                     std::to_string(contact.second.front().depth) + " m";
          }
        }
      }
      return false;
    }
  }
  const Eigen::Vector3d safety_position =
      (state.getGlobalLinkTransform(config_.base_link).inverse() *
       state.getGlobalLinkTransform(config_.safety_link)).translation();
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
  const Eigen::Isometry3d transform =
      state.getGlobalLinkTransform(config_.base_link).inverse() *
      state.getGlobalLinkTransform(link_name);
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

} // namespace fr10_motion_planning
