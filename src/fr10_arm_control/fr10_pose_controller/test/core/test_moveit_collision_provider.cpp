#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include "fr10_motion_planning/moveit_ik_provider.hpp"

namespace {

constexpr const char *kUrdf = R"(
<robot name="collision_fixture">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="link2"/>
  <link name="link3"/>
  <link name="link4"/>
  <link name="link5"/>
  <link name="tool0">
    <collision><geometry><sphere radius="0.05"/></geometry></collision>
  </link>
  <link name="fixed_clear">
    <collision><geometry><sphere radius="0.03"/></geometry></collision>
  </link>
  <link name="fixed_overlap">
    <collision><geometry><sphere radius="0.02"/></geometry></collision>
  </link>
  <joint name="fixed_clear_joint" type="fixed"><parent link="tool0"/><child link="fixed_clear"/><origin xyz="0.3 0 0"/></joint>
  <joint name="fixed_overlap_joint" type="fixed"><parent link="tool0"/><child link="fixed_overlap"/><origin xyz="0.04 0 0"/></joint>
  <link name="vehicle_obstacle">
    <collision><geometry><sphere radius="0.05"/></geometry></collision>
  </link>
  <joint name="j1" type="revolute"><parent link="base_link"/><child link="link1"/><origin xyz="0.2 0 0"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="10" velocity="1"/></joint>
  <joint name="j2" type="revolute"><parent link="link1"/><child link="link2"/><origin xyz="0.2 0 0"/><axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="10" velocity="1"/></joint>
  <joint name="j3" type="revolute"><parent link="link2"/><child link="link3"/><origin xyz="0.2 0 0"/><axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="10" velocity="1"/></joint>
  <joint name="j4" type="revolute"><parent link="link3"/><child link="link4"/><origin xyz="0.2 0 0"/><axis xyz="1 0 0"/><limit lower="-3" upper="3" effort="10" velocity="1"/></joint>
  <joint name="j5" type="revolute"><parent link="link4"/><child link="link5"/><origin xyz="0.2 0 0"/><axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="10" velocity="1"/></joint>
  <joint name="j6" type="revolute"><parent link="link5"/><child link="tool0"/><origin xyz="0.2 0 0"/><axis xyz="1 0 0"/><limit lower="-3" upper="3" effort="10" velocity="1"/></joint>
  <joint name="vehicle_slide" type="prismatic"><parent link="base_link"/><child link="vehicle_obstacle"/><origin xyz="1.2 -0.2 0"/><axis xyz="0 1 0"/><limit lower="0" upper="0.3" effort="10" velocity="1"/></joint>
</robot>)";

constexpr const char *kSrdf = R"(
<robot name="collision_fixture">
  <group name="arm"><chain base_link="base_link" tip_link="tool0"/></group>
  <disable_collisions link1="tool0" link2="fixed_overlap" reason="fixture"/>
</robot>)";

} // namespace

int main() {
  const auto urdf_model = urdf::parseURDF(kUrdf);
  auto srdf_model = std::make_shared<srdf::Model>();
  if (!urdf_model || !srdf_model->initString(*urdf_model, kSrdf)) {
    std::cerr << "FAIL COLLISION_FIXTURE_MODEL\n";
    return 1;
  }
  auto robot_model =
      std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
  auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model);

  moveit_msgs::msg::CollisionObject obstacle;
  obstacle.header.frame_id = "base_link";
  obstacle.id = "dynamic_tool_obstacle";
  obstacle.operation = moveit_msgs::msg::CollisionObject::ADD;
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {0.10, 0.10, 0.10};
  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = 1.20;
  box_pose.orientation.w = 1.0;
  obstacle.primitives.push_back(box);
  obstacle.primitive_poses.push_back(box_pose);
  if (!scene->processCollisionObjectMsg(obstacle)) {
    std::cerr << "FAIL COLLISION_OBJECT_ADD\n";
    return 1;
  }

  fr10_motion_planning::MoveItIKProviderConfig config;
  config.group_name = "arm";
  config.base_link = "base_link";  // This synthetic fixture owns its own frames.
  config.tip_link = "tool0";
  config.safety_link = "tool0";
  config.max_wrist_reach_m = 2.0;
  config.min_wrist_radius_m = 0.0;
  config.max_jacobian_condition = 1e9;
  config.collision_checking_enabled = true;
  config.planning_scene = scene;
  config.limits.min_joint_position_rad.fill(-3.0);
  config.limits.max_joint_position_rad.fill(3.0);
  config.limits.max_joint_velocity_rad_s.fill(1.0);
  config.limits.max_joint_acceleration_rad_s2.fill(1.0);
  config.limits.max_joint_jerk_rad_s3.fill(1.0);
  fr10_motion_planning::MoveItIKProvider provider(robot_model, config);
  const auto pinned_scene = planning_scene::PlanningScene::clone(scene);

  fr10_motion::KinematicState state;
  const auto collision = provider.validate(state, state);
  const bool collision_rejected =
      !collision.success && collision.message.find("collision") !=
                                std::string::npos;

  obstacle.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  const bool removed = scene->processCollisionObjectMsg(obstacle);
  const auto after_removal = provider.validate(state, state);
  const bool collision_layer_cleared =
      after_removal.message.find("collision") == std::string::npos;

  provider.setPlanningSceneSnapshot(pinned_scene);
  const auto pinned_result = provider.validate(state, state);
  const bool pinned_scene_still_rejects =
      !pinned_result.success &&
      pinned_result.message.find("collision") != std::string::npos;
  provider.setPlanningSceneSnapshot({});
  const auto restored_result = provider.validate(state, state);
  const bool live_scene_restored =
      restored_result.message.find("collision") == std::string::npos;

  // The collision scene also owns the non-arm vehicle joints. The arm
  // candidate must be checked against their measured/snapshotted state.
  scene->getCurrentStateNonConst().setVariablePosition("vehicle_slide", 0.2);
  scene->getCurrentStateNonConst().update();
  const auto moving_vehicle_result = provider.validate(state, state);
  const bool moving_vehicle_collision_rejected =
      !moving_vehicle_result.success &&
      moving_vehicle_result.message.find("collision") != std::string::npos;

  const bool clear_pairs_cached = provider.invariantClearPairCount() == 2;
  auto overlap_scene = planning_scene::PlanningScene::clone(scene);
  overlap_scene->getAllowedCollisionMatrixNonConst().setEntry("tool0", "fixed_overlap", false);
  provider.setPlanningSceneSnapshot(overlap_scene);
  const auto fixed_overlap_result = provider.validate(state, state);
  const bool fixed_overlap_rejected = fixed_overlap_result.message.find("collision") != std::string::npos;
  bool equivalent = true;
  for (unsigned sample = 0; sample < 50; ++sample) {
    for (unsigned joint = 0; joint < 6; ++joint)
      state.position_rad[joint] = .6*std::sin(.31*sample+.9*joint);
    moveit::core::RobotState reference(overlap_scene->getCurrentState());
    reference.setJointGroupPositions(robot_model->getJointModelGroup("arm"),state.position_rad.data());
    reference.update();
    const bool full_collision = overlap_scene->isStateColliding(reference, "arm");
    const bool cached_collision = provider.validate(state,state).message.find("collision") != std::string::npos;
    equivalent = equivalent && full_collision == cached_collision;
  }
  // The cache belongs only to this provider and must not modify scene ACM.
  collision_detection::AllowedCollision::Type entry;
  const bool scene_acm_preserved = overlap_scene->getAllowedCollisionMatrix().getEntry(
      "tool0","fixed_overlap",entry) && entry == collision_detection::AllowedCollision::NEVER;
  if (!clear_pairs_cached || !fixed_overlap_rejected || !equivalent || !scene_acm_preserved) {
    std::cerr << "FAIL invariant rigid pair cache " << provider.invariantClearPairCount()
              << " overlap=" << fixed_overlap_rejected << " equivalent=" << equivalent
              << " acm=" << scene_acm_preserved << '\n';
    return 1;
  }

  std::cout << "METRIC MOVEIT_COLLISION_PROVIDER"
            << " collision_rejected=" << (collision_rejected ? 1 : 0)
            << " object_removed=" << (removed ? 1 : 0)
            << " collision_layer_cleared="
            << (collision_layer_cleared ? 1 : 0)
            << " pinned_scene_still_rejects="
            << (pinned_scene_still_rejects ? 1 : 0)
            << " live_scene_restored=" << (live_scene_restored ? 1 : 0)
            << " moving_vehicle_collision_rejected="
            << (moving_vehicle_collision_rejected ? 1 : 0)
            << " post_remove_result=" << after_removal.message << "\n";
  return collision_rejected && removed && collision_layer_cleared &&
                 pinned_scene_still_rejects && live_scene_restored &&
                 moving_vehicle_collision_rejected
             ? 0
             : 1;
}
