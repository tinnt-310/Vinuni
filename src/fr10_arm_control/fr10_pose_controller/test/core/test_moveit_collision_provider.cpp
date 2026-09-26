#include <array>
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

#include "fr10_pose_controller/moveit_ik_provider.hpp"

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
  <joint name="j1" type="revolute"><parent link="base_link"/><child link="link1"/><origin xyz="0.2 0 0"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="10" velocity="1"/></joint>
  <joint name="j2" type="revolute"><parent link="link1"/><child link="link2"/><origin xyz="0.2 0 0"/><axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="10" velocity="1"/></joint>
  <joint name="j3" type="revolute"><parent link="link2"/><child link="link3"/><origin xyz="0.2 0 0"/><axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="10" velocity="1"/></joint>
  <joint name="j4" type="revolute"><parent link="link3"/><child link="link4"/><origin xyz="0.2 0 0"/><axis xyz="1 0 0"/><limit lower="-3" upper="3" effort="10" velocity="1"/></joint>
  <joint name="j5" type="revolute"><parent link="link4"/><child link="link5"/><origin xyz="0.2 0 0"/><axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="10" velocity="1"/></joint>
  <joint name="j6" type="revolute"><parent link="link5"/><child link="tool0"/><origin xyz="0.2 0 0"/><axis xyz="1 0 0"/><limit lower="-3" upper="3" effort="10" velocity="1"/></joint>
</robot>)";

constexpr const char *kSrdf = R"(
<robot name="collision_fixture">
  <group name="arm"><chain base_link="base_link" tip_link="tool0"/></group>
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

  fr10_pose_controller::MoveItIKProviderConfig config;
  config.group_name = "arm";
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
  fr10_pose_controller::MoveItIKProvider provider(robot_model, config);
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

  std::cout << "METRIC MOVEIT_COLLISION_PROVIDER"
            << " collision_rejected=" << (collision_rejected ? 1 : 0)
            << " object_removed=" << (removed ? 1 : 0)
            << " collision_layer_cleared="
            << (collision_layer_cleared ? 1 : 0)
            << " pinned_scene_still_rejects="
            << (pinned_scene_still_rejects ? 1 : 0)
            << " live_scene_restored=" << (live_scene_restored ? 1 : 0)
            << " post_remove_result=" << after_removal.message << "\n";
  return collision_rejected && removed && collision_layer_cleared &&
                 pinned_scene_still_rejects && live_scene_restored
             ? 0
             : 1;
}
