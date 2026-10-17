#pragma once

#include <memory>
#include <string>
#include <vector>
#include <utility>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include "fr10_motion_core/ik_provider.hpp"

namespace fr10_motion_planning {

struct MoveItIKProviderConfig {
  std::string group_name{"fairino10_v6_group"};
  std::string tip_link{"charging_tcp"};
  std::string base_link{"fr10_base_link"};
  std::string safety_link{"wrist3_link"};
  double timeout_s{0.05};
  double wrist_min_abs_rad{0.0};
  double elbow_min_abs_rad{0.0};
  double max_wrist_reach_m{1.30};
  double min_wrist_radius_m{0.200};
  double max_jacobian_condition{30.0};
  bool collision_checking_enabled{true};
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  // Deterministic/offline fixture path. Production uses the monitored scene.
  planning_scene::PlanningSceneConstPtr planning_scene;
  fr10_motion::KinematicLimits limits;
};

class MoveItIKProvider final : public fr10_motion::IKProvider {
public:
  MoveItIKProvider(const moveit::core::RobotModelPtr &robot_model,
                   MoveItIKProviderConfig config);

  fr10_motion::IKResult solve(const fr10_motion::Pose &target,
                              const fr10_motion::KinematicState &seed) override;

  fr10_motion::IKResult
  validate(const fr10_motion::KinematicState &target,
           const fr10_motion::KinematicState &seed) override;

  bool forward(const fr10_motion::KinematicState &state,
               fr10_motion::Pose &pose,
               std::string *error = nullptr) const override;

  bool forwardLink(const fr10_motion::KinematicState &state,
                   const std::string &link_name, fr10_motion::Pose &pose,
                   std::string *error = nullptr) const;

  // Planning workers pin collision checks to one immutable scene. Passing an
  // empty pointer restores the configured live-monitor/offline-scene source.
  void setPlanningSceneSnapshot(
      planning_scene::PlanningSceneConstPtr planning_scene);

  // Clear geometry pairs with a provably fixed relative transform. No world,
  // attached body or movable-link pair can enter this cache.
  std::size_t invariantClearPairCount() const { return invariant_clear_pairs_.size(); }

private:
  void initializeInvariantClearPairs(const planning_scene::PlanningScene &scene);
  moveit::core::RobotState stateFromScene() const;

  bool validateState(moveit::core::RobotState &state,
                     const fr10_motion::KinematicState &seed,
                     const fr10_motion::KinematicState &target,
                     double &manipulability, double &condition_number,
                     std::string &error) const;

  moveit::core::RobotModelPtr robot_model_;
  const moveit::core::JointModelGroup *joint_model_group_{nullptr};
  MoveItIKProviderConfig config_;
  planning_scene::PlanningSceneConstPtr planning_scene_snapshot_;
  bool invariant_pairs_initialized_{false};
  std::vector<std::pair<std::string, std::string>> invariant_clear_pairs_;
  std::unique_ptr<collision_detection::AllowedCollisionMatrix> snapshot_acm_;
  // Worker-owned scratch state; immutable snapshots keep non-arm state fixed.
  std::unique_ptr<moveit::core::RobotState> validation_state_;
};

} // namespace fr10_motion_planning
