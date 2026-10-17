#pragma once

#include <array>
#include <cstddef>
#include <functional>
#include <string>
#include <vector>

#include "fr10_motion_core/cartesian_path.hpp"
#include "fr10_motion_core/continuous_ik.hpp"

namespace fr10_motion {

enum class CartesianRouteMode : int { CARTESIAN_LINEAR = 0, FREE_SPACE_JOINT };

struct CartesianRoutePlannerConfig {
  CartesianRouteMode mode{CartesianRouteMode::CARTESIAN_LINEAR};
  CartesianPathConfig path;
  ContinuousIKConfig continuous_ik;
  bool singularity_avoidance_enabled{true};
  std::vector<double> detour_clearances_m{0.05, 0.10, 0.15};
  double joint_space_step_rad{0.03};
  double cylindrical_barrier_radius_m{0.200};
  double cylindrical_detour_radius_m{0.350};
  KinematicLimits limits;
};

struct CartesianRoutePlan {
  bool success{false};
  bool free_space_joint_route{false};
  bool used_detour{false};
  bool used_alternate_orientation{false};
  bool used_joint_space_detour{false};
  bool used_cylindrical_detour{false};
  std::size_t cylindrical_detour_waypoints{0};
  std::vector<Pose> cartesian_path;
  ContinuousIKResult continuous_ik;
  std::size_t direct_failure_index{0};
  std::size_t detour_attempts{0};
  std::size_t best_detour_failure_index{0};
  double best_detour_failure_fraction{0.0};
  double detour_clearance_m{0.0};
  std::array<double, 3> detour_direction{};
  std::string direct_failure_message;
  std::string best_detour_failure_message;
  std::string message;
};

// Plans a dense straight Cartesian route first. If continuous IK rejects an
// intermediate waypoint, it searches deterministic smooth positional detours
// without weakening any IK/singularity barrier. This is kinematic avoidance;
// it is not collision-scene planning.
class CartesianRoutePlanner {
public:
  using CancelCheck = std::function<bool()>;

  // Materialize and independently validate an externally planned joint route.
  static CartesianRoutePlan fromJointWaypoints(
      const std::vector<KinematicState> &waypoints, const Pose &goal,
      IKProvider &provider, const CartesianRoutePlannerConfig &config,
      const CancelCheck &cancel = {});

  static CartesianRoutePlan plan(const Pose &start, const Pose &goal,
                                 const KinematicState &seed,
                                 IKProvider &provider,
                                 const CartesianRoutePlannerConfig &config,
                                 const CancelCheck &cancel = {});
};

} // namespace fr10_motion
