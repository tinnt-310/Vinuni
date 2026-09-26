#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include "fr10_pose_controller/core_types.hpp"

namespace fr10_motion {

struct CartesianPathConfig {
  double max_translation_step_m{0.005};
  double max_rotation_step_rad{0.02};
  std::size_t max_waypoints{2000};
};

class CartesianPath {
public:
  static bool linear(const Pose &start, const Pose &goal,
                     const CartesianPathConfig &config,
                     std::vector<Pose> &waypoints,
                     std::string *error = nullptr);

  // Same linear translation, but follows the complementary quaternion arc.
  // It is a fallback for free-space reorientation when the shortest SLERP is
  // blocked by joint/singularity barriers.
  static bool alternateOrientation(const Pose &start, const Pose &goal,
                                   const CartesianPathConfig &config,
                                   std::vector<Pose> &waypoints,
                                   std::string *error = nullptr);

  // Creates a smooth positional bump around a failed point on the direct
  // route while preserving endpoint poses and quaternion SLERP. The offset is
  // expressed in base coordinates and reaches its maximum at peak_fraction.
  static bool detour(const Pose &start, const Pose &goal,
                     double peak_fraction,
                     const std::array<double, 3> &peak_offset_m,
                     const CartesianPathConfig &config,
                     std::vector<Pose> &waypoints,
                     std::string *error = nullptr);

  // Free-space reconfiguration route: translate to an offset staging point
  // with the initial orientation, rotate there, then translate to the goal
  // with the final orientation. Quintic easing gives zero first/second path
  // derivatives at both phase boundaries.
  static bool stagedDetour(const Pose &start, const Pose &goal,
                           double staging_fraction,
                           const std::array<double, 3> &staging_offset_m,
                           const CartesianPathConfig &config,
                           std::vector<Pose> &waypoints,
                           std::string *error = nullptr);

  static Pose interpolate(const Pose &start, const Pose &goal,
                          double fraction);

  // Interpolates a uniformly parameterized waypoint route. This is the same
  // path parameter used by the joint spline and scalar time law.
  static Pose interpolate(const std::vector<Pose> &waypoints,
                          double fraction);

  static std::array<double, 3>
  translationTangent(const std::vector<Pose> &waypoints, double fraction);

  static double translationDeviation(
      const Pose &reference, const std::array<double, 3> &unit_tangent,
      const Pose &sample);

  static double closestTranslationParameter(const Pose &start,
                                            const Pose &goal,
                                            const Pose &sample);

  static double translationDeviation(const Pose &start, const Pose &goal,
                                     const Pose &sample);

  static bool preInsertPose(const Pose &target,
                            const std::array<double, 3> &local_axis,
                            double distance_m, Pose &pre_insert,
                            std::string *error = nullptr);
};

} // namespace fr10_motion
