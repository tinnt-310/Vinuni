#include "fr10_insertion_manager/insertion_geometry_monitor.hpp"

#include <algorithm>
#include <cmath>
#include "fr10_motion_core/math_utils.hpp"

namespace fr10_insertion {
InsertionGeometryObservation observeInsertionGeometry(
    const fr10_force::CartesianPose &contact_pose,
    const fr10_force::Vector3 &axis_base,
    const fr10_force::CartesianPose &actual_pose,
    double lateral_tolerance_m, double orientation_tolerance_rad) {
  InsertionGeometryObservation result;
  fr10_force::RotationMatrix base_from_insertion{}, base_from_tcp{};
  if (!fr10_force::finite(contact_pose) || !fr10_force::finite(actual_pose) ||
      !fr10_force::finite(axis_base) ||
      std::abs(fr10_force::norm(axis_base) - 1.0) > 1e-6 ||
      !std::isfinite(lateral_tolerance_m) || lateral_tolerance_m <= 0.0 ||
      !std::isfinite(orientation_tolerance_rad) || orientation_tolerance_rad <= 0.0 ||
      !fr10_motion::math::quaternionToRotationMatrix(contact_pose.orientation_xyzw, base_from_insertion) ||
      !fr10_motion::math::quaternionToRotationMatrix(actual_pose.orientation_xyzw, base_from_tcp)) return result;
  fr10_force::Vector3 delta{};
  for (std::size_t i=0; i<3; ++i) {
    delta[i] = actual_pose.position_m[i] - contact_pose.position_m[i];
    result.depth_m += delta[i] * axis_base[i];
  }
  fr10_force::Vector3 lateral{};
  for (std::size_t i=0; i<3; ++i)
    lateral[i] = delta[i] - result.depth_m * axis_base[i];
  result.lateral_error_m = fr10_force::norm(lateral);
  auto &r = result.insertion_from_tcp.rotation_target_from_source;
  for (std::size_t row=0; row<3; ++row)
    for (std::size_t column=0; column<3; ++column) {
      r[row*3+column] = 0.0;
      for (std::size_t k=0; k<3; ++k)
        r[row*3+column] += base_from_insertion[k*3+row] * base_from_tcp[k*3+column];
    }
  result.orientation_error_rad = fr10_motion::math::quaternionAngularDistance(
      contact_pose.orientation_xyzw, actual_pose.orientation_xyzw);
  result.valid = true;
  result.lateral_valid = result.lateral_error_m <= lateral_tolerance_m;
  result.orientation_valid = result.orientation_error_rad <= orientation_tolerance_rad;
  return result;
}

InsertionGeometryObservation observeInsertionGeometry(
    const InsertionCorridor &corridor,const fr10_force::CartesianPose &actual_pose,
    double lateral_tolerance_m,double orientation_tolerance_rad) {
  return observeInsertionGeometry(corridor.mouth_pose,corridor.axis_base,actual_pose,
      lateral_tolerance_m,orientation_tolerance_rad);
}

bool withinMatingClearance(const InsertionGeometryObservation &observation,
                          double clearance_m, double feature_lever_m) {
  return observation.valid && std::isfinite(clearance_m) && clearance_m > 0.0 &&
      std::isfinite(feature_lever_m) && feature_lever_m > 0.0 &&
      std::isfinite(observation.lateral_error_m) &&
      std::isfinite(observation.orientation_error_rad) &&
      observation.lateral_error_m +
          2.0 * feature_lever_m * std::sin(observation.orientation_error_rad / 2.0) <= clearance_m;
}

} // namespace fr10_insertion
