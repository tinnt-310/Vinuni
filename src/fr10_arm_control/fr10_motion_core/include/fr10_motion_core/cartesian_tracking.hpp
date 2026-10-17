#pragma once

#include <array>
#include <cmath>
#include <limits>

#include "fr10_motion_core/cartesian_path.hpp"
#include "fr10_motion_core/math_utils.hpp"
#include "fr10_motion_core/motion_controller.hpp"

namespace fr10_motion {

// One geometric definition for the live fault monitor and status telemetry.
// Invalid measurements propagate as infinity so the monitor can fail closed.
inline CartesianTrackingErrors cartesianTrackingErrors(
    const Pose &reference, const std::array<double, 3> &tangent,
    const Pose &actual) {
  const double invalid = std::numeric_limits<double>::infinity();
  for (double value : reference.position_m) {
    if (!std::isfinite(value)) return {invalid, invalid, invalid, invalid};
  }
  for (double value : actual.position_m) {
    if (!std::isfinite(value)) return {invalid, invalid, invalid, invalid};
  }
  for (double value : tangent) {
    if (!std::isfinite(value)) return {invalid, invalid, invalid, invalid};
  }
  for (double value : reference.orientation_quat) {
    if (!std::isfinite(value)) return {invalid, invalid, invalid, invalid};
  }
  for (double value : actual.orientation_quat) {
    if (!std::isfinite(value)) return {invalid, invalid, invalid, invalid};
  }

  double squared_error = 0.0;
  double projection = 0.0;
  double tangent_norm_squared = 0.0;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const double delta = actual.position_m[axis] - reference.position_m[axis];
    squared_error += delta * delta;
    projection += delta * tangent[axis];
    tangent_norm_squared += tangent[axis] * tangent[axis];
  }
  CartesianTrackingErrors errors;
  errors.position_error_m = std::sqrt(squared_error);
  errors.along_track_error_m = tangent_norm_squared > 1e-12
      ? std::abs(projection) / std::sqrt(tangent_norm_squared) : 0.0;
  errors.perpendicular_error_m =
      CartesianPath::translationDeviation(reference, tangent, actual);
  errors.orientation_error_rad = math::quaternionAngularDistance(
      actual.orientation_quat, reference.orientation_quat);
  if (!std::isfinite(errors.position_error_m) ||
      !std::isfinite(errors.along_track_error_m) ||
      !std::isfinite(errors.perpendicular_error_m) ||
      !std::isfinite(errors.orientation_error_rad)) {
    return {invalid, invalid, invalid, invalid};
  }
  return errors;
}

} // namespace fr10_motion
