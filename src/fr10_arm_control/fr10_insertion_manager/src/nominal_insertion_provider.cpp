#include "fr10_insertion_manager/nominal_insertion_provider.hpp"

#include <algorithm>
#include <cmath>

namespace fr10_insertion {

double boundedSeatingSpeed(double speed, double remaining, double lead,
    double correction_velocity, double correction_acceleration) {
  if(!std::isfinite(speed) || speed<=0. || !std::isfinite(remaining) ||
     !std::isfinite(lead) || lead<=0. || !std::isfinite(correction_velocity) ||
     correction_velocity<=0. || !std::isfinite(correction_acceleration) || correction_acceleration<=0.)
    return std::numeric_limits<double>::quiet_NaN();
  // Cover both acknowledged reference lead and kinematic stopping distance.
  // C1 speed taper avoids driving a constant air speed into the hard shoulder;
  // positive floor lets nominal arrival complete before force-only seating.
  const double window=2.*(lead+speed*speed/(2.*correction_acceleration));
  const double u=std::clamp(remaining/window,0.,1.);
  const double floor=std::min(speed,correction_velocity);
  return floor+(speed-floor)*u*u*(3.-2.*u);
}

bool NominalInsertionProvider::reset(
    const fr10_force::CartesianPose &final_pose,
    const fr10_force::Vector3 &axis_base, double preinsert_distance_m,
    double insertion_depth_m, double insertion_speed_m_s) {
  const double axis_norm = fr10_force::norm(axis_base);
  if (!fr10_force::finite(final_pose) || !fr10_force::finite(axis_base) ||
      !std::isfinite(preinsert_distance_m) ||
      !std::isfinite(insertion_depth_m) ||
      !std::isfinite(insertion_speed_m_s) || !(axis_norm > 1e-12) ||
      preinsert_distance_m < 0.0 || !(insertion_depth_m > 0.0) ||
      !(insertion_speed_m_s > 0.0)) {
    return false;
  }
  final_pose_ = final_pose;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    axis_base_[axis] = axis_base[axis] / axis_norm;
  }
  preinsert_distance_m_ = preinsert_distance_m;
  insertion_depth_m_ = insertion_depth_m;
  insertion_speed_m_s_ = insertion_speed_m_s;
  nominal_depth_m_ = -preinsert_distance_m_;
  initialized_ = true;
  return true;
}

fr10_force::CartesianPose NominalInsertionProvider::advance(
    double dt_s, bool advance_motion, double maximum_depth_m) {
  if (initialized_ && advance_motion && std::isfinite(dt_s) && dt_s > 0.0) {
    const double proposed = std::min(insertion_depth_m_, nominal_depth_m_ + insertion_speed_m_s_ * dt_s);
    // Freeze advance when feedback lags; never integrate an air-path target
    // millimetres into a blocked socket. Compliance may still unload contact.
    nominal_depth_m_ = std::max(nominal_depth_m_,std::min(proposed,maximum_depth_m));
  }
  return poseAtDepth(nominal_depth_m_);
}

fr10_force::CartesianPose NominalInsertionProvider::poseAtDepth(
    double depth_m) const {
  if (!initialized_ || !std::isfinite(depth_m)) {
    return {};
  }
  fr10_force::CartesianPose pose = final_pose_;
  const double distance_from_final = insertion_depth_m_ - depth_m;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    pose.position_m[axis] -= axis_base_[axis] * distance_from_final;
  }
  return pose;
}

} // namespace fr10_insertion
