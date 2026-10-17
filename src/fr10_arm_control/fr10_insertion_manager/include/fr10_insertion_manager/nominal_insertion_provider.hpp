#pragma once
#include <limits>

#include "fr10_force_control/force_types.hpp"

namespace fr10_insertion {

double boundedSeatingSpeed(double requested_speed_m_s, double remaining_depth_m,
    double nominal_lead_m, double correction_velocity_m_s, double correction_acceleration_m_s2);

class NominalInsertionProvider {
public:
  bool reset(const fr10_force::CartesianPose &final_pose,
             const fr10_force::Vector3 &axis_base,
             double preinsert_distance_m, double insertion_depth_m,
             double insertion_speed_m_s);

  fr10_force::CartesianPose advance(double dt_s, bool advance_motion,
      double maximum_depth_m=std::numeric_limits<double>::infinity());
  fr10_force::CartesianPose poseAtDepth(double depth_m) const;
  double nominalDepth() const { return nominal_depth_m_; }
  const fr10_force::Vector3 &axisBase() const { return axis_base_; }

private:
  fr10_force::CartesianPose final_pose_;
  fr10_force::Vector3 axis_base_{{0.0, 0.0, 1.0}};
  double preinsert_distance_m_{0.0};
  double insertion_depth_m_{0.0};
  double insertion_speed_m_s_{0.0};
  double nominal_depth_m_{0.0};
  bool initialized_{false};
};

} // namespace fr10_insertion
