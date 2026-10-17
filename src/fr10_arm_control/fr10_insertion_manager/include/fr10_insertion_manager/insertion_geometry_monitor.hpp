#pragma once

#include "fr10_force_control/force_types.hpp"

namespace fr10_insertion {

struct InsertionCorridor {
  fr10_force::CartesianPose mouth_pose;
  fr10_force::Vector3 axis_base{};
};

struct InsertionGeometryObservation {
  fr10_force::RigidTransform insertion_from_tcp;
  double depth_m{0.0};
  double lateral_error_m{0.0};
  double orientation_error_rad{0.0};
  bool valid{false};
  bool lateral_valid{false};
  bool orientation_valid{false};
};

// Fixed-size measured feedback only. No IK, ROS, TF lookup or collision call.
InsertionGeometryObservation observeInsertionGeometry(
    const fr10_force::CartesianPose &contact_pose,
    const fr10_force::Vector3 &axis_base,
    const fr10_force::CartesianPose &actual_pose,
    double lateral_tolerance_m, double orientation_tolerance_rad);

InsertionGeometryObservation observeInsertionGeometry(
    const InsertionCorridor &corridor,const fr10_force::CartesianPose &actual_pose,
    double lateral_tolerance_m,double orientation_tolerance_rad);

// Conservative displacement budget at every mating feature: translation
// and rotation consume the SAME CAD clearance, not independent tolerances.
bool withinMatingClearance(const InsertionGeometryObservation &observation,
                          double clearance_m, double feature_lever_m);

} // namespace fr10_insertion
