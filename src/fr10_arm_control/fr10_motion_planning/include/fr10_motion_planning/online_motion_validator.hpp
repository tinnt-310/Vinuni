#pragma once
#include "fr10_motion_core/cartesian_route_planner.hpp"
#include "fr10_motion_core/continuous_ik_stepper.hpp"
#include "fr10_motion_core/joint_otg.hpp"

namespace fr10_motion_planning {
struct OnlineMotionValidationStatistics {
  unsigned transition_samples{0};
  unsigned braking_samples{0};
};

enum class ForceCorridorContract { FIXED_NOMINAL, CONTACT_ADAPTIVE };

struct ForceCorridorValidationConfig {
  double preinsert_distance_m{.03};
  double insertion_depth_m{.015};
  double axial_correction_m{.002};
  double depth_tolerance_m{.0005};
  double sample_step_m{.001};
  std::size_t max_waypoints{2000};
  double axial_speed_m_s{.000525};
  double period_s{.008};
  double max_reference_lead_rad{.017};
  fr10_motion::ContinuousIKConfig continuous_ik;
  fr10_motion::KinematicLimits limits;
};

struct ForceCorridorValidationResult {
  bool valid{false};
  std::size_t pose_samples{0};
  std::size_t braking_samples{0};
  double collision_end_depth_m{0.};
  double failure_depth_m{0.};
  std::string detail;
};

// Fixed nominal insertion screens the complete ray with the scene. Adaptive
// insertion screens complete branch kinematics, but collision admission only
// covers the clearance/entry ray. Contact-corrected motion is not known yet:
// every subsequent target and controlled stop still requires online screening.
ForceCorridorValidationResult validateForceAdmissionCorridor(
    const fr10_motion::Pose &contact, const std::array<double, 3> &axis,
    const fr10_motion::KinematicState &preinsert_seed,
    fr10_motion::IKProvider &kinematics, fr10_motion::IKProvider &collision,
    const ForceCorridorValidationConfig &config, ForceCorridorContract contract,
    const fr10_motion::CartesianRoutePlanner::CancelCheck &cancel = {});
// Same sampled jerk-limited stop used by force corridor and online screening.
bool validateControlledStop(const fr10_motion::KinematicState &seed,
    fr10_motion::IKProvider &provider, const fr10_motion::KinematicLimits &limits,
    double period_s, double max_reference_lead_rad,
    const fr10_motion::CartesianRoutePlanner::CancelCheck &cancel = {},
    unsigned *sample_count = nullptr);

// Worker-side screening from a captured q/v/a boundary. Sensor/scene/command
// freshness remains the consumer's responsibility at activation.
bool validateOnlineMotion(const fr10_motion::OnlineJointTarget &target,
    const fr10_motion::KinematicState &seed, fr10_motion::IKProvider &provider,
    const fr10_motion::KinematicLimits &limits, double period_s,
    double max_reference_lead_rad,
    const fr10_motion::CartesianRoutePlanner::CancelCheck &cancel = {},
    OnlineMotionValidationStatistics *statistics = nullptr);
}
