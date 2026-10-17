#include "fr10_motion_planning/online_motion_validator.hpp"
#include "fr10_motion_core/math_utils.hpp"

namespace fr10_motion_planning {
using namespace fr10_motion;
namespace {
bool sampleControlledStop(const KinematicState &, IKProvider &, const KinematicLimits &,
    double, double, const CartesianRoutePlanner::CancelCheck &, unsigned *);
}

ForceCorridorValidationResult validateForceAdmissionCorridor(
    const Pose &contact, const std::array<double, 3> &axis,
    const KinematicState &preinsert_seed, IKProvider &kinematics, IKProvider &collision,
    const ForceCorridorValidationConfig &config, ForceCorridorContract contract,
    const CartesianRoutePlanner::CancelCheck &cancel) {
  ForceCorridorValidationResult result;
  result.detail = "invalid force admission configuration";
  for (double value : {config.preinsert_distance_m, config.insertion_depth_m,
      config.axial_correction_m, config.depth_tolerance_m, config.sample_step_m,
      config.axial_speed_m_s, config.period_s, config.max_reference_lead_rad})
    if (!std::isfinite(value) || value <= 0.) return result;
  if (config.max_waypoints < 2 || std::abs(math::norm3(axis)-1.) > 1e-6 ||
      !std::isfinite(math::norm3(axis))) return result;
  const double start = -config.preinsert_distance_m - config.axial_correction_m;
  const double full_end = config.insertion_depth_m + config.axial_correction_m + config.depth_tolerance_m;
  result.collision_end_depth_m = contract == ForceCorridorContract::CONTACT_ADAPTIVE ? 0. : full_end;
  ContinuousIKStepper stepper;
  try { stepper.setConfig(config.continuous_ik); }
  catch (const std::invalid_argument &) { return result; }
  const auto screen = [&](double end, IKProvider &provider) {
      const double count = std::ceil((end-start)/config.sample_step_m);
      if (!std::isfinite(count) || count < 1. || count+1 > config.max_waypoints) return false;
      const auto samples = static_cast<std::size_t>(count);
      auto previous = preinsert_seed;
      previous.velocity_rad_s.fill(0.); previous.acceleration_rad_s2.fill(0.);
      for (std::size_t index=0; index<=samples; ++index) {
        if (cancel && cancel()) { result.detail = "admission canceled"; return false; }
        auto pose = contact;
        const double depth = start + (end-start)*index/samples;
        result.failure_depth_m = depth;
        for (std::size_t i=0; i<3; ++i) pose.position_m[i] += axis[i]*depth;
        ++result.pose_samples;
        const auto candidate = stepper.step(pose, previous, provider);
        if (!candidate.success) {
          result.detail = "admission IK/scene rejected: " + std::to_string(static_cast<int>(candidate.failure));
          return false;
        }
        if (index > 0) {
          const double segment = (end-start)/samples;
          for (double sign : {-1., 1.}) {
            auto braking_seed = candidate.state;
            for (std::size_t joint=0; joint<kJointCount; ++joint) {
              braking_seed.velocity_rad_s[joint] = sign * config.axial_speed_m_s *
                  (candidate.state.position_rad[joint]-previous.position_rad[joint])/segment;
              braking_seed.acceleration_rad_s2[joint] = 0.;
            }
            unsigned count = 0;
            if (!validateControlledStop(braking_seed, provider, config.limits, config.period_s,
                config.max_reference_lead_rad, cancel, &count)) {
              result.detail = "admission controlled-stop rejected"; return false;
            }
            result.braking_samples += count;
          }
        }
        previous = candidate.state;
      }
      return true;
    };
  if (contract == ForceCorridorContract::CONTACT_ADAPTIVE && !screen(full_end, kinematics)) return result;
  if (!screen(result.collision_end_depth_m, collision)) return result;
  result.valid = true; result.detail = "force admission validated";
  return result;
}


bool validateOnlineMotion(const OnlineJointTarget &target,
    const KinematicState &seed, IKProvider &provider,
    const KinematicLimits &limits, double period_s,
    double max_reference_lead_rad,
    const CartesianRoutePlanner::CancelCheck &cancel,
    OnlineMotionValidationStatistics *statistics) {
  if(statistics)*statistics={};
  if ((cancel && cancel()) || !provider.validate(seed, seed).success) return false;
  JointTarget joint;
  joint.position_rad = target.position_rad;
  joint.target_velocity_rad_s = target.target_velocity_rad_s;
  joint.target_acceleration_rad_s2 = target.target_acceleration_rad_s2;
  joint.velocity_scale = target.velocity_scale;
  joint.acceleration_scale = target.acceleration_scale;
  joint.jerk_scale = target.jerk_scale;
  joint.revision = target.sequence;
  joint.timestamp_s = target.timestamp_s;
  JointOtg otg(period_s, limits, max_reference_lead_rad);
  otg.reset(seed.position_rad, seed.velocity_rad_s, seed.acceleration_rad_s2);
  if (!otg.submitTarget(joint)) return false;
  auto last = seed;
  RobotState feedback; feedback.valid = true;
  bool finished = false;
  for (unsigned tick = 0; tick < 1000; ++tick) {
    if (statistics) ++statistics->transition_samples;
    if (cancel && cancel()) return false;
    feedback.joint_position_rad = last.position_rad;
    auto next = last;
    if (!otg.step(feedback, next.position_rad, next.velocity_rad_s,
                  next.acceleration_rad_s2) ||
        !provider.validate(next, last).success) return false;
    last = next;
    if (otg.isFinished()) { finished = true; break; }
  }
  return finished && sampleControlledStop(seed, provider, limits, period_s,
      max_reference_lead_rad, cancel, statistics ? &statistics->braking_samples : nullptr);
}

bool validateControlledStop(const KinematicState &seed, IKProvider &provider,
    const KinematicLimits &limits, double period_s, double max_reference_lead_rad,
    const CartesianRoutePlanner::CancelCheck &cancel, unsigned *sample_count) {
  if (sample_count) *sample_count = 0;
  if ((cancel && cancel()) || !provider.validate(seed, seed).success) return false;
  return sampleControlledStop(seed,provider,limits,period_s,max_reference_lead_rad,cancel,sample_count);
}

namespace {
bool sampleControlledStop(const KinematicState &seed, IKProvider &provider,
    const KinematicLimits &limits, double period_s, double max_reference_lead_rad,
    const CartesianRoutePlanner::CancelCheck &cancel, unsigned *sample_count) {
  if (sample_count) *sample_count = 0;
  JointOtg brake(period_s, limits, max_reference_lead_rad);
  brake.reset(seed.position_rad, seed.velocity_rad_s, seed.acceleration_rad_s2);
  if (!brake.startControlledStop()) return false;
  auto last = seed;
  RobotState feedback; feedback.valid = true;
  for (unsigned tick = 0; tick < 1000; ++tick) {
    if (sample_count) ++*sample_count;
    if (cancel && cancel()) return false;
    feedback.joint_position_rad = last.position_rad;
    auto next = last;
    if (!brake.step(feedback, next.position_rad, next.velocity_rad_s,
                    next.acceleration_rad_s2) ||
        !provider.validate(next, last).success) return false;
    last = next;
    if (brake.isFinished()) return !(cancel && cancel());
  }
  return false;
}
}
}
