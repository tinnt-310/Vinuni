#include <iostream>
#include "fr10_motion_planning/online_motion_validator.hpp"

using namespace fr10_motion;
class Obstacle final : public IKProvider {
public:
  bool blocked{false};
  unsigned moving_samples{0};
  IKResult solve(const Pose &, const KinematicState &) override { return {}; }
  IKResult validate(const KinematicState &q, const KinematicState &) override {
    IKResult result; result.state = q;
    result.success = !blocked || q.position_rad[0] < .03 || q.position_rad[0] > .07;
    if (q.velocity_rad_s[0] != 0.) ++moving_samples;
    return result;
  }
  bool forward(const KinematicState &q, Pose &pose, std::string *) const override {
    pose.position_m[0] = q.position_rad[0]; return true;
  }
};

class CorridorModel final : public IKProvider {
public:
  double blocked_at_m{1.};
  IKResult solve(const Pose &pose, const KinematicState &seed) override {
    auto target = seed; target.position_rad[0] = pose.position_m[2];
    return validate(target, seed);
  }
  IKResult validate(const KinematicState &target, const KinematicState &) override {
    IKResult result; result.state = target; result.success = target.position_rad[0] < blocked_at_m;
    result.manipulability = result.condition_number = 1.; return result;
  }
  bool forward(const KinematicState &state, Pose &pose, std::string *) const override {
    pose = {}; pose.position_m[2] = state.position_rad[0]; return true;
  }
};
int main() {
  Obstacle provider; KinematicState seed; KinematicLimits limits;
  limits.max_joint_velocity_rad_s.fill(1.);
  limits.max_joint_acceleration_rad_s2.fill(2.);
  limits.max_joint_jerk_rad_s3.fill(10.);
  OnlineJointTarget target; target.position_rad[0] = .1; target.sequence = 1;
  if (!fr10_motion_planning::validateOnlineMotion(target, seed, provider, limits, .008, .017) ||
      provider.moving_samples == 0) return 1;
  provider.blocked = true;
  if (fr10_motion_planning::validateOnlineMotion(target, seed, provider, limits, .008, .017)) return 2;
  provider.blocked = false;
  if (fr10_motion_planning::validateOnlineMotion(target, seed, provider, limits, .008, .017,
                                                [] { return true; })) return 3;
  seed.velocity_rad_s[0] = .05; seed.acceleration_rad_s2[0] = .01;
  if (!fr10_motion_planning::validateOnlineMotion(target, seed, provider, limits, .008, .017)) return 4;
  unsigned samples = 0;
  if (!fr10_motion_planning::validateControlledStop(seed,provider,limits,.008,.017,{},&samples) || !samples) return 5;
  provider.blocked = true;
  seed.position_rad[0] = .02; seed.velocity_rad_s[0] = .3;
  if (fr10_motion_planning::validateControlledStop(seed,provider,limits,.008,.017)) return 6;
  provider.blocked = false;
  if (fr10_motion_planning::validateControlledStop(seed,provider,limits,.008,.017,[]{return true;})) return 7;
  {
    using namespace fr10_motion_planning;
    CorridorModel kinematics, scene; scene.blocked_at_m = .017;
    ForceCorridorValidationConfig config; config.limits = limits;
    KinematicState preinsert; preinsert.position_rad[0] = -.03;
    const auto check = [&](ForceCorridorContract contract) {
        return validateForceAdmissionCorridor({}, {0.,0.,1.}, preinsert,
            kinematics, scene, config, contract);
      };
    if (check(ForceCorridorContract::FIXED_NOMINAL).valid) return 8;
    const auto adaptive = check(ForceCorridorContract::CONTACT_ADAPTIVE);
    if (!adaptive.valid || adaptive.collision_end_depth_m != 0. ||
        !adaptive.pose_samples || !adaptive.braking_samples) return 9;
    // Admission is not authorization to execute the blocked original ray.
    OnlineJointTarget corrected; corrected.sequence = 1; corrected.position_rad[0] = .0175;
    if (validateOnlineMotion(corrected, preinsert, scene, limits, .008, .017)) return 10;
    scene.blocked_at_m = -.005;
    if (check(ForceCorridorContract::CONTACT_ADAPTIVE).valid) return 11;
    scene.blocked_at_m = 1.; kinematics.blocked_at_m = .012;
    if (check(ForceCorridorContract::CONTACT_ADAPTIVE).valid) return 12;
    kinematics.blocked_at_m = 1.;
    if (validateForceAdmissionCorridor({}, {0.,0.,1.}, preinsert, kinematics, scene,
        config, ForceCorridorContract::CONTACT_ADAPTIVE, [] { return true; }).valid) return 13;
    config.sample_step_m = 0.;
    if (check(ForceCorridorContract::CONTACT_ADAPTIVE).valid) return 14;
    config.sample_step_m = .001; config.max_waypoints = 2;
    if (check(ForceCorridorContract::CONTACT_ADAPTIVE).valid) return 15;
  }
  std::cout << "Full online transition, live derivatives, obstacle and cancellation PASS\n";
}
