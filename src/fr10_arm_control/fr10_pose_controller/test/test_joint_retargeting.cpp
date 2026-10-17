#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

#include "fr10_motion_core/joint_otg.hpp"

int main() {
  constexpr double period_s = 0.008;
  fr10_motion::KinematicLimits limits;
  limits.max_joint_velocity_rad_s.fill(0.30);
  limits.max_joint_acceleration_rad_s2.fill(0.80);
  limits.max_joint_jerk_rad_s3.fill(4.0);

  fr10_motion::JointOtg otg(period_s, limits);
  std::array<double, 6> initial{};
  otg.reset(initial);

  fr10_motion::JointTarget target;
  target.position_rad = initial;
  target.position_rad[0] = 30.0 * fr10_motion::kDegToRad;
  target.revision = 1;
  if (!otg.submitTarget(target)) {
    return 1;
  }

  fr10_motion::RobotState feedback;
  std::array<double, 6> position{}, velocity{}, acceleration{};
  std::array<double, 6> previous_velocity{};
  double max_delta_velocity = 0.0;
  bool first_sample = true;

  for (int step = 1; step <= 82; ++step) {
    if (!otg.step(feedback, position, velocity, acceleration)) {
      return 1;
    }
    if (!first_sample) {
      max_delta_velocity = std::max(
          max_delta_velocity, std::abs(velocity[0] - previous_velocity[0]));
    }
    first_sample = false;
    previous_velocity = velocity;

    if (step == 38) {
      if (std::abs(velocity[0]) <= 0.10) {
        std::cerr << "J1 was not moving fast enough at t=0.304 s\n";
        return 1;
      }
      target.position_rad[0] = -30.0 * fr10_motion::kDegToRad;
      target.revision = 2;
      if (!otg.submitTarget(target)) {
        return 1;
      }
    } else if (step == 81) {
      target.position_rad[0] = 15.0 * fr10_motion::kDegToRad;
      target.revision = 3;
      if (!otg.submitTarget(target)) {
        return 1;
      }
    }
  }

  int steps = 82;
  while (!otg.isFinished() && steps < 2000) {
    if (!otg.step(feedback, position, velocity, acceleration)) {
      return 1;
    }
    max_delta_velocity = std::max(max_delta_velocity,
                                  std::abs(velocity[0] - previous_velocity[0]));
    previous_velocity = velocity;
    ++steps;
  }

  const double final_error =
      std::abs(position[0] - 15.0 * fr10_motion::kDegToRad);
  const double continuity_bound =
      limits.max_joint_acceleration_rad_s2[0] * period_s * 1.5;
  std::cout << "JOINT_RETARGET final_error_rad=" << final_error
            << " max_delta_qdot_rad_s=" << max_delta_velocity
            << " continuity_bound=" << continuity_bound << " samples=" << steps
            << "\n";
  return otg.isFinished() && final_error < 0.002 &&
                 max_delta_velocity <= continuity_bound
             ? 0
             : 1;
}
