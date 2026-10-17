#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>
#include "fr10_force_control/force_position_controller.hpp"
#include "fr10_motion_core/fr10_analytical_ik.hpp"
#include "fr10_motion_core/continuous_ik_stepper.hpp"

int main() {
  fr10_motion::Fr10AnalyticalIKConfig geometry;
  // Synthetic benchmark geometry, deliberately distinct from the deployment
  // tool calibration, which is resolved from robot_description at runtime.
  geometry.wrist3_to_tip = {{0.02,-0.03,0.22},{0.0,0.0,1.0,0.0}};
  geometry.wrist_min_abs_rad = 0.13962634015954636;
  geometry.elbow_min_abs_rad = 0.2617993877991494;
  geometry.max_wrist_reach_m = 1.65;
  geometry.limits.min_joint_position_rad.fill(-2.0*std::acos(-1.0));
  geometry.limits.max_joint_position_rad.fill(2.0*std::acos(-1.0));
  fr10_motion::Fr10AnalyticalIKProvider provider(geometry);
  fr10_motion::KinematicState seed;
  seed.position_rad = {0,-std::acos(-1.0)/2,std::acos(-1.0)/2,
                       -std::acos(-1.0)/2,-std::acos(-1.0)/2,0};
  fr10_motion::Pose nominal;
  if (!provider.forward(seed, nominal)) return 1;
  fr10_force::AdmittanceConfig gains;
  gains.selected_axes[2] = true;
  gains.damping.fill(500.0);
  gains.max_correction.fill(0.001);
  gains.max_velocity.fill(0.001);
  fr10_force::ForcePositionController force(gains, {1,0,0,0,1,0,0,0,1});
  fr10_motion::ContinuousIKStepper stepper;
  fr10_force::CartesianPose pose{nominal.position_m, nominal.orientation_quat};
  fr10_force::Wrench desired, measured;
  std::vector<double> timings;
  timings.reserve(50000); // Outside timed path; no ROS/MoveIt/backend.
  for (int i=-1000; i<50000; ++i) {
    desired.force_n[2] = 0.5*std::sin(2*std::acos(-1.0)*i/500.0);
    const auto start = std::chrono::steady_clock::now();
    const auto proposal = force.propose(pose,desired,measured,0.008);
    const auto ik = stepper.step({proposal.corrected_pose.position_m,
                                 proposal.corrected_pose.orientation_xyzw},seed,provider);
    if (!proposal.valid || !ik.success || !force.commit(proposal)) {
      std::cerr << "FAIL analytical force pipeline at cycle " << i << '\n';
      return 1;
    }
    seed = ik.state;
    const double us = std::chrono::duration<double,std::micro>(
        std::chrono::steady_clock::now()-start).count();
    if (i>=0) timings.push_back(us);
  }
  double sum=0;
  for (double us : timings) sum+=us;
  std::sort(timings.begin(),timings.end());
  std::cout << "FORCE_ANALYTICAL_PIPELINE cycles=" << timings.size()
            << " mean_us=" << sum/timings.size()
            << " p99_us=" << timings[timings.size()*99/100]
            << " max_us=" << timings.back() << '\n';
  // This measures userspace observed latency, not a certified WCET.
  return timings.back() < 8000.0 ? 0 : 1;
}
