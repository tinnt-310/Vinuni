#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "fr10_force_control/force_position_controller.hpp"
#include "fr10_force_control/force_safety_monitor.hpp"
#include "fr10_motion_core/math_utils.hpp"

namespace {

bool check(bool condition, const char *name) {
  if (!condition) std::cerr << "FAIL " << name << '\n';
  return condition;
}

bool near(double left, double right, double tolerance = 1e-12) {
  return std::abs(left - right) <= tolerance;
}

fr10_force::RotationMatrix identity() {
  return {1.0, 0.0, 0.0,
          0.0, 1.0, 0.0,
          0.0, 0.0, 1.0};
}

} // namespace

int main() {
  bool ok = true;
  {
    auto six = fr10_force::AdmittanceConfig{};
    six.selected_axes.fill(true);
    six.mass.fill(2.);
    six.damping.fill(3.);
    six.stiffness.fill(0.);
    six.max_correction = {.0075,.0075,.002,.10472,.10472,.10472};
    six.max_lateral_correction_norm_m = .0075;
    six.max_rotation_correction_norm_rad = .10472;
    six.max_velocity.fill(1.);
    six.max_acceleration.fill(100.);
    const fr10_force::RotationMatrix frame{0.,-1.,0.,1.,0.,0.,0.,0.,1.};
    fr10_force::CartesianPose tcp;
    tcp.position_m = {.4,-.2,.3};
    for (std::size_t axis = 0; axis < 6; ++axis) for (double sign : {-1.,1.}) {
      fr10_force::ForcePositionController one(six, frame);
      fr10_force::Vector6 sensed{};
      sensed[axis] = sign * (axis < 3 ? 1. : .05);
      const auto result = one.propose(tcp, {}, fr10_force::Wrench::fromVector(sensed), .008);
      ok &= check(result.valid && result.force_error[axis] == -sensed[axis] &&
          result.correction.position[axis]*sign < 0., "ALL_SIX_AXES_RESPOND_AGAINST_APPLIED_LOAD");
      const auto displacement = fr10_force::rotate(frame, {result.correction.position[0],
          result.correction.position[1],result.correction.position[2]});
      for (std::size_t i = 0; i < 3; ++i)
        ok &= check(near(result.corrected_pose.position_m[i]-tcp.position_m[i], displacement[i]),
            "SIX_AXIS_CORRECTION_USES_INSERTION_FRAME");
      const auto uncommitted = one.propose(tcp, {}, fr10_force::Wrench::fromVector(sensed), .008);
      ok &= check(uncommitted.correction.position == result.correction.position &&
          one.commit(result) && !one.commit(uncommitted), "SIX_AXIS_CORRECTION_ACCUMULATES_ONLY_ON_ADMISSION");
    }
    fr10_force::ForcePositionController pivot(six, frame);
    fr10_force::AdmittanceState tilted;
    tilted.correction[3] = 5. * 3.14159265358979323846 / 180.;
    pivot.reset(tilted);
    const auto rotated = pivot.propose(tcp, {}, {}, .008);
    ok &= check(rotated.valid && rotated.corrected_pose.position_m == tcp.position_m,
        "FIVE_DEGREE_ROTATION_PIVOTS_AT_TCP_WITHOUT_MOVING_TCP_ORIGIN");
  }
  fr10_force::AdmittanceConfig config;
  config.mass.fill(2.0);
  config.damping.fill(3.0);
  config.stiffness.fill(4.0);
  config.max_correction.fill(10.0);
  config.max_velocity.fill(10.0);
  config.max_acceleration.fill(100.0);
  config.selected_axes[2] = true;
  for (const auto &invalid_rotation : {
      fr10_force::RotationMatrix{},
      fr10_force::RotationMatrix{-1,0,0,0,1,0,0,0,1},
      fr10_force::RotationMatrix{1,0,0,1,0,0,0,0,1}}) {
    bool rejected = false;
    try {
      fr10_force::ForcePositionController invalid(config, invalid_rotation);
    } catch (const std::invalid_argument &) { rejected = true; }
    ok &= check(rejected, "INSERTION_FRAME_MUST_BE_PROPER_ROTATION");
  }
  fr10_force::AdmittanceModel model(config);

  fr10_force::Vector6 zero_error{};
  auto zero = model.propose(zero_error, 0.1);
  ok &= check(zero.valid && near(zero.output.position[2], 0.0) &&
                  near(zero.output.velocity[2], 0.0),
              "ZERO_ERROR_ZERO_STATE");

  fr10_force::AdmittanceState stored;
  stored.correction[2] = 0.1;
  model.reset(stored);
  zero = model.propose(zero_error, 0.1);
  ok &= check(zero.output.position[2] < stored.correction[2],
              "ZERO_ERROR_NONZERO_STATE_RETURNS_BY_STIFFNESS");
  model.reset();

  fr10_force::Vector6 error{};
  error[2] = 5.0;
  const auto proposal = model.propose(error, 0.1);
  const double expected_velocity = 0.1 * 5.0 /
      (2.0 + 0.1 * 3.0 + 0.1 * 0.1 * 4.0);
  ok &= check(near(proposal.output.velocity[2], expected_velocity) &&
                  near(proposal.output.position[2],
                       0.1 * expected_velocity),
              "IMPLICIT_EULER_CONSTANT_ERROR");
  ok &= check(near(model.state().correction[2], 0.0),
              "PROPOSAL_DOES_NOT_MUTATE_STATE");
  ok &= check(model.commit(proposal) &&
                  near(model.state().correction[2], proposal.output.position[2]),
              "COMMIT_ACCEPTED_TARGET_ONLY");
  ok &= check(near(proposal.output.position[0], 0.0),
              "DISABLED_AXES_ZERO");
  {
    fr10_force::AdmittanceModel variable(config);
    double x = 0.0, v = 0.0;
    for (double dt : {0.008, 0.004, 0.013, 0.001}) {
      v = (2.0*v + dt*(5.0-4.0*x))/(2.0+dt*3.0+dt*dt*4.0);
      x += dt*v;
      const auto next = variable.propose(error, dt);
      ok &= check(next.valid && near(next.output.position[2], x) &&
                      near(next.output.velocity[2], v) && variable.commit(next),
                  "VARIABLE_DT_IMPLICIT_EULER");
    }
    const auto handover = variable.state();
    fr10_force::AdmittanceModel resumed(config);
    resumed.reset(handover);
    ok &= check(near(resumed.propose(error, .008).output.position[2],
                     variable.propose(error, .008).output.position[2]),
                "RESET_HANDOVER_CONTINUITY");
    ok &= check(!resumed.propose(error, 0.0).valid &&
                    !resumed.propose(error, -0.008).valid &&
                    !resumed.propose(error, std::numeric_limits<double>::quiet_NaN()).valid,
                "INVALID_DT_DOES_NOT_INTEGRATE");
    error[2] = std::numeric_limits<double>::quiet_NaN();
    ok &= check(!resumed.propose(error, .008).valid, "NONFINITE_ERROR_REJECTED");
    error[2] = 5.0;
  }
  for (int invalid_parameter = 0; invalid_parameter < 4; ++invalid_parameter) {
    auto invalid = config;
    if (invalid_parameter == 0) invalid.mass[2] = 0.0;
    if (invalid_parameter == 1) invalid.damping[2] = -1.0;
    if (invalid_parameter == 2) invalid.stiffness[2] = -1.0;
    if (invalid_parameter == 3) invalid.mass[2] = std::numeric_limits<double>::infinity();
    bool rejected = false;
    try { fr10_force::AdmittanceModel invalid_model(invalid); }
    catch (const std::invalid_argument &) { rejected = true; }
    ok &= check(rejected, "INVALID_M_B_K_REJECTED");
  }

  fr10_force::AdmittanceConfig saturated_config = config;
  saturated_config.mass.fill(1.0);
  saturated_config.damping.fill(0.0);
  saturated_config.stiffness.fill(0.0);
  saturated_config.max_correction.fill(0.001);
  saturated_config.max_velocity.fill(0.01);
  saturated_config.max_acceleration.fill(0.1);
  fr10_force::AdmittanceModel saturated(saturated_config);
  error[2] = 1000.0;
  auto saturated_result = saturated.propose(error, 1.0);
  ok &= check(saturated_result.position_saturated[2] &&
                  saturated_result.velocity_saturated[2] &&
                  saturated_result.acceleration_saturated[2] &&
                  saturated_result.output.position[2] > 0.00099 &&
                  saturated_result.output.position[2] < 0.001 &&
                  near(saturated_result.output.acceleration[2],
                       saturated_result.output.velocity[2]),
              "SATURATION_BRAKES_BEFORE_CORRECTION_BOUNDARY");

  config.stiffness.fill(0.0);
  fr10_force::ForcePositionController controller(config, identity());
  fr10_force::CartesianPose nominal;
  nominal.position_m = {0.4, 0.0, 0.5};
  fr10_force::Wrench desired;
  desired.force_n[2] = 10.0;
  fr10_force::Wrench measured;
  measured.force_n[2] = 4.0;
  const auto controlled = controller.propose(nominal, desired, measured, 0.008);
  ok &= check(controlled.valid && near(controlled.force_error[2], 6.0) &&
                  controlled.corrected_pose.position_m[2] >
                      nominal.position_m[2],
              "FORCE_ERROR_SIGN_AND_POSE_CORRECTION");
  ok &= check(controller.commit(controlled), "CONTROLLER_COMMIT");

  fr10_force::AdmittanceConfig angular_config = config;
  angular_config.selected_axes.fill(false);
  angular_config.selected_axes[5] = true;
  fr10_force::ForcePositionController angular(angular_config, identity());
  desired = {};
  measured = {};
  desired.torque_nm[2] = 1.0;
  const auto angular_result = angular.propose(nominal, desired, measured, 0.01);
  ok &= check(angular_result.valid &&
                  std::abs(angular_result.corrected_pose.orientation_xyzw[2]) >
                      0.0 &&
                  near(angular_result.corrected_pose.orientation_xyzw[0], 0.0),
              "ROTATION_VECTOR_EXPONENTIAL_MAP");
  {
    auto six_config = saturated_config;
    six_config.selected_axes.fill(true);
    fr10_force::AdmittanceModel six(six_config);
    fr10_force::Vector6 demand{1000,-1000,1000,-1000,1000,-1000};
    const auto limited = six.propose(demand, 1.0);
    for (std::size_t axis = 0; axis < 6; ++axis) {
      ok &= check(limited.position_saturated[axis] &&
                      limited.output.position[axis] * (axis%2 ? -1. : 1.) > .00099 &&
                      std::abs(limited.output.position[axis]) < .001,
                  "SIX_AXES_BRAKE_WITHIN_CORRECTION_ENVELOPE");
    }
    six.commit(limited);
    for (int i = 0; i < 80; ++i) six.commit(six.propose(demand, .008));
    demand.fill(0.0);
    ok &= check(near(six.propose(demand, .008).output.position[5], -.001),
                "ZERO_ERROR_ZERO_K_RETAINS_CORRECTION");
  }
  {
    auto gains = saturated_config;
    gains.selected_axes.fill(true);
    fr10_force::AdmittanceModel bounded(gains);
    for (int i = 0; i < 3000; ++i) {
      const double dt = i%3 == 0 ? .048 : .008;
      fr10_force::Vector6 drive;
      drive.fill(i%700 < 350 ? 1000. : -1000.);
      const auto previous = bounded.state();
      const auto step = bounded.propose(drive, dt);
      ok &= check(step.valid && bounded.commit(step), "BOUNDARY_REVERSAL_ACCEPTED");
      for (std::size_t axis = 0; axis < 6; ++axis) {
        const double acceleration = (step.output.velocity[axis] - previous.velocity[axis])/dt;
        ok &= check(std::abs(step.output.position[axis]) <= gains.max_correction[axis] &&
            std::abs(acceleration) <= gains.max_acceleration[axis] + 1e-10 &&
            near(step.output.acceleration[axis], acceleration) &&
            near(step.output.position[axis] - previous.correction[axis], dt*step.output.velocity[axis]),
            "BOUNDARY_POSITION_VELOCITY_ACCELERATION_ARE_CONSISTENT");
      }
    }
    auto impossible = bounded.state(); impossible.correction[2] = .001; impossible.velocity[2] = .01;
    bool rejected = false;
    try { bounded.reset(impossible); } catch (const std::invalid_argument &) { rejected = true; }
    ok &= check(rejected, "INFEASIBLE_HANDOVER_REJECTED_WITHOUT_INSTANT_VELOCITY_RESET");
  }
  {
    auto gains = config;
    gains.selected_axes.fill(true);
    gains.stiffness.fill(4.0);
    gains.drive_deadband.fill(.5);
    fr10_force::AdmittanceModel quiet(gains);
    fr10_force::AdmittanceState equilibrium;
    equilibrium.correction.fill(.1);
    quiet.reset(equilibrium);
    fr10_force::Vector6 drive{.8,0.,.9,-.1,.4,.7};
    const auto retained = quiet.propose(drive,.008);
    for (std::size_t axis=0;axis<6;++axis)
      ok &= check(retained.valid && near(retained.output.position[axis],.1) &&
          near(retained.output.velocity[axis],0.), "NET_DRIVE_DEADBAND_PRESERVES_LOADED_EQUILIBRIUM");
    drive[0]=1.;drive[1]=-.2;
    const auto active=quiet.propose(drive,.008);
    ok &= check(active.output.velocity[0]>0. && active.output.velocity[1]<0.,
                "DEADBAND_SIGN_SYMMETRIC_OUTSIDE_EDGE");
    equilibrium.velocity.fill(.01);quiet.reset(equilibrium);drive.fill(.4);
    const auto damping=quiet.propose(drive,.008);
    ok &= check(damping.output.velocity[0]>0. && damping.output.velocity[0]<.01,
                "DEADBAND_DAMPS_LIVE_VELOCITY_WITHOUT_DISCONTINUITY");
    gains.drive_deadband[0]=-.1;
    bool rejected=false;
    try {fr10_force::AdmittanceModel invalid(gains);} catch(const std::invalid_argument&){rejected=true;}
    ok &= check(rejected,"NEGATIVE_DRIVE_DEADBAND_REJECTED");
    gains.drive_deadband[0]=std::numeric_limits<double>::quiet_NaN();rejected=false;
    try {fr10_force::AdmittanceModel invalid(gains);} catch(const std::invalid_argument&){rejected=true;}
    ok &= check(rejected,"NONFINITE_DRIVE_DEADBAND_REJECTED");
    gains.drive_deadband.fill(.5);gains.stiffness.fill(0.);
    fr10_force::ForcePositionController unmasked(gains,identity());
    desired={};measured={};desired.force_n[2]=5.;measured.force_n[2]=4.7;
    auto held=unmasked.propose(nominal,desired,measured,.008);
    ok &= check(held.valid && near(held.correction.position[2],0.) && near(held.force_error[2],.3),
                "DEADBAND_NEVER_MASKS_VERIFICATION_FORCE_ERROR");
  }
  {
    fr10_force::RawWrenchEnvelope envelope;
    envelope.observe({{60,-70,80},{11,-12,13}});
    envelope.observe({{1,2,3},{.1,.2,.3}});
    ok &= check(envelope.valid && envelope.populated &&
        envelope.peak.force_n==fr10_force::Vector3{60,-70,80} &&
        envelope.peak.torque_nm==fr10_force::Vector3{11,-12,13},
        "OVERSAMPLED_IMPULSE_RETAINED_FOR_RAW_SAFETY");
    fr10_force::ForceSafetyConfig safety_config;
    safety_config.max_force_n.fill(50.);safety_config.max_torque_nm.fill(10.);
    fr10_force::ForceSafetyMonitor safety(safety_config);
    fr10_force::ForceSafetyInput safety_input;
    safety_input.raw_sample={envelope.peak,1.,1.,1,envelope.valid,false};
    safety_input.now_steady_s=1.;
    ok &= check(safety.update(safety_input).fault==fr10_force::ForceSafetyFault::FORCE_LIMIT,
        "SHORT_RAW_IMPULSE_FAULTS_DESPITE_NORMAL_CONDITIONED_LOAD");
    auto invalid=fr10_force::Wrench{};invalid.force_n[0]=std::numeric_limits<double>::quiet_NaN();
    envelope.observe(invalid);envelope.observe({});
    ok &= check(!envelope.valid,"BRIEF_NONFINITE_PACKET_CANNOT_BE_OVERWRITTEN");
    envelope.reset();envelope.observe({{1,2,3},{.1,.2,.3}});
    ok &= check(envelope.valid && envelope.peak.force_n[0]==1,
        "RAW_ENVELOPE_CONSUMPTION_CLEARS_ONLY_PREVIOUS_INTERVAL");
  }
  return ok ? 0 : 1;
}
