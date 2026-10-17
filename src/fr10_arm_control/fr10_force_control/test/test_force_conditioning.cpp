#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "fr10_force_control/contact_detector.hpp"
#include "fr10_force_control/fake_wrench_source.hpp"
#include "fr10_force_control/force_safety_monitor.hpp"
#include "fr10_force_control/wrench_compensation.hpp"
#include "fr10_force_control/wrench_filter.hpp"
#include "fr10_force_control/wrench_transform.hpp"

namespace {

bool check(bool condition, const char *name) {
  if (!condition) std::cerr << "FAIL " << name << '\n';
  return condition;
}

bool near(double left, double right, double tolerance = 1e-10) {
  return std::abs(left - right) <= tolerance;
}

fr10_force::ForceSafetyInput safeInput() {
  fr10_force::ForceSafetyInput input;
  input.raw_sample.valid = true;
  input.raw_sample.sequence = 1;
  input.raw_sample.source_timestamp_s = 1.0;
  input.raw_sample.receipt_steady_s = 10.0;
  input.now_steady_s = 10.0;
  return input;
}

fr10_force::ForceSafetyConfig safetyConfig() {
  fr10_force::ForceSafetyConfig config;
  config.max_force_n.fill(10.0);
  config.max_torque_nm.fill(2.0);
  config.max_force_rate_n_s.fill(50.0);
  config.max_correction.fill(0.02);
  config.max_depth_m = 0.03;
  config.stale_timeout_s = 0.05;
  return config;
}

} // namespace

int main() {
  bool ok = true;
  fr10_force::RigidTransform transform;
  transform.rotation_target_from_source = {0.0, -1.0, 0.0,
                                            1.0, 0.0, 0.0,
                                            0.0, 0.0, 1.0};
  transform.target_to_source_m = {0.0, 0.0, 0.1};
  fr10_force::Wrench source;
  source.force_n = {1.0, 0.0, 0.0};
  const auto target = fr10_force::WrenchTransform::apply(source, transform);
  ok &= check(near(target.force_n[0], 0.0) && near(target.force_n[1], 1.0) &&
                  near(target.torque_nm[0], -0.1),
              "WRENCH_ROTATION_AND_MOMENT_ARM");

  fr10_force::WrenchFilter filter({10.0});
  fr10_force::WrenchSample sample;
  sample.valid = true;
  sample.sequence = 1;
  sample.source_timestamp_s = 1.0;
  sample.receipt_steady_s = 5.0;
  sample.wrench.force_n[0] = 4.0;
  const auto first = filter.update(sample, 0.01);
  ok &= check(first.accepted && near(first.sample.wrench.force_n[0], 4.0),
              "FILTER_INITIAL_SAMPLE");
  const auto duplicate = filter.update(sample, 0.01);
  ok &= check(!duplicate.accepted && duplicate.duplicate,
              "FILTER_DUPLICATE_REJECTED");
  sample.sequence = 2;
  sample.source_timestamp_s = 1.01;
  sample.wrench.force_n[0] = std::numeric_limits<double>::quiet_NaN();
  ok &= check(!filter.update(sample, 0.01).accepted,
              "FILTER_NAN_REJECTED_WITHOUT_COMMIT");
  {
    fr10_force::WrenchFilter step_filter({10.0});
    auto step = sample;
    step.wrench = {};
    step.sequence = 1;
    step.source_timestamp_s = 1.0;
    step_filter.update(step, .008);
    step.sequence++;
    step.source_timestamp_s += .008;
    step.wrench.force_n[0] = 10.0;
    const double alpha = .008/(1.0/(2.0*3.14159265358979323846*10.0)+.008);
    ok &= check(near(step_filter.update(step,.008).sample.wrench.force_n[0], 10*alpha),
                "FILTER_STEP_COEFFICIENT");
    for (int i=0; i<100; ++i) {
      ++step.sequence; step.source_timestamp_s += .008;
      step_filter.update(step, .008);
    }
    ++step.sequence; step.source_timestamp_s += .008;
    ok &= check(near(step_filter.update(step,.008).sample.wrench.force_n[0],10,1e-8),
                "FILTER_UNITY_DC_GAIN");
    double max_noise = 0.0;
    for (int i=0; i<100; ++i) {
      ++step.sequence; step.source_timestamp_s += .008;
      step.wrench.force_n[0] = 10 + (i%2 ? -1.0 : 1.0);
      const auto output = step_filter.update(step,.008);
      max_noise = std::max(max_noise,std::abs(output.sample.wrench.force_n[0]-10));
    }
    ok &= check(max_noise < .5, "FILTER_ALTERNATING_NOISE_ATTENUATION");
    step_filter.reset();
    ok &= check(near(step_filter.update(step,.008).sample.wrench.force_n[0],step.wrench.force_n[0]),
                "FILTER_RESET_REINITIALIZES_WITHOUT_OLD_BIAS");
  }

  fr10_force::WrenchCompensationConfig compensation_config;
  compensation_config.source_sign = -1.0;
  fr10_force::WrenchCompensator compensator(compensation_config);
  fr10_force::Wrench bias;
  bias.force_n[0] = 1.0;
  ok &= check(compensator.setBias(bias), "TARE_BIAS_ACCEPTED");
  fr10_force::Wrench raw;
  raw.force_n[0] = -5.0;
  ok &= check(near(compensator.apply(raw).force_n[0], 4.0),
              "SOURCE_SIGN_AND_TARE");
  bool gravity_failed_closed = false;
  try {
    fr10_force::WrenchCompensationConfig invalid_gravity;
    invalid_gravity.gravity_enabled = true;
    fr10_force::WrenchCompensator invalid(invalid_gravity);
    (void)invalid;
  } catch (const std::invalid_argument &) {
    gravity_failed_closed = true;
  }
  ok &= check(gravity_failed_closed, "GRAVITY_MISSING_MASS_FAILS_CLOSED");
  {
    fr10_force::WrenchCompensationConfig calibrated;
    calibrated.gravity_enabled = true;
    calibrated.gravity_calibrated = true;
    calibrated.tool_mass_kg = 2.0;
    calibrated.tool_cog_m = {0.1,0,0};
    fr10_force::WrenchCompensator gravity(calibrated);
    const fr10_force::RotationMatrix rotation{1,0,0,0,0,-1,0,1,0};
    const auto load = gravity.apply({{0,19.6133,0},{0,0,1.96133}}, rotation);
    ok &= check(fr10_force::norm(load.force_n) < 1e-10 &&
                    fr10_force::norm(load.torque_nm) < 1e-10,
                "GRAVITY_ROTATED_SOURCE_FRAME_AND_COG");
    calibrated.source_sign = -1.0;
    fr10_force::WrenchCompensator reversed(calibrated);
    fr10_force::Wrench native{{0,19.6133,0},{0,0,1.96133}};
    auto unloaded = reversed.apply(native, rotation);
    ok &= check(fr10_force::norm(unloaded.force_n) < 1e-10 &&
                    fr10_force::norm(unloaded.torque_nm) < 1e-10,
                "GRAVITY_SIGN_CONVERSION_APPLIES_TO_RAW_AND_MODELED_LOAD");
    native.force_n[0] = -3.0;
    fr10_force::Wrench canonical_bias; canonical_bias.force_n[0] = 1.0;
    reversed.setBias(canonical_bias);
    ok &= check(near(reversed.apply(native, rotation).force_n[0], 2.0),
                "GRAVITY_REVERSED_SOURCE_RETAINS_CONTACT_AND_CANONICAL_TARE");
    calibrated.gravity_calibrated = false;
    bool rejected = false;
    try { fr10_force::WrenchCompensator uncalibrated(calibrated); }
    catch (const std::invalid_argument &) { rejected = true; }
    ok &= check(rejected, "GRAVITY_UNCALIBRATED_FAILS_CLOSED");
  }

  fr10_force::ContactDetector contact({5.0, 3.0, 0.024, 0.016});
  ok &= check(contact.update(6.0, 0.008) ==
                  fr10_force::ContactState::POSSIBLE_CONTACT &&
                  contact.update(0.0, 0.008) ==
                  fr10_force::ContactState::NO_CONTACT,
              "CONTACT_SPIKE_REJECTED");
  contact.update(6.0, 0.008);
  contact.update(6.0, 0.008);
  ok &= check(contact.update(6.0, 0.008) ==
                  fr10_force::ContactState::CONTACT_CONFIRMED,
              "CONTACT_CONFIRM_TIME");
  contact.update(2.0, 0.008);
  ok &= check(contact.update(2.0, 0.008) ==
                  fr10_force::ContactState::NO_CONTACT,
              "CONTACT_RELEASE_HYSTERESIS");
  {
    fr10_force::ContactDetector spatial({5.,3.,.024,.016});
    fr10_force::Wrench load;
    load.force_n = {.8,.8,0.};
    load.torque_nm = {.016,.016,.04};
    fr10_force::ContactEvidence evidence;
    for (int i = 0; i < 3; ++i) evidence = spatial.update(load, 2, .008);
    ok &= check(evidence.axial == fr10_force::ContactState::NO_CONTACT &&
        evidence.lateral == fr10_force::ContactState::CONTACT_CONFIRMED &&
        evidence.tilt == fr10_force::ContactState::CONTACT_CONFIRMED &&
        evidence.torsion == fr10_force::ContactState::CONTACT_CONFIRMED,
        "SIDE_TILT_AND_TWIST_CONTACT_NEVER_MASQUERADE_AS_AXIAL_SEATING");
    load = {}; load.force_n[2] = 6.;
    for (int i = 0; i < 3; ++i) evidence = spatial.update(load, 2, .008);
    ok &= check(evidence.axial == fr10_force::ContactState::CONTACT_CONFIRMED &&
        evidence.lateral == fr10_force::ContactState::NO_CONTACT &&
        evidence.tilt == fr10_force::ContactState::NO_CONTACT &&
        evidence.torsion == fr10_force::ContactState::NO_CONTACT,
        "SPATIAL_CONTACT_CHANNELS_HAVE_INDEPENDENT_HYSTERESIS");
    load.force_n[0] = std::numeric_limits<double>::quiet_NaN();
    evidence = spatial.update(load, 2, .008);
    ok &= check(evidence.axial == fr10_force::ContactState::NO_CONTACT &&
        evidence.lateral == fr10_force::ContactState::NO_CONTACT,
        "INVALID_SPATIAL_WRENCH_CLEARS_CONTACT_EVIDENCE");
  }

  const auto sc = safetyConfig();
  {
    fr10_force::ForceSafetyMonitor monitor(sc);
    auto input = safeInput();
    input.conditioned_wrench.force_n[2] = 9.0;
    monitor.update(input);
    input.raw_sample.sequence++;
    input.raw_sample.source_timestamp_s += 0.008;
    input.raw_sample.receipt_steady_s += 0.008;
    input.now_steady_s += 0.008;
    input.conditioned_wrench.force_n[2] = 0.0;
    input.conditioned_reference_changed = true;
    ok &= check(monitor.update(input).safe(), "TARE_REBASELINES_RATE_NOT_PHYSICAL_STEP");
    input.conditioned_reference_changed = false;
    input.raw_sample.sequence++;
    input.raw_sample.source_timestamp_s += 0.008;
    input.raw_sample.receipt_steady_s += 0.008;
    input.now_steady_s += 0.008;
    input.conditioned_wrench.force_n[2] = 1.0;
    ok &= check(monitor.update(input).fault == fr10_force::ForceSafetyFault::FORCE_RATE_LIMIT,
                "POST_TARE_PHYSICAL_FORCE_RATE_STILL_CHECKED");
    input.conditioned_reference_changed = true;
    input.now_steady_s += sc.stale_timeout_s + 0.01;
    ok &= check(monitor.update(input).fault == fr10_force::ForceSafetyFault::STALE_WRENCH,
                "TARE_DOES_NOT_REFRESH_DUPLICATE_WATCHDOG");
  }
  {
    fr10_force::ForceSafetyMonitor monitor(sc);
    auto input = safeInput();
    input.conditioned_wrench.torque_nm[0] = 3.0;
    ok &= check(monitor.update(input).fault == fr10_force::ForceSafetyFault::TORQUE_LIMIT,
                "SAFETY_TCP_MOMENT_ARM_TORQUE_LIMIT");
  }
  {
    fr10_force::ForceSafetyMonitor monitor(sc);
    auto input = safeInput();
    input.conditioned_wrench.force_n[0] = 11.0;
    ok &= check(monitor.update(input).fault == fr10_force::ForceSafetyFault::FORCE_LIMIT,
                "SAFETY_INSERTION_AXES_FORCE_LIMIT");
  }
  {
    fr10_force::ForceSafetyMonitor monitor(sc);
    auto input = safeInput();
    input.raw_sample.valid = false;
    ok &= check(monitor.update(input).fault ==
                    fr10_force::ForceSafetyFault::INVALID_WRENCH,
                "SAFETY_INVALID_WRENCH");
  }
  {
    fr10_force::ForceSafetyMonitor monitor(sc);
    auto input = safeInput();
    input.raw_sample.overload = true;
    ok &= check(monitor.update(input).fault ==
                    fr10_force::ForceSafetyFault::SENSOR_OVERLOAD,
                "SAFETY_SENSOR_OVERLOAD");
  }
  {
    fr10_force::ForceSafetyMonitor monitor(sc);
    auto input = safeInput();
    ok &= check(monitor.update(input).safe(), "SAFETY_BASELINE");
    input.raw_sample.sequence = 2;
    input.raw_sample.source_timestamp_s = 1.01;
    input.raw_sample.receipt_steady_s = 10.01;
    input.now_steady_s = 10.01;
    input.raw_sample.wrench.force_n[0] = 11.0;
    input.conditioned_wrench.force_n[0] = 11.0;
    ok &= check(monitor.update(input).fault ==
                    fr10_force::ForceSafetyFault::FORCE_LIMIT,
                "SAFETY_FORCE_LIMIT");
  }
  {
    fr10_force::ForceSafetyMonitor monitor(sc);
    auto input = safeInput();
    monitor.update(input);
    input.raw_sample.sequence = 2;
    input.raw_sample.source_timestamp_s = 0.5;
    input.raw_sample.receipt_steady_s = 10.01;
    input.now_steady_s = 10.01;
    ok &= check(monitor.update(input).fault ==
                    fr10_force::ForceSafetyFault::CLOCK_JUMP,
                "SAFETY_BACKWARD_CLOCK_JUMP");
  }
  {
    fr10_force::ForceSafetyMonitor monitor(sc);
    auto input = safeInput();
    monitor.update(input);
    input.raw_sample.sequence = 2;
    input.raw_sample.source_timestamp_s = 1.01;
    input.raw_sample.receipt_steady_s = 10.01;
    input.now_steady_s = 10.01;
    input.raw_sample.wrench.force_n[0] = 1.0;
    input.conditioned_wrench.force_n[0] = 1.0;
    ok &= check(monitor.update(input).fault ==
                    fr10_force::ForceSafetyFault::FORCE_RATE_LIMIT,
                "SAFETY_FORCE_RATE_LIMIT");
  }
  {
    fr10_force::ForceSafetyMonitor monitor(sc);
    auto input = safeInput();
    input.raw_sample.wrench.torque_nm[0] = 3.0;
    input.conditioned_wrench = input.raw_sample.wrench;
    ok &= check(monitor.update(input).fault ==
                    fr10_force::ForceSafetyFault::TORQUE_LIMIT,
                "SAFETY_TORQUE_LIMIT");
  }
  {
    fr10_force::ForceSafetyMonitor monitor(sc);
    auto input = safeInput();
    monitor.update(input);
    input.now_steady_s = 10.06;
    ok &= check(monitor.update(input).fault ==
                    fr10_force::ForceSafetyFault::STALE_WRENCH,
                "DUPLICATE_DOES_NOT_REFRESH_WATCHDOG");
  }
  {
    fr10_force::ForceSafetyMonitor monitor(sc);
    auto input = safeInput();
    input.correction.position[0] = 0.03;
    ok &= check(monitor.update(input).fault ==
                    fr10_force::ForceSafetyFault::CORRECTION_LIMIT,
                "SAFETY_CORRECTION_LIMIT");
  }
  {
    fr10_force::ForceSafetyMonitor monitor(sc);
    auto input = safeInput();
    input.depth_m = 0.04;
    ok &= check(monitor.update(input).fault ==
                    fr10_force::ForceSafetyFault::DEPTH_LIMIT,
                "SAFETY_DEPTH_LIMIT");
  }
  {
    fr10_force::ForceSafetyMonitor monitor(sc);
    auto input = safeInput();
    input.contact_required = true;
    ok &= check(monitor.update(input).fault ==
                    fr10_force::ForceSafetyFault::LOST_CONTACT,
                "SAFETY_LOST_CONTACT");
  }
  {
    fr10_force::ForceSafetyMonitor monitor(sc);
    auto input = safeInput();
    input.ik_ok = false;
    ok &= check(monitor.update(input).fault ==
                    fr10_force::ForceSafetyFault::IK_FAILURE,
                "SAFETY_IK_FAILURE");
  }
  {
    fr10_force::ForceSafetyMonitor monitor(sc);
    auto input = safeInput();
    input.scene_unchanged = false;
    ok &= check(monitor.update(input).fault ==
                    fr10_force::ForceSafetyFault::SCENE_CHANGED,
                "SAFETY_SCENE_CHANGE");
  }
  {
    fr10_force::ForceSafetyMonitor monitor(sc);
    auto input = safeInput();
    input.collision_free = false;
    ok &= check(monitor.update(input).fault ==
                    fr10_force::ForceSafetyFault::COLLISION,
                "SAFETY_COLLISION");
  }
  {
    fr10_force::ForceSafetyMonitor monitor(sc);
    auto input = safeInput();
    input.tracking_ok = false;
    ok &= check(monitor.update(input).fault ==
                    fr10_force::ForceSafetyFault::TRACKING_FAILURE,
                "SAFETY_TRACKING_FAILURE");
  }
  {
    fr10_force::ForceSafetyMonitor monitor(sc);
    auto input = safeInput();
    input.robot_ok = false;
    ok &= check(monitor.update(input).fault ==
                    fr10_force::ForceSafetyFault::ROBOT_FAULT,
                "SAFETY_ROBOT_FAULT");
  }
  {
    fr10_force::ForceSafetyMonitor monitor(sc);
    auto input = safeInput();
    input.command_accepted = false;
    ok &= check(monitor.update(input).fault ==
                    fr10_force::ForceSafetyFault::COMMAND_REJECTED,
                "SAFETY_COMMAND_REJECTED");
  }

  fr10_force::FakeWrenchConfig fake_config;
  fake_config.contact_depth_m = 0.005;
  fake_config.stiffness_n_m = 1000.0;
  fake_config.damping_n_s_m = 10.0;
  fr10_force::FakeWrenchSource fake(fake_config);
  const auto generated = fake.sample(0.010, 0.1, 1.0, 2.0);
  ok &= check(generated.valid && near(generated.wrench.force_n[2], 6.0),
              "FAKE_SPRING_DAMPER");
  fr10_force::FakeWrenchFaults faults;
  faults.duplicate = true;
  const auto generated_duplicate = fake.sample(0.010, 0.1, 1.01, 2.01, faults);
  ok &= check(generated_duplicate.sequence == generated.sequence &&
                  near(generated_duplicate.source_timestamp_s,
                       generated.source_timestamp_s),
              "FAKE_DUPLICATE_INJECTION");
  faults = {};
  faults.nan = true;
  ok &= check(!fr10_force::finite(
                  fake.sample(0.010, 0.1, 1.02, 2.02, faults).wrench),
              "FAKE_NAN_INJECTION");
  faults = {};
  faults.dropout = true;
  ok &= check(!fake.sample(0.010, 0.1, 1.03, 2.03, faults).valid,
              "FAKE_DROPOUT_INJECTION");
  faults = {};
  faults.overload = true;
  ok &= check(fake.sample(0.010, 0.1, 1.04, 2.04, faults).overload,
              "FAKE_OVERLOAD_INJECTION");
  faults = {};
  faults.lost_contact = true;
  ok &= check(near(fake.sample(0.010, 0.1, 1.05, 2.05, faults)
                       .wrench.force_n[2], 0.0),
              "FAKE_LOST_CONTACT_INJECTION");
  // Verification measures the process trend, while raw safety is unchanged.
  fr10_force::WrenchVerificationSlope slope(.2);
  fr10_force::WrenchSample trend;
  trend.valid=true;
  for(unsigned tick=0;tick<=100;++tick) {
    const double elapsed=tick*.008;
    trend.sequence=tick+1;trend.source_timestamp_s=1.+elapsed;
    trend.wrench.force_n[2]=4.5+elapsed+.1*std::sin(2.*3.141592653589793*25.*elapsed);
    auto result=slope.update(trend);
    ok &= check(result.has_value()==(tick>=25),"VERIFICATION_REQUIRES_FULL_SOURCE_WINDOW");
    if(result) ok &= check(std::abs((*result)[2]-1.)<1e-9,
                          "PERIODIC_RIPPLE_DOES_NOT_HIDE_ACTUAL_FORCE_TREND");
  }
  const auto previous=slope.update(trend);trend.receipt_steady_s=1000.;
  const auto verification_duplicate=slope.update(trend);
  ok &= check(previous && verification_duplicate && *previous==*verification_duplicate,
              "DUPLICATE_DOES_NOT_RENEW_SOURCE_WINDOW");
  slope.reset();
  for(unsigned tick=0;tick<80;++tick) {
    const double elapsed=tick*.013;
    trend.sequence=tick+1;trend.source_timestamp_s=1.+elapsed;
    trend.wrench.force_n[2]=10.*elapsed;
    auto result=slope.update(trend);
    if(result) ok &= check(std::abs((*result)[2]-10.)<1e-9 && (*result)[2]>5.,
                          "ACTUAL_RAPID_TREND_REJECTED_WITH_OFF_GRID_INTERPOLATION");
  }
  trend.source_timestamp_s+=.051;++trend.sequence;
  ok &= check(!slope.update(trend),"SOURCE_GAP_DENIES_VERIFICATION");
  trend.source_timestamp_s-=.001;
  ok &= check(!slope.update(trend),"BACKWARD_SOURCE_DENIES_VERIFICATION");
  trend.wrench.force_n[2]=std::numeric_limits<double>::quiet_NaN();
  ok &= check(!slope.update(trend),"INVALID_FORCE_DENIES_VERIFICATION");
  slope.reset();trend.wrench.force_n[2]=5.;
  for(unsigned tick=0;tick<100;++tick) {
    trend.sequence=tick+1;trend.source_timestamp_s=1.+tick*.001;
    ok &= check(!slope.update(trend),"BOUNDED_HISTORY_CANNOT_SILENTLY_SHORTEN_WINDOW");
  }
  return ok ? 0 : 1;
}
