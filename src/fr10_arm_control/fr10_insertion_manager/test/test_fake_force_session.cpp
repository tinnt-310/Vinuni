#include <array>
#include <chrono>
#include <iostream>

#include "fr10_force_control/fake_wrench_source.hpp"
#include "fr10_insertion_manager/force_session.hpp"
#include "fr10_motion_core/motion_controller.hpp"

namespace {

bool check(bool condition, const char *name) {
  if (!condition) std::cerr << "FAIL " << name << '\n';
  return condition;
}

class CartesianEchoProvider final : public fr10_motion::IKProvider {
public:
  fr10_motion::IKResult solve(
      const fr10_motion::Pose &target,
      const fr10_motion::KinematicState &seed) override {
    std::array<fr10_motion::IKResult, kMaxCandidates> candidates{};
    const auto count = solveCandidates(target, seed, candidates, nullptr);
    return count == 0 ? fr10_motion::IKResult{} : candidates[0];
  }

  std::size_t solveCandidates(
      const fr10_motion::Pose &target,
      const fr10_motion::KinematicState &seed,
      std::array<fr10_motion::IKResult, kMaxCandidates> &candidates,
      std::string *) override {
    candidates = {};
    last_target_ = target;
    candidates[0].success = true;
    candidates[0].state = seed;
    candidates[0].state.position_rad[0] = target.position_m[2];
    candidates[0].manipulability = 1.0;
    candidates[0].condition_number = 1.0;
    return 1;
  }

  fr10_motion::IKResult validate(
      const fr10_motion::KinematicState &target,
      const fr10_motion::KinematicState &) override {
    fr10_motion::IKResult result;
    result.success = true;
    result.state = target;
    return result;
  }

  bool forward(const fr10_motion::KinematicState &,
               fr10_motion::Pose &pose, std::string *) const override {
    pose = last_target_;
    return true;
  }

private:
  mutable fr10_motion::Pose last_target_;
};

fr10_motion::MotionExecutorConfig executorConfig() {
  fr10_motion::MotionExecutorConfig config;
  config.control_rate_hz = 125.0;
  config.require_command_ack = true;
  config.max_joint_reference_lead_rad = 1.0;
  config.limits.min_joint_position_rad.fill(-3.0);
  config.limits.max_joint_position_rad.fill(3.0);
  config.limits.max_joint_velocity_rad_s.fill(1.0);
  config.limits.max_joint_acceleration_rad_s2.fill(5.0);
  config.limits.max_joint_jerk_rad_s3.fill(20.0);
  return config;
}

} // namespace

int main() {
  bool ok = true;
  fr10_insertion::ForceSessionConfig config;
  config.state_machine.preinsert_hold_s = 0.008;
  config.admittance.selected_axes[2] = true;
  config.admittance.mass.fill(1.0);
  config.admittance.damping.fill(20.0);
  config.admittance.max_correction.fill(0.01);
  config.admittance.max_velocity.fill(0.02);
  config.admittance.max_acceleration.fill(1.0);
  config.contact.enter_force_n = 5.0;
  config.contact.exit_force_n = 3.0;
  config.contact.confirm_time_s = 0.016;
  config.contact.release_time_s = 0.016;
  config.safety.max_force_n.fill(50.0);
  config.safety.max_torque_nm.fill(10.0);
  config.safety.max_force_rate_n_s.fill(10000.0);
  config.safety.max_correction.fill(0.02);
  config.safety.max_depth_m = 0.02;
  config.continuous_ik.max_joint_step_rad = 1.0;
  config.continuous_ik.max_position_error_m = 1e-9;
  config.continuous_ik.max_orientation_error_rad = 1e-9;
  fr10_insertion::ForceSession session(config);

  fr10_insertion::InsertionGoal goal;
  goal.preinsert_distance_m = 0.001;
  goal.insertion_depth_m = 0.005;
  goal.insertion_speed_m_s = 0.001;
  goal.selected_axes[2] = true;
  goal.insertion_axis = 2;
  goal.desired_axis_force_n = 10.0;
  goal.timeout_s = 5.0;
  fr10_force::CartesianPose final_pose;
  final_pose.position_m = {0.4, 0.0, 0.5};
  fr10_force::Wrench desired;
  desired.force_n[2] = 10.0;
  ok &= check(session.start(goal, final_pose, {0.0, 0.0, 1.0}, desired,
                            1, 7, 1.0),
              "SESSION_START");

  CartesianEchoProvider provider;
  fr10_force::FakeWrenchConfig fake_config;
  fake_config.contact_depth_m = -0.001;
  fake_config.stiffness_n_m = 6000.0;
  fr10_force::FakeWrenchSource fake(fake_config);
  fr10_insertion::ForceSessionInput input;
  input.source_time_s = 1.0;
  input.now_steady_s = 10.0;
  input.dt_s = 0.008;
  input.preinsert_reached = true;
  fr10_insertion::ForceSessionOutput output;
  for (int cycle = 0; cycle < 4; ++cycle) {
    input.source_time_s += input.dt_s;
    input.now_steady_s += input.dt_s;
    input.raw_sample = fake.sample(-0.001, 0.0, input.source_time_s,
                                   input.now_steady_s);
    input.conditioned_wrench = input.raw_sample.wrench;
    input.tare_complete = cycle >= 2;
    output = session.update(input, provider);
  }
  ok &= check(output.decision.request_online_motion &&
                  output.online_target_valid,
              "APPROACH_PRODUCES_ONLINE_TARGET");

  fr10_motion::MotionExecutor executor(executorConfig());
  fr10_motion::RobotState feedback;
  feedback.valid = true;
  feedback.safety_known = true;
  feedback.feedback_sequence = 1;
  feedback.timestamp_s = input.source_time_s;
  executor.update(feedback, 0.008);
  ok &= check(executor.submitOnlineJointTarget(output.online_target),
              "ONLINE_TARGET_ACCEPTED_BY_EXECUTOR");
  feedback.feedback_sequence++;
  feedback.timestamp_s += 0.008;
  executor.update(feedback, 0.008);
  const auto command = executor.command();
  const bool published = command.type ==
          fr10_motion::CommandType::JOINT_POSITION &&
      executor.acknowledgeCommand(command.sequence);
  ok &= check(session.commitOnlineTarget(output.online_target.sequence,
                                         published),
              "COMMIT_ONLY_AFTER_EXECUTOR_ACK");
  ok &= check(executor.status().owner ==
                  fr10_motion::MotionOwner::FORCE_ONLINE,
              "FORCE_ONLINE_OWNS_JOINT_SERVO_PATH");

  // Build persistent contact, then verify that a rejected publish does not
  // commit the admittance integrator.
  input.preinsert_reached = true;
  input.tare_complete = true;
  input.actual_depth_m = 0.001;
  fr10_insertion::ForceSessionOutput force_output;
  for (int cycle = 0; cycle < 4; ++cycle) {
    input.source_time_s += input.dt_s;
    input.now_steady_s += input.dt_s;
    input.raw_sample = fake.sample(0.0, 0.0, input.source_time_s,
                                   input.now_steady_s);
    input.conditioned_wrench = input.raw_sample.wrench;
    force_output = session.update(input, provider);
    if (force_output.online_target_valid) {
      session.commitOnlineTarget(force_output.online_target.sequence, true);
    }
  }
  input.source_time_s += input.dt_s;
  input.now_steady_s += input.dt_s;
  input.raw_sample = fake.sample(0.0, 0.0, input.source_time_s,
                                 input.now_steady_s);
  input.conditioned_wrench = input.raw_sample.wrench;
  input.joint_seed.velocity_rad_s.fill(0.2);
  input.joint_seed.acceleration_rad_s2.fill(0.1);
  input.command_source_time_s = 1000.0;
  const auto rejected = session.update(input, provider);
  ok &= check(rejected.online_target.timestamp_s == 1000.0 &&
                  std::abs(rejected.online_target.valid_until_s-1000.032)<1e-9,
              "MONOTONIC_PHASE_CLOCK_SEPARATE_FROM_FEEDBACK_EXPIRY_CLOCK");
  bool endpoint_stationary = true;
  for (std::size_t joint = 0; joint < fr10_motion::kJointCount; ++joint) {
    endpoint_stationary &=
        rejected.online_target.target_velocity_rad_s[joint] == 0.0 &&
        rejected.online_target.target_acceleration_rad_s2[joint] == 0.0;
  }
  ok &= check(endpoint_stationary,
              "IK_SEED_DERIVATIVES_ARE_NOT_ONLINE_ENDPOINT_DERIVATIVES");
  ok &= check(rejected.actual_depth_m == input.actual_depth_m,
              "REPORTED_DEPTH_IS_MEASURED_NOT_CLAMPED_PROGRESS");
  ok &= check(rejected.online_target_valid &&
                  rejected.correction.position[2] > 0.0,
              "FORCE_CORRECTION_REACHES_IK");
  ok &= check(!session.commitOnlineTarget(rejected.online_target.sequence,
                                          false),
              "REJECTED_PUBLISH_NOT_COMMITTED");
  input.source_time_s += input.dt_s;
  input.now_steady_s += input.dt_s;
  input.raw_sample = fake.sample(0.0, 0.0, input.source_time_s,
                                 input.now_steady_s);
  input.conditioned_wrench = input.raw_sample.wrench;
  const auto retry = session.update(input, provider);
  ok &= check(std::abs(retry.correction.position[2] -
                       rejected.correction.position[2]) < 1e-12,
              "REJECTED_PUBLISH_PRESERVES_INTEGRATOR_STATE");
  session.commitOnlineTarget(retry.online_target.sequence, true);
  fr10_insertion::ForceSessionOutput held;
  double previous_nominal_z = 0.0;
  bool nominal_advanced_during_verify = false;
  // A force step needs a complete source trend window followed by the
  // unchanged completion dwell, then the executor stop acknowledgement.
  for (int cycle=0; cycle<64; ++cycle) {
    input.source_time_s += input.dt_s;
    input.now_steady_s += input.dt_s;
    input.actual_depth_m = goal.insertion_depth_m;
    input.raw_sample.sequence++;
    input.raw_sample.source_timestamp_s = input.source_time_s;
    input.raw_sample.receipt_steady_s = input.now_steady_s;
    input.raw_sample.wrench = desired;
    input.conditioned_wrench = desired;
    held = session.update(input, provider);
    if(cycle<20) ok &= check(!held.decision.success,
        "FORCE_STEP_CANNOT_COMPLETE_BEFORE_TREND_AND_DWELL");
    if (held.decision.request_controlled_stop && held.decision.state == fr10_insertion::InsertionState::SUCCESS) {
      input.stop_complete = true; // Simulated executor acknowledgement, next tick.
    }
    if (cycle > 1 && held.decision.state == fr10_insertion::InsertionState::VERIFY_INSERTION) {
      nominal_advanced_during_verify |= held.nominal_pose.position_m[2] > previous_nominal_z + 1e-12;
    }
    previous_nominal_z = held.nominal_pose.position_m[2];
    if (held.online_target_valid) session.commitOnlineTarget(held.online_target.sequence, true);
  }
  ok &= check(held.decision.success && !held.online_target_valid &&
                  held.correction.position[2] > 0.0,
              "HOLD_RETAINS_ACKNOWLEDGED_CORRECTION_WITHOUT_INTEGRATION");
  ok &= check(nominal_advanced_during_verify,
              "VERIFY_DOES_NOT_FREEZE_NOMINAL_BEFORE_FINAL_DEPTH");

  // The online tick contains only fixed-size admittance math and analytical
  // candidate selection. Benchmark this pure path independently of ROS,
  // MoveIt, logging, networking, and the backend transport.
  fr10_force::AdmittanceConfig benchmark_admittance;
  benchmark_admittance.selected_axes[2] = true;
  benchmark_admittance.mass.fill(1.0);
  benchmark_admittance.damping.fill(20.0);
  benchmark_admittance.max_correction.fill(0.01);
  benchmark_admittance.max_velocity.fill(0.02);
  benchmark_admittance.max_acceleration.fill(1.0);
  fr10_force::ForcePositionController benchmark_controller(
      benchmark_admittance,
      {1.0, 0.0, 0.0,
       0.0, 1.0, 0.0,
       0.0, 0.0, 1.0});
  fr10_motion::ContinuousIKConfig benchmark_ik_config;
  benchmark_ik_config.max_joint_step_rad = 1.0;
  benchmark_ik_config.max_position_error_m = 1e-9;
  benchmark_ik_config.max_orientation_error_rad = 1e-9;
  fr10_motion::ContinuousIKStepper benchmark_ik(benchmark_ik_config);
  fr10_motion::KinematicState benchmark_seed;
  benchmark_seed.position_rad[0] = final_pose.position_m[2];
  fr10_force::Wrench zero_wrench;
  constexpr int benchmark_cycles = 50000;
  const auto benchmark_start = std::chrono::steady_clock::now();
  for (int cycle = 0; cycle < benchmark_cycles; ++cycle) {
    const auto proposal = benchmark_controller.propose(
        final_pose, zero_wrench, zero_wrench, 0.008);
    fr10_motion::Pose target;
    target.position_m = proposal.corrected_pose.position_m;
    target.orientation_quat = proposal.corrected_pose.orientation_xyzw;
    const auto ik = benchmark_ik.step(target, benchmark_seed, provider);
    if (!proposal.valid || !ik.success ||
        !benchmark_controller.commit(proposal)) {
      ok &= check(false, "FORCE_IK_BENCHMARK_VALID");
      break;
    }
    benchmark_seed = ik.state;
  }
  const double benchmark_elapsed_us =
      std::chrono::duration<double, std::micro>(
          std::chrono::steady_clock::now() - benchmark_start).count();
  const double benchmark_mean_us =
      benchmark_elapsed_us / static_cast<double>(benchmark_cycles);
  std::cout << "FORCE_IK_BENCHMARK mean_us=" << benchmark_mean_us
            << " cycles=" << benchmark_cycles << '\n';
  ok &= check(benchmark_mean_us < 8000.0,
              "FORCE_IK_MEAN_BELOW_8MS");
  return ok ? 0 : 1;
}
