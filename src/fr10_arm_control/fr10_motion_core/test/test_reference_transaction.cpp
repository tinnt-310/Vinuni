#include <cmath>
#include <iostream>
#include "fr10_motion_core/motion_controller.hpp"

namespace {
std::shared_ptr<fr10_motion::CartesianJointTrajectory> path(
    const fr10_motion::KinematicState &seed, double goal, uint64_t generation,
    const fr10_motion::MotionExecutorConfig &config) {
  using namespace fr10_motion;
  auto trajectory = std::make_shared<CartesianJointTrajectory>();
  trajectory->generation = generation;
  CartesianTrajectoryPoint first; first.joint_state = seed;
  trajectory->points.push_back(first);
  JointOtg planner(.008, config.limits, config.max_joint_reference_lead_rad);
  planner.reset(seed.position_rad, seed.velocity_rad_s, seed.acceleration_rad_s2);
  JointTarget target; target.position_rad = seed.position_rad; target.position_rad[0] = goal;
  planner.submitTarget(target);
  RobotState feedback; feedback.valid = true;
  for (int i = 1; i < 1000; ++i) {
    CartesianTrajectoryPoint point;
    feedback.joint_position_rad = trajectory->points.back().joint_state.position_rad;
    if (!planner.step(feedback, point.joint_state.position_rad,
                       point.joint_state.velocity_rad_s, point.joint_state.acceleration_rad_s2)) break;
    point.time_from_start_s = i * .008;
    trajectory->points.push_back(point);
    if (planner.isFinished()) break;
  }
  return trajectory;
}
}
int main() {
  using namespace fr10_motion;
  bool ok = true;
  auto check = [&](bool valid, const char *name) { if (!valid) { std::cerr << name << '\n'; ok = false; } };
  MotionExecutorConfig config; config.require_command_ack = true;
  config.max_joint_reference_lead_rad = kDegToRad;
  config.limits.min_joint_position_rad.fill(-3); config.limits.max_joint_position_rad.fill(3);
  config.limits.max_joint_velocity_rad_s.fill(.5);
  config.limits.max_joint_acceleration_rad_s2.fill(1);
  config.limits.max_joint_jerk_rad_s3.fill(5);
  MotionExecutor executor(config);
  RobotState feedback; feedback.valid = true; feedback.feedback_sequence = 1;
  executor.update(feedback, .008); executor.acknowledgeCommand(executor.command().sequence);
  const auto initial = executor.planningBoundary(10);
  check(initial.valid && initial.sequence == 0, "stationary boundary is deadline-free");
  auto trajectory = path(initial.reference, .2, 1, config);
  auto prepared = executor.prepareJointTrajectory(trajectory, JointTarget{});
  check(prepared != nullptr, "test path prepared");
  check(executor.activatePreparedPoseTrajectory(prepared) == PoseActivationResult::ACCEPTED, "path activated");
  executor.update(feedback, .008); const auto proposed = executor.command();
  executor.update(feedback, .008); const auto replay = executor.command();
  check(proposed.joint_position_rad == replay.joint_position_rad,
        "unpublished buffered reference cannot integrate");
  check(!executor.acknowledgeCommand(proposed.sequence) && executor.acknowledgeCommand(replay.sequence),
        "only current proposal commits");
  feedback.joint_position_rad = replay.joint_position_rad;
  feedback.joint_velocity_rad_s = replay.joint_velocity_rad_s;
  feedback.joint_acceleration_rad_s2 = replay.joint_acceleration_rad_s2;
  const auto future = executor.planningBoundary(10);
  for (int i = 0; i < 10; ++i) {
    ++feedback.feedback_sequence; executor.update(feedback, .008);
    const auto command = executor.command(); executor.acknowledgeCommand(command.sequence);
    feedback.joint_position_rad = command.joint_position_rad;
    feedback.joint_velocity_rad_s = command.joint_velocity_rad_s;
    feedback.joint_acceleration_rad_s2 = command.joint_acceleration_rad_s2;
  }
  check(future.valid && future.sequence == executor.command().sequence &&
        future.reference.position_rad == executor.command().joint_position_rad &&
        future.reference.velocity_rad_s == executor.command().joint_velocity_rad_s &&
        future.reference.acceleration_rad_s2 == executor.command().joint_acceleration_rad_s2,
        "future handover boundary predicts acknowledged q/v/a exactly");
  auto next = executor.prepareJointTrajectory(path(future.reference, .3, 2, config), JointTarget{});
  check(next != nullptr && executor.activatePreparedPoseTrajectory(next) == PoseActivationResult::ACCEPTED,
        "moving handover is accepted at scheduled reference");
  executor.update(feedback, .008); const auto joined = executor.command();
  check(joined.joint_position_rad[0] > future.reference.position_rad[0] &&
        std::abs(joined.joint_velocity_rad_s[0] - future.reference.velocity_rad_s[0]) <= .008 + 1e-10 &&
        std::abs(joined.joint_acceleration_rad_s2[0] - future.reference.acceleration_rad_s2[0]) <= .04 + 1e-10,
        "first handover command is next sample, not duplicated moving seed");

  executor.acknowledgeCommand(joined.sequence);
  feedback.joint_position_rad = joined.joint_position_rad;
  feedback.joint_velocity_rad_s = joined.joint_velocity_rad_s;
  feedback.joint_acceleration_rad_s2 = joined.joint_acceleration_rad_s2;
  executor.stop();
  bool stop_committed = false;
  for (int i = 0; i < 1000; ++i) {
    ++feedback.feedback_sequence;
    const auto proposed_status = executor.update(feedback, .008);
    const auto braking = executor.command();
    check(proposed_status.state == MotionState::STOPPING &&
          !executor.planningBoundary(10).valid,
          "unacknowledged braking cannot expose a stationary planning boundary");
    check(executor.acknowledgeCommand(braking.sequence), "braking command publication commits");
    feedback.joint_position_rad = braking.joint_position_rad;
    feedback.joint_velocity_rad_s = braking.joint_velocity_rad_s;
    feedback.joint_acceleration_rad_s2 = braking.joint_acceleration_rad_s2;
    if (executor.status().state == MotionState::HOLDING) {
      const auto stopped = executor.planningBoundary(10);
      stop_committed = stopped.valid && stopped.sequence == 0 &&
          stopped.reference.position_rad == braking.joint_position_rad &&
          stopped.reference.velocity_rad_s == braking.joint_velocity_rad_s &&
          stopped.reference.acceleration_rad_s2 == braking.joint_acceleration_rad_s2 &&
          std::abs(stopped.reference.velocity_rad_s[0]) < 1e-12 &&
          std::abs(stopped.reference.acceleration_rad_s2[0]) < 1e-12;
      break;
    }
  }
  check(stop_committed, "post-stop replan seeds acknowledged zero velocity and acceleration");

  JointOtg otg(.008, config.limits); otg.reset({});
  JointTarget target; target.position_rad[0] = .1; otg.submitTarget(target);
  std::array<double, 6> q{}, v{}, a{}, replay_q{}, replay_v{}, replay_a{};
  otg.step(feedback, q, v, a, false); otg.step(feedback, replay_q, replay_v, replay_a, false);
  check(q == replay_q && otg.currentPosition()[0] == 0, "unpublished Ruckig proposal cannot integrate");
  otg.commitStep(); check(otg.currentPosition() == q, "Ruckig commit follows publication");
  return ok ? 0 : 1;
}
