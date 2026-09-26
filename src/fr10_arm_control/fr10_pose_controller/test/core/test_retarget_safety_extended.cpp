#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "fr10_pose_controller/cartesian_path.hpp"
#include "fr10_pose_controller/joint_otg.hpp"
#include "fr10_pose_controller/motion_controller.hpp"
#include "test_support.hpp"

namespace {

enum class Outcome { PASS, FAIL, SKIP };
using Case = std::function<Outcome()>;

struct StreamMetrics {
  bool valid{true};
  int submissions{0};
  int updates{0};
  double max_velocity{0.0};
  double max_acceleration{0.0};
  double max_jerk{0.0};
};

StreamMetrics streamTargets(double input_rate_hz, bool alternate) {
  constexpr double control_rate_hz = 125.0;
  constexpr double dt = 1.0 / control_rate_hz;
  constexpr double duration_s = 5.0;
  fr10_motion::JointOtg otg(dt, fr10_motion_test::limits(), 10.0);
  std::array<double, 6> zero{};
  otg.reset(zero);
  fr10_motion::RobotState feedback;
  std::array<double, 6> q{}, qd{}, qdd{}, previous_qdd{};
  StreamMetrics metrics;
  double next_submission_s = 0.0;
  const int cycles = static_cast<int>(duration_s * control_rate_hz);
  for (int cycle = 0; cycle < cycles && metrics.valid; ++cycle) {
    const double time_s = cycle * dt;
    while (time_s + 1e-12 >= next_submission_s) {
      fr10_motion::JointTarget target;
      target.revision = static_cast<uint64_t>(metrics.submissions + 1);
      if (alternate) {
        target.position_rad[0] =
            metrics.submissions % 2 == 0 ? 0.20 : -0.20;
      } else {
        target.position_rad[0] = 0.18 * std::sin(0.7 * next_submission_s);
        target.position_rad[1] = 0.12 * std::sin(0.5 * next_submission_s);
      }
      metrics.valid = metrics.valid && otg.submitTarget(target);
      ++metrics.submissions;
      next_submission_s += 1.0 / input_rate_hz;
    }
    metrics.valid = metrics.valid && otg.step(feedback, q, qd, qdd);
    for (std::size_t joint = 0; joint < fr10_motion::kJointCount; ++joint) {
      metrics.max_velocity =
          std::max(metrics.max_velocity, std::abs(qd[joint]));
      metrics.max_acceleration =
          std::max(metrics.max_acceleration, std::abs(qdd[joint]));
      metrics.max_jerk = std::max(
          metrics.max_jerk,
          std::abs(qdd[joint] - previous_qdd[joint]) / dt);
    }
    previous_qdd = qdd;
    ++metrics.updates;
  }
  return metrics;
}

Outcome streamCase(double rate_hz, const std::string &id, bool alternate) {
  const auto metrics = streamTargets(rate_hz, alternate);
  const auto limits = fr10_motion_test::limits();
  const int expected = static_cast<int>(std::floor(5.0 * rate_hz));
  std::cout << "METRIC " << id << " input_rate_hz=" << rate_hz
            << " submissions=" << metrics.submissions
            << " updates=" << metrics.updates
            << " max_velocity_rad_s=" << metrics.max_velocity
            << " max_acceleration_rad_s2=" << metrics.max_acceleration
            << " max_jerk_rad_s3=" << metrics.max_jerk << "\n";
  return metrics.valid && metrics.submissions >= expected &&
                 metrics.updates == 625 &&
                 metrics.max_velocity <= limits.max_joint_velocity_rad_s[0] +
                                             1e-7 &&
                 metrics.max_acceleration <=
                     limits.max_joint_acceleration_rad_s2[0] + 1e-7 &&
                 metrics.max_jerk <= limits.max_joint_jerk_rad_s3[0] + 1e-5
             ? Outcome::PASS
             : Outcome::FAIL;
}

Outcome directionReversal() {
  constexpr double dt = 0.008;
  fr10_motion::JointOtg otg(dt, fr10_motion_test::limits(), 10.0);
  std::array<double, 6> zero{};
  otg.reset(zero);
  fr10_motion::JointTarget target;
  target.position_rad[0] = 0.80;
  target.revision = 1;
  bool ok = otg.submitTarget(target);
  fr10_motion::RobotState feedback;
  std::array<double, 6> q{}, qd{}, qdd{}, previous_qdd{};
  bool reversed = false;
  bool crossed_zero_velocity = false;
  double max_jerk = 0.0;
  double velocity_before = 0.0;
  double velocity_after = 0.0;
  for (int cycle = 0; cycle < 5000 && ok && !otg.isFinished(); ++cycle) {
    ok = otg.step(feedback, q, qd, qdd);
    max_jerk = std::max(max_jerk, std::abs(qdd[0] - previous_qdd[0]) / dt);
    previous_qdd = qdd;
    if (!reversed && qd[0] > 0.12) {
      velocity_before = qd[0];
      target.position_rad[0] = -0.30;
      target.revision = 2;
      ok = ok && otg.submitTarget(target);
      reversed = true;
    } else if (reversed && velocity_after == 0.0) {
      velocity_after = qd[0];
    }
    if (reversed && qd[0] < 0.0) {
      crossed_zero_velocity = true;
    }
  }
  std::cout << "METRIC RT_DIRECTION_REVERSAL velocity_before_rad_s="
            << velocity_before << " velocity_after_rad_s=" << velocity_after
            << " max_jerk_rad_s3=" << max_jerk
            << " final_error_rad=" << std::abs(q[0] + 0.30) << "\n";
  return ok && reversed && crossed_zero_velocity && otg.isFinished() &&
                 std::abs(q[0] + 0.30) < 1e-9 &&
                 std::abs(velocity_after - velocity_before) < 0.01 &&
                 max_jerk <= 4.00001
             ? Outcome::PASS
             : Outcome::FAIL;
}

Outcome poseTimestampPlayback(double second_period_s,
                              double expected_position_rad,
                              const std::string &id) {
  fr10_motion::MotionExecutorConfig config;
  config.control_rate_hz = 125.0;
  config.max_joint_reference_lead_rad = 1.0;
  config.limits = fr10_motion_test::limits();
  fr10_motion::MotionExecutor executor(config);
  fr10_motion::RobotState feedback;
  feedback.valid = true;
  executor.update(feedback, 0.008);

  auto trajectory = std::make_shared<fr10_motion::CartesianJointTrajectory>();
  trajectory->generation = 41;
  for (std::size_t index = 0; index < 4; ++index) {
    fr10_motion::CartesianTrajectoryPoint point;
    point.joint_state.position_rad[0] = 0.01 * index;
    point.time_from_start_s = 0.008 * index;
    point.path_parameter = static_cast<double>(index) / 3.0;
    trajectory->points.push_back(point);
  }
  if (!executor.submitPoseTrajectory(trajectory)) {
    return Outcome::FAIL;
  }

  // The first update emits t=0. A normal second update should emit t=8 ms;
  // a 16 ms callback still advances one nominal sample. Catch-up would skip
  // validated commands and introduce a larger position step at the driver.
  executor.update(feedback, 0.008);
  const auto first = executor.command();
  feedback.joint_position_rad = first.joint_position_rad;
  executor.update(feedback, second_period_s);
  const auto second = executor.command();
  const double error =
      std::abs(second.joint_position_rad[0] - expected_position_rad);
  std::cout << "METRIC " << id
            << " measured_period_s=" << second_period_s
            << " expected_q1_rad=" << expected_position_rad
            << " emitted_q1_rad=" << second.joint_position_rad[0]
            << " timestamp_error_rad=" << error << "\n";
  if (error > 1e-12 && second_period_s > 0.008) {
    std::cerr << "DEFECT " << id
              << " Pose playback skipped a nominal sample after a missed cycle\n";
  }
  return first.type == fr10_motion::CommandType::JOINT_POSITION &&
                 second.type == fr10_motion::CommandType::JOINT_POSITION &&
                 error <= 1e-12
             ? Outcome::PASS
             : Outcome::FAIL;
}

Outcome invalidPose(double value, const std::string &id) {
  fr10_motion::Pose start;
  start.position_m = {0.30, -0.20, 0.50};
  auto goal = start;
  goal.position_m[0] = value;
  std::vector<fr10_motion::Pose> path;
  std::string error;
  const bool accepted = fr10_motion::CartesianPath::linear(
      start, goal, fr10_motion::CartesianPathConfig{}, path, &error);
  std::cout << "METRIC " << id << " accepted=" << (accepted ? 1 : 0)
            << " reason=" << error << "\n";
  return !accepted && path.empty() ? Outcome::PASS : Outcome::FAIL;
}

Outcome invalidQuaternion() {
  fr10_motion::Pose start;
  start.position_m = {0.30, -0.20, 0.50};
  auto zero = start;
  zero.orientation_quat = {0.0, 0.0, 0.0, 0.0};
  auto nan = start;
  nan.orientation_quat[0] = std::numeric_limits<double>::quiet_NaN();
  auto gross = start;
  gross.orientation_quat = {0.0, 0.0, 0.0, 10.0};
  std::vector<fr10_motion::Pose> path;
  std::string error;
  const bool zero_accepted = fr10_motion::CartesianPath::linear(
      start, zero, fr10_motion::CartesianPathConfig{}, path, &error);
  path.clear();
  const bool nan_accepted = fr10_motion::CartesianPath::linear(
      start, nan, fr10_motion::CartesianPathConfig{}, path, &error);
  path.clear();
  const bool gross_accepted = fr10_motion::CartesianPath::linear(
      start, gross, fr10_motion::CartesianPathConfig{}, path, &error);
  const double final_norm = gross_accepted
                                ? std::sqrt(fr10_motion::math::quaternionNormSquared(
                                      path.back().orientation_quat))
                                : 0.0;
  std::cout << "METRIC S_INVALID_QUAT zero_accepted="
            << (zero_accepted ? 1 : 0)
            << " nan_accepted=" << (nan_accepted ? 1 : 0)
            << " gross_nonzero_accepted=" << (gross_accepted ? 1 : 0)
            << " normalized_endpoint_norm=" << final_norm << "\n";
  return !zero_accepted && !nan_accepted && gross_accepted &&
                 std::abs(final_norm - 1.0) < 1e-12
             ? Outcome::PASS
             : Outcome::FAIL;
}

Outcome ownershipCollision() {
  fr10_motion::MotionExecutorConfig config;
  config.control_rate_hz = 125.0;
  config.limits = fr10_motion_test::limits();
  fr10_motion::MotionExecutor executor(config);
  fr10_motion::RobotState feedback;
  feedback.valid = true;
  executor.update(feedback, 0.008);
  fr10_motion::JointTarget joint;
  joint.position_rad[0] = 0.25;
  joint.revision = 1;
  const bool joint_ok = executor.submitJointTarget(joint);
  executor.update(feedback, 0.008);
  auto trajectory = std::make_shared<fr10_motion::CartesianJointTrajectory>();
  trajectory->generation = 1;
  for (std::size_t index = 0; index < 3; ++index) {
    fr10_motion::CartesianTrajectoryPoint point;
    point.joint_state.position_rad[0] = 0.02 * index;
    point.path_parameter = 0.5 * index;
    point.time_from_start_s = 0.008 * index;
    trajectory->points.push_back(point);
  }
  const bool pose_ok = executor.submitPoseTrajectory(trajectory);
  const auto status = executor.update(feedback, 0.008);
  const bool one_owner = status.owner == fr10_motion::MotionOwner::POSE_PATH &&
                         executor.command().type ==
                             fr10_motion::CommandType::JOINT_POSITION;
  std::cout << "METRIC S_OWNERSHIP joint_accepted=" << (joint_ok ? 1 : 0)
            << " pose_accepted=" << (pose_ok ? 1 : 0)
            << " final_owner=" << fr10_motion::motionOwnerName(status.owner)
            << "\n";
  return joint_ok && pose_ok && one_owner ? Outcome::PASS : Outcome::FAIL;
}

Outcome controlledStopProfile() {
  fr10_motion::MotionExecutorConfig config;
  config.control_rate_hz = 125.0;
  config.limits = fr10_motion_test::limits();
  fr10_motion::MotionExecutor executor(config);
  fr10_motion::RobotState feedback;
  feedback.valid = true;
  executor.update(feedback, 0.008);
  fr10_motion::JointTarget target;
  target.position_rad[0] = 0.80;
  target.revision = 1;
  if (!executor.submitJointTarget(target)) {
    return Outcome::FAIL;
  }
  for (int cycle = 0; cycle < 60; ++cycle) {
    executor.update(feedback, 0.008);
    const auto command = executor.command();
    feedback.joint_position_rad = command.joint_position_rad;
    feedback.joint_velocity_rad_s = command.joint_velocity_rad_s;
    feedback.joint_acceleration_rad_s2 = command.joint_acceleration_rad_s2;
  }
  const double velocity_before =
      std::abs(executor.command().joint_velocity_rad_s[0]);
  const double acceleration_before =
      executor.command().joint_acceleration_rad_s2[0];
  executor.stop();
  bool emitted_deceleration_command =
      executor.command().type == fr10_motion::CommandType::JOINT_POSITION;
  bool limits_respected = true;
  bool reversed = false;
  int stop_cycles = 0;
  double max_acceleration = 0.0;
  double max_jerk = 0.0;
  double previous_acceleration = acceleration_before;
  while (executor.status().state == fr10_motion::MotionState::STOPPING &&
         stop_cycles++ < 1000) {
    executor.update(feedback, 0.008);
    const auto command = executor.command();
    emitted_deceleration_command = emitted_deceleration_command &&
        command.type == fr10_motion::CommandType::JOINT_POSITION;
    max_acceleration =
        std::max(max_acceleration,
                 std::abs(command.joint_acceleration_rad_s2[0]));
    max_jerk = std::max(
        max_jerk,
        std::abs(command.joint_acceleration_rad_s2[0] -
                 previous_acceleration) /
            0.008);
    previous_acceleration = command.joint_acceleration_rad_s2[0];
    reversed = reversed || command.joint_velocity_rad_s[0] < -1e-8;
    limits_respected = limits_respected &&
        std::abs(command.joint_velocity_rad_s[0]) <=
            config.limits.max_joint_velocity_rad_s[0] + 1e-7 &&
        std::abs(command.joint_acceleration_rad_s2[0]) <=
            config.limits.max_joint_acceleration_rad_s2[0] + 1e-7;
    feedback.joint_position_rad = command.joint_position_rad;
    feedback.joint_velocity_rad_s = command.joint_velocity_rad_s;
    feedback.joint_acceleration_rad_s2 = command.joint_acceleration_rad_s2;
  }
  const auto final_command = executor.command();
  std::cout << "METRIC S_CONTROLLED_STOP_PROFILE velocity_before_rad_s="
            << velocity_before
            << " deceleration_command_emitted="
            << (emitted_deceleration_command ? 1 : 0)
            << " stop_cycles=" << stop_cycles
            << " max_acceleration_rad_s2=" << max_acceleration
            << " max_jerk_rad_s3=" << max_jerk
            << " reversed=" << (reversed ? 1 : 0)
            << " final_velocity_rad_s="
            << std::abs(final_command.joint_velocity_rad_s[0])
            << " final_state="
            << fr10_motion::motionStateName(executor.status().state) << "\n";
  return velocity_before > 0.05 && emitted_deceleration_command &&
                 stop_cycles > 1 && stop_cycles < 1000 && limits_respected &&
                 max_jerk <= config.limits.max_joint_jerk_rad_s3[0] + 1e-5 &&
                 !reversed &&
                 std::abs(final_command.joint_velocity_rad_s[0]) < 1e-9 &&
                 executor.status().state == fr10_motion::MotionState::HOLDING
             ? Outcome::PASS
             : Outcome::FAIL;
}

Outcome pathFaultHysteresis() {
  fr10_motion::CartesianFaultMonitorConfig config;
  config.persistence_s = 0.024;
  fr10_motion::CartesianFaultMonitor monitor(config);
  fr10_motion::CartesianTrackingErrors violation;
  violation.along_track_error_m = 0.006;
  const auto first = monitor.update(violation, true, 0.008);
  const auto second = monitor.update(violation, true, 0.008);
  const auto third = monitor.update(violation, true, 0.008);
  fr10_motion::CartesianTrackingErrors normal;
  const auto cleared = monitor.update(normal, true, 0.008);
  const auto spike = monitor.update(violation, true, 0.008);
  const auto recovered = monitor.update(normal, true, 0.008);
  std::cout << "METRIC S_PATH_FAULT_HYSTERESIS first="
            << static_cast<int>(first) << " second=" << static_cast<int>(second)
            << " third=" << static_cast<int>(third)
            << " cleared=" << static_cast<int>(cleared)
            << " spike=" << static_cast<int>(spike)
            << " recovered=" << static_cast<int>(recovered) << "\n";
  return first == fr10_motion::CartesianFaultState::WARNING &&
                 second == fr10_motion::CartesianFaultState::WARNING &&
                 third ==
                     fr10_motion::CartesianFaultState::PERSISTENT_FAULT &&
                 cleared == fr10_motion::CartesianFaultState::NORMAL &&
                 spike == fr10_motion::CartesianFaultState::WARNING &&
                 recovered == fr10_motion::CartesianFaultState::NORMAL
             ? Outcome::PASS
             : Outcome::FAIL;
}

Outcome stalePoseActivation() {
  fr10_motion::MotionExecutorConfig config;
  config.control_rate_hz = 125.0;
  config.max_joint_reference_lead_rad = 0.01;
  config.limits = fr10_motion_test::limits();
  fr10_motion::MotionExecutor executor(config);
  fr10_motion::RobotState feedback;
  feedback.valid = true;
  executor.update(feedback, 0.008);

  auto trajectory = std::make_shared<fr10_motion::CartesianJointTrajectory>();
  trajectory->generation = 77;
  trajectory->planning_timestamp_s = 12.5;
  trajectory->seed_joint_state.position_rad = feedback.joint_position_rad;
  for (std::size_t index = 0; index < 2; ++index) {
    fr10_motion::CartesianTrajectoryPoint point;
    point.joint_state.position_rad[0] = 0.02 * index;
    point.time_from_start_s = 0.008 * index;
    point.path_parameter = static_cast<double>(index);
    trajectory->points.push_back(point);
  }
  auto prepared = executor.preparePoseTrajectory(trajectory);
  feedback.joint_position_rad[0] = 0.05;
  executor.update(feedback, 0.008);
  const auto activation =
      executor.activatePreparedPoseTrajectory(std::move(prepared));
  std::cout << "METRIC S_STALE_POSE_ACTIVATION result="
            << static_cast<int>(activation)
            << " executor_state="
            << fr10_motion::motionStateName(executor.status().state)
            << " generation=" << trajectory->generation
            << " plan_stamp=" << trajectory->planning_timestamp_s << "\n";
  return activation == fr10_motion::PoseActivationResult::STALE &&
                 executor.status().state != fr10_motion::MotionState::FAULT
             ? Outcome::PASS
             : Outcome::FAIL;
}

Outcome poseFaultControlledStop() {
  fr10_motion::MotionExecutorConfig config;
  config.control_rate_hz = 125.0;
  config.max_joint_reference_lead_rad = 1.0;
  config.limits = fr10_motion_test::limits();
  fr10_motion::MotionExecutor executor(config);
  fr10_motion::RobotState feedback;
  feedback.valid = true;
  executor.update(feedback, 0.008);

  auto trajectory = std::make_shared<fr10_motion::CartesianJointTrajectory>();
  trajectory->generation = 88;
  for (std::size_t index = 0; index < 201; ++index) {
    fr10_motion::CartesianTrajectoryPoint point;
    point.joint_state.position_rad[0] = 0.001 * index;
    point.joint_state.velocity_rad_s[0] = index == 200 ? 0.0 : 0.125;
    point.time_from_start_s = 0.008 * index;
    point.path_parameter = static_cast<double>(index) / 200.0;
    trajectory->points.push_back(point);
  }
  if (!executor.submitPoseTrajectory(trajectory)) {
    return Outcome::FAIL;
  }
  for (int cycle = 0; cycle < 30; ++cycle) {
    executor.update(feedback, 0.008);
    const auto command = executor.command();
    feedback.joint_position_rad = command.joint_position_rad;
    feedback.joint_velocity_rad_s = command.joint_velocity_rad_s;
    feedback.joint_acceleration_rad_s2 = command.joint_acceleration_rad_s2;
  }
  const double velocity_before =
      std::abs(executor.command().joint_velocity_rad_s[0]);
  executor.reportCartesianTrackingError(0.006, 0.03, 0.003, true);
  executor.update(feedback, 0.008);
  const bool entered_stopping =
      executor.status().state == fr10_motion::MotionState::STOPPING;
  int cycles = 0;
  while (executor.status().state == fr10_motion::MotionState::STOPPING &&
         cycles++ < 1000) {
    const auto command = executor.command();
    feedback.joint_position_rad = command.joint_position_rad;
    feedback.joint_velocity_rad_s = command.joint_velocity_rad_s;
    feedback.joint_acceleration_rad_s2 = command.joint_acceleration_rad_s2;
    executor.update(feedback, 0.008);
  }
  const auto status = executor.status();
  std::cout << "METRIC S_POSE_FAULT_CONTROLLED_STOP velocity_before_rad_s="
            << velocity_before
            << " entered_stopping=" << (entered_stopping ? 1 : 0)
            << " stop_cycles=" << cycles
            << " final_state=" << fr10_motion::motionStateName(status.state)
            << " final_owner=" << fr10_motion::motionOwnerName(status.owner)
            << "\n";
  return velocity_before > 0.05 && entered_stopping && cycles < 1000 &&
                 status.state == fr10_motion::MotionState::HOLDING &&
                 status.owner == fr10_motion::MotionOwner::POSE_PATH
             ? Outcome::PASS
             : Outcome::FAIL;
}

Outcome faultTransition(bool emergency_stop, bool collision,
                        const std::string &id) {
  fr10_motion::MotionExecutorConfig config;
  config.control_rate_hz = 125.0;
  config.limits = fr10_motion_test::limits();
  fr10_motion::MotionExecutor executor(config);
  fr10_motion::RobotState feedback;
  feedback.valid = true;
  executor.update(feedback, 0.008);
  fr10_motion::JointTarget target;
  target.position_rad[0] = 0.30;
  target.revision = 1;
  executor.submitJointTarget(target);
  executor.update(feedback, 0.008);
  feedback.emergency_stop = emergency_stop;
  feedback.collision_detected = collision;
  const auto status = executor.update(feedback, 0.008);
  std::cout << "METRIC " << id
            << " state=" << fr10_motion::motionStateName(status.state)
            << " owner=" << fr10_motion::motionOwnerName(status.owner)
            << "\n";
  return status.state == fr10_motion::MotionState::FAULT &&
                 status.owner == fr10_motion::MotionOwner::NONE
             ? Outcome::PASS
             : Outcome::FAIL;
}

Outcome recoverySemantics() {
  fr10_motion::MotionExecutorConfig config;
  config.control_rate_hz = 125.0;
  config.limits = fr10_motion_test::limits();
  fr10_motion::MotionExecutor executor(config);
  fr10_motion::RobotState feedback;
  feedback.valid = true;
  executor.update(feedback, 0.008);
  fr10_motion::JointTarget target;
  target.position_rad[0] = 0.30;
  target.revision = 1;
  executor.submitJointTarget(target);
  executor.update(feedback, 0.008);
  feedback.main_error_code = 7;
  executor.update(feedback, 0.008);
  const bool unsafe_reset = executor.resetFault();
  feedback.main_error_code = 0;
  const auto cleared_but_latched = executor.update(feedback, 0.008);
  const bool reset = executor.resetFault();
  const auto status = executor.update(feedback, 0.008);
  const auto command = executor.command();
  std::cout << "METRIC S_RECOVERY unsafe_reset=" << (unsafe_reset ? 1 : 0)
            << " cleared_state="
            << fr10_motion::motionStateName(cleared_but_latched.state)
            << " reset=" << (reset ? 1 : 0)
            << " state=" << fr10_motion::motionStateName(status.state)
            << " owner=" << fr10_motion::motionOwnerName(status.owner)
            << " command=" << fr10_motion::commandTypeName(command.type)
            << "\n";
  return !unsafe_reset &&
                 cleared_but_latched.state == fr10_motion::MotionState::FAULT &&
                 reset && status.state == fr10_motion::MotionState::IDLE &&
                 status.owner == fr10_motion::MotionOwner::NONE &&
                 command.type == fr10_motion::CommandType::NONE
             ? Outcome::PASS
             : Outcome::FAIL;
}

Outcome targetRejectedAfterEmergencyIndication() {
  fr10_motion::MotionExecutorConfig config;
  config.control_rate_hz = 125.0;
  config.limits = fr10_motion_test::limits();
  fr10_motion::MotionExecutor executor(config);
  fr10_motion::RobotState feedback;
  feedback.valid = true;
  executor.update(feedback, 0.008);
  feedback.emergency_stop = true;
  const auto fault_status = executor.update(feedback, 0.008);
  fr10_motion::JointTarget target;
  target.position_rad[0] = 0.10;
  target.revision = 1;
  const bool accepted = executor.submitJointTarget(target);
  const auto final_status = executor.status();
  std::cout << "METRIC S_TARGET_REJECTED_AFTER_ESTOP_INDICATION"
            << " target_accepted=" << (accepted ? 1 : 0)
            << " state="
            << fr10_motion::motionStateName(final_status.state)
            << " owner="
            << fr10_motion::motionOwnerName(final_status.owner) << "\n";
  return fault_status.state == fr10_motion::MotionState::FAULT && !accepted &&
                 final_status.state == fr10_motion::MotionState::FAULT &&
                 final_status.owner == fr10_motion::MotionOwner::NONE
             ? Outcome::PASS
             : Outcome::FAIL;
}

std::map<std::string, Case> cases() {
  return {
      {"RT_STREAM_10HZ", [] {
         return streamCase(10.0, "RT_STREAM_10HZ", false);
       }},
      {"RT_STREAM_20HZ", [] {
         return streamCase(20.0, "RT_STREAM_20HZ", false);
       }},
      {"RT_STREAM_50HZ", [] {
         return streamCase(50.0, "RT_STREAM_50HZ", false);
       }},
      {"RT_STREAM_100HZ", [] {
         return streamCase(100.0, "RT_STREAM_100HZ", false);
       }},
      {"RT_DIRECTION_REVERSAL", directionReversal},
      {"RT_RAPID_ALTERNATION", [] {
         return streamCase(100.0, "RT_RAPID_ALTERNATION", true);
       }},
      {"RT_POSE_NOMINAL_TIMESTAMP_PLAYBACK", [] {
         return poseTimestampPlayback(0.008, 0.01,
                                      "RT_POSE_NOMINAL_TIMESTAMP_PLAYBACK");
       }},
      {"RT_POSE_MISSED_CYCLE_PLAYBACK", [] {
         return poseTimestampPlayback(0.016, 0.01,
                                      "RT_POSE_MISSED_CYCLE_PLAYBACK");
       }},
      {"S_NAN_POSE", [] {
         return invalidPose(std::numeric_limits<double>::quiet_NaN(),
                            "S_NAN_POSE");
       }},
      {"S_INF_POSE", [] {
         return invalidPose(std::numeric_limits<double>::infinity(),
                            "S_INF_POSE");
       }},
      {"S_INVALID_QUAT", invalidQuaternion},
      {"S_OWNERSHIP", ownershipCollision},
      {"S_CONTROLLED_STOP_PROFILE", controlledStopProfile},
      {"S_PATH_FAULT_HYSTERESIS", pathFaultHysteresis},
      {"S_STALE_POSE_ACTIVATION", stalePoseActivation},
      {"S_POSE_FAULT_CONTROLLED_STOP", poseFaultControlledStop},
      {"S_FAULT_TRANSITION", [] {
         return faultTransition(false, true, "S_FAULT_TRANSITION");
       }},
      {"S_HARDWARE_ESTOP_INDICATION", [] {
         return faultTransition(true, false, "S_HARDWARE_ESTOP_INDICATION");
       }},
      {"S_RECOVERY", recoverySemantics},
      {"S_TARGET_REJECTED_AFTER_ESTOP_INDICATION",
       targetRejectedAfterEmergencyIndication},
  };
}

} // namespace

int main(int argc, char **argv) {
  const auto registry = cases();
  if (argc != 3 || std::string(argv[1]) != "--case") {
    std::cerr << "Usage: " << argv[0] << " --case CASE_ID\n";
    return 2;
  }
  const auto found = registry.find(argv[2]);
  if (found == registry.end()) {
    std::cerr << "Unknown case: " << argv[2] << "\n";
    return 2;
  }
  const Outcome outcome = found->second();
  if (outcome == Outcome::SKIP) {
    return 77;
  }
  std::cout << (outcome == Outcome::PASS ? "PASS " : "FAIL ")
            << found->first << "\n";
  return outcome == Outcome::PASS ? 0 : 1;
}
