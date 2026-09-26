#include <deque>
#include <vector>
#include "fr10_pose_controller/motion_controller.hpp"
#include "test_support.hpp"

using namespace fr10_motion;

int main() {
  fr10_motion_test::Check check;
  constexpr double dt = 0.008;
  MotionExecutorConfig config;
  config.require_command_ack = true;
  config.limits = fr10_motion_test::limits();
  config.limits.max_joint_velocity_rad_s.fill(1.5);
  config.limits.max_joint_acceleration_rad_s2.fill(3.5);
  config.limits.max_joint_jerk_rad_s3.fill(20.0);
  config.reference_lag_persistence_samples = 30; // Legacy setting must not delay braking.

  for (bool pose : {false, true}) {
    for (int delay : {1, 2, 4, 8}) {
      MotionExecutor executor(config);
      RobotState feedback;
      feedback.valid = true;
      feedback.feedback_sequence = 1;
      executor.update(feedback, dt);
      JointTarget target;
      target.position_rad[0] = 1.2;
      target.revision = 1;
      bool accepted = false;
      if (pose) {
        auto trajectory = std::make_shared<CartesianJointTrajectory>();
        trajectory->generation = 1;
        JointOtg planner(dt, config.limits, config.max_joint_reference_lead_rad);
        planner.reset(feedback.joint_position_rad);
        planner.submitTarget(target);
        CartesianTrajectoryPoint point;
        trajectory->points.push_back(point);
        for (int tick = 1; tick < 2000; ++tick) {
          planner.step(feedback, point.joint_state.position_rad,
                       point.joint_state.velocity_rad_s, point.joint_state.acceleration_rad_s2);
          point.time_from_start_s = tick * dt;
          point.path_parameter = point.joint_state.position_rad[0] / 1.2;
          trajectory->points.push_back(point);
          if (planner.isFinished()) { break; }
        }
        accepted = executor.submitPoseTrajectory(trajectory);
      } else {
        accepted = executor.submitJointTarget(target);
      }
      const std::string id = std::string(pose ? "POSE_" : "JOINT_") + std::to_string(delay * 8) + "MS";
      check.require(accepted, id + "_ACCEPT", "valid trajectory rejected");
      std::deque<MotionCommand> queue(delay);
      std::vector<double> positions{0.0};
      MotionCommand previous;
      ReferenceLagEvent event;
      bool smooth = true, same_tick = true, saw_stop = false;
      MotionStatus status;
      for (int tick = 0; tick < 1500; ++tick) {
        const auto delayed = queue.front();
        queue.pop_front();
        feedback.joint_position_rad = delayed.joint_position_rad;
        feedback.joint_velocity_rad_s = delayed.joint_velocity_rad_s;
        feedback.joint_acceleration_rad_s2 = delayed.joint_acceleration_rad_s2;
        ++feedback.feedback_sequence;
        feedback.timestamp_s += dt;
        status = executor.update(feedback, pose && tick % 7 == 0 ? 0.032 : dt);
        const auto command = executor.command();
        if (status.lag_event.sequence != 0 && !saw_stop) {
          saw_stop = true;
          event = status.lag_event;
          JointOtg expected_stop(dt, config.limits, config.max_joint_reference_lead_rad);
          expected_stop.reset(previous.joint_position_rad, previous.joint_velocity_rad_s,
                              previous.joint_acceleration_rad_s2);
          expected_stop.startControlledStop();
          std::array<double, 6> q, v, a;
          expected_stop.step(feedback, q, v, a);
          same_tick = std::abs(q[0] - command.joint_position_rad[0]) < 1e-12 &&
              status.reference_lag_samples == 1;
          check.require(!executor.submitJointTarget(target), id + "_STOP_REJECTS_TARGET",
                        "target interrupted braking");
        }
        if (saw_stop && status.state == MotionState::STOPPING) {
          executor.stop(); // repeated requests must not restart braking
        }
        smooth = smooth && std::isfinite(command.joint_position_rad[0]) &&
            std::abs(command.joint_velocity_rad_s[0]) <= 1.5 + 1e-8 &&
            std::abs(command.joint_acceleration_rad_s2[0]) <= 3.5 + 1e-8 &&
            std::abs(command.joint_acceleration_rad_s2[0] - previous.joint_acceleration_rad_s2[0]) <= 20.0 * dt + 1e-7;
        positions.push_back(command.joint_position_rad[0]);
        executor.acknowledgeCommand(command.sequence);
        previous = command;
        queue.push_back(command);
      }
      // Derive from actual position stream, not just the declared v/a fields.
      std::vector<double> velocity, acceleration;
      for (std::size_t i = 1; i < positions.size(); ++i) {
        velocity.push_back((positions[i] - positions[i-1]) / dt);
      }
      for (std::size_t i = 1; i < velocity.size(); ++i) {
        acceleration.push_back((velocity[i] - velocity[i-1]) / dt);
        smooth = smooth && std::abs(acceleration.back()) < 3.5 + 1e-6;
      }
      for (std::size_t i = 1; i < acceleration.size(); ++i) {
        smooth = smooth && std::abs((acceleration[i] - acceleration[i-1]) / dt) < 20.0 + 1e-5;
      }
      check.require(smooth, id + "_QVAJ_CONTINUITY", "position-derived limits exceeded");
      check.require(status.state == MotionState::HOLDING, id + "_TERMINATES", "braking never completed");
      // Last-sent comparison is one sample behind candidate generation:
      // two queue slots correspond to one 8 ms last-sent tracking gap.
      check.require(delay <= 2 ? !saw_stop : saw_stop, id + "_LAG_POLICY", "unexpected lag decision");
      if (saw_stop) {
        check.require(same_tick, id + "_FIRST_BRAKE_SAME_TICK", "lag delayed or not seeded from sent reference");
        check.require(status.outcome == MotionOutcome::STOPPED_BEFORE_GOAL && !status.in_goal_tolerance,
                      id + "_NOT_GOAL", "aborted goal reported successful");
        check.require(status.lag_event.sequence == 1 && status.lag_event.error_rad == event.error_rad &&
                          status.lag_event.feedback_sequence == event.feedback_sequence,
                      id + "_IMMUTABLE_EVENT", "event restarted or snapshot changed");
        check.require(executor.submitJointTarget(target), id + "_EXPLICIT_RESUBMIT", "same goal cannot be resubmitted");
      }
    }
  }

  MotionExecutor executor(config);
  RobotState feedback;
  feedback.valid = true;
  feedback.feedback_sequence = 1;
  executor.update(feedback, dt);
  JointTarget target;
  target.position_rad[0] = 1.2;
  executor.submitJointTarget(target);
  for (int tick = 0; tick < 100; ++tick) {
    executor.update(feedback, dt); // Same cached feedback, never a new observation.
    executor.acknowledgeCommand(executor.command().sequence);
  }
  check.require(executor.status().state == MotionState::TRACKING,
                "DUPLICATE_NOT_NEW_OBSERVATION", "counted same feedback repeatedly");
  ++feedback.feedback_sequence;
  executor.update(feedback, dt);
  check.require(executor.status().state == MotionState::STOPPING,
                "FRESH_SAMPLE_STOPS", "fresh excess lag did not stop");
  feedback.joint_position_rad[0] = std::numeric_limits<double>::quiet_NaN();
  executor.update(feedback, dt);
  check.require(executor.status().state == MotionState::FAULT && executor.command().type == CommandType::NONE,
                "NAN_NO_COMMAND", "non-finite feedback did not clear output");
  return check.result();
}
