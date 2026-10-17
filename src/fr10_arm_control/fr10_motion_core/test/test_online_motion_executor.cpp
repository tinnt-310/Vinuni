#include <iostream>
#include <limits>

#include "fr10_motion_core/motion_controller.hpp"

namespace {

bool check(bool condition, const char *name) {
  if (!condition) {
    std::cerr << "FAIL " << name << '\n';
  }
  return condition;
}

fr10_motion::MotionExecutorConfig configuration() {
  fr10_motion::MotionExecutorConfig config;
  config.control_rate_hz = 125.0;
  config.require_command_ack = true;
  config.max_joint_reference_lead_rad = 1.0;
  config.limits.min_joint_position_rad.fill(-3.0);
  config.limits.max_joint_position_rad.fill(3.0);
  config.limits.max_joint_velocity_rad_s.fill(0.5);
  config.limits.max_joint_acceleration_rad_s2.fill(1.0);
  config.limits.max_joint_jerk_rad_s3.fill(5.0);
  return config;
}

} // namespace

int main() {
  bool ok = true;
  fr10_motion::MotionExecutor executor(configuration());
  fr10_motion::RobotState feedback;
  feedback.valid = true;
  feedback.safety_known = true;
  feedback.feedback_sequence = 1;
  feedback.timestamp_s = 1.0;
  executor.update(feedback, 0.008);

  fr10_motion::OnlineJointTarget target;
  target.position_rad[0] = 0.1;
  target.corrected_pose.position_m = {0.4, 0.0, 0.5};
  target.desired_translation_tangent = {0.0, 0.0, -1.0};
  target.generation = 7;
  target.sequence = 1;
  target.wrench_sequence = 11;
  target.scene_revision = 5;
  target.timestamp_s = 1.0;
  target.valid_until_s = 1.050;
  ok &= check(executor.submitOnlineJointTarget(target),
              "ONLINE_TARGET_ACCEPTED");
  ok &= check(!executor.submitOnlineJointTarget(target),
              "ONLINE_TARGET_REPLAY_REJECTED");

  feedback.feedback_sequence++;
  feedback.timestamp_s += 0.008;
  auto status = executor.update(feedback, 0.008);
  const auto command = executor.command();
  ok &= check(status.state == fr10_motion::MotionState::TRACKING &&
                  status.owner == fr10_motion::MotionOwner::FORCE_ONLINE &&
                  status.active_online_sequence == 1 &&
                  status.active_wrench_sequence == 11 &&
                  status.active_scene_revision == 5 &&
                  status.cartesian_reference_valid &&
                  command.type == fr10_motion::CommandType::JOINT_POSITION,
              "ONLINE_TARGET_STATUS_AND_JOINT_OUTPUT");
  ok &= check(executor.acknowledgeCommand(command.sequence),
              "ONLINE_COMMAND_ACKNOWLEDGED");

  target.sequence = 2;
  target.wrench_sequence = 12;
  target.timestamp_s = feedback.timestamp_s;
  target.valid_until_s = feedback.timestamp_s + 0.050;
  ok &= check(executor.submitOnlineJointTarget(target),
              "ONLINE_RETARGET_ACCEPTED");

  feedback.feedback_sequence++;
  feedback.timestamp_s = target.valid_until_s + 0.001;
  status = executor.update(feedback, 0.008);
  ok &= check(status.state == fr10_motion::MotionState::STOPPING ||
                  status.state == fr10_motion::MotionState::HOLDING,
              "ONLINE_EXPIRY_CONTROLLED_STOP");

  fr10_motion::MotionExecutor delayed(configuration());
  feedback.timestamp_s = 2.032;
  delayed.update(feedback, .008);
  target.timestamp_s = 2.0; target.valid_until_s = 2.048;
  target.command_lease_s = .048;
  ok &= check(delayed.submitOnlineJointTarget(target), "FRESH_DELAYED_VALIDATION_ACCEPTED");
  feedback.timestamp_s = 2.056; feedback.feedback_sequence++;
  status = delayed.update(feedback, .008);
  ok &= check(status.state == fr10_motion::MotionState::TRACKING,
              "VALIDATION_LATENCY_DOES_NOT_CONSUME_ACCEPTED_COMMAND_LEASE");
  target.sequence++;
  ok &= check(!delayed.submitOnlineJointTarget(target), "EXPIRED_PROPOSAL_CANNOT_RENEW_LEASE");
  feedback.timestamp_s = 2.081; feedback.feedback_sequence++;
  status = delayed.update(feedback, .008);
  ok &= check(status.state == fr10_motion::MotionState::STOPPING ||
                  status.state == fr10_motion::MotionState::HOLDING,
              "ACCEPTED_COMMAND_LEASE_EXPIRY_STILL_STOPS");

  fr10_motion::MotionExecutor handover(configuration());
  feedback.timestamp_s = 3.040; handover.update(feedback, .008);
  target.timestamp_s = 3.; target.valid_until_s = 3.048; target.command_lease_s = .064;
  ok &= check(handover.submitOnlineJointTarget(target), "BOUNDED_HANDOVER_LEASE_ACCEPTED");
  feedback.timestamp_s = 3.088; feedback.feedback_sequence++;
  ok &= check(handover.update(feedback, .008).state == fr10_motion::MotionState::TRACKING,
              "WORKER_DEADLINE_PLUS_CONSUMER_PERIOD_DOES_NOT_PREMATURELY_BRAKE");
  target.timestamp_s = 3.048; target.valid_until_s = 3.096; target.sequence++;
  ok &= check(handover.submitOnlineJointTarget(target), "FRESH_HANDOVER_REPLACEMENT_ACCEPTED");
  target.sequence++; target.command_lease_s = .100;
  ok &= check(!handover.submitOnlineJointTarget(target), "UNBOUNDED_COMMAND_LEASE_REJECTED");

  fr10_motion::MotionExecutor before_tick(configuration());
  feedback.timestamp_s=4.;before_tick.update(feedback,.008);
  target.sequence++;target.timestamp_s=4.;target.valid_until_s=4.048;target.command_lease_s=.064;
  ok &= check(before_tick.submitOnlineJointTarget(target,4.040),"CURRENT_ADMISSION_CLOCK_ANCHORS_LEASE");
  ok &= check(std::abs(before_tick.status().online_valid_until_s-4.104)<1e-12,
      "ONLINE_LEASE_DEADLINE_IS_OBSERVABLE_AT_ADMISSION");
  feedback.timestamp_s=4.096;feedback.feedback_sequence++;
  ok &= check(before_tick.update(feedback,.008).state==fr10_motion::MotionState::TRACKING,
      "PREVIOUS_TICK_CLOCK_DOES_NOT_SHORTEN_ACCEPTED_LEASE");
  feedback.timestamp_s=4.105;feedback.feedback_sequence++;
  status=before_tick.update(feedback,.008);
  ok &= check(status.state==fr10_motion::MotionState::STOPPING || status.state==fr10_motion::MotionState::HOLDING,
      "CURRENT_CLOCK_LEASE_STILL_EXPIRES");
  fr10_motion::MotionExecutor clocks(configuration());
  feedback.timestamp_s=5.;clocks.update(feedback,.008);
  target.timestamp_s=5.;target.valid_until_s=5.048;target.sequence++;
  ok &= check(!clocks.submitOnlineJointTarget(target,4.99),"PAST_ADMISSION_CLOCK_REJECTED");
  ok &= check(!clocks.submitOnlineJointTarget(target,5.049),"EXPIRED_PROPOSAL_CANNOT_BE_ADMITTED");
  ok &= check(!clocks.submitOnlineJointTarget(target,std::numeric_limits<double>::quiet_NaN()),"NONFINITE_ADMISSION_CLOCK_REJECTED");
  return ok ? 0 : 1;
}
