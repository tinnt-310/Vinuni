#include "fr10_motion_core/motion_controller.hpp"
#include "test_support.hpp"

int main() {
  fr10_motion_test::Check check;
  using fr10_motion::ControlMode;
  using fr10_motion::StaleActivationRecovery;
  using fr10_motion::staleActivationRecovery;

  check.require(staleActivationRecovery(ControlMode::POSE, 0) ==
                    StaleActivationRecovery::REPLAN_LIVE &&
                    staleActivationRecovery(ControlMode::POSE, 2) ==
                        StaleActivationRecovery::REPLAN_LIVE,
                "POSE_STALE_LIVE_REPLANS",
                "first three stale pose activations should replan from live state");
  check.require(staleActivationRecovery(ControlMode::POSE, 3) ==
                    StaleActivationRecovery::CONTROLLED_STOP_REPLAN,
                "POSE_STALE_STOP_REPLAN",
                "exhausted moving replans should stop once before retry");
  check.require(staleActivationRecovery(ControlMode::POSE, 4) ==
                    StaleActivationRecovery::REJECT &&
                    staleActivationRecovery(ControlMode::JOINT, 4) ==
                        StaleActivationRecovery::REJECT,
                "STALE_RETRY_BOUND",
                "post-stop stale plans in either mode must not retry forever");
  check.require(staleActivationRecovery(ControlMode::JOINT, 3) ==
                    StaleActivationRecovery::CONTROLLED_STOP_REPLAN,
                "JOINT_STOP_REPLAN", "Joint and Pose share bounded stop/replan");
  check.require(fr10_motion::timingStopRetryAllowed(0) &&
                    !fr10_motion::timingStopRetryAllowed(1) &&
                    !fr10_motion::timingStopRetryAllowed(4),
                "TIMING_STOP_RETRY_BOUND",
                "one controlled-stop timing replan must be the maximum");
  return check.result();
}
