#include "fr10_motion_core/stop_reason_policy.hpp"

#include <atomic>
#include <iostream>

int main() {
  using fr10_motion::StopReason;
  using fr10_motion::recordWorkerStopReason;
  using fr10_motion::recordGoalReachedStopReason;
  std::atomic<StopReason> reason{StopReason::NONE};
  std::atomic<bool> stopped{false};
  std::atomic<StopReason> pending{StopReason::NONE};
  auto expect = [&](bool condition, const char *message) {
    if (!condition) std::cerr << message << '\n';
    return condition;
  };

  if (!expect(recordWorkerStopReason(reason, stopped, StopReason::IK_FAILURE) &&
                  reason.load() == StopReason::IK_FAILURE,
              "worker rejection must be visible while active")) return 1;
  reason.store(StopReason::COMM_FAILURE);
  if (!expect(!recordWorkerStopReason(reason, stopped, StopReason::IK_FAILURE) &&
                  reason.load() == StopReason::COMM_FAILURE,
              "worker must not overwrite a latched communication fault")) return 1;
  reason.store(StopReason::REFERENCE_LAG);
  if (!expect(!recordWorkerStopReason(reason, stopped, StopReason::PATH_DEVIATION) &&
                  reason.load() == StopReason::REFERENCE_LAG,
              "worker must not overwrite a reference-lag stop")) return 1;
  reason.store(StopReason::NONE);
  stopped.store(true);
  if (!expect(!recordWorkerStopReason(reason, stopped, StopReason::IK_FAILURE) &&
                  reason.load() == StopReason::NONE,
              "worker must not report after shutdown starts")) return 1;
  stopped.store(false);
  reason.store(StopReason::GOAL_REACHED);
  if (!expect(recordWorkerStopReason(reason, stopped, StopReason::PATH_DEVIATION) &&
                  reason.load() == StopReason::PATH_DEVIATION,
              "a new worker fault may replace a prior goal outcome")) return 1;
  reason.store(StopReason::NONE);
  if (!expect(recordGoalReachedStopReason(reason, stopped, pending, true) &&
                  reason.load() == StopReason::GOAL_REACHED,
              "control owner must record a settled goal")) return 1;
  reason.store(StopReason::COMM_FAILURE);
  if (!expect(!recordGoalReachedStopReason(reason, stopped, pending, true) &&
                  reason.load() == StopReason::COMM_FAILURE,
              "goal observation must not overwrite a latched fault")) return 1;
  reason.store(StopReason::NONE);
  pending.store(StopReason::PATH_DEVIATION);
  if (!expect(!recordGoalReachedStopReason(reason, stopped, pending, true) &&
                  reason.load() == StopReason::NONE,
              "pending safety stop must suppress goal outcome")) return 1;
  pending.store(StopReason::NONE);
  stopped.store(true);
  if (!expect(!recordGoalReachedStopReason(reason, stopped, pending, true) &&
                  reason.load() == StopReason::NONE,
              "shutdown must suppress goal outcome")) return 1;
  stopped.store(false);
  for (const auto fault : {StopReason::REFERENCE_LAG,StopReason::PATH_DEVIATION,
       StopReason::SCENE_CHANGED,StopReason::HARDWARE_ALARM,StopReason::COMM_FAILURE,
       StopReason::IK_FAILURE,StopReason::SINGULARITY,StopReason::STALE_TRAJECTORY}) {
    if (!expect(fr10_motion::invalidatesForceTracking(fault),
        "latched motion fault must terminate force staging")) return 1;
  }
  for (const auto normal : {StopReason::NONE,StopReason::GOAL_REACHED,
       StopReason::RETARGET_TIMING,StopReason::CONTROLLED_STOP}) {
    if (!expect(!fr10_motion::invalidatesForceTracking(normal),
        "planned timing stop and verified HOLD must preserve force session")) return 1;
  }
  if (!expect(!recordGoalReachedStopReason(reason, stopped, pending, false) &&
                  reason.load() == StopReason::NONE,
              "non-goal status must not become goal reached")) return 1;
  return 0;
}
