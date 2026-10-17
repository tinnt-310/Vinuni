#pragma once

#include <atomic>

namespace fr10_motion {

enum class StopReason : int {
  NONE = 0,
  USER_INTERRUPT,
  GOAL_REACHED,
  SINGULARITY,
  HARDWARE_ALARM,
  COMM_FAILURE,
  IK_FAILURE,
  PATH_DEVIATION,
  REFERENCE_LAG,
  SCENE_CHANGED,
  CONTROLLED_STOP,
  RETARGET_TIMING,
  STALE_TRAJECTORY
};

inline bool invalidatesForceTracking(StopReason reason) {
  switch (reason) {
  case StopReason::SINGULARITY:
  case StopReason::HARDWARE_ALARM:
  case StopReason::COMM_FAILURE:
  case StopReason::IK_FAILURE:
  case StopReason::PATH_DEVIATION:
  case StopReason::REFERENCE_LAG:
  case StopReason::SCENE_CHANGED:
  case StopReason::STALE_TRAJECTORY:
    return true;
  default:
    return false;
  }
}

// A planning worker may report rejection only while no stop is underway.
// Existing safety/controlled-stop causes remain latched until the control
// owner explicitly resets them for a new goal. The CAS also handles a stop
// racing with a worker rejection without a mutex in the 125 Hz path.
inline bool recordWorkerStopReason(std::atomic<StopReason> &reason,
                                   const std::atomic<bool> &stop_requested,
                                   StopReason candidate) {
  auto current = reason.load(std::memory_order_acquire);
  while (!stop_requested.load(std::memory_order_acquire) &&
         (current == StopReason::NONE ||
          current == StopReason::GOAL_REACHED ||
          current == StopReason::IK_FAILURE)) {
    if (reason.compare_exchange_weak(current, candidate,
                                     std::memory_order_acq_rel,
                                     std::memory_order_acquire)) {
      return true;
    }
  }
  return false;
}

// Only the control owner may turn a settled executor outcome into a stop
// reason. A delayed diagnostics snapshot must never hide a latched fault.
inline bool recordGoalReachedStopReason(
    std::atomic<StopReason> &reason,
    const std::atomic<bool> &stop_requested,
    const std::atomic<StopReason> &pending_stop,
    bool settled_goal) {
  if (!settled_goal || stop_requested.load(std::memory_order_acquire) ||
      pending_stop.load(std::memory_order_acquire) != StopReason::NONE) {
    return false;
  }
  auto expected = StopReason::NONE;
  return reason.compare_exchange_strong(expected, StopReason::GOAL_REACHED,
                                        std::memory_order_acq_rel,
                                        std::memory_order_acquire);
}

}  // namespace fr10_motion
