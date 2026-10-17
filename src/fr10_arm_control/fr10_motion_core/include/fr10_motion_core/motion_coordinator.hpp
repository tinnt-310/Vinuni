#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <optional>
#include "fr10_motion_core/core_types.hpp"
#include "fr10_motion_core/cartesian_trajectory.hpp"

namespace fr10_motion {

enum class CommandPhase : uint8_t {
  ACCEPTED, PLANNING, READY, EXECUTING, SUCCEEDED, STOPPING, STOPPED,
  REJECTED, SUPERSEDED, COALESCED
};
enum class CommandReason : uint16_t {
  NONE, DUPLICATE, BUSY, INVALID_INPUT, NOT_READY, PLANNING_FAILED,
  STALE_BOUNDARY, SCENE_CHANGED, USER_STOP, BACKEND_FAILURE, TRACKING_FAULT
};
struct CommandIntent {
  uint64_t session_id{0};
  ControlMode mode{ControlMode::JOINT};
  JointTarget joint;
  CartesianTarget pose;
  PoseExecutionPolicy policy{PoseExecutionPolicy::FREE_SPACE};
};
struct CommandReport {
  uint64_t id{0};
  uint64_t generation{0};
  uint64_t scene_revision{0};
  uint64_t boundary_sequence{0};
  ControlMode mode{ControlMode::JOINT};
  CommandPhase phase{CommandPhase::ACCEPTED};
  CommandReason reason{CommandReason::NONE};
};
struct CommandDecision {
  CommandReport report;
  bool accepted{false};
  bool coalesced{false};
};

// One intent owner across movement modes. This class knows neither ROS nor
// executors/planners. Generation checks protect every worker-result transition.
// A fixed event ring keeps diagnostic publication outside the control tick.
class MotionCoordinator {
public:
  MotionCoordinator(double joint_tolerance, double position_tolerance,
                    double angular_tolerance);
  CommandDecision request(const CommandIntent &intent, uint64_t scene_revision,
                          uint64_t boundary_sequence, bool exclusive_busy = false);
  CommandReport reject(ControlMode mode, CommandReason reason);
  bool transition(uint64_t id, CommandPhase phase,
                  CommandReason reason = CommandReason::NONE);
  bool owns(uint64_t id) const;
  void stop(CommandReason reason, bool retain_intent);
  std::optional<CommandReport> current() const;
  std::optional<CommandReport> nextEvent();
  uint64_t droppedEvents() const;

private:
  bool equivalent(const CommandIntent &a, const CommandIntent &b) const;
  void emit(const CommandReport &report);
  const double joint_tolerance_, position_tolerance_, angular_tolerance_;
  mutable std::mutex mutex_;
  std::optional<CommandIntent> intent_;
  std::optional<CommandReport> current_;
  uint64_t next_id_{0};
  uint64_t generation_{0};
  std::array<CommandReport, 64> events_{};
  std::size_t read_{0}, count_{0};
  uint64_t dropped_{0};
};
} // namespace fr10_motion
