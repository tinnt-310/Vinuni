#include <iostream>
#include "fr10_motion_core/motion_coordinator.hpp"

int main() {
  using namespace fr10_motion;
  bool ok = true;
  auto check = [&](bool valid, const char *name) { if (!valid) { std::cerr << name << '\n'; ok = false; } };
  MotionCoordinator coordinator(1e-5, 1e-5, 1e-5);
  CommandIntent a;
  a.joint.position_rad[0] = 0.1;
  const auto first = coordinator.request(a, 1, 2);
  check(first.accepted, "A accepted");
  check(coordinator.request(a, 1, 3).coalesced, "same active A coalesced");
  coordinator.transition(first.report.id, CommandPhase::EXECUTING);
  CommandIntent b; b.mode = ControlMode::POSE; b.pose.position_m[0] = 0.4;
  const auto second = coordinator.request(b, 1, 4);
  check(second.accepted && !coordinator.owns(first.report.id), "POSE owns new intent");
  check(!coordinator.transition(first.report.id, CommandPhase::EXECUTING), "old worker cannot activate");
  const auto again = coordinator.request(a, 1, 5);
  check(again.accepted && again.report.generation > second.report.generation,
        "JOINT A after POSE B is NEW intent");
  check(!coordinator.request(b, 1, 5, true).accepted && coordinator.owns(again.report.id),
        "BUSY cannot replace current command");
  coordinator.stop(CommandReason::USER_STOP, false);
  coordinator.transition(again.report.id, CommandPhase::STOPPED);
  check(coordinator.request(a, 1, 6).accepted, "same A after stop accepted");
  for (int i = 0; i < 100; ++i) coordinator.reject(ControlMode::JOINT, CommandReason::INVALID_INPUT);
  check(coordinator.droppedEvents() > 0, "bounded event ring");
  return ok ? 0 : 1;
}
