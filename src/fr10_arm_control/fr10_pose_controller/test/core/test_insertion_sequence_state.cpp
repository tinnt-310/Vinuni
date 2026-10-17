#include "fr10_motion_core/insertion_sequence_state.hpp"
#include "test_support.hpp"

int main() {
  fr10_motion_test::Check check;
  fr10_motion::InsertionSequenceState state;
  fr10_motion::CartesianTarget socket;
  socket.position_m = {0.2, 0.3, 0.4};
  socket.orientation_quat = {0.0, 0.0, 0.0, 1.0};
  fr10_motion::Pose staging_pose;
  staging_pose.position_m = {0.2, 0.3, 0.3};
  staging_pose.orientation_quat = socket.orientation_quat;

  check.require(!state.isDuplicate(socket, 0.001, 0.01),
                "empty_duplicate", "empty state has no duplicate target");
  state.arm(socket, staging_pose, 42);
  check.require(state.isDuplicate(socket, 0.001, 0.01),
                "staged_duplicate", "pending insertion is the duplicate reference");
  auto moved = socket;
  moved.position_m[0] += 0.002;
  check.require(!state.isDuplicate(moved, 0.001, 0.01),
                "changed_target", "changed insertion remains queueable");
  check.require(!state.pendingForGeneration(41).has_value(),
                "wrong_generation", "unrelated motion cannot consume staging");
  const auto staged = state.pendingForGeneration(42);
  check.require(staged && staged->staging_generation == 42 &&
                    staged->pre_insert_pose.position_m == staging_pose.position_m,
                "staged_snapshot", "staging snapshot preserves pose and generation");
  state.rejectStaging(41);
  check.require(state.pendingForGeneration(42).has_value(),
                "unrelated_rejection", "unrelated rejection retains staging");
  check.require(!state.takeIfGeneration(41),
                "unrelated_take", "wrong generation must not clear staging");
  check.require(state.takeIfGeneration(42) &&
                    !state.pendingForGeneration(42).has_value(),
                "matching_take", "matching generation consumes staging once");
  state.retain(socket);
  const auto retract = state.beginRetract();
  check.require(retract && retract->position_m == socket.position_m,
                "retract", "retract uses the last validated insertion target");

  state.arm(moved, staging_pose, 43);
  state.rejectStaging(43);
  check.require(!state.pendingForGeneration(43).has_value() &&
                    state.beginRetract().has_value(),
                "matching_rejection", "rejection clears pending but keeps last socket");
  state.markDirect(socket);
  check.require(!state.pendingForGeneration(43).has_value() &&
                    state.beginRetract().has_value(),
                "direct", "direct insertion replaces pending sequence");
  state.clear();
  check.require(!state.beginRetract().has_value() &&
                    !state.isDuplicate(socket, 0.001, 0.01),
                "stop_clear", "controlled stop clears pending and retained target");
  return check.result();
}
