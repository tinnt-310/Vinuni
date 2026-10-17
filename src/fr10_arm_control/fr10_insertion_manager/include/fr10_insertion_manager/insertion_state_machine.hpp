#pragma once
#include "fr10_insertion_manager/insertion_progress_monitor.hpp"
#include "fr10_insertion_manager/contact_reference.hpp"

#include <array>
#include <cstdint>
#include <limits>

#include "fr10_force_control/contact_detector.hpp"
#include "fr10_force_control/force_safety_monitor.hpp"

namespace fr10_insertion {

enum class InsertionState : int {
  IDLE = 0,
  MOVE_PREINSERT,
  HOLD_PREINSERT,
  FT_ZERO,
  APPROACH,
  CONTACT_DETECTED,
  FORCE_INSERTION,
  VERIFY_INSERTION,
  SUCCESS,
  HOLD,
  RETRACT,
  STOPPING,
  FAILED,
  CONTACT_ALIGN,
  INSERT_COMPLIANT,
  SEATING,
  APPROACH_COMPLIANT
};

enum class InsertionProcessMode { AXIAL_CONTACT, MATCHED_MATING };
bool validInsertionAxes(const std::array<bool, 6> &selected, std::size_t axis,
                        InsertionProcessMode mode);

enum class CompletionBehavior : int { HOLD = 0, RETRACT };

enum class InsertionFault : int {
  NONE = 0,
  INVALID_GOAL,
  TIMEOUT,
  CANCELED,
  FORCE_SAFETY,
  CLOCK_JUMP,
  NO_PROGRESS,
  PLANNING_FAILED
};

const char *insertionFaultName(InsertionFault fault);

struct InsertionGoal {
  double preinsert_distance_m{0.02};
  double insertion_depth_m{0.01};
  double insertion_speed_m_s{0.002};
  std::array<bool, 6> selected_axes{};
  std::size_t insertion_axis{2};
  double desired_axis_force_n{10.0};
  double timeout_s{20.0};
  CompletionBehavior completion_behavior{CompletionBehavior::HOLD};
};

struct InsertionStateConfig {
  InsertionProcessMode process_mode{InsertionProcessMode::AXIAL_CONTACT};
  double preinsert_hold_s{0.10};
  double depth_tolerance_m{0.0005};
  double force_error_tolerance_n{1.0};
  double force_rate_tolerance_n_s{5.0};
  double completion_hold_s{0.20};
  double alignment_hold_s{0.08};
  double alignment_force_tolerance_n{1.0};
  double alignment_torque_tolerance_nm{0.10};
  double no_progress_timeout_s{2.0};
  double minimum_progress_m{0.0001};
  double minimum_alignment_progress_fraction{0.05};
  // Tighten the moving pre-brake window while preserving the stationary HOLD
  // force contract. Infinity inherits the existing tolerance for legacy APIs.
  double pre_stop_force_error_tolerance_n{std::numeric_limits<double>::infinity()};
};

struct InsertionObservation {
  double source_time_s{0.0};
  double dt_s{0.0};
  double actual_depth_m{0.0};
  double axis_force_error_n{0.0};
  double axis_force_rate_n_s{0.0};
  double lateral_force_n{0.0};
  double torque_norm_nm{0.0};
  fr10_force::ContactState contact_state{
      fr10_force::ContactState::NO_CONTACT};
  fr10_force::ForceSafetyFault safety_fault{
      fr10_force::ForceSafetyFault::NONE};
  bool preinsert_reached{false};
  bool planning_failed{false};
  bool tare_complete{false};
  bool lateral_valid{true};
  bool orientation_valid{true};
  bool mating_valid{true};
  bool tracking_valid{true};
  bool robot_fault{false};
  bool stop_complete{false};
  bool retract_complete{false};
  bool retract_requested{false};
  bool canceled{false};
  ContactReferenceMode reference_mode{ContactReferenceMode::PASSIVE};
  double search_deadline_source_s{0.0};
  bool reference_required{false};
  // Residuals against the nominal task wrench, in the measured wrench frame.
  // Raw load norms above remain authoritative for initial contact alignment.
  double lateral_force_error_n{0.0};
  double torque_error_norm_nm{0.0};
};

struct InsertionDecision {
  InsertionState state{InsertionState::IDLE};
  InsertionFault fault{InsertionFault::NONE};
  fr10_force::ForceSafetyFault safety_fault{
      fr10_force::ForceSafetyFault::NONE};
  bool request_online_motion{false};
  bool request_controlled_stop{false};
  bool request_tare{false};
  bool terminal{false};
  bool success{false};
  double progress{0.0};
};

class InsertionStateMachine {
public:
  explicit InsertionStateMachine(InsertionStateConfig config = {});

  bool start(const InsertionGoal &goal, double source_time_s);
  InsertionDecision update(const InsertionObservation &observation);
  void reset();

  InsertionState state() const { return state_; }
  InsertionFault fault() const { return fault_; }
  const InsertionGoal &goal() const { return goal_; }
  // Sensor freshness must gate force motion, never prevent joint braking or
  // maintaining the acknowledged stopped state after a latched sensor fault.
  bool requiresFreshWrenchForResume() const {
    return fault_ == InsertionFault::NONE && state_ >= InsertionState::FT_ZERO &&
        state_ != InsertionState::SUCCESS;
  }

private:
  void enter(InsertionState state);
  void fail(InsertionFault fault,
            fr10_force::ForceSafetyFault safety_fault =
                fr10_force::ForceSafetyFault::NONE);
  InsertionDecision decision(double depth_m) const;

  InsertionStateConfig config_;
  InsertionGoal goal_;
  InsertionState state_{InsertionState::IDLE};
  InsertionFault fault_{InsertionFault::NONE};
  fr10_force::ForceSafetyFault safety_fault_{
      fr10_force::ForceSafetyFault::NONE};
  double start_time_s_{0.0};
  double previous_time_s_{0.0};
  double phase_elapsed_s_{0.0};
  double stable_elapsed_s_{0.0};
  InsertionProgressMonitor progress_;
};

} // namespace fr10_insertion
