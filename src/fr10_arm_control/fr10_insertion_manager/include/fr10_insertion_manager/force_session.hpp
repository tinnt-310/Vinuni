#pragma once
#include <optional>

#include "fr10_force_control/contact_detector.hpp"
#include "fr10_force_control/force_position_controller.hpp"
#include "fr10_force_control/force_safety_monitor.hpp"
#include "fr10_force_control/wrench_filter.hpp"
#include "fr10_insertion_manager/insertion_state_machine.hpp"
#include "fr10_insertion_manager/nominal_insertion_provider.hpp"
#include "fr10_insertion_manager/contact_reference.hpp"
#include "fr10_motion_core/continuous_ik_stepper.hpp"
#include "fr10_motion_core/core_types.hpp"

namespace fr10_insertion {

struct ForceSessionConfig {
  InsertionStateConfig state_machine;
  fr10_force::AdmittanceConfig admittance;
  fr10_force::ContactDetectorConfig contact;
  fr10_force::ForceSafetyConfig safety;
  fr10_motion::ContinuousIKConfig continuous_ik;
  ContactReferenceConfig contact_reference;
  fr10_force::RotationMatrix rotation_base_from_insertion{
      {1.0, 0.0, 0.0,
       0.0, 1.0, 0.0,
       0.0, 0.0, 1.0}};
  double target_validity_s{0.032};
  double max_nominal_lead_m{0.0001};
  double velocity_scale{0.10};
  double acceleration_scale{0.10};
  double jerk_scale{0.10};
};

struct ForceSessionInput {
  fr10_force::WrenchSample raw_sample;
  fr10_force::Wrench conditioned_wrench;
  fr10_motion::KinematicState joint_seed;
  double source_time_s{0.0};
  // Phase/integration clock can be monotonic; expiry uses feedback's clock.
  std::optional<double> command_source_time_s;
  double now_steady_s{0.0};
  double dt_s{0.0};
  double actual_depth_m{0.0};
  std::optional<fr10_force::Vector6> actual_correction;
  bool allow_motion_proposal{true};
  bool preinsert_reached{false};
  bool planning_failed{false};
  bool tare_complete{false};
  bool conditioned_reference_changed{false};
  bool lateral_valid{true};
  bool orientation_valid{true};
  bool mating_valid{true};
  bool tracking_valid{true};
  bool robot_ok{true};
  bool base_stationary{true};
  bool collision_free{true};
  bool scene_unchanged{true};
  bool command_accepted{true};
  bool stop_complete{false};
  bool retract_complete{false};
  bool retract_requested{false};
  bool canceled{false};
};

struct ForceSessionOutput {
  InsertionDecision decision;
  double actual_depth_m{0.0};
  fr10_force::CartesianPose nominal_pose;
  fr10_force::CartesianPose corrected_pose;
  fr10_force::CartesianCorrection correction;
  fr10_force::Vector6 force_error{};
  fr10_force::ContactState contact_state{
      fr10_force::ContactState::NO_CONTACT};
  fr10_force::ContactEvidence contact_evidence;
  fr10_force::ForceSafetyResult safety;
  std::optional<fr10_force::Vector3> verification_force_rate_n_s;
  fr10_motion::ContinuousIKStepResult ik;
  fr10_motion::OnlineJointTarget online_target;
  bool online_target_valid{false};
  ContactReferenceMode reference_mode{ContactReferenceMode::PASSIVE};
  KeyedYawProbeMode yaw_probe_mode{KeyedYawProbeMode::IDLE};
  std::array<ContactIntervalMode,2> lateral_probe_modes{ContactIntervalMode::IDLE,ContactIntervalMode::IDLE};
};

class ForceSession {
public:
  explicit ForceSession(ForceSessionConfig config);

  bool start(const InsertionGoal &goal,
             const fr10_force::CartesianPose &final_pose,
             const fr10_force::Vector3 &axis_base,
             const fr10_force::Wrench &desired_wrench,
             uint64_t generation, uint64_t scene_revision,
             double source_time_s);
  // Only before FT_ZERO: retain the original FSM deadline and safety clocks.
  bool reframePreinsert(const fr10_force::CartesianPose &final_pose,
                       const fr10_force::Vector3 &axis_base,
                       const fr10_force::RotationMatrix &rotation,
                       uint64_t scene_revision);
  ForceSessionOutput update(const ForceSessionInput &input,
                            fr10_motion::IKProvider &provider);
  bool commitOnlineTarget(uint64_t sequence,
                          bool accepted_and_published);
  bool canCommitOnlineTarget(uint64_t sequence) const;
  void reset();

  bool active() const;
  const InsertionStateMachine &stateMachine() const { return state_machine_; }
  const ContactReference &contactReference() const { return contact_reference_; }

private:
  static fr10_motion::Pose toMotionPose(
      const fr10_force::CartesianPose &pose);

  ForceSessionConfig config_;
  InsertionStateMachine state_machine_;
  NominalInsertionProvider nominal_provider_;
  fr10_force::ForcePositionController controller_;
  fr10_force::ContactDetector contact_detector_;
  fr10_force::ForceSafetyMonitor safety_monitor_;
  fr10_force::WrenchVerificationSlope verification_slope_;
  fr10_motion::ContinuousIKStepper ik_stepper_;
  fr10_force::Wrench desired_wrench_;
  fr10_force::ForcePositionResult pending_controller_result_;
  fr10_force::ForcePositionResult committed_controller_result_;
  NominalInsertionProvider pending_nominal_provider_;
  ContactReference contact_reference_;
  ContactReferenceProposal pending_reference_;
  uint64_t generation_{0};
  uint64_t scene_revision_{0};
  uint64_t online_sequence_{0};
  uint64_t pending_sequence_{0};
  double deferred_motion_dt_s_{0.0};
};

} // namespace fr10_insertion
