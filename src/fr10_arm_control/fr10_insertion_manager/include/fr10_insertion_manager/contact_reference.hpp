#pragma once

#include <cstdint>
#include <limits>
#include <optional>
#include "fr10_force_control/force_types.hpp"
#include "fr10_insertion_manager/keyed_yaw_probe.hpp"

namespace fr10_insertion {

enum class ContactReferenceMode { PASSIVE, SEARCH, ACQUIRED, EXHAUSTED, CAPTURED };
inline bool contactReferencePending(ContactReferenceMode mode) {
  return mode == ContactReferenceMode::SEARCH || mode == ContactReferenceMode::CAPTURED;
}
enum class ContactReferencePhase { INACTIVE, APPROACH, ALIGN, INSERT, SEAT };

struct ContactReferenceConfig {
  // Zero preserves legacy behavior. Reference cannot change physical CAD or
  // immutable input pose; it only defines the internal admittance equilibrium.
  double search_radius_m{0.0};
  // Keep contact-derived capture/locating pressure while disabling the
  // commanded spiral for passive six-axis alignment experiments.
  bool spiral_enabled{true};
  double spiral_pitch_m{0.00025};
  double search_speed_m_s{0.0005};
  double stall_time_s{0.5};
  double search_budget_s{180.0};
  double progress_step_m{0.0001};
  double acquisition_depth_m{0.004}; // Beyond the calibrated 3 mm lead-in.
  double acquisition_hold_s{0.08};
  double maximum_observation_gap_s{0.05};
  double contact_force_n{0.5};
  double search_force_n{1.0};
  double max_lateral_force_n{1.0};
  double max_torque_nm{0.1};
  double max_rotation_rad{0.03490658503988659};
  // Qualified model/encoder observation error, not additional command travel.
  // An estimate close to the boundary is projected onto the same hard set.
  double anchor_position_tolerance_m{0.00005};
  double anchor_rotation_tolerance_rad{0.00005};
  KeyedYawProbeConfig keyed_yaw;
  ContactIntervalConfig lateral_probe{0.,.5,1000.,.05,.25,.000025,.0025,.05,
      std::numeric_limits<double>::infinity()};
};

struct ContactReferenceObservation {
  bool valid{true};
  ContactReferencePhase phase{ContactReferencePhase::INACTIVE};
  fr10_force::Vector6 actual_correction{};
  fr10_force::Wrench measured_wrench;
  double depth_m{0.0};
  double source_time_s{0.0};
  double dt_s{0.0};
  fr10_force::Wrench nominal_wrench;
};

struct ContactReferenceState {
  ContactReferenceMode mode{ContactReferenceMode::PASSIVE};
  fr10_force::Vector6 equilibrium{};
  bool contact_observed{false};
  double best_depth_m{-std::numeric_limits<double>::infinity()};
  double stalled_s{0.0};
  double spiral_angle_rad{0.0};
  double search_start_source_s{0.0};
  double acquisition_elapsed_s{0.0};
  double acquisition_depth_m{0.0};
  double last_source_time_s{0.0};
  fr10_force::Vector3 capture_rotation_rad{};
};

struct ContactReferenceProposal {
  bool valid{false};
  uint64_t base_revision{0};
  uint64_t qualification_revision{0};
  ContactReferenceState next;
  KeyedYawProposal yaw_probe;
  std::array<ContactIntervalProposal,2> lateral_probes;
};

// ACQUIRED extrapolates the frozen axis. CAPTURED may transport its fixed
// entry point using an acknowledged correction orientation: it still has a
// zero rotational spring reference and does not certify the bore axis.
// No measured lateral displacement updates the anchor. Safety stays in the
// immutable input axes. Omitting accepted_correction retains the legacy API.
fr10_force::Vector6 contactEquilibriumAtDepth(const ContactReferenceState &state, double depth_m,
    const fr10_force::Vector6 *accepted_correction = nullptr);

class ContactReference {
public:
  explicit ContactReference(ContactReferenceConfig config = {});
  // Sensor history is observed every tick, independently of motion admission.
  // A rejected target cannot roll back an intervening unqualified sample.
  void observe(const ContactReferenceObservation &input);
  ContactReferenceProposal propose(const ContactReferenceObservation &input) const;
  bool commit(const ContactReferenceProposal &proposal);
  bool canCommit(const ContactReferenceProposal &proposal) const;
  void reset();
  bool enabled() const { return config_.search_radius_m > 0.0; }
  const ContactReferenceState &state() const { return state_; }
  const ContactReferenceConfig &config() const { return config_; }
  const KeyedYawProbe &yawProbe() const { return yaw_probe_; }
  const ContactIntervalProbe &lateralProbe(std::size_t axis) const {return lateral_probes_.at(axis);}
  fr10_force::Wrench desiredWrench(const ContactReferenceProposal &proposal,
      const ContactReferenceObservation &input,const fr10_force::Wrench &nominal) const;

private:
  void clearAcquisitionWindow();
  ContactIntervalObservation lateralObservation(const ContactReferenceObservation &input,std::size_t axis) const;
  ContactReferenceConfig config_;
  KeyedYawProbe yaw_probe_;
  std::array<ContactIntervalProbe,2> lateral_probes_;
  ContactReferenceState state_;
  uint64_t revision_{0};
  uint64_t qualification_revision_{0};
  double last_observation_source_s_{-std::numeric_limits<double>::infinity()};
  std::optional<double> qualified_since_source_s_;
};

} // namespace fr10_insertion
