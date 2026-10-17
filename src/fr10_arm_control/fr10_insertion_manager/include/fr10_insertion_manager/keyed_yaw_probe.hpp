#pragma once

#include <cstdint>
#include <limits>
#include "fr10_force_control/force_types.hpp"
#include "fr10_insertion_manager/contact_interval_probe.hpp"

namespace fr10_insertion {

using KeyedYawProbeMode = ContactIntervalMode;
struct KeyedYawProbeConfig {
  double drive_torque_nm{0.0}; // Zero retains the previously commissioned API.
  double contact_torque_nm{0.10};
  double center_gain_nm_rad{25.0};
  double contact_hold_s{0.05};
  double center_hold_s{0.25};
  double angle_tolerance_rad{0.00005};
  double max_rotation_rad{0.03490658503988659};
  double maximum_observation_gap_s{0.05};
  double max_yaw_rad{std::numeric_limits<double>::infinity()};
  // Mechanical keyed clearance, measured or derived from commissioned CAD.
  // Unknown preserves the legacy observed-interval midpoint; an actuator
  // envelope alone cannot identify the true center of a one-sided interval.
  double contact_free_half_span_rad{std::numeric_limits<double>::infinity()};
};
struct KeyedYawObservation {
  bool valid{false};
  fr10_force::Vector3 rotation_rad{};
  double torque_z_nm{0.0};
  double source_time_s{0.0};
};
struct KeyedYawProbeState {
  KeyedYawProbeMode mode{KeyedYawProbeMode::IDLE};
  double negative_boundary_rad{0.0};
  double positive_boundary_rad{0.0};
  double center_rad{0.0};
  double negative_contact_strength{0.0};
  double positive_contact_strength{0.0};
};
struct KeyedYawProposal {
  bool valid{false};
  uint64_t base_revision{0};
  uint64_t qualification_revision{0};
  KeyedYawProbeState next;
};

// Probe the contact-free keyed yaw interval under locating pressure. The
// rotation envelope may truncate either endpoint. Its contact-weighted center is an estimate,
// not a certificate of CAD/electrical fit. Caller owns timeout, all physical
// safety, IK and command publication. No joint or pose command is emitted here.
class KeyedYawProbe {
public:
  explicit KeyedYawProbe(KeyedYawProbeConfig config = {});
  void observe(const KeyedYawObservation &input);
  KeyedYawProposal propose(const KeyedYawObservation &input, bool start) const;
  bool canCommit(const KeyedYawProposal &proposal) const;
  bool commit(const KeyedYawProposal &proposal);
  double torque(const KeyedYawProbeState &state, double measured_yaw_rad) const;
  // Select the absolute canonical desired moment. CENTERING is a bounded
  // position objective, with measured reaction feedforward; SEEK adds the
  // locating moment to the nominal force objective. Other phases restore that
  // nominal objective. Raw safety/loaded-axis qualification are unchanged.
  double desiredTorque(const KeyedYawProbeState &state, double measured_yaw_rad,
      double measured_reaction_nm, double nominal_moment_nm) const;
  void reset();
  bool enabled() const { return config_.drive_torque_nm > 0.; }
  const KeyedYawProbeState &state() const { return state_; }
private:
  KeyedYawProbeConfig config_;
  ContactIntervalProbe interval_;
  // Legacy API view; the shared interval owns qualification and transactions.
  KeyedYawProbeState state_;
};

} // namespace fr10_insertion
