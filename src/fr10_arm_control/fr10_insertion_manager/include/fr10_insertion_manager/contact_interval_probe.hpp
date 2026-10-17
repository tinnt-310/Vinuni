#pragma once

#include <cstdint>
#include <limits>
#include <optional>

namespace fr10_insertion {

enum class ContactIntervalMode { IDLE, SEEK_NEGATIVE, SEEK_POSITIVE, CENTERING, COMPLETE };
// SI coordinate/load pair: metres/Newtons or rotation-vector radians/Nm.
// This observer/proposal has no pose, IK, robot or actuator ownership.
struct ContactIntervalConfig {
  double drive_load{0.};
  double contact_load{.1};
  double center_gain{25.};
  double contact_hold_s{.05};
  double center_hold_s{.25};
  double coordinate_tolerance{.00005};
  double max_coordinate{.03490658503988659};
  double maximum_observation_gap_s{.05};
  double contact_free_half_span{std::numeric_limits<double>::infinity()};
};
struct ContactIntervalObservation {
  bool valid{false};
  double coordinate{0.};
  double reaction{0.};
  double lower_limit{0.};
  double upper_limit{0.};
  double source_time_s{0.};
  bool contact_confirmed{true};
};
struct ContactIntervalState {
  ContactIntervalMode mode{ContactIntervalMode::IDLE};
  double negative_boundary{0.},positive_boundary{0.},center{0.};
  double negative_contact_strength{0.},positive_contact_strength{0.};
};
struct ContactIntervalProposal {
  bool valid{false};
  uint64_t base_revision{0},qualification_revision{0};
  ContactIntervalState next;
};
class ContactIntervalProbe {
public:
  explicit ContactIntervalProbe(ContactIntervalConfig config={});
  void observe(const ContactIntervalObservation &input);
  ContactIntervalProposal propose(const ContactIntervalObservation &input,bool start) const;
  bool canCommit(const ContactIntervalProposal &proposal) const;
  bool commit(const ContactIntervalProposal &proposal);
  double desiredLoad(const ContactIntervalState &state,double coordinate,
      double measured_reaction,double nominal_load) const;
  void reset();
  bool enabled() const {return config_.drive_load>0.;}
  const ContactIntervalState &state() const {return state_;}
private:
  void clearWindow();
  ContactIntervalConfig config_;
  ContactIntervalState state_;
  uint64_t revision_{0},qualification_revision_{0};
  double last_source_time_s_{-std::numeric_limits<double>::infinity()};
  std::optional<double> qualified_since_source_s_;
};
} // namespace fr10_insertion
