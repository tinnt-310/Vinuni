#pragma once

#include "fr10_force_control/force_types.hpp"

namespace fr10_force {

enum class ContactState : int {
  NO_CONTACT = 0,
  POSSIBLE_CONTACT,
  CONTACT_CONFIRMED
};

struct ContactDetectorConfig {
  double enter_force_n{5.0};
  double exit_force_n{3.0};
  double confirm_time_s{0.040};
  double release_time_s{0.040};
  double lateral_enter_force_n{1.0};
  double lateral_exit_force_n{0.5};
  double tilt_enter_torque_nm{0.02};
  double tilt_exit_torque_nm{0.01};
  double torsion_enter_torque_nm{0.02};
  double torsion_exit_torque_nm{0.01};
};

struct ContactEvidence {
  ContactState axial{ContactState::NO_CONTACT};
  ContactState lateral{ContactState::NO_CONTACT};
  ContactState tilt{ContactState::NO_CONTACT};
  ContactState torsion{ContactState::NO_CONTACT};
};

class ContactDetector {
public:
  explicit ContactDetector(ContactDetectorConfig config = {});

  ContactState update(double signed_axis_force_n, double dt_s);
  ContactEvidence update(const Wrench &wrench, std::size_t insertion_axis, double dt_s);
  void reset();
  ContactState state() const { return channels_[0].state; }

private:
  ContactDetectorConfig config_;
  struct Channel {
    ContactState state{ContactState::NO_CONTACT};
    double enter_duration_s{0.0};
    double exit_duration_s{0.0};
  };
  std::array<Channel, 4> channels_{};
  ContactState updateChannel(Channel &channel, double value, double enter, double exit, double dt_s);
};

} // namespace fr10_force
