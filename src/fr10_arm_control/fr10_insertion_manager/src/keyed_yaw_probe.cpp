#include "fr10_insertion_manager/keyed_yaw_probe.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fr10_insertion {
namespace {
ContactIntervalConfig config(const KeyedYawProbeConfig &input) {
  if(std::isnan(input.max_yaw_rad) || input.max_yaw_rad<=0. ||
      (std::isfinite(input.contact_free_half_span_rad) && input.contact_free_half_span_rad>=1.5707963267948966))
    throw std::invalid_argument("invalid keyed yaw envelope");
  return {input.drive_torque_nm,input.contact_torque_nm,input.center_gain_nm_rad,
      input.contact_hold_s,input.center_hold_s,input.angle_tolerance_rad,
      std::min(input.max_rotation_rad,input.max_yaw_rad),input.maximum_observation_gap_s,
      input.contact_free_half_span_rad};
}
ContactIntervalObservation observation(const KeyedYawObservation &input,const KeyedYawProbeConfig &config) {
  const bool valid=input.valid && fr10_force::finite(input.rotation_rad) &&
      fr10_force::norm(input.rotation_rad)<=config.max_rotation_rad+config.angle_tolerance_rad;
  const double extent=std::min(config.max_yaw_rad,std::sqrt(std::max(0.,
      config.max_rotation_rad*config.max_rotation_rad-input.rotation_rad[0]*input.rotation_rad[0]-
      input.rotation_rad[1]*input.rotation_rad[1])));
  return {valid,input.rotation_rad[2],input.torque_z_nm,-extent,extent,input.source_time_s};
}
ContactIntervalState generic(const KeyedYawProbeState &input) {
  return {input.mode,input.negative_boundary_rad,input.positive_boundary_rad,input.center_rad,
      input.negative_contact_strength,input.positive_contact_strength};
}
ContactIntervalProposal generic(const KeyedYawProposal &input) {
  return {input.valid,input.base_revision,input.qualification_revision,generic(input.next)};
}
KeyedYawProbeState legacy(const ContactIntervalState &input) {
  return {input.mode,input.negative_boundary,input.positive_boundary,input.center,
      input.negative_contact_strength,input.positive_contact_strength};
}
}
KeyedYawProbe::KeyedYawProbe(KeyedYawProbeConfig input):config_(input),interval_(config(input)) {}
void KeyedYawProbe::observe(const KeyedYawObservation &input) {interval_.observe(observation(input,config_));}
KeyedYawProposal KeyedYawProbe::propose(const KeyedYawObservation &input,bool start) const {
  const auto result=interval_.propose(observation(input,config_),start);
  return {result.valid,result.base_revision,result.qualification_revision,legacy(result.next)};
}
bool KeyedYawProbe::canCommit(const KeyedYawProposal &proposal) const {return interval_.canCommit(generic(proposal));}
bool KeyedYawProbe::commit(const KeyedYawProposal &proposal) {
  if(!interval_.commit(generic(proposal)))return false;
  state_=legacy(interval_.state());return true;
}
double KeyedYawProbe::torque(const KeyedYawProbeState &state,double measured) const {
  return desiredTorque(state,measured,0.,0.);
}
double KeyedYawProbe::desiredTorque(const KeyedYawProbeState &state,double measured,
    double reaction,double nominal) const {
  return interval_.desiredLoad(generic(state),measured,reaction,nominal);
}
void KeyedYawProbe::reset() {interval_.reset();state_=legacy(interval_.state());}
} // namespace fr10_insertion
