#include "fr10_insertion_manager/contact_interval_probe.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fr10_insertion {
namespace {
bool valid(const ContactIntervalObservation &input) {
  return input.valid && std::isfinite(input.coordinate) && std::isfinite(input.reaction) &&
      std::isfinite(input.lower_limit) && std::isfinite(input.upper_limit) &&
      input.lower_limit<=input.upper_limit && std::isfinite(input.source_time_s);
}
}
ContactIntervalProbe::ContactIntervalProbe(ContactIntervalConfig config):config_(config) {
  if(!std::isfinite(config.drive_load) || config.drive_load<0.)
    throw std::invalid_argument("invalid contact interval drive load");
  for(double value:{config.contact_load,config.center_gain,config.contact_hold_s,
      config.center_hold_s,config.coordinate_tolerance,config.max_coordinate,
      config.maximum_observation_gap_s})
    if(!std::isfinite(value) || value<=0.) throw std::invalid_argument("invalid contact interval configuration");
  if((enabled() && config.contact_load>=config.drive_load) ||
      config.coordinate_tolerance>=config.max_coordinate ||
      std::isnan(config.contact_free_half_span) || config.contact_free_half_span<=0.)
    throw std::invalid_argument("invalid contact interval load/coordinate envelope");
}
void ContactIntervalProbe::clearWindow() {qualified_since_source_s_.reset();++qualification_revision_;}
void ContactIntervalProbe::observe(const ContactIntervalObservation &input) {
  const double elapsed=input.source_time_s-last_source_time_s_;
  const bool continuous=!std::isfinite(last_source_time_s_) ||
      (elapsed>0. && elapsed<=config_.maximum_observation_gap_s);
  last_source_time_s_=input.source_time_s;
  if(!enabled() || !valid(input) || !input.contact_confirmed || !continuous) {clearWindow();return;}
  bool qualified=false;
  switch(state_.mode) {
  case ContactIntervalMode::SEEK_NEGATIVE:
    qualified=input.reaction<=-config_.contact_load ||
        input.coordinate<=input.lower_limit+config_.coordinate_tolerance;break;
  case ContactIntervalMode::SEEK_POSITIVE:
    qualified=input.reaction>=config_.contact_load ||
        input.coordinate>=input.upper_limit-config_.coordinate_tolerance;break;
  case ContactIntervalMode::CENTERING:
    qualified=std::abs(input.coordinate-state_.center)<=config_.coordinate_tolerance;break;
  default:break;
  }
  if(!qualified) {clearWindow();return;}
  if(!qualified_since_source_s_) qualified_since_source_s_=input.source_time_s;
}
ContactIntervalProposal ContactIntervalProbe::propose(const ContactIntervalObservation &input,bool start) const {
  ContactIntervalProposal result;result.base_revision=revision_;
  result.qualification_revision=qualification_revision_;result.next=state_;
  if(!valid(input) || (enabled() && input.source_time_s!=last_source_time_s_)) return result;
  result.valid=true;
  if(!enabled())return result;
  if(state_.mode==ContactIntervalMode::IDLE && start && input.contact_confirmed) {
    result.next.mode=ContactIntervalMode::SEEK_NEGATIVE;return result;
  }
  const double hold=state_.mode==ContactIntervalMode::CENTERING ? config_.center_hold_s : config_.contact_hold_s;
  if(!qualified_since_source_s_ || input.source_time_s-*qualified_since_source_s_<hold)return result;
  switch(state_.mode) {
  case ContactIntervalMode::SEEK_NEGATIVE:
    result.next.negative_boundary=input.coordinate;
    result.next.negative_contact_strength=std::clamp(-input.reaction/config_.contact_load,0.,1.);
    result.next.mode=ContactIntervalMode::SEEK_POSITIVE;break;
  case ContactIntervalMode::SEEK_POSITIVE: {
    result.next.positive_boundary=input.coordinate;
    result.next.positive_contact_strength=std::clamp(input.reaction/config_.contact_load,0.,1.);
    const double lower=result.next.negative_contact_strength,upper=result.next.positive_contact_strength;
    const double midpoint=.5*(result.next.negative_boundary+result.next.positive_boundary);
    // These are contact-weighted coordinate observations, not an ordered
    // pair of certified walls. A changing coupled envelope can collapse the
    // scan; quiet encoder samples can then cross within their uncertainty.
    // Estimate from their evidence and project onto the current hard set.
    const double half_span=std::isfinite(config_.contact_free_half_span) ? config_.contact_free_half_span :
        .5*std::abs(result.next.positive_boundary-result.next.negative_boundary);
    const double feasible_lower=std::max(-config_.max_coordinate,input.lower_limit);
    const double feasible_upper=std::min(config_.max_coordinate,input.upper_limit);
    if(feasible_lower>feasible_upper){result.valid=false;return result;}
    result.next.center=std::clamp((lower*(result.next.negative_boundary+half_span)+
        upper*(result.next.positive_boundary-half_span)+std::max(0.,1.-lower-upper)*midpoint)/
        std::max(1.,lower+upper),feasible_lower,feasible_upper);
    result.next.mode=ContactIntervalMode::CENTERING;break;
  }
  case ContactIntervalMode::CENTERING:result.next.mode=ContactIntervalMode::COMPLETE;break;
  default:break;
  }
  return result;
}
bool ContactIntervalProbe::canCommit(const ContactIntervalProposal &proposal) const {
  const auto &next=proposal.next;
  const auto mode=static_cast<int>(state_.mode),next_mode=static_cast<int>(next.mode);
  return next_mode>=0 && next_mode<=static_cast<int>(ContactIntervalMode::COMPLETE) &&
      (next_mode==mode || next_mode==mode+1) &&
      proposal.valid && proposal.base_revision==revision_ &&
      (next.mode==state_.mode || state_.mode==ContactIntervalMode::IDLE ||
          proposal.qualification_revision==qualification_revision_) &&
      std::isfinite(next.negative_boundary) && std::isfinite(next.positive_boundary) &&
      std::isfinite(next.negative_contact_strength) && next.negative_contact_strength>=0. && next.negative_contact_strength<=1. &&
      std::isfinite(next.positive_contact_strength) && next.positive_contact_strength>=0. && next.positive_contact_strength<=1. &&
      std::isfinite(next.center) && std::abs(next.center)<=config_.max_coordinate;
}
bool ContactIntervalProbe::commit(const ContactIntervalProposal &proposal) {
  if(!canCommit(proposal))return false;
  if(proposal.next.mode!=state_.mode)clearWindow();
  state_=proposal.next;++revision_;return true;
}
double ContactIntervalProbe::desiredLoad(const ContactIntervalState &state,double coordinate,
    double reaction,double nominal) const {
  if(!enabled())return nominal;
  switch(state.mode) {
  case ContactIntervalMode::SEEK_NEGATIVE:return nominal-config_.drive_load;
  case ContactIntervalMode::SEEK_POSITIVE:return nominal+config_.drive_load;
  case ContactIntervalMode::CENTERING:
    return std::clamp(reaction+config_.center_gain*(state.center-coordinate),-config_.drive_load,config_.drive_load);
  default:return nominal;
  }
}
void ContactIntervalProbe::reset() {state_={};last_source_time_s_=-std::numeric_limits<double>::infinity();clearWindow();++revision_;}
} // namespace fr10_insertion
