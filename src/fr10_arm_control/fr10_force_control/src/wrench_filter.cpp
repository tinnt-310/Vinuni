#include "fr10_force_control/wrench_filter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fr10_force {

WrenchFilter::WrenchFilter(WrenchFilterConfig config) : config_(config) {
  if (!std::isfinite(config_.cutoff_hz) || !(config_.cutoff_hz > 0.0)) {
    throw std::invalid_argument("wrench filter cutoff must be positive");
  }
}

WrenchFilterResult WrenchFilter::update(const WrenchSample &sample,
                                        double dt_s) {
  WrenchFilterResult result;
  result.duplicate = initialized_ &&
      (sample.sequence <= last_sequence_ ||
       sample.source_timestamp_s <= last_source_timestamp_s_);
  if (!sample.valid || sample.sequence == 0 || !finite(sample.wrench) ||
      !std::isfinite(sample.source_timestamp_s) ||
      !std::isfinite(sample.receipt_steady_s) || !std::isfinite(dt_s) ||
      !(dt_s > 0.0) || result.duplicate) {
    return result;
  }

  if (!initialized_) {
    filtered_ = sample.wrench;
    initialized_ = true;
  } else {
    constexpr double pi = 3.14159265358979323846;
    const double time_constant_s = 1.0 / (2.0 * pi * config_.cutoff_hz);
    const double alpha = dt_s / (time_constant_s + dt_s);
    auto filtered = filtered_.vector();
    const auto input = sample.wrench.vector();
    for (std::size_t axis = 0; axis < kCartesianDofs; ++axis) {
      filtered[axis] += alpha * (input[axis] - filtered[axis]);
    }
    filtered_ = Wrench::fromVector(filtered);
  }
  last_sequence_ = sample.sequence;
  last_source_timestamp_s_ = sample.source_timestamp_s;
  result.accepted = true;
  result.sample = sample;
  result.sample.wrench = filtered_;
  return result;
}

void WrenchFilter::reset() {
  filtered_ = Wrench{};
  last_sequence_ = 0;
  last_source_timestamp_s_ = 0.0;
  initialized_ = false;
}

WrenchVerificationSlope::WrenchVerificationSlope(double window_s,double maximum_gap_s)
    : window_s_(window_s),maximum_gap_s_(maximum_gap_s) {
  if(!std::isfinite(window_s_) || window_s_<=0. ||
      !std::isfinite(maximum_gap_s_) || maximum_gap_s_<=0.)
    throw std::invalid_argument("verification source window must be positive");
}
const WrenchVerificationSlope::Observation &WrenchVerificationSlope::at(std::size_t index) const {
  return history_[(head_+index)%history_.size()];
}
void WrenchVerificationSlope::reset() {
  head_=size_=0;sequence_=0;last_result_.reset();
}
std::optional<Vector3> WrenchVerificationSlope::update(const WrenchSample &sample) {
  if(!sample.valid || !finite(sample.wrench) || sample.sequence==0 ||
      !std::isfinite(sample.source_timestamp_s)) {reset();return {};}
  if(size_) {
    const double previous=at(size_-1).source_s;
    if(sample.sequence==sequence_ && sample.source_timestamp_s==previous) return last_result_;
    if(sample.sequence<=sequence_ || sample.source_timestamp_s<=previous ||
        sample.source_timestamp_s-previous>maximum_gap_s_) {reset();return {};}
  }
  sequence_=sample.sequence;
  if(size_==history_.size()) {head_=(head_+1)%history_.size();--size_;}
  history_[(head_+size_)%history_.size()]={sample.source_timestamp_s,sample.wrench.force_n};++size_;
  const double target=sample.source_timestamp_s-window_s_;
  while(size_>2 && at(1).source_s<=target) {head_=(head_+1)%history_.size();--size_;}
  if(size_<2 || at(0).source_s>target+1e-12) {last_result_.reset();return {};}
  const auto &first=at(0);const auto &second=at(1);
  const double fraction=std::clamp((target-first.source_s)/(second.source_s-first.source_s),0.,1.);
  Vector3 result{};
  for(unsigned axis=0;axis<3;++axis) {
    const double old=first.force[axis]+fraction*(second.force[axis]-first.force[axis]);
    result[axis]=(sample.wrench.force_n[axis]-old)/window_s_;
  }
  last_result_=result;return result;
}

} // namespace fr10_force
