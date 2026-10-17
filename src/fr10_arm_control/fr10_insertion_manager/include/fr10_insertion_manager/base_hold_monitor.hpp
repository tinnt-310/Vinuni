#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include "fr10_motion_core/math_utils.hpp"

namespace fr10_insertion {

struct BaseFeedback {
  std::array<double, 3> position{};
  std::array<double, 4> orientation{{0, 0, 0, 1}};
  std::array<double, 6> twist{};
  int64_t source_ns{0};
  double receipt_s{0};
  bool valid{false};
};

// Pose-authoritative localization: finite source-window rigid-body velocity.
// Raw pose, receipt and source stamps remain intact for drift/freshness guards.
// Startup, discontinuity or insufficient bounded history never implies zero.
class BasePoseVelocity {
public:
  explicit BasePoseVelocity(double window_s=.016) noexcept
      : window_ns_(std::isfinite(window_s) && window_s>=.001 && window_s<=.020
                   ? static_cast<int64_t>(std::llround(window_s*1e9)) : 0) {}
  BaseFeedback update(BaseFeedback sample) noexcept {
    using namespace fr10_motion::math;
    bool valid=window_ns_>0 && sample.valid && sample.source_ns>0 &&
               normalizeQuaternion(sample.orientation);
    for(double p:sample.position) valid &= std::isfinite(p);
    if(!valid) {size_=0;sample.valid=false;return sample;}
    const BaseFeedback current=sample;
    if(size_) {
      const auto delta=sample.source_ns-at(size_-1).source_ns;
      if(delta==0) {sample.valid=false;return sample;}
      if(delta<0 || delta>20000000) {
        size_=0;head_=0;history_[0]=current;size_=1;
        sample.valid=false;return sample;
      }
    }
    if(size_==history_.size()) {head_=(head_+1)%history_.size();--size_;}
    history_[(head_+size_)%history_.size()]=current;++size_;
    const auto target=sample.source_ns-window_ns_;
    // Retain the two samples bracketing the exact source-time window.
    while(size_>2 && at(1).source_ns<=target) {
      head_=(head_+1)%history_.size();--size_;
    }
    sample.valid=size_>=2 && at(0).source_ns<=target;
    if(!sample.valid) return sample;
    const auto &first=at(0);const auto &second=at(1);
    const double fraction=static_cast<double>(target-first.source_ns)/
                          static_cast<double>(second.source_ns-first.source_ns);
    std::array<double,3> world_velocity{};
    const double dt=static_cast<double>(window_ns_)*1e-9;
    for(unsigned i=0;i<3;++i) {
      const double old=first.position[i]+fraction*(second.position[i]-first.position[i]);
      world_velocity[i]=(sample.position[i]-old)/dt;
    }
    const auto velocity=rotateVector(quaternionConjugate(sample.orientation),world_velocity);
    const auto old_orientation=quaternionSlerp(first.orientation,second.orientation,fraction);
    // Express both translational and angular displacement in the current child frame.
    const auto rotation_world=rotateVector(old_orientation,quaternionToRotationVector(
        quaternionMultiply(quaternionConjugate(old_orientation),sample.orientation)));
    const auto rotation=rotateVector(quaternionConjugate(sample.orientation),rotation_world);
    for(unsigned i=0;i<3;++i) {sample.twist[i]=velocity[i];sample.twist[i+3]=rotation[i]/dt;}
    return sample;
  }
private:
  const BaseFeedback &at(std::size_t index) const noexcept {
    return history_[(head_+index)%history_.size()];
  }
  // Maximum supported 20 ms source window at the 50 us minimum physics step,
  // including interpolation brackets. Fixed storage keeps update allocation-free.
  std::array<BaseFeedback,512> history_{};
  std::size_t head_{0},size_{0};
  int64_t window_ns_;
};

enum class BaseHoldFault { NONE, INVALID, STALE, CLOCK_JUMP, MOVING, DRIFT, LEASE_LOST };
struct BaseHoldResult { bool ready{false}; BaseHoldFault fault{BaseHoldFault::NONE}; };

class BaseHoldMonitor {
public:
  BaseHoldResult update(const BaseFeedback &sample, double now_s, bool lease_valid) noexcept {
    if (fault_ != BaseHoldFault::NONE) return {false, fault_};
    if (!lease_valid) return invalidate(BaseHoldFault::LEASE_LOST);
    double norm = 0;
    bool finite = sample.valid && std::isfinite(now_s) && std::isfinite(sample.receipt_s);
    for (double value : sample.position) finite &= std::isfinite(value);
    for (double value : sample.orientation) { finite &= std::isfinite(value); norm += value*value; }
    for (double value : sample.twist) finite &= std::isfinite(value);
    if (!finite || norm < 1e-12 || sample.source_ns <= 0) return invalidate(BaseHoldFault::INVALID);
    if (sample.source_ns < last_source_ns_) return invalidate(BaseHoldFault::CLOCK_JUMP);
    if (sample.source_ns > last_source_ns_) {
      last_source_ns_ = sample.source_ns;
      last_receipt_s_ = sample.receipt_s; // Duplicate messages cannot renew freshness.
    }
    if (now_s < last_receipt_s_ || now_s-last_receipt_s_ > .050)
      return invalidate(BaseHoldFault::STALE);
    bool stationary = true;
    for (double value : sample.twist) stationary &= std::abs(value) <= 1e-4;
    if (!stationary) {
      have_anchor_ = false;
      return invalidate(BaseHoldFault::MOVING);
    }
    if (!have_anchor_) {
      anchor_ = sample;
      anchor_source_ns_ = sample.source_ns;
      have_anchor_ = true;
    }
    double distance_squared = 0, dot = 0, anchor_norm = 0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const double difference = sample.position[axis]-anchor_.position[axis];
      distance_squared += difference*difference;
    }
    for (std::size_t axis = 0; axis < 4; ++axis) {
      dot += sample.orientation[axis]*anchor_.orientation[axis];
      anchor_norm += anchor_.orientation[axis]*anchor_.orientation[axis];
    }
    const double cosine = std::fmin(1., std::abs(dot)/std::sqrt(norm*anchor_norm));
    // Simulation engineering limits, not a hardware safety certification.
    if (std::sqrt(distance_squared) > .0002 || 2*std::acos(cosine) > .0005)
      return invalidate(BaseHoldFault::DRIFT);
    ready_ = sample.source_ns-anchor_source_ns_ >= 200000000;
    return {ready_, BaseHoldFault::NONE};
  }
  void reset() noexcept { *this = BaseHoldMonitor{}; }

private:
  BaseHoldResult invalidate(BaseHoldFault fault) noexcept {
    if (ready_) fault_ = fault; // Waiting admission may recover; an active lease cannot.
    else have_anchor_ = false;
    return {false, fault};
  }
  BaseFeedback anchor_;
  int64_t last_source_ns_{0}, anchor_source_ns_{0};
  double last_receipt_s_{0};
  bool have_anchor_{false}, ready_{false};
  BaseHoldFault fault_{BaseHoldFault::NONE};
};

} // namespace fr10_insertion
