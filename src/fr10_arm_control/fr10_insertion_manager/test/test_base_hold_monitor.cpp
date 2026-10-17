#include <iostream>
#include <limits>
#include "fr10_insertion_manager/base_hold_monitor.hpp"

int main() {
  using namespace fr10_insertion;
  bool ok = true;
  auto check = [&](bool value, const char *name) { if (!value) { std::cerr << name << '\n'; ok = false; } };
  BaseHoldMonitor monitor;
  BaseFeedback sample; sample.valid = true;
  sample.source_ns = 1000000000; sample.receipt_s = 1;
  check(!monitor.update(sample, 1, true).ready, "admission requires source-time dwell");
  sample.source_ns += 210000000; sample.receipt_s = 1.21;
  check(monitor.update(sample, 1.21, true).ready, "stationary source dwell");
  check(monitor.update(sample, 1.261, true).fault == BaseHoldFault::STALE,
        "duplicate stamp does not renew freshness");
  monitor.reset(); sample.source_ns = 1000000000; sample.receipt_s = 1;
  monitor.update(sample, 1, true);
  sample.source_ns += 210000000; sample.receipt_s = 1.21;
  monitor.update(sample, 1.21, true);
  sample.source_ns += 33000000; sample.receipt_s = 1.243; sample.position[0] = .0003;
  check(monitor.update(sample, 1.243, true).fault == BaseHoldFault::DRIFT, "zero twist cannot hide pose drift");
  sample.position[0] = 0;
  check(monitor.update(sample, 1.243, true).fault == BaseHoldFault::DRIFT, "active fault stays latched");
  monitor.reset(); sample.twist[0] = .001;
  check(monitor.update(sample, 1.243, true).fault == BaseHoldFault::MOVING, "moving base denied");
  sample.twist[0] = 0;
  check(!monitor.update(sample, 1.243, false).ready, "lease acknowledgement required");
  check(!monitor.update(sample, 1.243, true).ready, "recovery must repeat dwell");
  sample.source_ns += 210000000; sample.receipt_s = 1.453;
  check(monitor.update(sample, 1.453, true).ready, "waiting admission can recover");
  check(monitor.update(sample, 1.453, false).fault == BaseHoldFault::LEASE_LOST, "active lease loss");
  for (unsigned axis=0;axis<6;++axis) {
    monitor.reset();sample=BaseFeedback{};sample.valid=true;
    sample.source_ns=1000000000;sample.receipt_s=1.;
    monitor.update(sample,1.,true);
    sample.source_ns+=210000000;sample.receipt_s=1.21;
    check(monitor.update(sample,1.21,true).ready,"six-axis stationary anchor");
    sample.source_ns+=8000000;sample.receipt_s=1.218;sample.twist[axis]=.001;
    check(monitor.update(sample,1.218,true).fault==BaseHoldFault::MOVING,
          "full physical twist including vertical/roll/pitch motion is guarded");
  }
  monitor.reset();sample=BaseFeedback{};sample.valid=true;
  sample.source_ns=1000000000;sample.receipt_s=1.;
  monitor.update(sample,1.,true);
  sample.source_ns+=210000000;sample.receipt_s=1.21;
  monitor.update(sample,1.21,true);
  sample.source_ns+=8000000;sample.receipt_s=1.218;sample.position[2]=.0003;
  check(monitor.update(sample,1.218,true).fault==BaseHoldFault::DRIFT,
        "vertical drift cannot be hidden by planar wheel odometry");
  // Exact 16 ms source window, including off-grid interpolation and child-frame twist.
  for(int64_t step:{50000,62500,125000,1000000,3000000,8000000}) {
    // Include the maximum configured 20 ms window at the supported 20 kHz
    // physics cadence; a full window must survive bounded ring wrap.
    const int64_t window=step==50000 ? 20000000 : 16000000;
    BasePoseVelocity estimator(window*1e-9);
    BaseFeedback derived;
    sample=BaseFeedback{};sample.valid=true;
    const double yaw=fr10_motion::math::kHalfPi;
    for(int64_t elapsed=0;elapsed<=80000000;elapsed+=step) {
      const double t=elapsed*1e-9;
      sample.source_ns=1000000000+elapsed;sample.receipt_s=2.+t;
      sample.position={1.+t,2.,3.};
      sample.orientation={0,0,std::sin((yaw+t)/2),std::cos((yaw+t)/2)};
      derived=estimator.update(sample);
      check(derived.valid==(elapsed>=window),"full source window required before admission");
      if(derived.valid) {
        check(std::abs(derived.twist[0]+std::sin(t))<1e-8 &&
              std::abs(derived.twist[1]+std::cos(t))<1e-8 &&
              std::abs(derived.twist[5]-1.)<1e-8,
              "source-window translation/rotation on rotated child frame");
        check(derived.receipt_s==sample.receipt_s && derived.source_ns==sample.source_ns &&
              derived.position==sample.position,"velocity estimate preserves raw pose and clocks");
      }
    }
    sample.receipt_s+=.1;
    check(!estimator.update(sample).valid,"duplicate cannot renew derived velocity");
    sample.source_ns+=21000000;
    check(!estimator.update(sample).valid,"source gap invalidates window");
    sample.source_ns-=1000000;
    check(!estimator.update(sample).valid,"backward source invalidates window");
    sample.position[0]=std::numeric_limits<double>::quiet_NaN();
    check(!estimator.update(sample).valid,"nonfinite raw pose cannot admit parking");
  }
  BasePoseVelocity estimator;
  monitor.reset();sample=BaseFeedback{};sample.valid=true;
  BaseFeedback derived;
  for(unsigned tick=0;tick<=220;++tick) {
    sample.source_ns=1000000000+tick*1000000;sample.receipt_s=1.+tick*.001;
    if(tick%2) sample.orientation={0,0,0,-2};else sample.orientation={0,0,0,1};
    derived=estimator.update(sample);
    if(derived.valid) check(monitor.update(derived,sample.receipt_s,true).fault==BaseHoldFault::NONE,
                           "antipodal stationary pose remains stationary through bounded ring wrap");
  }
  check(monitor.update(derived,sample.receipt_s,true).ready,"source window followed by full admission dwell");
  sample.source_ns+=1000000;sample.receipt_s+=.001;sample.position[2]+=.000002;
  derived=estimator.update(sample);
  check(derived.valid && std::abs(derived.twist[2]-.000125)<1e-12,
        "one-sample two-micrometre displacement remains above velocity barrier");
  check(monitor.update(derived,sample.receipt_s,true).fault==BaseHoldFault::MOVING,
        "actual movement is detected at the first new source sample");
  // Sustained 0.2 mm/s movement crosses 0.1 mm/s by two control periods.
  estimator=BasePoseVelocity{};sample=BaseFeedback{};sample.valid=true;
  for(unsigned tick=0;tick<=40;++tick) {
    sample.source_ns=1000000000+tick*1000000;
    sample.position[0]=tick>24 ? (tick-24)*.001*.0002 : 0.;
    derived=estimator.update(sample);
    if(tick==40) check(derived.valid && derived.twist[0]>.0001,
                      "sustained above-limit movement detected within 16 ms");
  }
  BasePoseVelocity invalid(0.);
  check(!invalid.update(sample).valid,"invalid window cannot imply zero velocity");
  // Sampling denser than the fixed ring can retain is rejected, not shortened.
  estimator=BasePoseVelocity{};
  for(unsigned tick=0;tick<2500;++tick) {
    sample.source_ns=1000000000+tick*10000;
    check(!estimator.update(sample).valid,"insufficient bounded history is invalid");
  }
  return ok ? 0 : 1;
}
