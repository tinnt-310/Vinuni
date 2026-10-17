#pragma once
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace fr10_motion {
enum class TickDecision { WAIT, STEP, BACKWARD_JUMP, INVALID };
enum class ResumeDecision { WAIT, READY, OBSERVATION_EXPIRED, FEEDBACK_EXPIRED };

// Only missing actuator feedback prevents executing a controlled stop. Other
// expired observations return to their safety monitor after the same bounded
// handover window, so a sensor fault cannot destroy its actuator owner.
inline ResumeDecision resumeReadiness(bool feedback_ready, bool observations_ready,
    double elapsed_s, double maximum_wait_s) {
  if (!std::isfinite(elapsed_s) || elapsed_s < 0. ||
      !std::isfinite(maximum_wait_s) || maximum_wait_s <= 0.)
    return ResumeDecision::FEEDBACK_EXPIRED;
  if (feedback_ready && observations_ready) return ResumeDecision::READY;
  if (elapsed_s <= maximum_wait_s) return ResumeDecision::WAIT;
  return feedback_ready ? ResumeDecision::OBSERVATION_EXPIRED : ResumeDecision::FEEDBACK_EXPIRED;
}
// A single source-time contract for JOINT, POSE and FORCE. Hardware scheduling
// uses the wall timer; simulation emits at most one 8 ms step per callback.
class TickClock {
public:
  explicit TickClock(double period = 0.008) : period_(period) {
    if (!std::isfinite(period) || period <= 0) throw std::invalid_argument("tick period");
  }
  TickDecision advance(double source, bool simulation) {
    if (!std::isfinite(source) || source < 0) return TickDecision::INVALID;
    if (!initialized_) {
      initialized_ = true; last_source_ = boundary_ = source;
      return simulation ? TickDecision::WAIT : TickDecision::STEP;
    }
    if (source + 1e-12 < last_source_) {
      last_source_ = boundary_ = source; return TickDecision::BACKWARD_JUMP;
    }
    last_source_ = source;
    if (!simulation) { boundary_ = source; return TickDecision::STEP; }
    const double elapsed = source - boundary_;
    if (elapsed + 1e-12 < period_) return TickDecision::WAIT;
    if (elapsed >= 2 * period_) {
      ++missed_windows_; boundary_ = source;
    } else { boundary_ += period_; }
    return TickDecision::STEP;
  }
  uint64_t missedWindows() const { return missed_windows_; }
private:
  double period_, last_source_{0}, boundary_{0};
  bool initialized_{false};
  uint64_t missed_windows_{0};
};
} // namespace fr10_motion
