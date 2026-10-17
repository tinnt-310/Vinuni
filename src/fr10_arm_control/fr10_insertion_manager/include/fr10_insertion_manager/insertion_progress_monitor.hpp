#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace fr10_insertion {

// Directional advancement or sustained reduction of alignment/seating error. Neither
// oscillation nor a phase change alone renews the no-progress watchdog.
class InsertionProgressMonitor {
public:
  InsertionProgressMonitor(double timeout_s, double depth_step_m, double fraction)
      : timeout_s_(timeout_s), depth_step_m_(depth_step_m), fraction_(fraction) {
    if (!std::isfinite(timeout_s) || timeout_s <= 0 ||
        !std::isfinite(depth_step_m) || depth_step_m <= 0 ||
        !std::isfinite(fraction) || fraction <= 0 || fraction >= 1)
      throw std::invalid_argument("invalid insertion progress profile");
  }
  bool update(double dt_s, double depth_m, bool improving, double load,
              std::size_t objective = 0) {
    if (!std::isfinite(dt_s) || dt_s < 0 || !std::isfinite(depth_m) ||
        !std::isfinite(load) || load < 0 || objective >= best_load_.size()) return false;
    if (!initialized_) { depth_reference_m_ = depth_m; initialized_ = true; }
    elapsed_s_ += dt_s;
    if (depth_m >= depth_reference_m_ + depth_step_m_) {
      depth_reference_m_ = depth_m; elapsed_s_ = 0.;
    }
    if (improving && !objective_initialized_[objective]) {
      best_load_[objective] = load;
      objective_initialized_[objective] = true;
    }
    if (improving && load <= best_load_[objective] -
        std::max(.05, fraction_ * best_load_[objective])) {
      best_load_[objective] = load; elapsed_s_ = 0.;
    }
    return elapsed_s_ < timeout_s_;
  }
  void reset() {
    initialized_ = false; elapsed_s_ = 0.; best_load_.fill(0.);
    objective_initialized_.fill(false);
  }
private:
  double timeout_s_, depth_step_m_, fraction_;
  double depth_reference_m_{0.}, elapsed_s_{0.};
  std::array<double, 2> best_load_{};
  std::array<bool, 2> objective_initialized_{};
  bool initialized_{false};
};

} // namespace fr10_insertion
