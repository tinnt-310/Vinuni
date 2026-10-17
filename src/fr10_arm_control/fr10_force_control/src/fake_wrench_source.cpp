#include "fr10_force_control/fake_wrench_source.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace fr10_force {

FakeWrenchSource::FakeWrenchSource(FakeWrenchConfig config)
    : config_(config) {
  if (config_.insertion_axis >= 3 ||
      !std::isfinite(config_.contact_depth_m) ||
      !std::isfinite(config_.stiffness_n_m) ||
      !std::isfinite(config_.damping_n_s_m) ||
      !std::isfinite(config_.bias_n) ||
      !std::isfinite(config_.noise_amplitude_n) ||
      config_.stiffness_n_m < 0.0 || config_.damping_n_s_m < 0.0 ||
      config_.noise_amplitude_n < 0.0) {
    throw std::invalid_argument("fake wrench configuration is invalid");
  }
}

WrenchSample FakeWrenchSource::sample(
    double depth_m, double depth_rate_m_s, double source_timestamp_s,
    double receipt_steady_s, const FakeWrenchFaults &faults) {
  WrenchSample result;
  if (faults.dropout) {
    return result;
  }
  result.valid = true;
  result.overload = faults.overload;
  result.sequence = faults.duplicate ? sequence_ : ++sequence_;
  result.source_timestamp_s = faults.duplicate ? last_timestamp_s_
                                                : source_timestamp_s;
  result.receipt_steady_s = receipt_steady_s;
  if (!faults.duplicate) {
    last_timestamp_s_ = source_timestamp_s;
  }
  double force = config_.bias_n;
  if (!faults.lost_contact && depth_m > config_.contact_depth_m) {
    force += config_.stiffness_n_m * (depth_m - config_.contact_depth_m) +
        config_.damping_n_s_m * std::max(0.0, depth_rate_m_s);
  }
  // Deterministic bounded pseudo-noise; no RNG/global state is involved.
  force += config_.noise_amplitude_n *
      std::sin(static_cast<double>(result.sequence) * 1.61803398875);
  result.wrench.force_n[config_.insertion_axis] = force;
  if (faults.nan) {
    result.wrench.force_n[config_.insertion_axis] =
        std::numeric_limits<double>::quiet_NaN();
  }
  return result;
}

void FakeWrenchSource::reset() {
  sequence_ = 0;
  last_timestamp_s_ = 0.0;
}

} // namespace fr10_force
