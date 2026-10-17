#include "fr10_force_control/wrench_tare.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fr10_force {

WrenchTare::WrenchTare(WrenchTareConfig config) : config_(config) {
  const double values[]{config.minimum_duration_s, config.maximum_sample_gap_s,
      config.maximum_force_residual_n, config.maximum_torque_residual_nm,
      config.maximum_force_stddev_n, config.maximum_torque_stddev_nm};
  if (config.minimum_samples < 2)
    throw std::invalid_argument("tare requires at least two observations");
  for (double value : values)
    if (!std::isfinite(value) || value <= 0.)
      throw std::invalid_argument("invalid no-contact tare limits");
}

void WrenchTare::clearWindow() {
  count_ = control_observations_ = 0;
  mean_ = {};
  m2_ = {};
  first_time_s_ = last_time_s_ = 0.;
}

void WrenchTare::reset(const Wrench &reference_bias) {
  if (!finite(reference_bias)) throw std::invalid_argument("invalid tare bias");
  reference_bias_ = reference_bias;
  last_sequence_ = 0;
  clearWindow();
}

std::optional<Wrench> WrenchTare::observe(
    const WrenchSample &sample, const Wrench &gravity_removed,
    bool stationary_no_contact) {
  return observeBatch(&sample, &gravity_removed, 1, stationary_no_contact);
}

std::optional<bool> WrenchTare::accumulate(
    const WrenchSample &sample, const Wrench &gravity_removed) {
  const bool unique = sample.sequence > last_sequence_;
  last_sequence_ = std::max(last_sequence_, sample.sequence);
  if (!sample.valid || sample.overload || !finite(gravity_removed) ||
      !std::isfinite(sample.source_timestamp_s) || sample.source_timestamp_s <= 0.) {
    clearWindow();
    return false;
  }
  if (!unique) return std::nullopt;
  const auto value = gravity_removed.vector();
  const auto reference = reference_bias_.vector();
  Vector3 force{}, torque{};
  for (std::size_t i = 0; i < 3; ++i) {
    force[i] = value[i] - reference[i];
    torque[i] = value[i + 3] - reference[i + 3];
  }
  if (norm(force) > config_.maximum_force_residual_n ||
      norm(torque) > config_.maximum_torque_residual_nm) {
    clearWindow();
    return false;
  }
  if (count_ && (sample.source_timestamp_s <= last_time_s_ ||
      sample.source_timestamp_s - last_time_s_ > config_.maximum_sample_gap_s)) {
    clearWindow();
    // A discontinuity must never complete an earlier tare window.
    return false;
  }
  if (!count_) first_time_s_ = sample.source_timestamp_s;
  last_time_s_ = sample.source_timestamp_s;
  ++count_;
  for (std::size_t i = 0; i < 6; ++i) {
    const double delta = value[i] - mean_[i];
    mean_[i] += delta / static_cast<double>(count_);
    m2_[i] += delta * (value[i] - mean_[i]);
  }
  return true;
}

std::optional<Wrench> WrenchTare::observeBatch(const WrenchSample *samples,
    const Wrench *gravity_removed, std::size_t count, bool stationary_no_contact) {
  const auto consume = [&] {
    if (samples) for (std::size_t i=0; i<count; ++i)
      last_sequence_ = std::max(last_sequence_, samples[i].sequence);
  };
  if (!stationary_no_contact || (count && (!samples || !gravity_removed))) {
    clearWindow(); consume(); return std::nullopt;
  }
  bool observed = false;
  for (std::size_t i=0; i<count; ++i) {
    const auto accepted = accumulate(samples[i], gravity_removed[i]);
    if (accepted && !*accepted) { consume(); return std::nullopt; }
    observed = observed || (accepted && *accepted);
  }
  if (!observed) return std::nullopt;
  ++control_observations_;
  if (control_observations_ < config_.minimum_samples || count_ < 2 ||
      last_time_s_ - first_time_s_ + 1e-12 < config_.minimum_duration_s)
    return std::nullopt;
  for (std::size_t i = 0; i < 6; ++i) {
    const double limit = i < 3 ? config_.maximum_force_stddev_n
                               : config_.maximum_torque_stddev_nm;
    if (m2_[i] / static_cast<double>(count_ - 1) > limit * limit) {
      clearWindow();
      return std::nullopt;
    }
  }
  return Wrench::fromVector(mean_);
}

} // namespace fr10_force
