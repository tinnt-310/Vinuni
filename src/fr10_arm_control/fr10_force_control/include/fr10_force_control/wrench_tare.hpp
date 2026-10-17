#pragma once

#include <optional>
#include "fr10_force_control/force_types.hpp"

namespace fr10_force {

// Residual bias after the declared gravity owner has removed the rigid load.
// Small force alone cannot prove no contact: the caller must establish a
// stationary, collision-free clearance pose independently of the sensor.
struct WrenchTareConfig {
  std::size_t minimum_samples{25};
  double minimum_duration_s{0.192};
  double maximum_sample_gap_s{0.050};
  double maximum_force_residual_n{5.0};
  double maximum_torque_residual_nm{0.5};
  double maximum_force_stddev_n{0.25};
  double maximum_torque_stddev_nm{0.03};
};

class WrenchTare {
public:
  explicit WrenchTare(WrenchTareConfig config = {});
  // At least minimum_samples qualified control observations, not FT substeps.
  // Every unique source sample contributes to unfiltered mean and variance.
  // Returned bias is canonical, gravity removed, including the prior bias.
  std::optional<Wrench> observe(const WrenchSample &sample,
                               const Wrench &gravity_removed,
                               bool stationary_no_contact);
  std::optional<Wrench> observeBatch(const WrenchSample *samples,
      const Wrench *gravity_removed, std::size_t count, bool stationary_no_contact);
  void reset(const Wrench &reference_bias = {});
  std::size_t sampleCount() const { return count_; }

private:
  void clearWindow();
  std::optional<bool> accumulate(const WrenchSample &, const Wrench &);
  WrenchTareConfig config_;
  Wrench reference_bias_;
  Vector6 mean_{};
  Vector6 m2_{};
  std::size_t count_{0};
  std::size_t control_observations_{0};
  uint64_t last_sequence_{0};
  double first_time_s_{0.0};
  double last_time_s_{0.0};
};

} // namespace fr10_force
