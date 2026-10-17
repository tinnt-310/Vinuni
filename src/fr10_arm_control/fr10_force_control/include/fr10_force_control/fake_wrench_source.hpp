#pragma once

#include "fr10_force_control/force_types.hpp"

namespace fr10_force {

struct FakeWrenchConfig {
  std::size_t insertion_axis{2};
  double contact_depth_m{0.005};
  double stiffness_n_m{2000.0};
  double damping_n_s_m{10.0};
  double bias_n{0.0};
  double noise_amplitude_n{0.0};
};

struct FakeWrenchFaults {
  bool dropout{false};
  bool duplicate{false};
  bool nan{false};
  bool overload{false};
  bool lost_contact{false};
};

class FakeWrenchSource {
public:
  explicit FakeWrenchSource(FakeWrenchConfig config = {});

  WrenchSample sample(double depth_m, double depth_rate_m_s,
                      double source_timestamp_s, double receipt_steady_s,
                      const FakeWrenchFaults &faults = {});
  void reset();

private:
  FakeWrenchConfig config_;
  uint64_t sequence_{0};
  double last_timestamp_s_{0.0};
};

} // namespace fr10_force
