#pragma once

#include <optional>
#include "fr10_force_control/force_types.hpp"

namespace fr10_force {

struct WrenchFilterConfig {
  double cutoff_hz{20.0};
};

struct WrenchFilterResult {
  bool accepted{false};
  bool duplicate{false};
  WrenchSample sample;
};

class WrenchFilter {
public:
  explicit WrenchFilter(WrenchFilterConfig config = {});

  [[nodiscard]] WrenchFilterResult update(const WrenchSample &sample,
                                          double dt_s);
  void reset();

private:
  WrenchFilterConfig config_;
  Wrench filtered_;
  uint64_t last_sequence_{0};
  double last_source_timestamp_s_{0.0};
  bool initialized_{false};
};

// Process verification slope over a finite source interval. This is separate
// from the immediate raw/conditioned rate protection in ForceSafetyMonitor.
// Called at control-observation cadence; insufficient bounded history denies
// verification. Duplicates retain the last measurement without renewing time.
class WrenchVerificationSlope {
public:
  explicit WrenchVerificationSlope(double window_s, double maximum_gap_s=.050);
  std::optional<Vector3> update(const WrenchSample &sample);
  void reset();
private:
  struct Observation {double source_s{0};Vector3 force{};};
  const Observation &at(std::size_t index) const;
  std::array<Observation,64> history_{};
  std::size_t head_{0},size_{0};
  uint64_t sequence_{0};
  double window_s_,maximum_gap_s_;
  std::optional<Vector3> last_result_;
};

} // namespace fr10_force
