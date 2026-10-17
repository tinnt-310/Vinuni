#pragma once
#include <limits>

#include "fr10_force_control/force_types.hpp"

namespace fr10_force {

enum class ForceSafetyFault : int {
  NONE = 0,
  INVALID_WRENCH,
  SENSOR_OVERLOAD,
  STALE_WRENCH,
  CLOCK_JUMP,
  FORCE_LIMIT,
  TORQUE_LIMIT,
  FORCE_RATE_LIMIT,
  CORRECTION_LIMIT,
  DEPTH_LIMIT,
  LOST_CONTACT,
  IK_FAILURE,
  COLLISION,
  SCENE_CHANGED,
  TRACKING_FAILURE,
  ROBOT_FAULT,
  COMMAND_REJECTED
};

struct ForceSafetyConfig {
  Vector3 max_force_n{{100.0, 100.0, 100.0}};
  Vector3 max_torque_nm{{20.0, 20.0, 20.0}};
  Vector3 max_force_rate_n_s{{1000.0, 1000.0, 1000.0}};
  Vector6 max_correction{{0.03, 0.03, 0.03, 0.15, 0.15, 0.15}};
  double max_depth_m{0.05};
  double stale_timeout_s{0.050};
  double max_lateral_correction_norm_m{std::numeric_limits<double>::infinity()};
  double max_rotation_correction_norm_rad{std::numeric_limits<double>::infinity()};
  std::size_t insertion_axis{2};
};

struct ForceSafetyInput {
  WrenchSample raw_sample;
  Wrench conditioned_wrench;
  CartesianCorrection correction;
  double depth_m{0.0};
  double now_steady_s{0.0};
  bool contact_required{false};
  bool contact_confirmed{false};
  bool ik_ok{true};
  bool collision_free{true};
  bool scene_unchanged{true};
  bool tracking_ok{true};
  bool robot_ok{true};
  bool command_accepted{true};
  // Only set by the conditioning owner when an idle/FT_ZERO tare changes
  // the measurement reference. Never resets source freshness or raw limits.
  bool conditioned_reference_changed{false};
};

struct ForceSafetyResult {
  ForceSafetyFault fault{ForceSafetyFault::NONE};
  double source_age_s{0.0};
  Vector3 force_rate_n_s{};

  bool safe() const { return fault == ForceSafetyFault::NONE; }
};

class ForceSafetyMonitor {
public:
  explicit ForceSafetyMonitor(ForceSafetyConfig config = {});

  ForceSafetyResult update(const ForceSafetyInput &input);
  void reset();

private:
  ForceSafetyConfig config_;
  uint64_t last_sequence_{0};
  double last_unique_receipt_s_{0.0};
  double last_source_timestamp_s_{0.0};
  Vector3 last_force_n_{};
  Vector3 last_force_rate_n_s_{};
  bool initialized_{false};
};

} // namespace fr10_force
