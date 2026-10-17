#pragma once

#include <array>
#include <cstdint>
#include <limits>

#include "fr10_force_control/force_types.hpp"

namespace fr10_force {

struct AdmittanceConfig {
  Vector6 mass{{1.0, 1.0, 1.0, 0.1, 0.1, 0.1}};
  Vector6 damping{{50.0, 50.0, 50.0, 5.0, 5.0, 5.0}};
  Vector6 stiffness{};
  // Continuous dead zone on F_error - K*x, in N / N m. Zero preserves the
  // original model. It never changes measured force or safety/completion data.
  Vector6 drive_deadband{};
  Vector6 max_correction{{0.02, 0.02, 0.02, 0.1, 0.1, 0.1}};
  Vector6 max_velocity{{0.02, 0.02, 0.02, 0.1, 0.1, 0.1}};
  Vector6 max_acceleration{{0.2, 0.2, 0.2, 1.0, 1.0, 1.0}};
  std::array<bool, kCartesianDofs> selected_axes{};
  // Infinity retains the legacy component-only contract. A session binds
  // these to its safety envelopes; they are not extra independently tuned limits.
  double max_lateral_correction_norm_m{std::numeric_limits<double>::infinity()};
  double max_rotation_correction_norm_rad{std::numeric_limits<double>::infinity()};
  std::size_t insertion_axis{2};
};

struct AdmittanceState {
  Vector6 correction{};
  Vector6 velocity{};
};

struct AdmittanceProposal {
  bool valid{false};
  CartesianCorrection output;
  AdmittanceState next_state;
  std::array<bool, kCartesianDofs> position_saturated{};
  std::array<bool, kCartesianDofs> velocity_saturated{};
  std::array<bool, kCartesianDofs> acceleration_saturated{};
  uint64_t base_revision{0};
};

class AdmittanceModel {
public:
  explicit AdmittanceModel(AdmittanceConfig config = {});

  [[nodiscard]] AdmittanceProposal propose(const Vector6 &force_error,
                                           double dt_s) const;
  [[nodiscard]] AdmittanceProposal propose(const Vector6 &force_error,
                                           double dt_s,
                                           const Vector6 &equilibrium) const;
  bool commit(const AdmittanceProposal &proposal);
  bool canCommit(const AdmittanceProposal &proposal) const;
  void reset(const AdmittanceState &state = {});
  void setConfig(const AdmittanceConfig &config);

  const AdmittanceConfig &config() const { return config_; }
  const AdmittanceState &state() const { return state_; }

private:
  static bool validConfig(const AdmittanceConfig &config);

  AdmittanceConfig config_;
  AdmittanceState state_;
  uint64_t revision_{0};
};

} // namespace fr10_force
