#pragma once

#include "fr10_force_control/force_types.hpp"

namespace fr10_force {

enum class GravityOwner { APPLICATION, SOURCE };

struct WrenchCompensationConfig {
  double source_sign{1.0};
  bool gravity_enabled{false};
  bool gravity_calibrated{false};
  double tool_mass_kg{0.0};
  Vector3 tool_cog_m{};
  Vector3 gravity_m_s2{{0.0, 0.0, -9.80665}};
  // SOURCE means an independently calibrated upstream already removes load.
  // Fairino ft_sensor_data (RCS) alone does not establish this ownership.
  GravityOwner gravity_owner{GravityOwner::APPLICATION};
};

class WrenchCompensator {
public:
  explicit WrenchCompensator(WrenchCompensationConfig config = {});

  [[nodiscard]] Wrench apply(const Wrench &raw) const;
  [[nodiscard]] Wrench apply(const Wrench &raw,
                             const RotationMatrix &source_from_gravity_frame) const;
  bool setBias(const Wrench &bias);
  const Wrench &bias() const { return bias_; }

private:
  WrenchCompensationConfig config_;
  Wrench bias_;
};

} // namespace fr10_force
