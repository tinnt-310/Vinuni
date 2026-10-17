#include "fr10_force_control/wrench_compensation.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include "fr10_force_control/wrench_transform.hpp"

namespace fr10_force {

WrenchCompensator::WrenchCompensator(WrenchCompensationConfig config)
    : config_(config) {
  const bool sign_valid = std::isfinite(config_.source_sign) &&
      std::abs(std::abs(config_.source_sign) - 1.0) < 1e-12;
  const bool gravity_valid = finite(config_.tool_cog_m) &&
      finite(config_.gravity_m_s2) &&
      (!config_.gravity_enabled ||
       (config_.gravity_calibrated && std::isfinite(config_.tool_mass_kg) && config_.tool_mass_kg > 0.0));
  if (!sign_valid || !gravity_valid ||
      (config_.gravity_owner == GravityOwner::SOURCE && config_.gravity_enabled)) {
    throw std::invalid_argument("wrench compensation configuration is invalid");
  }
}

Wrench WrenchCompensator::apply(const Wrench &raw) const {
  return apply(raw, {1,0,0,0,1,0,0,0,1});
}

Wrench WrenchCompensator::apply(
    const Wrench &raw, const RotationMatrix &source_from_gravity_frame) const {
  RigidTransform rotation;
  rotation.rotation_target_from_source = source_from_gravity_frame;
  if (!finite(raw) || !WrenchTransform::valid(rotation)) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    return {{nan, nan, nan}, {nan, nan, nan}};
  }
  auto native = raw.vector();
  const auto bias = bias_.vector();
  if (config_.gravity_enabled) {
    Vector3 gravity_force = rotate(source_from_gravity_frame, config_.gravity_m_s2);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      gravity_force[axis] =
          config_.tool_mass_kg * gravity_force[axis];
      native[axis] -= gravity_force[axis];
    }
    const auto gravity_torque = cross(config_.tool_cog_m, gravity_force);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      native[axis + 3] -= gravity_torque[axis];
    }
  }
  // The rigid load is expressed in the sensor's native force convention.
  // Convert both raw and modeled load together, then remove canonical bias.
  // Applying sign only to raw doubles gravity for a reversed sensor source.
  auto compensated = native;
  for (std::size_t axis = 0; axis < kCartesianDofs; ++axis) {
    compensated[axis] = config_.source_sign * native[axis] - bias[axis];
  }
  return Wrench::fromVector(compensated);
}

bool WrenchCompensator::setBias(const Wrench &bias) {
  if (!finite(bias)) {
    return false;
  }
  bias_ = bias;
  return true;
}

} // namespace fr10_force
