#pragma once

#include "fr10_force_control/admittance_model.hpp"

namespace fr10_force {

// A convex position set (component box intersected with lateral/rotation
// balls) and a straight, acceleration-bounded stopping corridor. No pose or
// reported acceleration is clipped after integration.
bool admissibleAdmittanceState(const AdmittanceState &state, const AdmittanceConfig &config);
bool constrainAdmittanceVelocity(const AdmittanceState &state, const AdmittanceConfig &config,
    double dt_s, Vector6 &velocity, std::array<bool, kCartesianDofs> &position_saturated);

} // namespace fr10_force
