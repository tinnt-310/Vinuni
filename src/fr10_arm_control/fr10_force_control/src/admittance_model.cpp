#include "fr10_force_control/admittance_model.hpp"
#include "fr10_force_control/admittance_envelope.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "fr10_motion_core/math_utils.hpp"

namespace fr10_force {
namespace {

// A semi-implicit step travels dt*v before the remaining continuous braking
// distance v^2/(2*a). Keep both inside the correction envelope. Rationalizing
// the quadratic root avoids cancellation near the boundary.
double brakingVelocity(double distance, double acceleration, double dt_s) {
  const double step = acceleration * dt_s;
  return 2.0 * acceleration * distance /
      (std::hypot(step, std::sqrt(2.0 * acceleration * distance)) + step);
}


} // namespace

AdmittanceModel::AdmittanceModel(AdmittanceConfig config) {
  setConfig(config);
}

bool AdmittanceModel::validConfig(const AdmittanceConfig &config) {
  if (config.insertion_axis >= 3 ||
      !(config.max_lateral_correction_norm_m > 0.) ||
      !(config.max_rotation_correction_norm_rad > 0.) ||
      !finite(config.mass) || !finite(config.damping) ||
      !finite(config.stiffness) || !finite(config.drive_deadband) || !finite(config.max_correction) ||
      !finite(config.max_velocity) || !finite(config.max_acceleration)) {
    return false;
  }
  for (std::size_t axis = 0; axis < kCartesianDofs; ++axis) {
    if (!(config.mass[axis] > 0.0) || config.damping[axis] < 0.0 ||
        config.stiffness[axis] < 0.0 || config.drive_deadband[axis] < 0.0 ||
        !(config.max_correction[axis] > 0.0) ||
        !(config.max_velocity[axis] > 0.0) ||
        !(config.max_acceleration[axis] > 0.0)) {
      return false;
    }
  }
  return true;
}

void AdmittanceModel::setConfig(const AdmittanceConfig &config) {
  if (!validConfig(config) || !admissibleAdmittanceState(state_, config)) {
    throw std::invalid_argument("admittance configuration is invalid");
  }
  config_ = config;
  ++revision_;
  for (std::size_t axis = 0; axis < kCartesianDofs; ++axis) {
    if (!config_.selected_axes[axis]) {
      state_.correction[axis] = 0.0;
      state_.velocity[axis] = 0.0;
    }
  }
}

AdmittanceProposal AdmittanceModel::propose(const Vector6 &force_error,
                                            double dt_s) const {
  return propose(force_error, dt_s, {});
}

AdmittanceProposal AdmittanceModel::propose(const Vector6 &force_error,
    double dt_s, const Vector6 &equilibrium) const {
  AdmittanceProposal proposal;
  proposal.base_revision = revision_;
  if (!finite(force_error) || !finite(equilibrium) ||
      !std::isfinite(dt_s) || !(dt_s > 0.0)) {
    return proposal;
  }
  using namespace fr10_motion::math;
  Vector6 spring_error{};
  for (std::size_t i = 0; i < 3; ++i)
    spring_error[i] = state_.correction[i] - equilibrium[i];
  // Both rotations are expressed in the fixed insertion axes. This is the
  // SO(3) error of actual correction against equilibrium, not Euler addition
  // or subtraction of rotation vectors for noncommuting rotations.
  const auto error_rotation = quaternionToRotationVector(quaternionMultiply(
      rotationVectorToQuaternion({state_.correction[3], state_.correction[4], state_.correction[5]}),
      quaternionConjugate(rotationVectorToQuaternion({equilibrium[3], equilibrium[4], equilibrium[5]}))));
  for (std::size_t i = 0; i < 3; ++i) spring_error[i + 3] = error_rotation[i];
  proposal.valid = true;
  proposal.next_state = state_;
  for (std::size_t axis = 0; axis < kCartesianDofs; ++axis) {
    if (!config_.selected_axes[axis]) {
      proposal.next_state.correction[axis] = 0.0;
      proposal.next_state.velocity[axis] = 0.0;
      continue;
    }

    const double old_position = state_.correction[axis];
    const double old_velocity = state_.velocity[axis];
    const double denominator = config_.mass[axis] +
        dt_s * config_.damping[axis] +
        dt_s * dt_s * config_.stiffness[axis];
    if (!std::isfinite(denominator)) { proposal.valid = false; return proposal; }
    const double net_drive = force_error[axis] - config_.stiffness[axis]*spring_error[axis];
    const double drive = std::copysign(
        std::max(0.0,std::abs(net_drive)-config_.drive_deadband[axis]),net_drive);
    double velocity =
        (config_.mass[axis] * old_velocity +
         dt_s * drive) /
        denominator;

    const double minimum_velocity_from_acceleration =
        old_velocity - config_.max_acceleration[axis] * dt_s;
    const double maximum_velocity_from_acceleration =
        old_velocity + config_.max_acceleration[axis] * dt_s;
    const double acceleration_limited = std::clamp(
        velocity, minimum_velocity_from_acceleration,
        maximum_velocity_from_acceleration);
    proposal.acceleration_saturated[axis] =
        std::abs(acceleration_limited - velocity) > 1e-15;
    velocity = acceleration_limited;

    const double velocity_limited = std::clamp(
        velocity, -config_.max_velocity[axis], config_.max_velocity[axis]);
    proposal.velocity_saturated[axis] =
        std::abs(velocity_limited - velocity) > 1e-15;
    velocity = velocity_limited;

    const double forward = brakingVelocity(config_.max_correction[axis] - old_position,
        config_.max_acceleration[axis], dt_s);
    const double backward = brakingVelocity(config_.max_correction[axis] + old_position,
        config_.max_acceleration[axis], dt_s);
    const double bounded_velocity = std::clamp(velocity, -backward, forward);
    proposal.position_saturated[axis] = std::abs(bounded_velocity - velocity) > 1e-15;
    velocity = bounded_velocity;
    // An infeasible handover must not be concealed by an instantaneous stop
    // or by clipping its diagnostic acceleration.
    if (!std::isfinite(velocity) ||
        velocity < minimum_velocity_from_acceleration - 1e-12 ||
        velocity > maximum_velocity_from_acceleration + 1e-12) {
      proposal.valid = false; return proposal;
    }
    const double position = old_position + dt_s * velocity;

    proposal.next_state.correction[axis] = position;
    proposal.next_state.velocity[axis] = velocity;
    proposal.output.position[axis] = position;
    proposal.output.velocity[axis] = velocity;
    proposal.output.acceleration[axis] = (velocity - old_velocity) / dt_s;
  }
  if (!constrainAdmittanceVelocity(state_, config_, dt_s,
      proposal.next_state.velocity, proposal.position_saturated)) {
    proposal.valid = false; return proposal;
  }
  for (std::size_t axis = 0; axis < kCartesianDofs; ++axis) {
    if (!config_.selected_axes[axis]) continue;
    const double velocity = proposal.next_state.velocity[axis];
    const double position = state_.correction[axis] + dt_s * velocity;
    proposal.next_state.correction[axis] = position;
    proposal.output.position[axis] = position;
    proposal.output.velocity[axis] = velocity;
    proposal.output.acceleration[axis] = (velocity-state_.velocity[axis])/dt_s;
  }
  proposal.valid = admissibleAdmittanceState(proposal.next_state, config_);
  return proposal;
}

bool AdmittanceModel::canCommit(const AdmittanceProposal &proposal) const {
  return proposal.valid && proposal.base_revision == revision_ &&
      finite(proposal.next_state.correction) && finite(proposal.next_state.velocity) &&
      admissibleAdmittanceState(proposal.next_state, config_);
}

bool AdmittanceModel::commit(const AdmittanceProposal &proposal) {
  if (!canCommit(proposal)) return false;
  state_ = proposal.next_state;
  ++revision_;
  return true;
}

void AdmittanceModel::reset(const AdmittanceState &state) {
  if (!finite(state.correction) || !finite(state.velocity)) {
    throw std::invalid_argument("admittance reset state is invalid");
  }
  if (!admissibleAdmittanceState(state, config_))
    throw std::invalid_argument("admittance handover cannot brake inside its correction envelope");
  state_ = state;
  ++revision_;
  for (std::size_t axis = 0; axis < kCartesianDofs; ++axis) {
    if (!config_.selected_axes[axis]) {
      state_.correction[axis] = 0.0;
      state_.velocity[axis] = 0.0;
    }
  }
}

} // namespace fr10_force
