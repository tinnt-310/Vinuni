#include "fr10_force_control/admittance_envelope.hpp"

#include <algorithm>
#include <cmath>

namespace fr10_force {
namespace {

struct CorrectionGroup {
  std::array<std::size_t, 3> axes{};
  std::size_t count{0};
  double radius{0.};
};

std::array<CorrectionGroup, 2> groups(const AdmittanceConfig &config) {
  CorrectionGroup lateral, rotation;
  lateral.radius = config.max_lateral_correction_norm_m;
  rotation.radius = config.max_rotation_correction_norm_rad;
  for (std::size_t i=0; i<3; ++i) {
    if (i != config.insertion_axis && config.selected_axes[i]) lateral.axes[lateral.count++] = i;
    if (config.selected_axes[i+3]) rotation.axes[rotation.count++] = i+3;
  }
  return {lateral, rotation};
}

double stoppingTime(const Vector6 &velocity, const AdmittanceConfig &config, const CorrectionGroup &group) {
  double duration = 0.;
  for (std::size_t i=0; i<group.count; ++i) {
    const auto axis = group.axes[i];
    duration = std::max(duration, std::abs(velocity[axis])/config.max_acceleration[axis]);
  }
  return duration;
}

bool admissibleGroup(const AdmittanceState &state, const AdmittanceConfig &config,
    const CorrectionGroup &group) {
  if (!std::isfinite(group.radius)) return true;
  const double duration = stoppingTime(state.velocity, config, group);
  double position_squared = 0., stop_squared = 0.;
  for (std::size_t i=0; i<group.count; ++i) {
    const auto axis = group.axes[i];
    const double position = state.correction[axis];
    const double stop = position + .5*duration*state.velocity[axis];
    if (std::abs(stop) > config.max_correction[axis]) return false;
    position_squared += position*position;
    stop_squared += stop*stop;
  }
  return position_squared <= group.radius*group.radius && stop_squared <= group.radius*group.radius;
}

// For a fixed stopping duration T, admissible velocity is a box intersected
// with a translated ball: x=q+(dt+T/2)*v. Project the requested velocity onto
// that convex set. Unlike scaling the whole vector, this retains tangent
// motion and supplies the small inward velocity needed by curved motion.
bool projectAtStoppingTime(const AdmittanceState &state, const AdmittanceConfig &config,
    const CorrectionGroup &group, double dt, double duration,
    const Vector6 &requested, Vector6 &candidate) {
  const double horizon = dt + .5*duration;
  std::array<double,3> lower{}, upper{}, desired{}, velocity_lower{}, velocity_upper{};
  double nearest_squared = 0.;
  for (std::size_t i=0; i<group.count; ++i) {
    const auto axis=group.axes[i];
    const double position=state.correction[axis], old=state.velocity[axis];
    const double acceleration=config.max_acceleration[axis];
    const double lo=std::max({-config.max_velocity[axis], old-acceleration*dt,
        -acceleration*duration, (-config.max_correction[axis]-position)/horizon});
    const double hi=std::min({config.max_velocity[axis], old+acceleration*dt,
        acceleration*duration, (config.max_correction[axis]-position)/horizon});
    if (lo>hi) return false;
    velocity_lower[i]=lo;velocity_upper[i]=hi;
    lower[i]=position+horizon*lo; upper[i]=position+horizon*hi;
    desired[i]=position+horizon*requested[axis];
    const double nearest=std::clamp(0.,lower[i],upper[i]);
    nearest_squared+=nearest*nearest;
  }
  const double radius_squared=group.radius*group.radius;
  if (nearest_squared>radius_squared) return false;
  const auto projected = [&](double multiplier) {
    auto velocity=requested;
    for (std::size_t i=0; i<group.count; ++i) {
      const auto axis=group.axes[i];
      const double point=std::clamp(desired[i]/(1.+multiplier),lower[i],upper[i]);
      // Inverting q+h*v can round outside the original velocity interval.
      // Restore that same closed interval before validating the integrated
      // position and real stop; neither pose nor its derivatives are hidden.
      velocity[axis]=std::clamp((point-state.correction[axis])/horizon,
          velocity_lower[i],velocity_upper[i]);
    }
    return velocity;
  };
  const auto feasible = [&](double multiplier) {
    double squared=0.;
    for (std::size_t i=0; i<group.count; ++i) {
      const double point=std::clamp(desired[i]/(1.+multiplier),lower[i],upper[i]);
      squared+=point*point;
    }
    auto next=state;next.velocity=projected(multiplier);
    for (std::size_t i=0;i<group.count;++i) {
      const auto axis=group.axes[i];
      next.correction[axis]+=dt*next.velocity[axis];
    }
    // Validate the implemented two-add integration/braking arithmetic too.
    // Reconstructing velocity from the projected stop can round one ULP
    // outward; the feasibility search must not waive the safety boundary.
    return squared<=radius_squared && admissibleGroup(next,config,group);
  };
  double multiplier=0.;
  if (!feasible(0.)) {
    double lo=0.,hi=1.;
    for (unsigned i=0; i<64 && !feasible(hi); ++i) hi*=2.;
    if (!feasible(hi)) return false;
    for (unsigned i=0;i<48;++i) {
      const double mid=.5*(lo+hi);
      if (feasible(mid)) hi=mid; else lo=mid;
    }
    multiplier=hi;
  }
  candidate=projected(multiplier);
  return true;
}

} // namespace

bool admissibleAdmittanceState(const AdmittanceState &state, const AdmittanceConfig &config) {
  if (!finite(state.correction) || !finite(state.velocity)) return false;
  for (std::size_t axis=0; axis<kCartesianDofs; ++axis) {
    if (!config.selected_axes[axis]) continue;
    const double position = state.correction[axis], velocity = state.velocity[axis];
    if (std::abs(position) > config.max_correction[axis] ||
        std::abs(velocity) > config.max_velocity[axis]) return false;
    // Directional remaining distance is limit-x for v>=0, limit+x for v<0.
    const double directional_remaining = velocity >= 0. ?
        config.max_correction[axis] - position : config.max_correction[axis] + position;
    if (velocity*velocity > 2.*config.max_acceleration[axis]*directional_remaining + 1e-18) return false;
  }
  for (const auto &group : groups(config))
    if (!admissibleGroup(state, config, group)) return false;
  return true;
}

bool constrainAdmittanceVelocity(const AdmittanceState &state, const AdmittanceConfig &config,
    double dt_s, Vector6 &velocity, std::array<bool, kCartesianDofs> &position_saturated) {
  for (const auto &group : groups(config)) {
    if (!std::isfinite(group.radius) || group.count == 0) continue;
    const auto admissible = [&](const Vector6 &candidate) {
      auto next = state;
      for (std::size_t i=0; i<group.count; ++i) {
        const auto axis = group.axes[i];
        next.correction[axis] += dt_s*candidate[axis];
        next.velocity[axis] = candidate[axis];
      }
      return admissibleGroup(next, config, group);
    };
    if (admissible(velocity)) continue;
    // Uniform deceleration preserves a straight stop inside the convex set.
    // T=max(|v_i|/a_i) bounds each actual acceleration. This baseline is
    // feasible whenever the admitted state has its reserved stopping corridor.
    const double duration = stoppingTime(state.velocity, config, group);
    const double retention = duration > dt_s ? 1.-dt_s/duration : 0.;
    auto braking = velocity;
    for (std::size_t i=0; i<group.count; ++i) {
      const auto axis = group.axes[i];
      braking[axis] = retention*state.velocity[axis];
    }
    if (!admissible(braking)) return false;
    const auto requested = velocity;
    const auto distanceSquared = [&](const Vector6 &candidate) {
      double squared=0.;
      for (std::size_t i=0;i<group.count;++i) {
        const auto axis=group.axes[i];
        const double error=candidate[axis]-requested[axis];
        squared+=error*error;
      }
      return squared;
    };
    velocity=braking;
    double best_distance=distanceSquared(braking);
    const double minimum_duration=std::max(0.,duration-dt_s);
    const double maximum_duration=duration+dt_s;
    // Bounded one-dimensional sampling of stopping time; each subproblem is
    // convex. Tmin includes the admitted braking baseline. Final admission
    // uses the actual stopping time, rather than assuming the chosen T.
    for (unsigned sample=0;sample<=16;++sample) {
      const double trial_duration=minimum_duration+
          (maximum_duration-minimum_duration)*static_cast<double>(sample)/16.;
      Vector6 candidate;
      if (!projectAtStoppingTime(state,config,group,dt_s,trial_duration,requested,candidate) ||
          !admissible(candidate)) continue;
      const double distance=distanceSquared(candidate);
      if (distance<best_distance) {best_distance=distance;velocity=candidate;}
    }
    for (std::size_t i=0; i<group.count; ++i) {
      const auto axis = group.axes[i];
      position_saturated[axis] = position_saturated[axis] || std::abs(velocity[axis]-requested[axis]) > 1e-15;
    }
  }
  return true;
}

} // namespace fr10_force
