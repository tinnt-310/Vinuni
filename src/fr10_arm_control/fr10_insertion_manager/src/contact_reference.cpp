#include "fr10_insertion_manager/contact_reference.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "fr10_motion_core/math_utils.hpp"

namespace fr10_insertion {
namespace {
ContactIntervalConfig lateralConfig(const ContactReferenceConfig &reference) {
  auto config=reference.lateral_probe;
  if(reference.search_radius_m>0.)config.max_coordinate=std::min(config.max_coordinate,reference.search_radius_m);
  config.maximum_observation_gap_s=reference.maximum_observation_gap_s;
  return config;
}
}

fr10_force::Vector6 contactEquilibriumAtDepth(const ContactReferenceState &state, double depth_m,
    const fr10_force::Vector6 *accepted_correction) {
  auto equilibrium = state.equilibrium;
  const bool captured = state.mode == ContactReferenceMode::CAPTURED && accepted_correction;
  if (!captured && state.mode != ContactReferenceMode::ACQUIRED) return equilibrium;
  using namespace fr10_motion::math;
  const auto &orientation = captured ? *accepted_correction : equilibrium;
  const auto direction = rotateVector(rotationVectorToQuaternion(
      {orientation[3], orientation[4], orientation[5]}), {0., 0., 1.});
  const auto entry_direction = captured ? rotateVector(
      rotationVectorToQuaternion(state.capture_rotation_rad), {0., 0., 1.}) : direction;
  // Infer the same mouth point from the captured tip/depth/direction, then
  // transport it along the accepted tool direction. Keeping the entry XY at
  // every depth instead produces a spring load opposing physical alignment.
  // d here is axial projection in the immutable input frame. Parameterizing
  // by that projection avoids changing the meaning of existing goal depth.
  for (std::size_t i = 0; i < 2; ++i)
    equilibrium[i] += depth_m * direction[i] / direction[2] -
        state.acquisition_depth_m * entry_direction[i] / entry_direction[2];
  return equilibrium;
}

ContactReference::ContactReference(ContactReferenceConfig config) : config_(config),
    yaw_probe_([&] {
      auto probe = config.keyed_yaw;
      probe.max_rotation_rad = config.max_rotation_rad;
      probe.angle_tolerance_rad = config.anchor_rotation_tolerance_rad;
      probe.maximum_observation_gap_s = config.maximum_observation_gap_s;
      return probe;
    }()),lateral_probes_{ContactIntervalProbe(lateralConfig(config)),ContactIntervalProbe(lateralConfig(config))} {
  if ((yaw_probe_.enabled() || lateral_probes_[0].enabled()) && !enabled())
    throw std::invalid_argument("contact interval probes require bounded contact search");
  if (!std::isfinite(config.search_radius_m) || config.search_radius_m < 0.)
    throw std::invalid_argument("invalid contact search radius");
  const double positive[]{config.spiral_pitch_m, config.search_speed_m_s,
      config.stall_time_s, config.search_budget_s, config.progress_step_m,
      config.acquisition_depth_m, config.acquisition_hold_s, config.contact_force_n,
      config.maximum_observation_gap_s,
      config.search_force_n, config.max_lateral_force_n, config.max_torque_nm,
      config.max_rotation_rad, config.anchor_position_tolerance_m, config.anchor_rotation_tolerance_rad};
  for (double value : positive)
    if (!std::isfinite(value) || value <= 0.)
      throw std::invalid_argument("invalid bounded contact reference configuration");
  if (config.max_rotation_rad >= fr10_motion::math::kHalfPi)
    throw std::invalid_argument("contact corridor rotation must retain a positive axial projection");
  if ((enabled() && config.anchor_position_tolerance_m >= config.search_radius_m) ||
      config.anchor_rotation_tolerance_rad >= config.max_rotation_rad)
    throw std::invalid_argument("anchor observation uncertainty must be smaller than its hard correction envelope");
}

void ContactReference::clearAcquisitionWindow() {
  qualified_since_source_s_.reset();
  ++qualification_revision_;
}

void ContactReference::observe(const ContactReferenceObservation &input) {
  for(std::size_t i=0;i<2;++i)lateral_probes_[i].observe(lateralObservation(input,i));
  yaw_probe_.observe({input.valid && state_.mode == ContactReferenceMode::CAPTURED &&
      input.depth_m >= config_.acquisition_depth_m &&
      input.measured_wrench.force_n[2] >= config_.contact_force_n,
      {input.actual_correction[3], input.actual_correction[4], input.actual_correction[5]},
      input.measured_wrench.torque_nm[2], input.source_time_s});
  const double elapsed = input.source_time_s - last_observation_source_s_;
  const bool continuous = !std::isfinite(last_observation_source_s_) ||
      (elapsed > 0. && elapsed <= config_.maximum_observation_gap_s);
  last_observation_source_s_ = input.source_time_s;
  const bool final_axis = state_.mode == ContactReferenceMode::CAPTURED;
  const bool qualified = enabled() && input.valid && continuous &&
      fr10_force::finite(input.actual_correction) && fr10_force::finite(input.measured_wrench) &&
      std::isfinite(input.source_time_s) && std::isfinite(input.depth_m) &&
      std::isfinite(input.dt_s) && input.dt_s > 0. &&
      input.phase != ContactReferencePhase::INACTIVE &&
      input.depth_m >= config_.acquisition_depth_m &&
      std::hypot(input.measured_wrench.force_n[0], input.measured_wrench.force_n[1]) <= config_.max_lateral_force_n &&
      fr10_force::norm(input.measured_wrench.torque_nm) <= config_.max_torque_nm &&
      (!final_axis || (input.phase == ContactReferencePhase::SEAT &&
          input.measured_wrench.force_n[2] >= config_.contact_force_n));
  if (!qualified) { clearAcquisitionWindow(); return; }
  if (!qualified_since_source_s_) qualified_since_source_s_ = input.source_time_s;
}

ContactReferenceProposal ContactReference::propose(const ContactReferenceObservation &input) const {
  ContactReferenceProposal proposal;
  proposal.base_revision = revision_;
  proposal.qualification_revision = qualification_revision_;
  proposal.next = state_;
  if (!input.valid || (enabled() && input.source_time_s != last_observation_source_s_) ||
      !fr10_force::finite(input.actual_correction) || !fr10_force::finite(input.measured_wrench) ||
      !std::isfinite(input.depth_m) || !std::isfinite(input.source_time_s) ||
      input.source_time_s < state_.last_source_time_s || !std::isfinite(input.dt_s) || input.dt_s <= 0.)
    return proposal;
  proposal.valid = true;
  auto &next = proposal.next;
  next.last_source_time_s = input.source_time_s;
  if (!enabled() || input.phase == ContactReferencePhase::INACTIVE ||
      state_.mode == ContactReferenceMode::EXHAUSTED || state_.mode == ContactReferenceMode::ACQUIRED)
    return proposal;
  if (contactReferencePending(state_.mode) &&
      input.source_time_s - state_.search_start_source_s >= config_.search_budget_s) {
    next.mode = ContactReferenceMode::EXHAUSTED;
    return proposal;
  }
  next.contact_observed = state_.contact_observed ||
      input.measured_wrench.force_n[2] >= config_.contact_force_n;
  if (input.depth_m >= state_.best_depth_m + config_.progress_step_m) {
    next.best_depth_m = input.depth_m;
    next.stalled_s = 0.;
  } else {
    next.stalled_s += input.dt_s;
  }
  // Bore advancement only qualifies a translation equilibrium. Its clearance
  // does not identify the true axis. Qualify the final axis on the loaded
  // shoulder, after moment alignment; never freeze an erroneous free-space
  // orientation merely because the connector has passed the lead-in.
  const bool final_axis = state_.mode == ContactReferenceMode::CAPTURED;
  next.acquisition_elapsed_s = qualified_since_source_s_
      ? input.source_time_s - *qualified_since_source_s_ : 0.;
  if (final_axis && yaw_probe_.enabled() && yaw_probe_.state().mode != KeyedYawProbeMode::COMPLETE) {
    const bool start = next.acquisition_elapsed_s >= config_.acquisition_hold_s;
    proposal.yaw_probe = yaw_probe_.propose({input.valid && input.depth_m >= config_.acquisition_depth_m,
        {input.actual_correction[3], input.actual_correction[4], input.actual_correction[5]},
        input.measured_wrench.torque_nm[2], input.source_time_s}, start);
    if (!proposal.yaw_probe.valid) { proposal.valid = false; return proposal; }
    const auto mode = proposal.yaw_probe.next.mode;
    if (mode == KeyedYawProbeMode::CENTERING || mode == KeyedYawProbeMode::COMPLETE)
      next.equilibrium[5] = proposal.yaw_probe.next.center_rad;
    // The completed probe must first be admitted, then loaded low-moment
    // qualification restarts before acquiring the final six-axis corridor.
    if (yaw_probe_.state().mode != KeyedYawProbeMode::COMPLETE) return proposal;
  }
  if(final_axis && lateral_probes_[0].enabled()) {
    bool preceding_complete=true;
    for(std::size_t i=0;i<2;++i) {
      // Completed scans have already committed their center. They no longer
      // own feasibility of another compliant axis; final acquisition and the
      // independent command/tracking safety still qualify the full pose.
      if(lateral_probes_[i].state().mode==ContactIntervalMode::COMPLETE)continue;
      auto &probe=proposal.lateral_probes[i];
      probe=lateral_probes_[i].propose(lateralObservation(input,i),
          preceding_complete && next.acquisition_elapsed_s>=config_.acquisition_hold_s);
      if(!probe.valid) {proposal.valid=false;return proposal;}
      if(probe.next.mode==ContactIntervalMode::CENTERING || probe.next.mode==ContactIntervalMode::COMPLETE)
        next.equilibrium[i]=probe.next.center;
      preceding_complete=preceding_complete && lateral_probes_[i].state().mode==ContactIntervalMode::COMPLETE;
    }
    // Commit centering, then qualify a fresh loaded six-axis corridor. No
    // quiet clearance edge is treated as a measured socket center.
    if(!preceding_complete)return proposal;
  }
  if (next.acquisition_elapsed_s >= config_.acquisition_hold_s) {
    const double lateral = std::hypot(input.actual_correction[0], input.actual_correction[1]);
    const double rotation = fr10_force::norm({input.actual_correction[3], input.actual_correction[4],
                                             input.actual_correction[5]});
    if (lateral > config_.search_radius_m + config_.anchor_position_tolerance_m ||
        rotation > config_.max_rotation_rad + config_.anchor_rotation_tolerance_rad) {
      proposal.valid = false;
      return proposal;
    }
    next.mode = final_axis ? ContactReferenceMode::ACQUIRED : ContactReferenceMode::CAPTURED;
    next.equilibrium = final_axis ? input.actual_correction : fr10_force::Vector6{
        input.actual_correction[0], input.actual_correction[1], 0., 0., 0., 0.};
    // Bounded estimate from a qualified observation; command constraints are
    // unchanged. Larger disagreement faults rather than being projected away.
    if (lateral > config_.search_radius_m)
      for (std::size_t i=0; i<2; ++i) next.equilibrium[i] *= config_.search_radius_m/lateral;
    if (final_axis && rotation > config_.max_rotation_rad)
      for (std::size_t i=3; i<6; ++i) next.equilibrium[i] *= config_.max_rotation_rad/rotation;
    next.equilibrium[2] = 0.; // Axial force remains independent of the anchor.
    next.acquisition_depth_m = input.depth_m;
    if (!final_axis)
      next.capture_rotation_rad = {input.actual_correction[3], input.actual_correction[4],
                                   input.actual_correction[5]};
    next.acquisition_elapsed_s = 0.;
    if (state_.mode == ContactReferenceMode::PASSIVE)
      next.search_start_source_s = input.source_time_s;
    return proposal;
  }
  if (final_axis) return proposal;
  // A matched connector may travel through clearance without touching until
  // its shoulder seats. Do not require artificial contact inside that gap.
  // Depth here is measured advancement; final acceptance still requires load.
  if (!next.contact_observed) return proposal;
  if (input.phase == ContactReferencePhase::SEAT) return proposal;
  if (state_.mode == ContactReferenceMode::PASSIVE &&
      next.stalled_s >= config_.stall_time_s) {
    next.mode = ContactReferenceMode::SEARCH;
    next.search_start_source_s = input.source_time_s;
  }
  if (next.mode != ContactReferenceMode::SEARCH) return proposal;
  if (input.source_time_s - next.search_start_source_s >= config_.search_budget_s) {
    next.mode = ContactReferenceMode::EXHAUSTED;
    return proposal;
  }
  if (!config_.spiral_enabled) return proposal;
  // While measured insertion progresses, keep the same equilibrium; let the
  // chamfer/contact compliance align before asking for another lateral step.
  if (next.stalled_s < config_.stall_time_s) return proposal;
  const double growth = config_.spiral_pitch_m / fr10_motion::math::kTwoPi;
  const double radius = growth * state_.spiral_angle_rad;
  const auto length = [growth](double angle) {
      return 0.5 * growth * (angle * std::hypot(1., angle) + std::asinh(angle));
    };
  const double target_length = length(state_.spiral_angle_rad) + config_.search_speed_m_s * input.dt_s;
  double lower = state_.spiral_angle_rad;
  double upper = lower + config_.search_speed_m_s * input.dt_s / std::hypot(growth, radius);
  // Invert monotonically increasing spiral arclength with bounded work. The
  // lower bracket never exceeds requested path speed even at the origin.
  for (unsigned i = 0; i < 20; ++i) {
    const double middle = 0.5 * (lower + upper);
    if (length(middle) <= target_length) lower = middle;
    else upper = middle;
  }
  next.spiral_angle_rad = lower;
  const double next_radius = growth * next.spiral_angle_rad;
  if (next_radius > config_.search_radius_m) {
    next.mode = ContactReferenceMode::EXHAUSTED;
    return proposal;
  }
  next.equilibrium[0] = next_radius * std::cos(next.spiral_angle_rad);
  next.equilibrium[1] = next_radius * std::sin(next.spiral_angle_rad);
  return proposal;
}

bool ContactReference::commit(const ContactReferenceProposal &proposal) {
  if (!canCommit(proposal)) return false;
  bool changed=proposal.next.mode!=state_.mode || (proposal.yaw_probe.valid &&
      proposal.yaw_probe.next.mode!=yaw_probe_.state().mode);
  for(std::size_t i=0;i<2;++i) changed=changed || (proposal.lateral_probes[i].valid &&
      proposal.lateral_probes[i].next.mode!=lateral_probes_[i].state().mode);
  if(changed)clearAcquisitionWindow();
  if (proposal.yaw_probe.valid) yaw_probe_.commit(proposal.yaw_probe);
  for(std::size_t i=0;i<2;++i) if(proposal.lateral_probes[i].valid)lateral_probes_[i].commit(proposal.lateral_probes[i]);
  state_ = proposal.next;
  ++revision_;
  return true;
}

bool ContactReference::canCommit(const ContactReferenceProposal &proposal) const {
  for(std::size_t i=0;i<2;++i)
    if(proposal.lateral_probes[i].valid && !lateral_probes_[i].canCommit(proposal.lateral_probes[i]))return false;
  const bool acquiring = proposal.next.mode != state_.mode &&
      (proposal.next.mode == ContactReferenceMode::CAPTURED || proposal.next.mode == ContactReferenceMode::ACQUIRED);
  bool starting_probe = proposal.yaw_probe.valid && yaw_probe_.state().mode == KeyedYawProbeMode::IDLE &&
      proposal.yaw_probe.next.mode == KeyedYawProbeMode::SEEK_NEGATIVE;
  for(std::size_t i=0;i<2;++i)starting_probe=starting_probe || (proposal.lateral_probes[i].valid &&
      lateral_probes_[i].state().mode==ContactIntervalMode::IDLE &&
      proposal.lateral_probes[i].next.mode==ContactIntervalMode::SEEK_NEGATIVE);
  return proposal.valid && proposal.base_revision == revision_ &&
      (!(acquiring || starting_probe) || proposal.qualification_revision == qualification_revision_) &&
      (!proposal.yaw_probe.valid || yaw_probe_.canCommit(proposal.yaw_probe)) &&
      fr10_force::finite(proposal.next.equilibrium);
}

void ContactReference::reset() {
  state_ = {};
  yaw_probe_.reset();
  for(auto &probe:lateral_probes_)probe.reset();
  last_observation_source_s_ = -std::numeric_limits<double>::infinity();
  clearAcquisitionWindow();
  ++revision_;
}

ContactIntervalObservation ContactReference::lateralObservation(
    const ContactReferenceObservation &input,std::size_t axis) const {
  const auto transported=contactEquilibriumAtDepth(state_,input.depth_m,&input.actual_correction);
  const double shift=transported[axis]-state_.equilibrium[axis];
  const double other=input.actual_correction[1-axis];
  const double extent=std::min(config_.lateral_probe.max_coordinate,std::sqrt(std::max(0.,
      config_.search_radius_m*config_.search_radius_m-other*other)));
  const bool valid=input.valid && state_.mode==ContactReferenceMode::CAPTURED &&
      input.depth_m>=config_.acquisition_depth_m &&
      fr10_force::finite(input.actual_correction) && fr10_force::finite(input.measured_wrench) &&
      fr10_force::finite(input.nominal_wrench);
  // Probe one fixed mouth coordinate, removing the existing depth transport.
  // Changing tool tilt must not be mistaken for traversal of a bore interval.
  return {valid,input.actual_correction[axis]-shift,
      input.measured_wrench.force_n[axis]-input.nominal_wrench.force_n[axis],
      -extent-shift,extent-shift,input.source_time_s,
      input.measured_wrench.force_n[2]>=config_.contact_force_n};
}

fr10_force::Wrench ContactReference::desiredWrench(const ContactReferenceProposal &proposal,
    const ContactReferenceObservation &input,const fr10_force::Wrench &nominal) const {
  auto desired=nominal;
  if(!proposal.valid)return desired;
  if(proposal.yaw_probe.valid)desired.torque_nm[2]=yaw_probe_.desiredTorque(
      proposal.yaw_probe.next,input.actual_correction[5],input.measured_wrench.torque_nm[2],nominal.torque_nm[2]);
  for(std::size_t i=0;i<2;++i) if(proposal.lateral_probes[i].valid) {
    const auto observation=lateralObservation(input,i);
    desired.force_n[i]=lateral_probes_[i].desiredLoad(proposal.lateral_probes[i].next,
        observation.coordinate,input.measured_wrench.force_n[i],nominal.force_n[i]);
  }
  return desired;
}

} // namespace fr10_insertion
