#include <cmath>
#include <iostream>
#include <limits>
#include "fr10_insertion_manager/contact_reference.hpp"

int main() {
  using namespace fr10_insertion;
  bool ok=true;
  const auto check=[&](bool value,const char *name) {
    if(!value){std::cerr<<"FAIL "<<name<<'\n';ok=false;}
  };
  ContactIntervalConfig config{1.5,1.,1000.,.05,.25,.000025,.0025,.05,.00025};
  for(double center:{-.005,-.002,-.001,0.,.001,.002,.005}) {
    auto envelope = config;
    if (std::abs(center) > .0025) envelope.max_coordinate = .0075;
    ContactIntervalProbe probe(envelope);
    ContactIntervalObservation input{true,0.,0.,-envelope.max_coordinate,envelope.max_coordinate,1.};
    const auto sample=[&]{input.source_time_s+=.008;probe.observe(input);return probe.propose(input,false);};
    sample();check(probe.commit(probe.propose(input,true)),"START_REQUIRES_ADMISSION");
    input.coordinate=center-.00025;input.reaction=-1.2;
    ContactIntervalProposal lower;
    for(int i=0;i<9;++i)lower=sample();
    check(probe.state().mode==ContactIntervalMode::SEEK_NEGATIVE,"PENDING_ENDPOINT_HAS_NO_SIDE_EFFECT");
    input.reaction=0.;sample();
    check(!probe.canCommit(lower),"CONTACT_LOSS_REVOKES_PENDING_ENDPOINT");
    input.reaction=-1.2;for(int i=0;i<9;++i)lower=sample();
    check(probe.commit(lower),"NEGATIVE_ENDPOINT_ADMITTED");
    input.coordinate=center+.00025;input.reaction=1.2;
    ContactIntervalProposal upper;
    for(int i=0;i<9;++i)upper=sample();
    check(upper.next.mode==ContactIntervalMode::CENTERING &&
        std::abs(upper.next.center-center)<1e-15,"BIDIRECTIONAL_WALLS_IDENTIFY_TRANSLATION_CENTER");
    check(probe.commit(upper),"CENTER_TARGET_ADMITTED");
    input.coordinate=center;input.reaction=.07;
    check(std::abs(probe.desiredLoad(probe.state(),center,.07,.07)-.07)<1e-15,
        "CENTERING_PRESERVES_MEASURED_REACTION_WITH_ZERO_POSITION_ERROR");
    ContactIntervalProposal complete;
    for(int i=0;i<34;++i)complete=sample();
    check(complete.next.mode==ContactIntervalMode::COMPLETE,"CENTER_REQUIRES_SOURCE_DWELL");
    input.source_time_s+=.051;probe.observe(input);
    check(!probe.commit(complete),"SOURCE_GAP_REVOKES_UNPUBLISHED_COMPLETION");
    for(int i=0;i<34;++i)complete=sample();
    check(probe.commit(complete) && probe.desiredLoad(probe.state(),center,0.,.07)==.07,
        "COMPLETE_RESTORES_NOMINAL_LOAD");
    auto forged=probe.propose(input,false);forged.next.mode=static_cast<ContactIntervalMode>(99);
    check(!probe.canCommit(forged),"INVALID_MODE_CANNOT_ENTER_COMMITTED_STATE");
  }
  // Actuator envelope is not a wall. A known mechanical half-clearance lets
  // the opposite measured wall identify a center at a truncated endpoint.
  ContactIntervalProbe truncated(config);
  ContactIntervalObservation input{true,0.,0.,-.0018,.0025,1.};
  const auto sample=[&]{input.source_time_s+=.008;truncated.observe(input);return truncated.propose(input,false);};
  sample();check(truncated.commit(truncated.propose(input,true)),"TRUNCATED_START");
  input.coordinate=-.0018;
  ContactIntervalProposal lower;
  for(int i=0;i<9;++i)lower=sample();
  check(truncated.commit(lower) && lower.next.negative_contact_strength==0.,"QUIET_ENVELOPE_IS_NOT_CONTACT");
  input.coordinate=-.00145;input.reaction=1.2;
  ContactIntervalProposal upper;
  for(int i=0;i<9;++i)upper=sample();
  check(std::abs(upper.next.center+.0017)<1e-15,"KNOWN_CLEARANCE_RECOVERS_ONE_SIDED_CENTER");
  input.coordinate=std::numeric_limits<double>::quiet_NaN();sample();
  check(!truncated.canCommit(upper) && !truncated.propose(input,false).valid,"NONFINITE_OBSERVATION_FAILS_CLOSED");

  // Other compliant axes can consume the radial command envelope while this
  // scan is pending. Quiet envelope samples are coordinates, not ordered
  // physical walls; encoder uncertainty can reverse their apparent order.
  ContactIntervalProbe shrinking(config);
  ContactIntervalObservation narrow{true,.000005,0.,0.,0.,1.};
  const auto narrow_tick=[&]{narrow.source_time_s+=.008;shrinking.observe(narrow);return shrinking.propose(narrow,false);};
  narrow_tick();check(shrinking.commit(shrinking.propose(narrow,true)),"COLLAPSED_ENVELOPE_START");
  for(int i=0;i<9;++i)lower=narrow_tick();
  check(shrinking.commit(lower),"QUIET_LOWER_ENVELOPE_SAMPLE");
  narrow.coordinate=-.000005;
  for(int i=0;i<9;++i)upper=narrow_tick();
  check(upper.valid && upper.next.mode==ContactIntervalMode::CENTERING && upper.next.center==0. &&
      upper.next.negative_contact_strength==0. && upper.next.positive_contact_strength==0.,
      "QUIET_REVERSED_SAMPLES_STILL_PRODUCE_A_BOUNDED_ESTIMATE");

  // A completed yaw scan no longer owns the changing Rx/Ry feasibility set
  // during XY scans. Final corridor acquisition still qualifies that norm.
  ContactReferenceConfig completed_config;
  completed_config.search_radius_m=.0025;completed_config.acquisition_depth_m=.008;
  completed_config.acquisition_hold_s=.016;completed_config.lateral_probe=config;
  completed_config.keyed_yaw.drive_torque_nm=.15;
  ContactReference completed(completed_config);
  ContactReferenceObservation completed_input;
  completed_input.valid=true;completed_input.dt_s=.008;completed_input.source_time_s=1.;
  completed_input.phase=ContactReferencePhase::INSERT;completed_input.depth_m=.009;
  const auto completed_tick=[&]{completed_input.source_time_s+=.008;completed.observe(completed_input);return completed.propose(completed_input);};
  for(int i=0;i<5;++i)check(completed.commit(completed_tick()),"COMPLETED_SCAN_ENTRY_CAPTURE");
  completed_input.phase=ContactReferencePhase::SEAT;completed_input.depth_m=.015;
  completed_input.measured_wrench.force_n[2]=2.;
  for(int i=0;i<5;++i)check(completed.commit(completed_tick()),"COMPLETED_SCAN_YAW_START");
  completed_input.actual_correction[5]=-.007;completed_input.measured_wrench.torque_nm[2]=-.12;
  for(int i=0;i<9;++i)check(completed.commit(completed_tick()),"COMPLETED_SCAN_NEGATIVE_WALL");
  completed_input.actual_correction[5]=.007;completed_input.measured_wrench.torque_nm[2]=.12;
  for(int i=0;i<9;++i)check(completed.commit(completed_tick()),"COMPLETED_SCAN_POSITIVE_WALL");
  completed_input.actual_correction[5]=0.;completed_input.measured_wrench.torque_nm[2]=0.;
  for(int i=0;i<36;++i)check(completed.commit(completed_tick()),"COMPLETED_SCAN_CENTER_ADMISSION");
  check(completed.yawProbe().state().mode==ContactIntervalMode::COMPLETE &&
      completed.state().mode==ContactReferenceMode::CAPTURED,"XY_STILL_OWNS_UNFINISHED_ACQUISITION");
  completed_input.actual_correction[3]=completed_config.max_rotation_rad+.0001;
  auto changed_tilt=completed_tick();
  check(changed_tilt.valid && !changed_tilt.yaw_probe.valid &&
      changed_tilt.next.mode==ContactReferenceMode::CAPTURED,
      "COMPLETED_YAW_CANNOT_REJECT_LIVE_XY_OBSERVATIONS");
  completed_input.actual_correction[3]=std::numeric_limits<double>::quiet_NaN();
  check(!completed_tick().valid,"COMPLETED_PROBE_NEVER_BYPASSES_NONFINITE_SENSOR_GUARD");

  // Complete reference transaction, with nonzero nominal XY force and a
  // changing SO(3) tilt/depth transport. Only reaction relative to nominal is
  // wall evidence; only the fixed mouth coordinate is a translation sample.
  ContactReferenceConfig ref_config;
  ref_config.search_radius_m=.0025;ref_config.acquisition_depth_m=.008;
  ref_config.acquisition_hold_s=.032;ref_config.lateral_probe=config;
  ContactReference reference(ref_config);
  ContactReferenceObservation measured;
  measured.valid=true;measured.dt_s=.008;measured.source_time_s=1.;
  measured.phase=ContactReferencePhase::INSERT;measured.depth_m=.009;
  measured.actual_correction={-.0015,.001,0.,0.,0.,0.};
  const auto tick=[&]{measured.source_time_s+=.008;reference.observe(measured);return reference.propose(measured);};
  for(int i=0;i<10;++i)check(reference.commit(tick()),"BORE_CAPTURE_ADMISSION");
  check(reference.state().mode==ContactReferenceMode::CAPTURED,"ENTRY_IS_NOT_AXIS_ACQUISITION");
  measured.phase=ContactReferencePhase::SEAT;measured.depth_m=.015;
  measured.nominal_wrench.force_n={.07,-.03,2.};measured.measured_wrench=measured.nominal_wrench;
  ContactReferenceProposal pending;
  for(int i=0;i<8;++i)pending=tick();
  check(pending.lateral_probes[0].next.mode==ContactIntervalMode::SEEK_NEGATIVE &&
      pending.lateral_probes[1].next.mode==ContactIntervalMode::IDLE,"XY_PROBES_START_SEQUENTIALLY");
  measured.measured_wrench.torque_nm[0]=.2;tick();
  check(!reference.canCommit(pending) && reference.lateralProbe(0).state().mode==ContactIntervalMode::IDLE,
      "PENDING_XY_START_RECHECKS_LOADED_QUALIFICATION");
  measured.measured_wrench.torque_nm[0]=0.;
  const double centers[2]={-.0017,.0008};
  int ticks=0;
  for(;ticks<500 && reference.state().mode!=ContactReferenceMode::ACQUIRED;++ticks) {
    measured.actual_correction[3]=.01*std::sin(.02*ticks);
    measured.actual_correction[4]=.015*std::cos(.02*ticks);
    const auto transported=contactEquilibriumAtDepth(reference.state(),measured.depth_m,&measured.actual_correction);
    measured.measured_wrench=measured.nominal_wrench;
    for(std::size_t i=0;i<2;++i) {
      const auto &state=reference.lateralProbe(i).state();
      const double shift=transported[i]-reference.state().equilibrium[i];
      const auto mode=state.mode;
      const double mouth=mode==ContactIntervalMode::SEEK_NEGATIVE ? centers[i]-.00025 :
          mode==ContactIntervalMode::SEEK_POSITIVE ? centers[i]+.00025 :
          mode==ContactIntervalMode::CENTERING || mode==ContactIntervalMode::COMPLETE ? state.center : centers[i];
      measured.actual_correction[i]=mouth+shift;
      measured.measured_wrench.force_n[i]+=mode==ContactIntervalMode::SEEK_NEGATIVE ? -1.2 :
          mode==ContactIntervalMode::SEEK_POSITIVE ? 1.2 : 0.;
    }
    auto proposal=tick();
    check(proposal.valid,"SIX_AXIS_REFERENCE_PROPOSAL_VALID");
    const auto desired=reference.desiredWrench(proposal,measured,measured.nominal_wrench);
    if(proposal.lateral_probes[0].valid && proposal.lateral_probes[0].next.mode==ContactIntervalMode::SEEK_NEGATIVE)
      check(std::abs(desired.force_n[0]-(.07-1.5))<1e-15,"PROBE_LOAD_USES_EXISTING_DESIRED_WRENCH");
    check(reference.commit(proposal),"REFERENCE_AND_PROBES_COMMIT_TOGETHER");
    if(reference.lateralProbe(0).state().mode!=ContactIntervalMode::COMPLETE)
      check(reference.lateralProbe(1).state().mode==ContactIntervalMode::IDLE,"Y_CANNOT_RACE_X_OBSERVATION");
    if(reference.lateralProbe(1).state().mode!=ContactIntervalMode::COMPLETE)
      check(reference.state().mode==ContactReferenceMode::CAPTURED,"CLEARANCE_EDGE_CANNOT_ACQUIRE_AXIS");
  }
  check(ticks<500 && reference.state().mode==ContactReferenceMode::ACQUIRED,"XY_CENTERING_REQUIRES_FRESH_LOADED_CORRIDOR");
  for(std::size_t i=0;i<2;++i)check(std::abs(reference.lateralProbe(i).state().center-centers[i])<1e-14,
      "TILT_AND_NOMINAL_FT_ARE_NOT_MISTAKEN_FOR_LATERAL_WALLS");
  std::cout<<(ok ? "PASS contact interval/reference transactions\n" : "FAIL contact interval/reference transactions\n");
  return ok ? 0 : 1;
}
