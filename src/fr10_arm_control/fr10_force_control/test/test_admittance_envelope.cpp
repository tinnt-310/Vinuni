#include <cmath>
#include <iostream>
#include <stdexcept>
#include "fr10_force_control/admittance_model.hpp"

int main() {
  using namespace fr10_force;
  bool ok = true;
  auto check = [&](bool value, const char *name) {
    if (!value) { std::cerr << name << '\n'; ok = false; }
  };
  AdmittanceConfig cfg;
  cfg.selected_axes.fill(true);
  cfg.max_correction = {.0025,.0025,.002,.035,.035,.035};
  cfg.max_lateral_correction_norm_m = .0025;
  cfg.max_rotation_correction_norm_rad = .035;
  cfg.max_velocity = {.0005,.0005,.000025,.002,.002,.002};
  cfg.max_acceleration = {.002,.001,.002,.02,.01,.015};
  AdmittanceModel model(cfg);
  unsigned constrained = 0;
  for (unsigned tick=0; tick<16000 && ok; ++tick) {
    const double dt = tick%3==0 ? .024 : tick%3==1 ? .008 : .016;
    const double sign = (tick/1800)%2==0 ? 1. : -1.;
    Vector6 drive{100.*sign,80.*sign,1.,4.*sign,3.*sign,2.*sign};
    const auto old = model.state();
    const auto p = model.propose(drive,dt);
    check(p.valid && model.canCommit(p), "VECTOR_BRAKING_ALWAYS_ADMISSIBLE");
    if (!p.valid) {
      std::cerr << "INVALID tick=" << tick << " dt=" << dt << "\n";
      for(unsigned i=0;i<6;++i) std::cerr<<i<<" oldq="<<old.correction[i]<<" oldv="<<old.velocity[i]<<" q="<<p.next_state.correction[i]<<" v="<<p.next_state.velocity[i]<<"\n";
      break;
    }
    double lateral=0., rotation=0.;
    for (unsigned i=0; i<6; ++i) {
      check(std::abs((p.output.position[i]-old.correction[i])/dt-p.output.velocity[i])<1e-12,
          "NO_POSITION_PROJECTION_OR_HIDDEN_VELOCITY_RESET");
      check(std::abs((p.output.velocity[i]-old.velocity[i])/dt-p.output.acceleration[i])<1e-12 &&
          std::abs(p.output.acceleration[i])<=cfg.max_acceleration[i]+1e-12,
          "ACTUAL_ACCELERATION_WITH_ANISOTROPIC_BRAKING");
      if (i<2) lateral=std::hypot(lateral,p.output.position[i]);
      if (i>=3) rotation=std::hypot(rotation,p.output.position[i]);
      constrained += p.position_saturated[i];
    }
    check(lateral<=cfg.max_lateral_correction_norm_m && rotation<=cfg.max_rotation_correction_norm_rad,
        "COUPLED_ENVELOPES_WITHOUT_EPSILON_WAIVER");
    check(model.commit(p), "TRANSACTION_COMMIT");
  }
  check(constrained>100, "EXERCISE_CORNER_AND_REVERSAL_LIMITS");
  AdmittanceModel tangent(cfg);
  AdmittanceState boundary;boundary.correction[1]=cfg.max_lateral_correction_norm_m;
  tangent.reset(boundary);
  const auto tangent_step=tangent.propose({1.,100.,0.,0.,0.,0.},.008);
  check(tangent_step.valid && tangent_step.output.velocity[0]>1e-6 &&
      tangent_step.output.velocity[1]<0. && tangent.canCommit(tangent_step),
      "BOUNDARY_TANGENT_REMAINS_AVAILABLE_WITH_RESERVED_CURVED_BRAKING");
  AdmittanceState outside;outside.correction[0]=.002;outside.correction[1]=.002;
  bool rejected=false;try { model.reset(outside); } catch(const std::invalid_argument &) { rejected=true; }
  check(rejected,"REJECT_INDIVIDUALLY_VALID_BUT_RADIAL_INVALID_HANDOVER");
  cfg.max_lateral_correction_norm_m=INFINITY;cfg.max_rotation_correction_norm_rad=INFINITY;
  AdmittanceModel legacy(cfg);legacy.reset(outside);
  check(legacy.propose({},.008).valid,"LEGACY_COMPONENT_ENVELOPE_UNCHANGED");
  cfg.max_lateral_correction_norm_m=.0025;cfg.insertion_axis=0;
  AdmittanceModel alternate(cfg);outside={};outside.correction[0]=.0024;outside.correction[1]=.002;
  alternate.reset(outside);
  check(alternate.propose({},.008).valid,"INSERTION_AXIS_EXCLUDED_FROM_LATERAL_NORM");
  return ok?0:1;
}
