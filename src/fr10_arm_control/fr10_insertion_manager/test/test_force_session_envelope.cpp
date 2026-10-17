#include <iostream>
#include <stdexcept>
#include <string>
#include "fr10_insertion_manager/force_session.hpp"

class TranslationIK : public fr10_motion::IKProvider {
 public:
  fr10_motion::IKResult solve(const fr10_motion::Pose &pose,
      const fr10_motion::KinematicState &seed) override {
    fr10_motion::IKResult result;result.success=true;result.state=seed;
    for(unsigned i=0;i<3;++i) result.state.position_rad[i]=pose.position_m[i];
    result.condition_number=result.manipulability=1.;return result;
  }
  fr10_motion::IKResult validate(const fr10_motion::KinematicState &target,
      const fr10_motion::KinematicState &) override {
    fr10_motion::IKResult result;result.success=true;result.state=target;return result;
  }
  bool forward(const fr10_motion::KinematicState &state,fr10_motion::Pose &pose,
      std::string *) const override {
    pose={};for(unsigned i=0;i<3;++i) pose.position_m[i]=state.position_rad[i];return true;
  }
};

int main() {
  using namespace fr10_insertion;
  ForceSessionConfig config;
  config.state_machine.preinsert_hold_s=.008;
  config.admittance.selected_axes[0]=true;
  config.admittance.max_velocity.fill(.002);
  config.admittance.max_acceleration.fill(.01);
  // Leave the default config axis Z deliberately; the accepted goal axis X
  // must be the single authority for both admittance and safety.
  config.safety.max_lateral_correction_norm_m=.00001;
  ForceSession session(config);TranslationIK provider;
  InsertionGoal goal;goal.insertion_axis=0;goal.selected_axes=config.admittance.selected_axes;
  goal.preinsert_distance_m=.001;goal.insertion_depth_m=.015;goal.desired_axis_force_n=5.;
  fr10_force::CartesianPose final;final.position_m[0]=.015;
  fr10_force::Wrench desired;desired.force_n[0]=5.;
  if(!session.start(goal,final,{1.,0.,0.},desired,1,1,1.)) return 1;
  ForceSessionInput input;input.dt_s=.008;
  input.preinsert_reached=input.tare_complete=true;
  input.raw_sample.valid=true;input.raw_sample.wrench.force_n[0]=8.;
  input.conditioned_wrench=input.raw_sample.wrench;
  double largest=0.;bool ok=true;
  for(unsigned tick=0;tick<80;++tick) {
    input.source_time_s=input.now_steady_s=1.+.008*(tick+1);
    input.raw_sample.source_timestamp_s=input.raw_sample.receipt_steady_s=input.source_time_s;
    input.raw_sample.sequence=tick+1;
    input.actual_depth_m=0.;
    const auto output=session.update(input,provider);
    largest=std::max(largest,std::abs(output.correction.position[0]));
    ok &= output.safety.safe() && output.decision.state!=InsertionState::FAILED;
    if(output.online_target_valid) {
      ok &= session.canCommitOnlineTarget(output.online_target.sequence);
      ok &= session.commitOnlineTarget(output.online_target.sequence,true);
      input.joint_seed.position_rad=output.online_target.position_rad;
      input.joint_seed.velocity_rad_s=output.online_target.target_velocity_rad_s;
      input.joint_seed.acceleration_rad_s2=output.online_target.target_acceleration_rad_s2;
    }
  }
  ok &= largest>.0001;
  if(!ok) std::cerr<<"Goal insertion axis must govern both correction envelopes; axial="<<largest<<'\n';
  // A six-axis quiet drive region larger than the completion objective can
  // leave a physically fitted connector forever outside VERIFY's wrench
  // gate. Reject that incompatible configuration before actuator admission.
  ForceSessionConfig matched;
  matched.state_machine.process_mode=InsertionProcessMode::MATCHED_MATING;
  matched.admittance.selected_axes.fill(true);
  const auto rejected=[&](const ForceSessionConfig &candidate) {
    try { ForceSession invalid(candidate);return false; }
    catch(const std::invalid_argument &error) {
      return std::string(error.what()).find("deadband")!=std::string::npos;
    }
  };
  matched.admittance.drive_deadband={.5,.5,.5,.15,.15,.15};
  ok &= rejected(matched);
  matched.admittance.drive_deadband={.8,.8,.5,.03,.03,.03};
  ok &= rejected(matched); // Individual XY values fit; their norm does not.
  matched.admittance.drive_deadband={.5,.5,.7,.03,.03,.03};
  matched.state_machine.pre_stop_force_error_tolerance_n=.6;
  ok &= rejected(matched);
  matched.admittance.drive_deadband[2]=.5;
  try { ForceSession valid(matched); }
  catch(const std::invalid_argument &) {ok=false;}
  if(!ok)std::cerr<<"Matched deadband must fit the full-wrench completion and pre-stop objective\n";
  return ok?0:1;
}
