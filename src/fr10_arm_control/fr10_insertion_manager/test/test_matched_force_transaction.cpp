#include <iostream>
#include "fr10_motion_core/math_utils.hpp"
#include "fr10_insertion_manager/force_session.hpp"

class EchoIK final : public fr10_motion::IKProvider {
public:
  fr10_motion::IKResult solve(const fr10_motion::Pose &pose,
      const fr10_motion::KinematicState &seed) override {
    last_ = pose; fr10_motion::IKResult result;
    result.success = true; result.state = seed;
    result.state.position_rad[0] = pose.position_m[2];
    result.manipulability = result.condition_number = 1.; return result;
  }
  fr10_motion::IKResult validate(const fr10_motion::KinematicState &target,
      const fr10_motion::KinematicState &) override {
    fr10_motion::IKResult result; result.success = true; result.state = target; return result;
  }
  bool forward(const fr10_motion::KinematicState &, fr10_motion::Pose &pose,
      std::string *) const override { pose = last_; return true; }
private:
  fr10_motion::Pose last_;
};

int main() {
  using namespace fr10_insertion;
  bool ok = true;
  auto check = [&](bool valid, const char *name) { if (!valid) { std::cerr << name << '\n'; ok = false; } };
  double previous_speed=0.;
  for(unsigned i=0;i<=2000;++i) {
    const double remaining=i*1e-6;
    const double speed=boundedSeatingSpeed(.0005,remaining,.0001,.0001,.002);
    check(speed>=.0001 && speed<=.0005 && speed>=previous_speed,
          "seating taper is positive, bounded and monotone with distance");
    previous_speed=speed;
  }
  check(boundedSeatingSpeed(.0005,0.,.0001,.0001,.002)==.0001 &&
        boundedSeatingSpeed(.0005,.01,.0001,.0001,.002)==.0005,
        "seating is slow while distant air approach keeps requested speed");
  check(boundedSeatingSpeed(.00005,0.,.0001,.0001,.002)==.00005,
        "seating taper cannot raise an already slow requested speed");
  const double window=2.*(.0001+.0005*.0005/(2.*.002));
  check(std::abs(boundedSeatingSpeed(.0005,1e-9,.0001,.0001,.002)-.0001)<1e-12 &&
        std::abs(boundedSeatingSpeed(.0005,window-1e-9,.0001,.0001,.002)-.0005)<1e-12,
        "taper joins have zero speed gradient without an abrupt contact step");
  check(!std::isfinite(boundedSeatingSpeed(.0005,0.,.0001,.0001,0.)),
        "invalid motion bounds cannot produce a seating speed");
  check(boundedSeatingSpeed(.0005,.000075,.0001+.0005,.00005,.002)<.00006,
        "accepted depth uncertainty and command lead slow before an early physical shoulder");
  ForceSessionConfig config;
  config.state_machine.process_mode = InsertionProcessMode::MATCHED_MATING;
  config.state_machine.preinsert_hold_s = .008;
  config.admittance.selected_axes.fill(true);
  ForceSession session(config); EchoIK provider;
  InsertionGoal goal; goal.selected_axes.fill(true); goal.preinsert_distance_m = .003;
  goal.insertion_depth_m = .015; goal.desired_axis_force_n = 5;
  fr10_force::CartesianPose final_pose; final_pose.position_m[2] = .015;
  fr10_force::Wrench desired; desired.force_n[2] = 5;
  const fr10_motion::Pose world_from_base{{.5,.1,1.17},
      fr10_motion::math::fixedXyzRpyToQuaternion({.0002,-.0001,3.141592653589793})};
  const fr10_motion::Pose base_goal{{.7,.05,.3},{0,0,0,1}};
  const auto world_goal=fr10_motion::math::composePose(world_from_base,base_goal);
  const auto recovered=fr10_motion::math::composePose(fr10_motion::math::inversePose(world_from_base),world_goal);
  check(fr10_motion::math::distance3D(recovered.position_m,base_goal.position_m)<1e-12 &&
      fr10_motion::math::quaternionAngularDistance(recovered.orientation_quat,base_goal.orientation_quat)<1e-8,
      "task anchor rigid transform round-trip retains position and orientation");
  auto moved_base=world_from_base;
  moved_base.orientation_quat=fr10_motion::math::fixedXyzRpyToQuaternion({.0004,-.0001,3.141592653589793});
  const auto adjusted=fr10_motion::math::composePose(fr10_motion::math::inversePose(moved_base),world_goal);
  check(fr10_motion::math::distance3D(adjusted.position_m,base_goal.position_m)>1e-5 &&
      fr10_motion::math::distance3D(fr10_motion::math::composePose(moved_base,adjusted).position_m,
                                  world_goal.position_m)<1e-12,
      "changing base attitude changes the arm goal while world socket stays immutable");
  ForceSession reframed(config);
  auto short_goal=goal;short_goal.timeout_s=.032;
  check(reframed.start(short_goal,final_pose,{0,0,1},desired,99,1,1.),"start bounded approach reframe");
  auto new_final=final_pose;new_final.position_m[0]=.0003;
  auto invalid_rotation=config.rotation_base_from_insertion;invalid_rotation[0]=2.;
  check(!reframed.reframePreinsert(new_final,{0,0,1},invalid_rotation,2),
        "invalid task rotation is rejected without throwing or mutating the session");
  check(reframed.reframePreinsert(new_final,{0,0,1},config.rotation_base_from_insertion,2),
      "stationary preinsert can be reframed before tare");
  ForceSessionInput boundary;
  boundary.dt_s=.008;boundary.source_time_s=boundary.now_steady_s=1.008;
  boundary.raw_sample.valid=true;boundary.raw_sample.sequence=1;
  boundary.raw_sample.source_timestamp_s=boundary.source_time_s;
  boundary.raw_sample.receipt_steady_s=boundary.now_steady_s;
  auto framed=reframed.update(boundary,provider);
  check(std::abs(framed.nominal_pose.position_m[0]-.0003)<1e-12,
      "preinsert nominal follows corrected goal without integrating a force step");
  check(reframed.reframePreinsert(final_pose,{0,0,1},config.rotation_base_from_insertion,3),"second reframe retains deadline");
  boundary.source_time_s=boundary.now_steady_s=1.04;boundary.raw_sample.sequence=2;
  boundary.raw_sample.source_timestamp_s=boundary.source_time_s;
  boundary.raw_sample.receipt_steady_s=boundary.now_steady_s;
  check(reframed.update(boundary,provider).decision.fault==InsertionFault::TIMEOUT,
      "reframe cannot reset the original session timeout");
  check(session.start(goal, final_pose, {0,0,1}, desired, 1, 1, 1.), "start matched session");
  ForceSessionInput input;
  input.dt_s = .008; input.source_time_s = input.now_steady_s = 1;
  input.actual_depth_m = -.003; input.preinsert_reached = input.tare_complete = true;
  input.raw_sample.valid = true;
  auto tick = [&] {
    input.source_time_s += .008; input.now_steady_s += .008;
    ++input.raw_sample.sequence;
    input.raw_sample.source_timestamp_s = input.source_time_s;
    input.raw_sample.receipt_steady_s = input.now_steady_s;
    return session.update(input, provider);
  };
  for (unsigned i=0; i<3; ++i) {
    const auto output = tick();
    if (output.online_target_valid) session.commitOnlineTarget(output.online_target.sequence, true);
  }
  auto first = tick();
  check(!session.reframePreinsert(final_pose,{0,0,1},config.rotation_base_from_insertion,3),
        "task frame is immutable once tare/contact sequence begins");
  check(first.online_target_valid, "air approach proposes online target");
  check(first.correction.position[2] == 0., "desired seat force is not integrated in air");
  session.commitOnlineTarget(first.online_target.sequence, false);
  auto repeated = tick();
  check(repeated.online_target_valid && repeated.nominal_pose.position_m[2] == first.nominal_pose.position_m[2],
        "failed publish rolls nominal trajectory back with admittance");
  session.commitOnlineTarget(repeated.online_target.sequence, true);
  auto next = tick();
  check(next.nominal_pose.position_m[2] > repeated.nominal_pose.position_m[2], "successful publish commits nominal advance");
  check(next.correction.position[2] == 0., "no desired load air plunge after commit");
  input.allow_motion_proposal = false;
  for (unsigned i=0; i<3; ++i) {
    const auto waiting = tick();
    check(!waiting.online_target_valid, "pending collision check does not replace unacknowledged proposal");
    check(waiting.safety.safe(), "safety continues while worker validates proposal");
  }
  check(session.commitOnlineTarget(next.online_target.sequence, true),
        "delayed worker acknowledgement still commits exactly the original proposal");
  input.allow_motion_proposal = true;
  const auto resumed = tick();
  check(resumed.nominal_pose.position_m[2]-next.nominal_pose.position_m[2] > goal.insertion_speed_m_s*.016,
        "motion integration accounts for elapsed worker wait without slowing commanded speed");
  session.commitOnlineTarget(resumed.online_target.sequence,true);
  for(unsigned i=0;i<30;++i) {
    const auto bounded=tick();
    check(bounded.nominal_pose.position_m[2] <= input.actual_depth_m+config.max_nominal_lead_m+1e-12,
          "blocked physical feedback bounds nominal advance before contact force grows");
    if(bounded.online_target_valid) session.commitOnlineTarget(bounded.online_target.sequence,true);
  }
  input.allow_motion_proposal = false;
  input.raw_sample.overload = true;
  check(tick().decision.request_controlled_stop,
        "sensor overload stops during pending worker validation");

  config.state_machine.completion_hold_s=.040;
  ForceSession seating(config);
  check(seating.start(goal,final_pose,{0,0,1},desired,2,1,input.source_time_s),"seating recovery session");
  input.raw_sample.overload=false;input.allow_motion_proposal=true;input.actual_depth_m=.015;
  input.conditioned_wrench.force_n[2]=input.raw_sample.wrench.force_n[2]=5.;
  auto seat_tick=[&] {
    input.source_time_s+=.008;input.now_steady_s+=.008;
    ++input.raw_sample.sequence;
    input.raw_sample.source_timestamp_s=input.source_time_s;
    input.raw_sample.receipt_steady_s=input.now_steady_s;
    auto out=seating.update(input,provider);
    if(out.online_target_valid) seating.commitOnlineTarget(out.online_target.sequence,true);
    return out;
  };
  for(unsigned i=0;i<50 && seating.stateMachine().state()!=InsertionState::VERIFY_INSERTION;++i) seat_tick();
  check(seating.stateMachine().state()==InsertionState::VERIFY_INSERTION,"seat contact reaches verification");
  input.conditioned_wrench.force_n[2]=input.raw_sample.wrench.force_n[2]=0.;
  ForceSessionOutput lost;
  for(unsigned i=0;i<15;++i) lost=seat_tick();
  check(lost.safety.safe() && !lost.decision.success &&
        lost.decision.state==InsertionState::SEATING,
        "intermittent unverified seat contact can reseat under all safety guards");
  input.conditioned_wrench.force_n[2]=input.raw_sample.wrench.force_n[2]=5.;
  input.conditioned_wrench.torque_nm[0]=input.raw_sample.wrench.torque_nm[0]=.13;
  for(unsigned i=0;i<50;++i) seat_tick();
  check(seating.stateMachine().state()==InsertionState::VERIFY_INSERTION,
        "full seating load with residual moment cannot freeze correction into HOLD");
  input.conditioned_wrench.torque_nm[0]=input.raw_sample.wrench.torque_nm[0]=0.;
  for(unsigned i=0;i<50 && seating.stateMachine().state()!=InsertionState::SUCCESS;++i) seat_tick();
  input.stop_complete=true;
  const auto held=seat_tick();
  check(held.decision.success && held.decision.state==InsertionState::HOLD,"reseated load verifies stationary HOLD");
  input.conditioned_wrench.force_n[2]=input.raw_sample.wrench.force_n[2]=0.;input.stop_complete=false;
  for(unsigned i=0;i<15 && seating.stateMachine().state()!=InsertionState::STOPPING;++i) lost=seat_tick();
  check(lost.decision.request_controlled_stop && lost.safety.fault==fr10_force::ForceSafetyFault::LOST_CONTACT,
        "accepted seated HOLD still faults if its contact load disappears");
  ForceSession retracting(config);
  input.retract_requested=false;input.stop_complete=false;
  input.conditioned_wrench.force_n[2]=input.raw_sample.wrench.force_n[2]=5.;
  check(retracting.start(goal,final_pose,{0,0,1},desired,3,1,input.source_time_s),"retract session start");
  auto retract_tick=[&] {
    input.source_time_s+=.008;input.now_steady_s+=.008;++input.raw_sample.sequence;
    input.raw_sample.source_timestamp_s=input.source_time_s;input.raw_sample.receipt_steady_s=input.now_steady_s;
    auto out=retracting.update(input,provider);
    if(out.online_target_valid)retracting.commitOnlineTarget(out.online_target.sequence,true);
    return out;
  };
  for(unsigned i=0;i<100 && retracting.stateMachine().state()!=InsertionState::SUCCESS;++i)retract_tick();
  input.stop_complete=true;retract_tick();input.stop_complete=false;
  input.retract_requested=true;
  check(retract_tick().decision.state==InsertionState::RETRACT,"explicit retract leaves seated HOLD");
  input.retract_requested=false;input.raw_sample.overload=true;
  const auto retract_fault=retract_tick();
  check(retract_fault.decision.request_controlled_stop &&
        retract_fault.safety.fault==fr10_force::ForceSafetyFault::SENSOR_OVERLOAD,
        "raw safety monitor remains active during explicit retract");
  ForceSession loaded_goal(config);
  desired.torque_nm[0]=.15;
  input={};input.dt_s=.008;input.source_time_s=input.now_steady_s=10.;
  input.actual_depth_m=.015;input.preinsert_reached=input.tare_complete=true;
  input.raw_sample.valid=true;input.conditioned_wrench=desired;
  input.raw_sample.wrench=desired;
  check(loaded_goal.start(goal,final_pose,{0,0,1},desired,4,1,10.),
        "nonzero nominal torque session start");
  for(unsigned i=0;i<100 && loaded_goal.stateMachine().state()!=InsertionState::SUCCESS;++i) {
    input.source_time_s+=.008;input.now_steady_s+=.008;++input.raw_sample.sequence;
    input.raw_sample.source_timestamp_s=input.source_time_s;
    input.raw_sample.receipt_steady_s=input.now_steady_s;
    const auto out=loaded_goal.update(input,provider);
    if(out.online_target_valid)loaded_goal.commitOnlineTarget(out.online_target.sequence,true);
  }
  check(loaded_goal.stateMachine().state()==InsertionState::SUCCESS,
        "ForceSession completion compares nominal moment error instead of raw moment");
  return ok ? 0 : 1;
}
