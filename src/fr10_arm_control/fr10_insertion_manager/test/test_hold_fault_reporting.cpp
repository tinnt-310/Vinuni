#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>
#include <vector>
#include "fr10_insertion_manager/insertion_ros_adapter.hpp"

// A synthetic acknowledged plant; this test has no joint-command publisher.
class EchoIK final : public fr10_motion::IKProvider {
public:
  fr10_motion::IKResult solve(const fr10_motion::Pose &pose,
      const fr10_motion::KinematicState &seed) override {
    last_=pose;fr10_motion::IKResult result;result.success=true;result.state=seed;
    result.state.position_rad[0]=pose.position_m[2];
    result.manipulability=result.condition_number=1.;return result;
  }
  fr10_motion::IKResult validate(const fr10_motion::KinematicState &target,
      const fr10_motion::KinematicState &) override {
    fr10_motion::IKResult result;result.success=true;result.state=target;return result;
  }
  bool forward(const fr10_motion::KinematicState &,fr10_motion::Pose &pose,
      std::string *) const override {pose=last_;return true;}
private:
  fr10_motion::Pose last_;
};

int main(int argc,char **argv) {
  using namespace std::chrono_literals;
  using namespace fr10_insertion;
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("hold_fault_reporting_regression");
  ForceSessionConfig config;
  config.admittance.selected_axes={{false,false,true,false,false,false}};
  config.admittance.damping.fill(100000.);
  config.state_machine.preinsert_hold_s=.008;
  config.state_machine.completion_hold_s=.016;
  config.contact.confirm_time_s=.008;
  WrenchAdapterConfig wrench_config;wrench_config.filter.cutoff_hz=1000.;
  InsertionRosAdapter adapter(node.get(),config,"fr10_base_link",wrench_config,"test",true,[] {return true;});
  bool diagnostic_ok=adapter.diagnosticPhase()==nullptr;
  auto publisher=node->create_publisher<geometry_msgs::msg::WrenchStamped>("/ft_sensor/wrench",rclcpp::SensorDataQoS());
  std::vector<InsertionRosAdapter::Insert::Result> results;
  auto result_sub=node->create_subscription<InsertionRosAdapter::Insert::Result>("/fr10_force/result",
      rclcpp::QoS(10).reliable().transient_local(),
      [&](InsertionRosAdapter::Insert::Result::ConstSharedPtr message) {results.push_back(*message);});
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
  const auto discovery=std::chrono::steady_clock::now()+2s;
  while(publisher->get_subscription_count()==0 && std::chrono::steady_clock::now()<discovery) {
    executor.spin_some();std::this_thread::sleep_for(2ms);
  }
  InsertionRosAdapter::Insert::Goal goal;
  goal.target_pose.header.frame_id="fr10_base_link";goal.target_pose.header.stamp=node->now();
  goal.target_pose.pose.position.z=.001;goal.target_pose.pose.orientation.w=1.;
  // Tare requires independently qualified clearance, even in this synthetic
  // plant. A 0.1 mm offset cannot establish no contact within pose uncertainty.
  goal.preinsert_distance=.001;goal.insertion_depth=.001;goal.insertion_speed=.02;
  goal.desired_wrench.force.z=5.;goal.selected_axes=config.admittance.selected_axes;
  goal.max_force={{50.,50.,50.}};goal.max_torque={{10.,10.,10.}};
  goal.timeout.sec=5;goal.profile_id="test";goal.completion_behavior=InsertionRosAdapter::Insert::Goal::HOLD;
  bool ok=adapter.submitPoseGoal(goal);
  ok &= diagnostic_ok && adapter.diagnosticPhase() &&
        std::strcmp(adapter.diagnosticPhase(),"WAITING_ADMISSION")==0;
  ok &= !adapter.requiresFreshWrenchForResume(); // Admission has no active force motion yet.
  const auto pending=adapter.takePendingGoal();
  ok &= pending.has_value();
  if(pending) ok &= adapter.startSession(*pending,{0,0,1},1,1,node->now().seconds());
  ok &= !adapter.requiresFreshWrenchForResume(); // Geometric preinsert precedes force admission.
  InsertionRosAdapter::RuntimeInput input;
  input.measured_joint_velocity_rad_s = fr10_force::Vector6{};
  input.actual_pose_valid=true;input.preinsert_reached=true;input.current_scene_revision=1;
  input.actual_pose=adapter.preinsertPose();EchoIK provider;
  bool overload=false;
  auto tick=[&](bool publish_sample=true) {
    geometry_msgs::msg::WrenchStamped message;
    message.header.frame_id="charging_tcp";message.header.stamp=node->now();
    message.wrench.force.z=overload?1000.:(input.actual_pose.position_m[2]>=.0005?5.:0.);
    if (publish_sample) publisher->publish(message);
    const auto receive_deadline=std::chrono::steady_clock::now()+30ms;
    if (publish_sample) {
      do {executor.spin_some();std::this_thread::sleep_for(1ms);}
      while(!adapter.freshUniqueWrench() && std::chrono::steady_clock::now()<receive_deadline);
    }
    input.stop_complete=adapter.state()==InsertionState::SUCCESS || adapter.state()==InsertionState::HOLD ||
                        adapter.state()==InsertionState::STOPPING || adapter.state()==InsertionState::FAILED;
    const auto output=adapter.update(input,provider);
    if(output.online_target_valid) {
      adapter.commitOnlineTarget(output.online_target.sequence,true);
      input.actual_pose=output.corrected_pose;input.joint_seed=output.ik.state;
    }
    executor.spin_some();std::this_thread::sleep_for(8ms);executor.spin_some();
  };
  // A stopped command is not evidence that the physical arm is stationary.
  input.measured_joint_velocity_rad_s->fill(.02);
  for (unsigned i=0;i<30;++i) tick();
  ok &= adapter.state()==InsertionState::FT_ZERO && results.empty();
  ok &= adapter.diagnosticPhase() && std::strcmp(adapter.diagnosticPhase(),"FT_ZERO")==0;
  input.measured_joint_velocity_rad_s.reset();
  for (unsigned i=0;i<30;++i) tick();
  ok &= adapter.state()==InsertionState::FT_ZERO && results.empty();
  input.measured_joint_velocity_rad_s = fr10_force::Vector6{};
  for (unsigned i=0;i<20;++i) tick();
  input.measured_joint_velocity_rad_s->fill(.02);
  tick(false); // Encoder movement with no newly published FT packet.
  input.measured_joint_velocity_rad_s->fill(0.);
  for (unsigned i=0;i<6;++i) tick();
  ok &= adapter.state()==InsertionState::FT_ZERO && results.empty();
  const auto deadline=std::chrono::steady_clock::now()+4s;
  while(ok && adapter.state()!=InsertionState::HOLD && std::chrono::steady_clock::now()<deadline) tick();
  ok &= adapter.state()==InsertionState::HOLD && results.size()==1 && results[0].success;
  ok &= adapter.diagnosticPhase() && std::strcmp(adapter.diagnosticPhase(),"HOLD")==0;
  ok &= adapter.requiresFreshWrenchForResume(); // Healthy HOLD still monitors FT.
  for(unsigned i=0;i<4;++i) tick();
  ok &= results.size()==1; // A continuing healthy HOLD cannot duplicate success.
  overload=true;tick();
  ok &= adapter.state()==InsertionState::FAILED && adapter.busy();
  ok &= adapter.diagnosticPhase() && std::strcmp(adapter.diagnosticPhase(),"FAILED")==0;
  ok &= !adapter.requiresFreshWrenchForResume(); // Stale/faulted FT cannot kill joint HOLD.
  for(unsigned i=0;i<4;++i) tick();
  ok &= results.size()==2 && !results.back().success && results.back().controlled_stop_completed &&
        results.back().message.find("safety_fault=5;")!=std::string::npos;
  if(!ok) std::cerr<<"HOLD fault must emit exactly one new terminal failure; state="
                  <<static_cast<int>(adapter.state())<<" results="<<results.size()<<'\n';
  executor.cancel();rclcpp::shutdown();
  return ok?0:1;
}
