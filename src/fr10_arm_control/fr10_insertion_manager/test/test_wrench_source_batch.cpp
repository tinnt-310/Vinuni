#include <chrono>
#include <iostream>
#include <limits>
#include <thread>
#include "fr10_insertion_manager/insertion_ros_adapter.hpp"

class BatchEchoIK : public fr10_motion::IKProvider {
public:
  fr10_motion::IKResult solve(const fr10_motion::Pose &,const fr10_motion::KinematicState &seed) override {
    fr10_motion::IKResult result;result.success=true;result.state=seed;return result;
  }
  fr10_motion::IKResult validate(const fr10_motion::KinematicState &target,
      const fr10_motion::KinematicState &) override {
    fr10_motion::IKResult result;result.success=true;result.state=target;return result;
  }
  bool forward(const fr10_motion::KinematicState &,fr10_motion::Pose &pose,
      std::string *) const override {pose={};return true;}
};

int main(int argc,char **argv) {
  using namespace fr10_insertion;using namespace std::chrono_literals;
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("wrench_source_batch_regression");
  ForceSessionConfig config;
  config.state_machine.preinsert_hold_s=.008;
  WrenchAdapterConfig wrench_config;wrench_config.filter.cutoff_hz=1.;
  InsertionRosAdapter adapter(node.get(),config,"fr10_base_link",wrench_config,"test",true,[] {return true;});
  auto pub=node->create_publisher<geometry_msgs::msg::WrenchStamped>("/ft_sensor/wrench",rclcpp::SensorDataQoS());
  double filtered=0.;unsigned reports=0;
  auto sub=node->create_subscription<geometry_msgs::msg::WrenchStamped>("/fr10_force/filtered_wrench",
      rclcpp::SensorDataQoS(),[&](geometry_msgs::msg::WrenchStamped::ConstSharedPtr value) {
        filtered=value->wrench.force.z;++reports;
      });
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
  const auto discovery=std::chrono::steady_clock::now()+2s;
  while(pub->get_subscription_count()==0 && std::chrono::steady_clock::now()<discovery) {
    executor.spin_some();std::this_thread::sleep_for(2ms);
  }
  InsertionRosAdapter::Insert::Goal goal;
  goal.target_pose.header.frame_id="fr10_base_link";goal.target_pose.header.stamp=node->now();
  goal.target_pose.pose.position.z=.015;goal.target_pose.pose.orientation.w=1.;
  goal.preinsert_distance=.001;goal.insertion_depth=.015;goal.insertion_speed=.0005;
  goal.desired_wrench.force.z=5.;goal.selected_axes={{false,false,true,false,false,false}};
  goal.max_force={{50.,50.,50.}};goal.max_torque={{10.,10.,10.}};
  goal.timeout.sec=10;goal.profile_id="test";goal.completion_behavior=InsertionRosAdapter::Insert::Goal::HOLD;
  bool ok=adapter.submitPoseGoal(goal);
  const auto pending=adapter.takePendingGoal();ok &= pending.has_value();
  if(pending) ok &= adapter.startSession(*pending,{0,0,1},1,1,node->now().seconds());
  InsertionRosAdapter::RuntimeInput input;
  input.measured_joint_velocity_rad_s = fr10_force::Vector6{};
  input.actual_pose_valid=true;input.preinsert_reached=true;input.current_scene_revision=1;
  input.actual_pose=adapter.preinsertPose();input.allow_motion_proposal=false;
  BatchEchoIK provider;
  auto send=[&](double force) {
    geometry_msgs::msg::WrenchStamped message;
    message.header.frame_id="charging_tcp";message.header.stamp=node->now();message.wrench.force.z=force;
    pub->publish(message);std::this_thread::sleep_for(2ms);executor.spin_some();
  };
  // Qualify 25 control observations using symmetric unfiltered FT bursts.
  // Latest-only tare would mistake the final +0.2 N phase for DC bias.
  for(unsigned tick=0;tick<32;++tick) {
    send(-.2);send(.2);adapter.update(input,provider);executor.spin_some();std::this_thread::sleep_for(8ms);
  }
  for(unsigned tick=0;tick<80;++tick) {
    send(0.);adapter.update(input,provider);executor.spin_some();std::this_thread::sleep_for(8ms);
  }
  ok &= std::abs(filtered)<.02;
  if(!ok) std::cerr<<"Symmetric burst tare must not introduce an aliased bias; force="<<filtered<<'\n';
  // Sensor bursts carry a 50% duty load; the last sample is always zero.
  // A latest-only 125 Hz consumer aliases the entire physical load to zero.
  for(unsigned tick=0;tick<48;++tick) {
    for(unsigned sample=0;sample<8;++sample) send(sample<4?10.:0.);
    adapter.update(input,provider);executor.spin_some();
  }
  ok &= reports>40 && filtered>3. && filtered<7.;
  if(!ok) std::cerr<<"All-source anti-alias conditioning required; reports="<<reports<<" force="<<filtered<<'\n';
  // An invalid packet inside a burst remains a safety fault even if the last
  // sample is healthy and the conditioned channel is finite.
  send(std::numeric_limits<double>::quiet_NaN());send(0.);
  input.stop_complete=true;
  const auto output=adapter.update(input,provider);
  ok &= output.decision.state==InsertionState::FAILED &&
        output.decision.safety_fault==fr10_force::ForceSafetyFault::INVALID_WRENCH;
  if(!ok) std::cerr<<"Invalid raw burst must remain observable to safety\n";
  executor.cancel();rclcpp::shutdown();return ok?0:1;
}
