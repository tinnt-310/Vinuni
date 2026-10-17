#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>
#include "fr10_insertion_manager/parking_ros_adapter.hpp"

namespace {
double steadyNow() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}

int main(int argc,char **argv) {
  using namespace std::chrono_literals;
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("parking_transport_regression");
  fr10_insertion::ParkingRosAdapter adapter(node.get(),"/test_physical_base","world",true);
  auto publisher=node->create_publisher<nav_msgs::msg::Odometry>("/test_physical_base",rclcpp::SensorDataQoS());
  auto status_pub=node->create_publisher<fr10_motion_interfaces::msg::VehicleParkingState>("/vehicle/parking_state",10);
  std::atomic<uint64_t> owner{0},lease_sequence{0};
  auto lease_sub=node->create_subscription<fr10_motion_interfaces::msg::VehicleParkingLease>(
    "/vehicle/parking_lease",10,[&](fr10_motion_interfaces::msg::VehicleParkingLease::ConstSharedPtr message) {
      if(!message->release) {lease_sequence.store(message->sequence);owner.store(message->owner_id);}
    });
  adapter.acquire();
  std::atomic<bool> publish{true},block_default{false};
  std::atomic<unsigned> blocked_callbacks{0};
  // Saturate all three ordinary executor workers with unrelated callbacks.
  // The owned feedback executor must remain independent of that saturation.
  auto slow=node->create_wall_timer(1ms,[&] {
    if(block_default.load()) {++blocked_callbacks;std::this_thread::sleep_for(80ms);}
  });
  std::array<rclcpp::CallbackGroup::SharedPtr,2> groups;
  std::array<rclcpp::TimerBase::SharedPtr,2> blockers;
  std::array<std::atomic<unsigned>,2> extra_blocked{};
  for(auto &value:extra_blocked) value.store(0);
  for(unsigned i=0;i<2;++i) {
    groups[i]=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    blockers[i]=node->create_wall_timer(1ms,[&,i] {
      if(block_default.load()) {++extra_blocked[i];std::this_thread::sleep_for(80ms);}
    },groups[i]);
  }
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),3);
  executor.add_node(node);
  std::thread spin([&] {executor.spin();});
  std::thread source([&] {
    while(publish.load()) {
      nav_msgs::msg::Odometry message;
      const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
      message.header.stamp=rclcpp::Time(ns);
      message.header.frame_id="world";message.child_frame_id="base_footprint";
      message.pose.pose.orientation.w=1.;
      publisher->publish(message);
      if(owner.load()!=0) {
        fr10_motion_interfaces::msg::VehicleParkingState status;
        status.owner_id=owner.load();status.sequence=lease_sequence.load();status.state=2;
        status_pub->publish(status);
      }
      std::this_thread::sleep_for(2ms);
    }
  });
  const double deadline=steadyNow()+5.;
  while(!adapter.freshFeedback() && steadyNow()<deadline) std::this_thread::sleep_for(2ms);
  while(!adapter.update().ready && steadyNow()<deadline) std::this_thread::sleep_for(2ms);
  bool ok=adapter.freshFeedback() && adapter.update().ready;
  block_default.store(true);
  const double end=steadyNow()+.35;
  while(steadyNow()<end) {
    ok &= adapter.freshFeedback() && adapter.update().ready;
    std::this_thread::sleep_for(2ms);
  }
  ok &= blocked_callbacks.load()>=2 && extra_blocked[0].load()>=2 && extra_blocked[1].load()>=2;
  publish.store(false);source.join();
  std::this_thread::sleep_for(100ms);
  ok &= !adapter.freshFeedback() && !adapter.update().ready; // Dedicated group cannot renew a stopped source.
  block_default.store(false);executor.cancel();spin.join();rclcpp::shutdown();
  if(!ok) std::cerr<<"Parking feedback isolation/freshness regression failed\n";
  return ok?0:1;
}
