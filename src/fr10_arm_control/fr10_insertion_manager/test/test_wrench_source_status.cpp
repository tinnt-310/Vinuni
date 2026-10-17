#include <chrono>
#include <iostream>
#include <thread>
#include "fr10_insertion_manager/insertion_ros_adapter.hpp"

int main(int argc, char **argv) {
  using namespace std::chrono_literals;
  using namespace fr10_insertion;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("wrench_source_status_regression");
  ForceSessionConfig config;
  WrenchAdapterConfig wrench;
  wrench.require_source_status = true;
  InsertionRosAdapter adapter(node.get(), config, "base", wrench, "test", true, [] { return true; });
  auto raw = node->create_publisher<geometry_msgs::msg::WrenchStamped>(wrench.wrench_topic, rclcpp::SensorDataQoS());
  auto source = node->create_publisher<fr10_motion_interfaces::msg::WrenchSourceStatus>(wrench.source_status_topic, 1);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while ((raw->get_subscription_count() == 0 || source->get_subscription_count() == 0) &&
         std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(); std::this_thread::sleep_for(2ms);
  }
  bool ok = true;
  auto check = [&](bool value, const char *name) {
    if (!value) { ok = false; std::cerr << "FAIL " << name << '\n'; }
  };
  auto sendRaw = [&] {
    geometry_msgs::msg::WrenchStamped msg;
    msg.header.frame_id = wrench.source_frame; msg.header.stamp = node->now();
    raw->publish(msg); std::this_thread::sleep_for(3ms); executor.spin_some();
  };
  fr10_motion_interfaces::msg::WrenchSourceStatus status;
  auto sendStatus = [&] {
    source->publish(status); std::this_thread::sleep_for(3ms); executor.spin_some();
  };
  sendRaw();
  check(!adapter.freshUniqueWrench(), "RAW_WITHOUT_REQUIRED_STATUS_CANNOT_ADMIT");
  status.sequence = 1; status.online = true; sendStatus(); sendRaw();
  check(adapter.freshUniqueWrench(), "UNIQUE_ONLINE_STATUS_ADMITS_FRESH_RAW");
  status.online = false; sendStatus(); sendRaw();
  check(!adapter.freshUniqueWrench(), "OFFLINE_DUPLICATE_FAILS_IMMEDIATELY");
  status.online = true; sendStatus(); sendRaw();
  check(!adapter.freshUniqueWrench(), "ONLINE_DUPLICATE_CANNOT_REENABLE");
  status.sequence = 2; sendStatus(); sendRaw();
  check(adapter.freshUniqueWrench(), "NEW_ONLINE_STATUS_RECOVERS_SOURCE_BEFORE_SESSION");
  std::this_thread::sleep_for(60ms); sendRaw();
  check(!adapter.freshUniqueWrench(), "NEW_RAW_CANNOT_REFRESH_STALE_STATUS");
  status.sequence = 3; sendStatus(); sendRaw();
  check(adapter.freshUniqueWrench(), "UNIQUE_STATUS_AND_RAW_RECOVER");
  status.overload = true; sendStatus(); sendRaw();
  check(!adapter.freshUniqueWrench(), "DUPLICATE_OVERLOAD_CANNOT_ADMIT");
  executor.cancel(); rclcpp::shutdown();
  return ok ? 0 : 1;
}
