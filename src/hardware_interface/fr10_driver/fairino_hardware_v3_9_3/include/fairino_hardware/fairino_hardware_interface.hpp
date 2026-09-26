#ifndef _FR_HARDWARE_INTERFACE_
#define _FR_HARDWARE_INTERFACE_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "visibility_control.h"
#include <array>
#include <vector>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "libfairino/include/robot.h"


#define CONTROLLER_IP_ADDRESS "192.168.58.2"

namespace fairino_hardware
{

class FairinoHardwareInterface: public hardware_interface::SystemInterface{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FairinoHardwareInterface)

  // Some FAIRINO SDK releases leave their CNDE receive thread alive after CloseRPC().
  // Destroying FRRobot then causes a verified use-after-free during shutdown.
  // Keep one reusable instance for this process and let the OS reclaim it.
  ~FairinoHardwareInterface() override { (void)_ptr_robot.release(); }

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  //FAIRINO_HARDWARE_PUBLIC
  //hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;
  
  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  
  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  // hardware_interface::return_type prepare_command_mode_switch(
  //   const std::vector<std::string> & start_interfaces,
  //   const std::vector<std::string> & stop_interfaces) override;
  // hardware_interface::return_type perform_command_mode_switch(
  //   const std::vector<std::string>& start_interfaces,
  //   const std::vector<std::string>& stop_interfaces) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
private:
  hardware_interface::CallbackReturn stopAndClose(const char * context);
  void publishHealth(int read_error);
  rclcpp::Node::SharedPtr health_node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr health_pub_;
  std_msgs::msg::Float64MultiArray health_message_;
  ROBOT_STATE_PKG health_state_{};
  uint64_t health_sequence_{0};
  uint64_t read_errors_{0};
  uint64_t write_errors_{0};
  int last_write_error_{0};
  int last_frame_{-1};
  std::chrono::steady_clock::time_point last_frame_time_{};

  double _jnt_position_command[6];
  double _jnt_velocity_command[6];
  double _jnt_torque_command[6];
  double _jnt_position_state[6];
  double _jnt_velocity_state[6];
  double _jnt_torque_state[6];
  int _control_mode;
  std::string _controller_ip = CONTROLLER_IP_ADDRESS;
  std::unique_ptr<FRRobot> _ptr_robot;
  bool _servo_mode_started{false};
  bool _rpc_connected{false};
  bool _first_write{true};
  bool _tcp_nodelay_verified{false};
  std::array<double, 6> _previous_command_deg{};
  rclcpp::Clock _steady_clock{RCL_STEADY_TIME};
};

} //end namespace


#endif
