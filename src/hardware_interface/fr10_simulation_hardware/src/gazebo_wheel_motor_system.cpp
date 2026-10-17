#include <array>
#include <atomic>
#include <cmath>
#include <algorithm>
#include <gazebo/common/Events.hh>
#include <gazebo_ros2_control/gazebo_system_interface.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace fr10_simulation_hardware {
// ODE constraint motors apply bounded physical torque. GazeboSystem's direct
// SetPosition/SetVelocity write path cannot provide a precision parking plant
// on a floating, heavy chassis, even when its encoder odometry reports zero.
class GazeboWheelMotorSystem final : public gazebo_ros2_control::GazeboSystemInterface {
 public:
  bool initSim(rclcpp::Node::SharedPtr &node,gazebo::physics::ModelPtr model,
               const hardware_interface::HardwareInfo &info,sdf::ElementPtr) override {
    node_=node;model_=model;
    if (!model || info.joints.size()!=8 || !info.sensors.empty() ||
        model->GetWorld()->Physics()->GetType()!="ode") return false;
    try { for (unsigned i=0;i<8;++i) {
      const auto &spec=info.joints[i];
      const bool steering=spec.command_interfaces.size()==1 && spec.command_interfaces[0].name=="position";
      const bool drive=spec.command_interfaces.size()==1 && spec.command_interfaces[0].name=="velocity";
      if ((!steering && !drive) ||
          (steering && spec.name.rfind("Revolute",0)!=0) ||
          (drive && spec.name.rfind("Wheel",0)!=0)) return false;
      auto &joint=joints_[i];joint.name=spec.name;joint.steering=steering;
      joint.handle=model->GetJoint(spec.name);
      if (!joint.handle || joint.handle->DOF()!=1) return false;
      joint.limit=joint.handle->GetEffortLimit(0);
      if (!std::isfinite(joint.limit) || joint.limit<=0.) return false;
      joint.command_min=std::stod(spec.command_interfaces[0].min);
      joint.command_max=std::stod(spec.command_interfaces[0].max);
      if (!std::isfinite(joint.command_min) || !std::isfinite(joint.command_max) ||
          joint.command_min>=joint.command_max) return false;
      if(steering) {
        joint.gain=std::stod(spec.parameters.at("motor_position_gain"));
        joint.max_velocity=std::stod(spec.parameters.at("motor_velocity_limit"));
        if (!std::isfinite(joint.gain) || joint.gain<=0. ||
            !std::isfinite(joint.max_velocity) || joint.max_velocity<=0. ||
            joint.max_velocity>joint.handle->GetVelocityLimit(0)) return false;
      }
      joint.command=steering ? joint.handle->Position(0) : 0.;
      if (!validCommand(joint)) return false;
      if (!joint.handle->SetParam("fmax",0,joint.limit) ||
          !joint.handle->SetParam("vel",0,0.)) return false;
    }} catch(const std::exception &error) {
      RCLCPP_ERROR(node_->get_logger(),"Invalid vehicle motor configuration: %s",error.what());return false;
    }
    update_=gazebo::event::Events::ConnectWorldUpdateBegin(
        [this](const gazebo::common::UpdateInfo &) { advance(); });
    RCLCPP_INFO(node_->get_logger(),"Vehicle: eight physical ODE motors, URDF torque limits, no joint teleportation");
    return true;
  }
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> result;
    for (auto &joint:joints_) {
      result.emplace_back(joint.name,"position",&joint.position);
      result.emplace_back(joint.name,"velocity",&joint.velocity);
    }
    return result;
  }
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> result;
    for (auto &joint:joints_)
      result.emplace_back(joint.name,joint.steering ? "position" : "velocity",&joint.command);
    return result;
  }
  hardware_interface::return_type read(const rclcpp::Time &,const rclcpp::Duration &) override {
    if (failed_.load()) return hardware_interface::return_type::ERROR;
    for(auto &joint:joints_) {
      joint.position=joint.handle->Position(0);joint.velocity=joint.handle->GetVelocity(0);
      if (!std::isfinite(joint.position) || !std::isfinite(joint.velocity))
        {failed_.store(true);return hardware_interface::return_type::ERROR;}
    }
    return hardware_interface::return_type::OK;
  }
  hardware_interface::return_type write(const rclcpp::Time &,const rclcpp::Duration &) override {
    if (failed_.load()) return hardware_interface::return_type::ERROR;
    for (const auto &joint:joints_)
      if (!validCommand(joint)) {failed_.store(true);return hardware_interface::return_type::ERROR;}
    return hardware_interface::return_type::OK;
  }
 private:
  void advance() {
    // Validate the complete motor set before applying any proposal. Invalid
    // commands latch braking for all eight joints, including earlier indices.
    for(const auto &joint:joints_)
      if(!validCommand(joint) || !std::isfinite(joint.handle->Position(0))) failed_.store(true);
    for (auto &joint:joints_) {
      const double position=joint.handle->Position(0);
      // Position error drives a velocity motor with the same steering speed
      // ceiling as swerve; zero wheel speed physically brakes under the lease.
      const double velocity=failed_.load() ? 0. : joint.steering ?
          std::clamp(joint.gain*(joint.command-position),-joint.max_velocity,joint.max_velocity) : joint.command;
      if (!joint.handle->SetParam("vel",0,velocity)) failed_.store(true);
    }
  }
  struct Joint {
    std::string name;gazebo::physics::JointPtr handle;
    bool steering{false};double command{0},position{0},velocity{0},limit{0};
    double command_min{0},command_max{0},gain{0},max_velocity{0};
  };
  static bool validCommand(const Joint &joint) {
    return std::isfinite(joint.command) && joint.command>=joint.command_min && joint.command<=joint.command_max;
  }
  std::array<Joint,8> joints_;
  gazebo::physics::ModelPtr model_;
  rclcpp::Node::SharedPtr node_;
  std::atomic<bool> failed_{false};
  gazebo::event::ConnectionPtr update_;
};
}
PLUGINLIB_EXPORT_CLASS(fr10_simulation_hardware::GazeboWheelMotorSystem,gazebo_ros2_control::GazeboSystemInterface)
