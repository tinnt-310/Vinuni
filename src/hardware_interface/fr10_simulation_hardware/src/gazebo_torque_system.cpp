#include <array>
#include <memory>
#include <string>
#include <vector>
#include <unordered_set>
#include <atomic>
#include <gazebo/common/Events.hh>
#include <gazebo_ros2_control/gazebo_system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "fr10_simulation_hardware/torque_servo.hpp"

namespace fr10_simulation_hardware {
class GazeboTorqueSystem final : public gazebo_ros2_control::GazeboSystemInterface {
 public:
  bool initSim(rclcpp::Node::SharedPtr &node,gazebo::physics::ModelPtr model,
               const hardware_interface::HardwareInfo &info,sdf::ElementPtr) override {
    nh_=node;
    if (info.joints.size()!=6 || !info.sensors.empty()) return false;
    try {
      JointVector kp,kd,limits;
      for (std::size_t i=0;i<6;++i) {
        const auto &spec=info.joints[i];
        if (spec.name!="j"+std::to_string(i+1) || spec.command_interfaces.size()!=1 ||
            spec.command_interfaces[0].name!="position") return false;
        joints_[i]=model->GetJoint(spec.name);
        if (!joints_[i] || joints_[i]->DOF()!=1) return false;
        std::vector<gazebo::physics::LinkPtr> pending{joints_[i]->GetChild()};
        std::unordered_set<std::string> visited;
        while(!pending.empty()) {
          auto link=pending.back();pending.pop_back();
          if(!link || !visited.insert(link->GetScopedName()).second) continue;
          const double mass=link->GetInertial()->Mass();
          if(!std::isfinite(mass) || mass<0.) return false;
          gravity_links_[i].emplace_back(link,mass);
          for(const auto &child:link->GetChildJoints()) pending.push_back(child->GetChild());
        }
        if(gravity_links_[i].empty()) return false;
        auto number=[&](const char *key) {return std::stod(spec.parameters.at(key));};
        if(number("pos_ki")!=0.) return false;
        kp[i]=number("pos_kp");kd[i]=number("pos_kd");limits[i]=joints_[i]->GetEffortLimit(0);
        double initial=0.;
        for (const auto &state:spec.state_interfaces)
          if (state.name=="position" && !state.initial_value.empty()) initial=std::stod(state.initial_value);
        if (!std::isfinite(initial)) return false;
        // Initialization only: match the URDF's configured starting joints.
        joints_[i]->SetPosition(0,initial,true);
        target_[i]=position_[i]=initial;
      }
      servo_=std::make_unique<StableTorqueServo>(kp,kd,limits);
      for(const auto &[link,mass]:gravity_links_[0]) {
        ServoLink item{link,mass,{}};
        for(unsigned i=0;i<6;++i)
          for(const auto &descendant:gravity_links_[i])
            if(descendant.first==link) {item.influenced[i]=true;break;}
        servo_links_.push_back(item);
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(nh_->get_logger(),"FR10 torque servo init failed: %s",e.what());return false;
    }
    last_time_=model->GetWorld()->SimTime().Double();
    model_=model;
    encoder_velocity_.initialize(Eigen::Map<const JointVector>(position_.data()),last_time_);
    servo_encoder_velocity_.initialize(Eigen::Map<const JointVector>(position_.data()),last_time_);
    servo_update_=gazebo::event::Events::ConnectWorldUpdateBegin(
        [this](const gazebo::common::UpdateInfo &) {advanceServo();});
    RCLCPP_INFO(nh_->get_logger(),"FR10 torque servo: six joints, physics-step encoder stable PD, physical articulated inertia/gravity, URDF effort limits");
    return true;
  }
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> result;
    for (std::size_t i=0;i<6;++i) {
      const auto name="j"+std::to_string(i+1);
      result.emplace_back(name,"position",&position_[i]);
      result.emplace_back(name,"velocity",&velocity_[i]);
      result.emplace_back(name,"effort",&effort_[i]);
    }
    return result;
  }
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> result;
    for (std::size_t i=0;i<6;++i) result.emplace_back("j"+std::to_string(i+1),"position",&target_[i]);
    return result;
  }
  hardware_interface::return_type read(const rclcpp::Time &,const rclcpp::Duration &) override {
    if(servo_error_.load()) return hardware_interface::return_type::ERROR;
    for (std::size_t i=0;i<6;++i) {
      position_[i]=joints_[i]->Position(0);
      effort_[i]=joints_[i]->GetForce(0);
      if (!std::isfinite(position_[i]) || !std::isfinite(effort_[i]))
        return hardware_interface::return_type::ERROR;
    }
    try {
      const auto measured=encoder_velocity_.observe(
          Eigen::Map<const JointVector>(position_.data()),model_->GetWorld()->SimTime().Double());
      Eigen::Map<JointVector>(velocity_.data())=measured;
    } catch(const std::exception &e) {
      RCLCPP_ERROR(nh_->get_logger(),"FR10 encoder feedback rejected: %s",e.what());
      servo_error_.store(true);
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }
  hardware_interface::return_type write(const rclcpp::Time &,const rclcpp::Duration &) override {
    if(servo_error_.load()) return hardware_interface::return_type::ERROR;
    for(double q:target_) if(!std::isfinite(q)) return hardware_interface::return_type::ERROR;
    return hardware_interface::return_type::OK;
  }
 private:
  void advanceServo() {
    if(servo_error_.load()) return;
    const double time=model_->GetWorld()->SimTime().Double(),dt=time-last_time_;
    last_time_=time;
    if (dt==0.) return;
    JointVector torque;
    try {
      const auto gravity=model_->GetWorld()->Gravity();
      JointInertia mass=JointInertia::Zero();
      JointVector feedforward=JointVector::Zero(),target,position,velocity;
      std::array<Eigen::Vector3d,6> axes,anchors;
      for (std::size_t i=0;i<6;++i) {
        const auto axis=joints_[i]->GlobalAxis(0).Normalized(),anchor=joints_[i]->Anchor(0);
        axes[i]={axis.X(),axis.Y(),axis.Z()};anchors[i]={anchor.X(),anchor.Y(),anchor.Z()};
        target[i]=target_[i];position[i]=joints_[i]->Position(0);
      }
      velocity=servo_encoder_velocity_.observe(position,time);
      const Eigen::Vector3d g(gravity.X(),gravity.Y(),gravity.Z());
      for(const auto &item:servo_links_) {
        const auto center=item.link->WorldCoGPose().Pos();
        const Eigen::Vector3d cog(center.X(),center.Y(),center.Z());
        const auto world_inertia=item.link->WorldInertiaMatrix();
        Eigen::Matrix3d inertia;
        for(unsigned row=0;row<3;++row) for(unsigned col=0;col<3;++col)
          inertia(row,col)=world_inertia(row,col);
        std::array<Eigen::Vector3d,6> linear;
        for(unsigned i=0;i<6;++i) if(item.influenced[i]) {
          linear[i]=axes[i].cross(cog-anchors[i]);
          feedforward[i]-=item.mass*linear[i].dot(g);
        }
        for(unsigned i=0;i<6;++i) if(item.influenced[i])
          for(unsigned j=0;j<=i;++j) if(item.influenced[j]) {
            const double value=item.mass*linear[i].dot(linear[j])+axes[i].dot(inertia*axes[j]);
            mass(i,j)+=value;
            if(i!=j) mass(j,i)+=value;
          }
        }
      torque=servo_->update(target,position,velocity,dt,mass,feedforward);
    } catch (const std::exception &e) {
      servo_error_.store(true);
      for (auto &joint:joints_) joint->SetForce(0,0.);
      RCLCPP_ERROR_THROTTLE(nh_->get_logger(),*nh_->get_clock(),1000,"FR10 servo rejected: %s",e.what());
      return;
    }
    for (std::size_t i=0;i<6;++i) joints_[i]->SetForce(0,torque[i]);
  }
  gazebo::physics::ModelPtr model_;
  std::array<gazebo::physics::JointPtr,6> joints_;
  struct ServoLink {
    gazebo::physics::LinkPtr link;
    double mass;
    std::array<bool,6> influenced;
  };
  std::unique_ptr<StableTorqueServo> servo_;
  JointEncoderVelocity encoder_velocity_;
  JointEncoderVelocity servo_encoder_velocity_;
  std::vector<ServoLink> servo_links_;
  std::array<std::vector<std::pair<gazebo::physics::LinkPtr,double>>,6> gravity_links_;
  std::array<double,6> target_{},position_{},velocity_{},effort_{};
  double last_time_{0.};
  std::atomic<bool> servo_error_{false};
  gazebo::event::ConnectionPtr servo_update_;
};
}
PLUGINLIB_EXPORT_CLASS(fr10_simulation_hardware::GazeboTorqueSystem,gazebo_ros2_control::GazeboSystemInterface)
