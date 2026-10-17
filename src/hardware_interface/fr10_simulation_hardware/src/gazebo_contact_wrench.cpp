#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <Eigen/Geometry>
#include <unordered_set>
#include "fr10_simulation_hardware/contact_wrench.hpp"
#include "fr10_simulation_hardware/gazebo_ode_contact.hpp"

namespace fr10_simulation {
namespace {
Eigen::Vector3d vector(const ignition::math::Vector3d &value) {
  return {value.X(), value.Y(), value.Z()};
}
}

class GazeboContactWrench final : public gazebo::ModelPlugin {
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;
    node_ = gazebo_ros::Node::Get(sdf);
    tcp_link_ = model_->GetLink(sdf->Get<std::string>("tcp_link"));
    if (!tcp_link_) throw std::runtime_error("contact wrench physical TCP link missing");
    tcp_offset_ = sdf->Get<ignition::math::Vector3d>("tcp_offset");
    if (!vector(tcp_offset_).allFinite()) throw std::runtime_error("invalid contact wrench TCP offset");
    frame_ = sdf->Get<std::string>("frame_name");
    if (frame_.empty()) throw std::runtime_error("contact wrench TCP frame missing");
    // Fixed-joint lumping may merge the nose/shoulder into the connector body.
    // Both representations contribute the same external resultant.
    for (const auto &name : {"charging_connector_link", "charging_mating_nose", "charging_mating_shoulder"}) {
      const auto link = model_->GetLink(name);
      if (link) tool_links_.insert(link.get());
    }
    auto manager = model_->GetWorld()->Physics()->GetContactManager();
    if (model_->GetWorld()->Physics()->GetType() != "ode")
      throw std::runtime_error("external contact wrench requires commissioned Gazebo 11 ODE feedback");
    manager->SetNeverDropContacts(true);
    publisher_ = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
        "/fr10_simulation/contact_wrench", rclcpp::SensorDataQoS().keep_last(1024));
    connection_ = gazebo::event::Events::ConnectWorldUpdateEnd([this] { update(); });
    RCLCPP_INFO(node_->get_logger(), "Physical external-contact wrench at TCP; no gravity, joint or pose commands");
  }
private:
  void update() {
    const auto pose = tcp_link_->WorldPose();
    const auto world_tcp = pose.Pos()+pose.Rot().RotateVector(tcp_offset_);
    ContactWrenchAccumulator sum;
    const auto manager = model_->GetWorld()->Physics()->GetContactManager();
    for (unsigned i=0; i<manager->GetContactCount(); ++i) {
      const auto contact = manager->GetContact(i);
      if (!contact || !contact->collision1 || !contact->collision2) continue;
      const auto first = contact->collision1->GetLink();
      const auto second = contact->collision2->GetLink();
      const bool first_tool = tool_links_.count(first.get());
      const bool second_tool = tool_links_.count(second.get());
      if (first_tool == second_tool) continue; // Internal loads cancel at the sensor boundary.
      accumulateOdeToolContact(*contact, first_tool, world_tcp, sum);
    }
    const auto &q = pose.Rot();
    const Eigen::Quaterniond rotation(q.W(),q.X(),q.Y(),q.Z());
    const auto wrench = sum.inFrame(rotation.toRotationMatrix());
    geometry_msgs::msg::WrenchStamped message;
    message.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(model_->GetWorld()->SimTime());
    message.header.frame_id = frame_;
    message.wrench.force.x=wrench[0]; message.wrench.force.y=wrench[1]; message.wrench.force.z=wrench[2];
    message.wrench.torque.x=wrench[3]; message.wrench.torque.y=wrench[4]; message.wrench.torque.z=wrench[5];
    publisher_->publish(message);
  }
  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr tcp_link_;
  gazebo_ros::Node::SharedPtr node_;
  gazebo::event::ConnectionPtr connection_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_;
  std::unordered_set<gazebo::physics::Link *> tool_links_;
  ignition::math::Vector3d tcp_offset_;
  std::string frame_;
};
GZ_REGISTER_MODEL_PLUGIN(GazeboContactWrench)
} // namespace fr10_simulation
