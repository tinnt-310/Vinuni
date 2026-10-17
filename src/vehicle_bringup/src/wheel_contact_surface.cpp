#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ode/ODESurfaceParams.hh>

namespace gazebo {

// URDF's <gazebo reference="Wheel..."> <slip1> is not translated into the
// collision's ODE surface by Gazebo Classic's URDF-to-SDF converter. Set the
// four wheel surfaces after conversion, before physics begins. This plugin
// changes contact compliance only; joint commands and robot limits are owned
// by ros2_control and the robot description.
class WheelContactSurface final : public ModelPlugin {
 public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    if (!model) {
      throw std::runtime_error("Wheel contact model is null");
    }
    const double tangential_slip = sdf && sdf->HasElement("slip")
                                       ? sdf->Get<double>("slip") : 1e-5;
    if (!std::isfinite(tangential_slip) || tangential_slip < 0.0 ||
        tangential_slip > 0.1) {
      throw std::runtime_error("Wheel contact slip must be finite in [0, 0.1]");
    }
    const std::array<const char *, 4> names{
        "Wheel1_1", "Wheel2_1", "Wheel3_1", "Wheel4_1"};
    for (const char *name : names) {
      auto link = model->GetLink(name);
      if (!link || link->GetCollisions().size() != 1) {
        throw std::runtime_error(std::string("Expected one collision on ") + name);
      }
      auto surface = boost::dynamic_pointer_cast<physics::ODESurfaceParams>(
          link->GetCollisions().front()->GetSurface());
      if (!surface) {
        throw std::runtime_error(std::string("Expected ODE collision surface on ") + name);
      }
      surface->slip1 = tangential_slip;
      surface->slip2 = tangential_slip;
    }
    gzmsg << "[wheel_contact_surface] Applied ODE slip1=slip2="
          << tangential_slip << " to four wheel collisions\n";
  }
};

GZ_REGISTER_MODEL_PLUGIN(WheelContactSurface)

}  // namespace gazebo
