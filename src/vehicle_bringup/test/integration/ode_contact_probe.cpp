// Diagnostic-only world plugin. No forces, poses, or solver settings are changed.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/ode/ODESurfaceParams.hh>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>

namespace gazebo {
class OdeContactProbe : public ModelPlugin {
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  double simulation_time_{0};
  event::ConnectionPtr begin_;
  event::ConnectionPtr update_;
  std::ofstream output_;
  static void vector(std::ostream &out, const ignition::math::Vector3d &v) {
    out << '[' << v.X() << ',' << v.Y() << ',' << v.Z() << ']';
  }
  void sample() {
    const double wall = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    output_ << "{\"wall\":" << wall << ",\"sim\":" << simulation_time_
            << ",\"models\":[";
    bool first = true;
    for (auto &model : std::vector<physics::ModelPtr>{model_}) {
      if (model->IsStatic()) continue;
      if (!first) output_ << ',';
      first = false;
      output_ << "{\"name\":\"" << model->GetName() << "\",\"position\":";
      vector(output_, model->WorldPose().Pos());
      output_ << ",\"rpy\":"; vector(output_, model->WorldPose().Rot().Euler());
      auto body = model->GetLink();
      output_ << ",\"linear\":"; vector(output_, body ? body->WorldLinearVel() : ignition::math::Vector3d::Zero);
      output_ << ",\"angular\":"; vector(output_, body ? body->WorldAngularVel() : ignition::math::Vector3d::Zero);
      output_ << ",\"joints\":[";
      bool joint_first = true;
      for (auto &joint : model->GetJoints()) {
        if (!joint->DOF()) continue;
        if (!joint_first) output_ << ',';
        joint_first = false;
        output_ << "{\"name\":\"" << joint->GetName() << "\",\"q\":"
                << joint->Position(0) << ",\"v\":" << joint->GetVelocity(0)
                << ",\"applied_force\":" << joint->GetForce(0) << '}';
      }
      output_ << "]}";
    }
    output_ << "],\"contacts\":[";
    auto manager = world_->Physics()->GetContactManager();
    for (unsigned i = 0; i < manager->GetContactCount(); ++i) {
      auto c = manager->GetContact(i);
      if (i) output_ << ',';
      output_ << "{\"a\":\"" << c->collision1->GetScopedName()
              << "\",\"b\":\"" << c->collision2->GetScopedName() << "\",\"fdir1_world\":";
      auto friction = c->collision1->GetSurface()->FrictionPyramid();
      const auto direction = friction ? c->collision1->WorldPose().Rot().RotateVector(friction->direction1)
                                      : ignition::math::Vector3d::Zero;
      vector(output_, direction);
      auto ode_surface = boost::dynamic_pointer_cast<physics::ODESurfaceParams>(c->collision1->GetSurface());
      output_ << ",\"slip1\":" << (ode_surface ? ode_surface->slip1 : -1.0)
              << ",\"slip2\":" << (ode_surface ? ode_surface->slip2 : -1.0);
      output_ << ",\"points\":[";
      for (int j = 0; j < c->count; ++j) {
        if (j) output_ << ',';
        output_ << "{\"position\":"; vector(output_, c->positions[j]);
        output_ << ",\"normal\":"; vector(output_, c->normals[j]);
        output_ << ",\"depth\":" << c->depths[j] << ",\"body1_force\":";
        vector(output_, c->wrench[j].body1Force);
        output_ << '}';
      }
      output_ << "]}";
    }
    output_ << "]}\n";
  }
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr) override {
    const char *directory = std::getenv("ODE_OUT");
    if (!directory) throw std::runtime_error("ODE_OUT required for contact probe");
    model_ = model;
    world_ = model->GetWorld();
    output_.open(std::string(directory) + "/physics.jsonl");
    if (!output_) throw std::runtime_error("Cannot open physics trace");
    output_ << std::setprecision(17);
    if (const char *slip = std::getenv("ODE_PROBE_SLIP")) {
      for (auto &link : model_->GetLinks()) {
        if (link->GetName().find("Wheel") != 0) continue;
        for (auto &collision : link->GetCollisions()) {
          auto surface = boost::dynamic_pointer_cast<physics::ODESurfaceParams>(collision->GetSurface());
          if (!surface) throw std::runtime_error("ODE surface required");
          surface->slip1 = surface->slip2 = std::stod(slip);
        }
      }
    }
    world_->Physics()->GetContactManager()->SetNeverDropContacts(true);
    begin_ = event::Events::ConnectWorldUpdateBegin([this](const common::UpdateInfo &info) { simulation_time_ = info.simTime.Double(); });
    update_ = event::Events::ConnectWorldUpdateEnd([this]() { sample(); });
  }
  ~OdeContactProbe() override { begin_.reset(); update_.reset(); }
};
GZ_REGISTER_MODEL_PLUGIN(OdeContactProbe)
}
