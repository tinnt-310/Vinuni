#include "fr10_simulation_hardware/ode_mesh_contact.hpp"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ode/ODECollision.hh>
#include <gazebo/physics/MeshShape.hh>

namespace fr10_simulation {
namespace {
// This shape retains the public CAD metadata. ODE obtains live transforms from
// its custom geom; there is no old trimesh history buffer to update each tick.
class ImmutableCadMeshShape final : public gazebo::physics::MeshShape {
public:
  explicit ImmutableCadMeshShape(gazebo::physics::CollisionPtr collision)
      : MeshShape(collision) {}
  void Update() override {}
  void SetScale(const ignition::math::Vector3d &scale) override {
    if(scale!=sdf->Get<ignition::math::Vector3d>("scale"))
      throw std::invalid_argument("matched CAD cell cannot be rescaled after contact initialization");
    MeshShape::SetScale(scale);
  }
  void ProcessMsg(const gazebo::msgs::Geometry &message) override {
    gazebo::msgs::Geometry current;FillMsg(current);
    if(message.SerializeAsString()!=current.SerializeAsString())
      throw std::invalid_argument("matched CAD cell cannot be replaced during a contact session");
  }
};
struct Replacement {
  gazebo::physics::ODECollisionPtr collision;
  gazebo::physics::ShapePtr shape;
  std::unique_ptr<dxGeom,decltype(&dGeomDestroy)> geometry{nullptr,&dGeomDestroy};
};
}
class GazeboCadContact final : public gazebo::ModelPlugin {
public:
  void Load(gazebo::physics::ModelPtr model,sdf::ElementPtr) override {
    if(model->GetWorld()->Physics()->GetType()!="ode")
      throw std::invalid_argument("matched CAD contact requires ODE dynamics");
    boost::recursive_mutex::scoped_lock lock(*model->GetWorld()->Physics()->GetPhysicsUpdateMutex());
    const auto context=std::make_shared<OdeMeshContactContext>();
    std::vector<Replacement> replacements;
    for(const auto &link:model->GetLinks()) for(const auto &collision:link->GetCollisions()) {
      const auto name=collision->GetName();
      const bool tool_link=link->GetName()=="charging_mating_nose" ||
          link->GetName()=="charging_connector_link";
      if(!tool_link || name.find("mating_surface_")==std::string::npos) continue;
      const auto ode=boost::dynamic_pointer_cast<gazebo::physics::ODECollision>(collision);
      if(!ode) throw std::runtime_error("matched CAD collision is not an ODE collision");
      const auto old=ode->GetCollisionId();
      if(dGeomGetClass(old)==dBoxClass) continue;
      const auto original=collision->GetShape();
      if(!boost::dynamic_pointer_cast<gazebo::physics::MeshShape>(original))
        throw std::runtime_error("matched nose cell is not a CAD mesh");
      auto shape=boost::make_shared<ImmutableCadMeshShape>(collision);
      shape->Load(original->GetSDF()->Clone());shape->Init();
      Replacement replacement;replacement.collision=ode;replacement.shape=shape;
      replacement.geometry.reset(createOdeConvexMeshContact(odeMeshTriangles(old),context));
      replacements.push_back(std::move(replacement));
    }
    if(replacements.empty())
      throw std::runtime_error("matched CAD contact found no canonical convex nose cells");
    // Warm immutable static CAD before any motion/FT stream starts. A first
    // contact must not pay for BVH/SAT construction and starve the50 ms encoder
    // watchdog. Native primitive dispatch/body dynamics are unchanged.
    for(const auto &environment:model->GetWorld()->Models()) {
      if(!environment->IsStatic())continue;
      for(const auto &link:environment->GetLinks())for(const auto &collision:link->GetCollisions()) {
        const auto ode=boost::dynamic_pointer_cast<gazebo::physics::ODECollision>(collision);
        if(ode && ode->GetCollisionId() && dGeomGetClass(ode->GetCollisionId())==dTriMeshClass)
          context->prepareNativeMesh(ode->GetCollisionId());
      }
    }
    // Validate every cell before mutating any live collision. Invalid CAD
    // cannot leave half the robot on a different collision representation.
    for(auto &replacement:replacements) {
      const auto &ode=replacement.collision;
      const auto old=ode->GetCollisionId(),geom=replacement.geometry.get();
      const auto body=dGeomGetBody(old);
      const auto category=dGeomGetCategoryBits(old), mask=dGeomGetCollideBits(old);
      dVector3 position{0.,0.,0.,0.};dQuaternion rotation{1.,0.,0.,0.};
      const auto old_position=body ? dGeomGetOffsetPosition(old) : dGeomGetPosition(old);
      for(unsigned i=0;i<3;++i) position[i]=old_position[i];
      if(body) dGeomGetOffsetQuaternion(old,rotation);else dGeomGetQuaternion(old,rotation);
      // Remove the original geom before its ODEMeshShape frees vertex data.
      dGeomDestroy(old);
      ode->SetShape(replacement.shape);ode->SetCollision(geom,true);
      replacement.geometry.release();
      dGeomSetBody(geom,body);dGeomSetCategoryBits(geom,category);dGeomSetCollideBits(geom,mask);
      if(body) {
        dGeomSetOffsetPosition(geom,position[0],position[1],position[2]);
        dGeomSetOffsetQuaternion(geom,rotation);
      } else {
        dGeomSetPosition(geom,position[0],position[1],position[2]);dGeomSetQuaternion(geom,rotation);
      }
    }
    gzmsg<<"Matched CAD contact: "<<replacements.size()
         <<" immutable convex cells; native ODE body dynamics and surfaces retained\n";
  }
};
GZ_REGISTER_MODEL_PLUGIN(GazeboCadContact)
} // namespace fr10_simulation
