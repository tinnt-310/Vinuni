#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ode/ODEPhysics.hh>
#include <gazebo/physics/ode/ODECollision.hh>
#include <gazebo/physics/ode/ODESurfaceParams.hh>
#include <iostream>
#include "fr10_simulation_hardware/gazebo_ode_contact.hpp"
#include "fr10_simulation_hardware/ode_mesh_contact.hpp"

namespace {
std::vector<fr10_simulation::Triangle> boxTriangles(const dVector3 &size) {
  Eigen::Vector3d p[8];
  for(unsigned i=0;i<8;++i) for(unsigned j=0;j<3;++j)
    p[i][j]=.5*size[j]*(i&(1<<j) ? 1. : -1.);
  const unsigned faces[][3]={{0,1,3},{0,3,2},{4,6,7},{4,7,5},
      {0,4,5},{0,5,1},{2,3,7},{2,7,6},{0,2,6},{0,6,4},{1,5,7},{1,7,3}};
  std::vector<fr10_simulation::Triangle> triangles;
  for(const auto &face:faces) triangles.push_back({p[face[0]],p[face[1]],p[face[2]]});
  return triangles;
}
void replace(gazebo::physics::ODECollisionPtr collision,dGeomID geom) {
  const auto old=collision->GetCollisionId();
  const auto body=dGeomGetBody(old);
  const auto category=dGeomGetCategoryBits(old),mask=dGeomGetCollideBits(old);
  dVector3 position{};dQuaternion rotation{};
  const auto p=body ? dGeomGetOffsetPosition(old) : dGeomGetPosition(old);
  for(unsigned i=0;i<3;++i) position[i]=p[i];
  if(body)dGeomGetOffsetQuaternion(old,rotation);else dGeomGetQuaternion(old,rotation);
  dGeomDestroy(old);collision->SetCollision(geom,true);dGeomSetBody(geom,body);
  dGeomSetCategoryBits(geom,category);dGeomSetCollideBits(geom,mask);
  if(body) {dGeomSetOffsetPosition(geom,position[0],position[1],position[2]);dGeomSetOffsetQuaternion(geom,rotation);}
  else {dGeomSetPosition(geom,position[0],position[1],position[2]);dGeomSetQuaternion(geom,rotation);}
}
struct NativeData {
  std::vector<dReal> vertices;
  std::vector<dTriIndex> indices;
  dTriMeshDataID data{};
  ~NativeData() {if(data)dGeomTriMeshDataDestroy(data);}
};
}

int main(int argc, char **argv) {
  if ((argc != 4 && argc != 5) || !gazebo::setupServer()) return 2;
  NativeData native;
  const bool first_tool = std::string(argv[2]) == "tool_first";
  const bool two_bodies = std::string(argv[3]) == "two_bodies";
  auto world = gazebo::loadWorld(argv[1]);
  if (!world) { gazebo::shutdown(); return 2; }
  auto ground_model = world->ModelByName("ground");
  if (two_bodies) {
    ground_model->SetStatic(false);
    ground_model->GetLink()->SetKinematic(true);
    ground_model->GetLink()->SetGravityMode(false);
  }
  auto tool_link = world->ModelByName("tool")->GetLink();
  auto ground_link = ground_model->GetLink();
  auto ground = boost::dynamic_pointer_cast<gazebo::physics::ODECollision>(ground_link->GetCollision("floor"));
  auto tool = boost::dynamic_pointer_cast<gazebo::physics::ODECollision>(tool_link->GetCollision("box"));
  if(argc==5) {
    // Use the actual matched station's finite CAD contact calibration. Native
    // decoder tests above retain their original rigid-box surface settings.
    // Single-precision narrow-phase geometry cannot qualify a 1e12 N/m body.
    for(const auto &collision:{tool,ground}) {
      collision->GetODESurface()->kp=20000.;collision->GetODESurface()->kd=100.;
    }
    const auto context=std::make_shared<fr10_simulation::OdeMeshContactContext>();
    bool rejected=false;
    try {context->prepareNativeMesh(nullptr);}catch(const std::invalid_argument &){rejected=true;}
    if(!rejected)return 1;
    dVector3 size;dGeomBoxGetLengths(tool->GetCollisionId(),size);
    replace(tool,fr10_simulation::createOdeConvexMeshContact(boxTriangles(size),
        context));
    if(std::string(argv[4])=="cad_surface") {
      dGeomBoxGetLengths(ground->GetCollisionId(),size);
      for(const auto &triangle:boxTriangles(size)) for(const auto &point:triangle) {
        native.indices.push_back(native.indices.size());
        for(unsigned i=0;i<3;++i)native.vertices.push_back(point[i]);
      }
      native.data=dGeomTriMeshDataCreate();
      dGeomTriMeshDataBuildDouble(native.data,native.vertices.data(),3*sizeof(dReal),native.indices.size(),
          native.indices.data(),native.indices.size(),3*sizeof(dTriIndex));
      replace(ground,dCreateTriMesh(nullptr,native.data,nullptr,nullptr,nullptr));
      context->prepareNativeMesh(ground->GetCollisionId());
    } else {
      rejected=false;
      try {context->prepareNativeMesh(ground->GetCollisionId());}
      catch(const std::invalid_argument &){rejected=true;}
      if(!rejected)return 1;
    }
  }
  auto engine = boost::dynamic_pointer_cast<gazebo::physics::ODEPhysics>(world->Physics());
  auto manager = engine->GetContactManager();
  manager->SetNeverDropContacts(true);
  gazebo::runWorld(world, 1000);
  if(argc==5) {
    std::cout<<"cad_cell_pose "<<tool_link->WorldPose()<<" velocity "<<tool_link->WorldLinearVel()
             <<" angular "<<tool_link->WorldAngularVel()<<'\n';
    dContactGeom preview[250];
    const int n=dCollide(tool->GetCollisionId(),ground->GetCollisionId(),250,preview,sizeof(preview[0]));
    std::cout<<"cad_cell_points "<<n<<'\n';
    for(int i=0;i<n && i<12;++i) std::cout<<preview[i].depth<<" pos "<<preview[i].pos[0]<<' '
        <<preview[i].pos[1]<<' '<<preview[i].pos[2]<<" normal "<<preview[i].normal[0]<<' '
        <<preview[i].normal[1]<<' '<<preview[i].normal[2]<<'\n';
  }
  // Build precisely one physical constraint in the requested collision order.
  // This exercises real ODE feedback, including dJointAttach's static reversal,
  // without relying on the broad-phase traversal order on a particular machine.
  ground->GetSurface()->collideBitmask = 0;
  engine->UpdateCollision();
  ground->GetSurface()->collideBitmask = 0xffff;
  dContactGeom points[MAX_CONTACT_JOINTS];
  engine->Collide(first_tool ? tool.get() : ground.get(), first_tool ? ground.get() : tool.get(), points);
  engine->UpdatePhysics();
  bool ok = manager->GetContactCount() == 1;
  const auto tcp = tool_link->WorldCoGPose().Pos() + tool_link->WorldPose().Rot().RotateVector({.01,.02,.03});
  fr10_simulation::ContactWrenchAccumulator sum;
  if (ok) {
    const auto contact = manager->GetContact(0);
    ok = contact->count > 0 && ((contact->collision1 == tool.get()) == first_tool);
    fr10_simulation::accumulateOdeToolContact(*contact, first_tool, tcp, sum);
  }
  const auto wrench = sum.inFrame(Eigen::Matrix3d::Identity());
  for (double value : wrench) ok = ok && std::isfinite(value);
  const Eigen::Vector3d force(wrench[0],wrench[1],wrench[2]);
  const Eigen::Vector3d moment(wrench[3],wrench[4],wrench[5]);
  const auto com = tool_link->WorldCoGPose().Pos();
  const Eigen::Vector3d arm(com.X()-tcp.X(),com.Y()-tcp.Y(),com.Z()-tcp.Z());
  // Independent 1 kg load and moment-arm check; source is tool-on-environment.
  ok = ok && std::abs(force.z()+9.80665) < .2 && force.head<2>().norm() < .1;
  ok = ok && (moment-arm.cross(force)).norm() < .01;
  std::cout << argv[2] << ' ' << argv[3] << " force " << force.transpose()
            << " moment " << moment.transpose() << " result " << (ok ? "PASS" : "FAIL") << '\n';
  // A malformed contact must propagate invalidity, never a fresh zero.
  gazebo::physics::Contact invalid;
  invalid.collision1 = tool.get(); invalid.collision2 = ground.get(); invalid.count = MAX_CONTACT_JOINTS+1;
  fr10_simulation::accumulateOdeToolContact(invalid, true, tcp, sum);
  for (double value : sum.inFrame(Eigen::Matrix3d::Identity())) ok = ok && std::isnan(value);
  gazebo::shutdown();
  return ok ? 0 : 1;
}
