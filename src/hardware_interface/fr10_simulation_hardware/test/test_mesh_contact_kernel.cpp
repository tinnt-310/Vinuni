#include "fr10_simulation_hardware/mesh_contact_kernel.hpp"
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <filesystem>
#include <btBulletCollisionCommon.h>

using namespace fr10_simulation;
std::vector<Triangle> cube() {
  const Eigen::Vector3d p[]={{-.005,-.005,-.005},{.005,-.005,-.005},
      {.005,.005,-.005},{-.005,.005,-.005},{-.005,-.005,.005},
      {.005,-.005,.005},{.005,.005,.005},{-.005,.005,.005}};
  const unsigned faces[][3]={{0,2,1},{0,3,2},{4,5,6},{4,6,7},{0,1,5},{0,5,4},
      {1,2,6},{1,6,5},{2,3,7},{2,7,6},{3,0,4},{3,4,7}};
  std::vector<Triangle> result;
  for (const auto &f:faces) result.push_back({p[f[0]],p[f[1]],p[f[2]]});
  return result;
}
std::vector<Triangle> stl(const std::string &path) {
  std::ifstream file(path,std::ios::binary); file.seekg(80);
  uint32_t count{};file.read(reinterpret_cast<char *>(&count),4);
  if (!file || count>1000000) throw std::runtime_error("invalid test STL");
  std::vector<Triangle> result(count);
  for (auto &triangle:result) {
    file.seekg(12,std::ios::cur);
    for(auto &p:triangle) {float v[3];file.read(reinterpret_cast<char *>(v),12);p={v[0],v[1],v[2]};}
    file.seekg(2,std::ios::cur);
  }
  if (!file) throw std::runtime_error("incomplete test STL");
  return result;
}
Eigen::Isometry3d pose(const Eigen::Vector3d &position,const Eigen::Vector3d &rpy) {
  Eigen::Isometry3d result=Eigen::Isometry3d::Identity();result.translation()=position;
  result.linear()=(Eigen::AngleAxisd(rpy.z(),Eigen::Vector3d::UnitZ())*
      Eigen::AngleAxisd(rpy.y(),Eigen::Vector3d::UnitY())*
      Eigen::AngleAxisd(rpy.x(),Eigen::Vector3d::UnitX())).toRotationMatrix();
  return result;
}
// Authoritative uncached contactPairTest path from the installed Bullet API.
// It is test-only: compare the optimized immutable-triangle path without a
// production mode flag, copied collider implementation or stored contacts.
struct OfficialGeometry {
  std::unique_ptr<btTriangleMesh> mesh;
  std::unique_ptr<btCollisionShape> shape;
  explicit OfficialGeometry(const std::vector<Triangle> &triangles,bool convex) {
    const auto mm=[](const Eigen::Vector3d &p){return btVector3(p.x()*1000.,p.y()*1000.,p.z()*1000.);};
    if(convex) {
      auto hull=std::make_unique<btConvexHullShape>();hull->setMargin(0.);
      for(const auto &triangle:triangles)for(const auto &p:triangle)hull->addPoint(mm(p),false);
      hull->recalcLocalAabb();hull->optimizeConvexHull();
      if(!hull->initializePolyhedralFeatures())throw std::runtime_error("invalid reference hull");
      shape=std::move(hull);
    } else {
      mesh=std::make_unique<btTriangleMesh>();
      for(const auto &t:triangles)mesh->addTriangle(mm(t[0]),mm(t[1]),mm(t[2]),false);
      shape=std::make_unique<btBvhTriangleMeshShape>(mesh.get(),true);shape->setMargin(0.);
    }
  }
};
struct OfficialReference {
  btDefaultCollisionConfiguration config;
  btCollisionDispatcher dispatcher{&config};
  btDbvtBroadphase broad;
  btCollisionWorld world{&dispatcher,&broad,&config};
  OfficialReference(){world.getDispatchInfo().m_enableSatConvex=true;}
  MeshContactResult query(const OfficialGeometry &cell,const Eigen::Isometry3d &cell_pose,
      const OfficialGeometry &mesh,const Eigen::Isometry3d &mesh_pose) {
    const Eigen::Isometry3d relative=mesh_pose.inverse()*cell_pose;
    const auto &r=relative.linear();
    btTransform transform;
    transform.setOrigin(btVector3(relative.translation().x()*1000.,relative.translation().y()*1000.,relative.translation().z()*1000.));
    transform.setBasis(btMatrix3x3(r(0,0),r(0,1),r(0,2),r(1,0),r(1,1),r(1,2),r(2,0),r(2,1),r(2,2)));
    btCollisionObject a,b;a.setCollisionFlags(0);b.setCollisionFlags(0);
    a.setCollisionShape(cell.shape.get());a.setWorldTransform(transform);
    b.setCollisionShape(mesh.shape.get());b.setWorldTransform(btTransform::getIdentity());
    struct Points final : btCollisionWorld::ContactResultCallback {
      MeshContactResult result;const btCollisionObject *first;
      const Eigen::Isometry3d &world_mesh;
      Points(const btCollisionObject &cell,const Eigen::Isometry3d &mesh):first(&cell),world_mesh(mesh){}
      btScalar addSingleResult(btManifoldPoint &point,const btCollisionObjectWrapper *object,int,int,
          const btCollisionObjectWrapper *,int,int) override {
        const double depth=-point.getDistance()/1000.;if(depth<=0. || result.count==250)return 0.;
        auto &p=result.points[result.count++];p.depth_m=depth;
        const auto midpoint=(point.getPositionWorldOnA()+point.getPositionWorldOnB())*.0005;
        p.position_world_m=world_mesh*Eigen::Vector3d(midpoint.x(),midpoint.y(),midpoint.z());
        const auto normal=(object->getCollisionObject()==first ? 1. : -1.)*point.m_normalWorldOnB;
        p.normal_toward_first=world_mesh.linear()*Eigen::Vector3d(normal.x(),normal.y(),normal.z());return 0.;
      }
    } points(a,mesh_pose);
    world.contactPairTest(&a,&b,points);return points.result;
  }
};
int main(int argc,char **argv) {
  bool ok=true;
  const auto check=[&](bool condition,const char *name) {
    if (!condition) {std::cerr<<"FAIL "<<name<<'\n';ok=false;}
  };
  MeshContactKernel kernel;
  MeshContactGeometry tool(cube(),true);
  MeshContactGeometry slab({Triangle{Eigen::Vector3d(-1,-1,0),Eigen::Vector3d(1,-1,0),Eigen::Vector3d(1,1,0)},
      Triangle{Eigen::Vector3d(-1,-1,0),Eigen::Vector3d(1,1,0),Eigen::Vector3d(-1,1,0)}},false);
  for(double depth:{-.0001,.000001,.00001}) {
    for(const auto &base:{pose({0,0,0},{0,0,0}),pose({1000000,-2000000,3000000},{.2,-.3,.7})}) {
      auto local=Eigen::Isometry3d::Identity();local.translation().z()=.005-depth;
      const Eigen::Isometry3d measured=base*local;
      const auto direct=kernel.collide(tool,measured,slab,base);
      const auto reverse=kernel.collide(slab,base,tool,measured);
      check(direct.valid && reverse.valid,"FINITE_CONTACT_IN_BOTH_COLLISION_ORDERS");
      if(depth<0.) {check(!direct.count && !reverse.count,"SEPARATED_CUBE_IS_NOT_A_FRESH_CONTACT");continue;}
      check(direct.count && reverse.count,"MICROMETRE_PENETRATION_IS_OBSERVED_WITHOUT_MARGIN");
      for(const auto &result:{direct,reverse}) for(unsigned i=0;i<result.count;++i)
        check(std::abs(result.points[i].depth_m-depth)<2e-7,"INDEPENDENT_DEPTH_UNDER_ROTATION_AND_LARGE_WORLD_OFFSET");
      for(unsigned i=0;i<direct.count;++i)
        check((direct.points[i].normal_toward_first-base.linear()*Eigen::Vector3d::UnitZ()).norm()<1e-5,
            "NORMAL_POINTS_TO_FIRST_BODY_IN_WORLD_FRAME");
      for(unsigned i=0;i<reverse.count;++i)
        check((reverse.points[i].normal_toward_first+base.linear()*Eigen::Vector3d::UnitZ()).norm()<1e-5,
            "REVERSED_COLLISION_NORMAL_HAS_OPPOSITE_SIGN");
    }
  }
  // Independent support-function contact sign for edges/corners. Rotate the
  // cell relative to the plane, not merely the complete world coordinate frame.
  for(double roll:{-.52,-.13,0.,.27,.61})for(double pitch:{-.43,0.,.38}) {
    auto tilted=pose({0,0,0},{roll,pitch,.17});
    const double support=.005*tilted.linear().row(2).cwiseAbs().sum();
    for(double expected:{-.00001,.000001,.00001}) {
      tilted.translation().z()=support-expected;
      for(bool reverse:{false,true}) {
        const auto result=reverse ? kernel.collide(slab,Eigen::Isometry3d::Identity(),tool,tilted) :
            kernel.collide(tool,tilted,slab,Eigen::Isometry3d::Identity());
        check(result.valid,"EDGE_CORNER_CONTACT_QUERY_VALID");
        double deepest=0.;for(unsigned i=0;i<result.count;++i)deepest=std::max(deepest,result.points[i].depth_m);
        if(expected<0.)check(result.count==0,"SEPARATED_TILTED_CELL_HAS_NO_CONTACT");
        else check(result.count>0 && deepest>0.,
            "TILTED_EDGE_CORNER_PENETRATION_IS_OBSERVED");
      }
    }
  }
  auto invalid=Eigen::Isometry3d::Identity();invalid.translation().x()=std::numeric_limits<double>::quiet_NaN();
  check(!kernel.collide(tool,invalid,slab,Eigen::Isometry3d::Identity()).valid,
      "INVALID_GEOMETRY_POSE_IS_NOT_A_VALID_ZERO_LOAD");
  invalid=Eigen::Isometry3d::Identity();invalid.linear()(0,0)=2.;
  check(!kernel.collide(tool,invalid,slab,Eigen::Isometry3d::Identity()).valid,
      "SCALING_CANNOT_BE_PASSED_AS_A_RIGID_CONTACT_POSE");
  invalid=Eigen::Isometry3d::Identity();invalid.linear()(0,0)=-1.;
  check(!kernel.collide(tool,invalid,slab,Eigen::Isometry3d::Identity()).valid,
      "REFLECTION_CANNOT_BE_PASSED_AS_A_RIGID_CONTACT_POSE");
  auto open=cube();open.pop_back();
  try {MeshContactGeometry rejected(open,true);check(false,"OPEN_CONVEX_CELL_REJECTED");}
  catch(const std::invalid_argument &) {}
  if(argc!=2) return 2;
  const std::string model=argv[1];
  unsigned cells=0;
  std::vector<std::string> canonical_paths;
  for(const auto &file:std::filesystem::directory_iterator(model+"/collision")) {
    if(file.path().extension()!=".stl" || file.path().filename()=="mating_nose_bvh.stl") continue;
    try {MeshContactGeometry canonical(stl(file.path().string()),true);++cells;canonical_paths.push_back(file.path().string());}
    catch(const std::exception &e) {std::cerr<<file.path()<<": "<<e.what()<<"\n";return 1;}
  }
  check(cells>100,"EVERY_CANONICAL_CONTACT_CELL_VALIDATES_WITHOUT_CONCAVE_HULL_REPLACEMENT");
  try {
    MeshContactGeometry assembly(stl(model+"/collision/mating_nose_bvh.stl"),true);
    check(false,"NONCONVEX_NOSE_ASSEMBLY_CANNOT_BE_REPLACED_BY_ITS_HULL");
  } catch(const std::invalid_argument &) {}
  MeshContactGeometry socket(stl(model+"/socket_contact_surface.stl"),false);
  MeshContactGeometry collar(stl(model+"/collision/collar_0_22_0.stl"),true);
  const auto socket_pose=pose({-.72,-.159,.32},{-M_PI,0,-M_PI/2});
  const auto tool_pose=pose({-.7199601914721936,-.1590069172206743,.3439973794633448},
      {3.141098729427141,-.00019017098920521286,-1.5790615508223267});
  const auto contact=kernel.collide(collar,tool_pose,socket,socket_pose);
  check(contact.valid && contact.count,"ARCHIVED_ODE_ARTIFACT_STILL_RETAINS_REAL_SMALL_CONTACT");
  double maximum=0.;for(unsigned i=0;i<contact.count;++i) maximum=std::max(maximum,contact.points[i].depth_m);
  check(maximum>1e-6 && maximum<2e-5,"UNCHANGED_CAD_HAS_MICROMETRE_CONTACT_NOT_ODE_1P56MM_DEPTH");
  auto loaded=tool_pose;loaded.translation().y()-=.001;
  const auto positive=kernel.collide(collar,loaded,socket,socket_pose);
  double deeper=0.;for(unsigned i=0;i<positive.count;++i) deeper=std::max(deeper,positive.points[i].depth_m);
  check(positive.valid && deeper>.0005,"DEEP_CONTACT_IS_REPORTED_WITHOUT_DEPTH_CLAMP");
  OfficialReference reference;
  OfficialGeometry official_socket(stl(model+"/socket_contact_surface.stl"),false);
  unsigned queries=0;
  // Every authored cell, varied contact/free/deep states. No controller or
  // actual robot is involved; all physical acceptance gates remain separate.
  for(const auto &path:canonical_paths) {
    const auto triangles=stl(path);
    MeshContactGeometry cached_cell(triangles,true);OfficialGeometry official_cell(triangles,true);
    for(unsigned sample=0;sample<120;++sample) {
      auto moved=tool_pose;
      moved.translation()+=socket_pose.linear()*Eigen::Vector3d(.002*std::sin(sample*.91),
          .002*std::cos(sample*.47),.012*std::sin(sample*.23));
      moved.linear()=tool_pose.linear()*pose({0,0,0},{.0349*std::sin(sample*.41),
          .0349*std::cos(sample*.71),.0349*std::sin(sample*.37)}).linear();
      const auto expected=reference.query(official_cell,moved,official_socket,socket_pose);
      const auto actual=kernel.collide(cached_cell,moved,socket,socket_pose);
      bool equal=actual.valid && actual.count==expected.count;
      for(unsigned i=0;i<std::min(actual.count,expected.count);++i)
        equal=equal && std::abs(actual.points[i].depth_m-expected.points[i].depth_m)<2e-7 &&
            (actual.points[i].position_world_m-expected.points[i].position_world_m).norm()<2e-7 &&
            (actual.points[i].normal_toward_first-expected.points[i].normal_toward_first).norm()<1e-4;
      if(!equal)std::cerr<<"reference mismatch "<<path<<" sample="<<sample<<'\n';
      check(equal,"IMMUTABLE_TRIANGLE_CACHE_PRESERVES_OFFICIAL_CONTACT_PAIR_RESULTS");++queries;
    }
  }
  check(queries==cells*120,"ALL_CANONICAL_CELLS_EXERCISE_REFERENCE_CONTACT_MATRIX");
  std::cout<<"official_contact_queries="<<queries<<'\n';
  std::cout<<"archived_depth_m="<<maximum<<" positive_depth_m="<<deeper<<'\n';
  return ok?0:1;
}
