#include "fr10_simulation_hardware/ode_mesh_contact.hpp"
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/MeshShape.hh>
#include <map>
#include <mutex>
#include <stdexcept>

namespace fr10_simulation {
namespace {
Eigen::Isometry3d pose(dGeomID geom) {
  Eigen::Isometry3d result=Eigen::Isometry3d::Identity();
  const auto p=dGeomGetPosition(geom); const auto r=dGeomGetRotation(geom);
  result.translation()=Eigen::Vector3d(p[0],p[1],p[2]);
  for(unsigned i=0;i<3;++i) for(unsigned j=0;j<3;++j) result.linear()(i,j)=r[4*i+j];
  return result;
}
struct NativeMesh {
  dTriMeshDataID data{};
  int count{};
  boost::weak_ptr<gazebo::physics::Shape> owner;
  ignition::math::Vector3d scale;
  std::string mesh_uri;
  std::unique_ptr<MeshContactGeometry> geometry;
};
struct Context {
  std::mutex mutex;
  MeshContactKernel kernel;
  std::map<dGeomID,NativeMesh> meshes;
  const MeshContactGeometry &mesh(dGeomID geom) {
    auto collision=static_cast<gazebo::physics::Collision *>(dGeomGetData(geom));
    const auto shape=collision ? collision->GetShape() : gazebo::physics::ShapePtr{};
    const auto mesh_shape=boost::dynamic_pointer_cast<gazebo::physics::MeshShape>(shape);
    const auto data=dGeomTriMeshGetData(geom);
    const int count=dGeomTriMeshGetTriangleCount(geom);
    const auto scale=mesh_shape ? mesh_shape->Scale() : ignition::math::Vector3d::One;
    const auto uri=mesh_shape ? mesh_shape->GetMeshURI() : std::string{};
    auto &entry=meshes[geom];
    // Keep only geometry backed by a live Gazebo shape. Standalone ODE users
    // have no ownership token and are rebuilt instead of caching stale IDs.
    if (!entry.geometry || !shape || entry.owner.lock()!=shape || entry.data!=data ||
        entry.count!=count || entry.scale!=scale || entry.mesh_uri!=uri) {
      entry.geometry=std::make_unique<MeshContactGeometry>(odeMeshTriangles(geom),false);
      entry.owner=shape;entry.data=data;entry.count=count;entry.scale=scale;entry.mesh_uri=uri;
    }
    return *entry.geometry;
  }
};
struct Cell {
  std::shared_ptr<Context> context;
  MeshContactGeometry geometry;
  Eigen::AlignedBox3d bounds;
  // Passive native mesh has no body and is never in a collision space. It
  // preserves original ODE primitive/ray behavior without a global override.
  std::vector<dReal> vertices;
  std::vector<dTriIndex> indices;
  dTriMeshDataID native_data{};
  dGeomID native{};
  Cell(const std::vector<Triangle> &triangles,std::shared_ptr<Context> shared)
      : context(std::move(shared)),geometry(triangles,true) {
    for(const auto &triangle:triangles) for(const auto &point:triangle) {
      bounds.extend(point);
      indices.push_back(indices.size());
      for(unsigned i=0;i<3;++i) vertices.push_back(point[i]);
    }
    native_data=dGeomTriMeshDataCreate();
    dGeomTriMeshDataBuildDouble(native_data,vertices.data(),3*sizeof(dReal),indices.size(),
        indices.data(),indices.size(),3*sizeof(dTriIndex));
    native=dCreateTriMesh(nullptr,native_data,nullptr,nullptr,nullptr);
  }
  ~Cell() {dGeomDestroy(native);dGeomTriMeshDataDestroy(native_data);}
};
Cell &cell(dGeomID geom) {return **static_cast<Cell **>(dGeomGetClassData(geom));}
void destroy(dGeomID geom) {delete &cell(geom);}
void aabb(dGeomID geom,dReal result[6]) {
  const auto world=pose(geom);
  Eigen::AlignedBox3d bounds;
  for(unsigned i=0;i<8;++i) bounds.extend(world*cell(geom).bounds.corner(
      static_cast<Eigen::AlignedBox3d::CornerType>(i)));
  for(unsigned i=0;i<3;++i) {result[2*i]=bounds.min()[i];result[2*i+1]=bounds.max()[i];}
}
int customClass();
int collide(dGeomID first,dGeomID second,int flags,dContactGeom *points,int stride) {
  auto &source=cell(first);
  std::lock_guard<std::mutex> lock(source.context->mutex);
  const int type=dGeomGetClass(second);
  if(type!=dTriMeshClass && type!=customClass()) {
    dQuaternion rotation;dGeomGetQuaternion(first,rotation);
    const auto position=dGeomGetPosition(first);
    dGeomSetPosition(source.native,position[0],position[1],position[2]);
    dGeomSetQuaternion(source.native,rotation);
    const int count=dCollide(source.native,second,flags,points,stride);
    for(int i=0;i<count;++i) {
      auto &point=*reinterpret_cast<dContactGeom *>(reinterpret_cast<char *>(points)+i*stride);
      if(point.g1==source.native) point.g1=first;
      if(point.g2==source.native) point.g2=first;
    }
    return count;
  }
  const auto &target=type==customClass() ? cell(second).geometry : source.context->mesh(second);
  const unsigned limit=std::min<unsigned>(flags&0xffff,250);
  const auto contact=source.context->kernel.collide(source.geometry,pose(first),target,pose(second),limit);
  if(!contact.valid) throw std::runtime_error("invalid CAD collision result; physical contact cannot be discarded");
  for(unsigned i=0;i<contact.count;++i) {
    auto &output=*reinterpret_cast<dContactGeom *>(reinterpret_cast<char *>(points)+i*stride);
    output={};output.g1=first;output.g2=second;output.side1=output.side2=-1;
    output.depth=contact.points[i].depth_m;
    for(unsigned j=0;j<3;++j) {
      output.pos[j]=contact.points[i].position_world_m[j];
      output.normal[j]=contact.points[i].normal_toward_first[j];
    }
  }
  return contact.count;
}
dColliderFn *dispatcher(int) {return &collide;}
int customClass() {
  static const int type=[] {
    dGeomClass definition{};definition.bytes=sizeof(Cell *);
    definition.collider=&dispatcher;definition.aabb=&aabb;definition.dtor=&destroy;
    return dCreateGeomClass(&definition);
  }();
  return type;
}
} // namespace
struct OdeMeshContactContext::Impl {std::shared_ptr<Context> context=std::make_shared<Context>();};
OdeMeshContactContext::OdeMeshContactContext() : impl_(std::make_shared<Impl>()) {}
OdeMeshContactContext::~OdeMeshContactContext()=default;
void OdeMeshContactContext::prepareNativeMesh(dGeomID geometry) {
  if(!geometry || dGeomGetClass(geometry)!=dTriMeshClass)
    throw std::invalid_argument("native CAD preparation requires an ODE mesh");
  std::lock_guard<std::mutex> lock(impl_->context->mutex);
  impl_->context->mesh(geometry);
}
dGeomID createOdeConvexMeshContact(const std::vector<Triangle> &triangles,
    const std::shared_ptr<OdeMeshContactContext> &context) {
  if(!context) throw std::invalid_argument("CAD contact context missing");
  auto data=std::make_unique<Cell>(triangles,context->impl_->context);
  auto geom=dCreateGeom(customClass());
  *static_cast<Cell **>(dGeomGetClassData(geom))=data.release();
  return geom;
}
std::vector<Triangle> odeMeshTriangles(dGeomID geom) {
  if(!geom || dGeomGetClass(geom)!=dTriMeshClass) throw std::invalid_argument("expected native ODE CAD mesh");
  const auto local=pose(geom).inverse();
  const int count=dGeomTriMeshGetTriangleCount(geom);
  if(count<=0) throw std::invalid_argument("empty native ODE CAD mesh");
  std::vector<Triangle> triangles(count);
  for(int i=0;i<count;++i) {
    dVector3 p[3];dGeomTriMeshGetTriangle(geom,i,&p[0],&p[1],&p[2]);
    for(unsigned j=0;j<3;++j) triangles[i][j]=local*Eigen::Vector3d(p[j][0],p[j][1],p[j][2]);
  }
  return triangles;
}
} // namespace fr10_simulation
