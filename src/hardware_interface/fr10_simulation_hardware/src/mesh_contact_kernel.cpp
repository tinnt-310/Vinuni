// Triangle polyhedron construction below is adapted from Bullet 3.05
// btConvexConvexAlgorithm.cpp; this is an altered integration, not upstream.
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#include "fr10_simulation_hardware/mesh_contact_kernel.hpp"
#include <btBulletCollisionCommon.h>
#include <BulletCollision/CollisionShapes/btConvexPolyhedron.h>
#include <BulletCollision/NarrowPhaseCollision/btPolyhedralContactClipping.h>
#include <BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.h>
#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>

namespace fr10_simulation {
namespace {
constexpr double units_per_metre = 1000.;
// Authored convex cells contain float-STL rounding on planar quad facets.
// This bound is below the independently tested 0.2 um kernel resolution;
// it does not permit hulls across bores or nonconvex socket/tool assemblies.
constexpr double convex_surface_rounding_m = 1e-7;
bool rigid(const Eigen::Isometry3d &pose) {
  return pose.matrix().allFinite() &&
      (pose.linear().transpose()*pose.linear()-Eigen::Matrix3d::Identity()).norm()<1e-9 &&
      std::abs(pose.linear().determinant()-1.)<1e-9 &&
      (pose.matrix().row(3)-Eigen::RowVector4d(0.,0.,0.,1.)).norm()<1e-12;
}
btVector3 internal(const Eigen::Vector3d &v) {
  return {btScalar(v.x()*units_per_metre), btScalar(v.y()*units_per_metre),
          btScalar(v.z()*units_per_metre)};
}
Eigen::Vector3d vector(const btVector3 &v) { return {v.x(), v.y(), v.z()}; }
void validate(const std::vector<Triangle> &triangles, bool convex) {
  if (triangles.empty()) throw std::invalid_argument("empty contact geometry");
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  std::map<std::array<double,3>, unsigned> vertices;
  std::map<std::pair<unsigned,unsigned>, unsigned> edges;
  for (const auto &triangle : triangles) {
    unsigned ids[3];
    for (unsigned i=0; i<3; ++i) {
      const auto &p=triangle[i];
      if (!p.allFinite()) throw std::invalid_argument("nonfinite contact vertex");
      const auto inserted=vertices.emplace(std::array<double,3>{p.x(),p.y(),p.z()}, vertices.size());
      ids[i]=inserted.first->second; center+=p;
    }
    for (unsigned i=0; i<3; ++i) ++edges[std::minmax(ids[i], ids[(i+1)%3])];
    if ((triangle[1]-triangle[0]).cross(triangle[2]-triangle[0]).norm() <= 1e-16)
      throw std::invalid_argument("degenerate contact triangle");
  }
  if (!convex) return;
  for (const auto &edge : edges)
    if (edge.second!=2) throw std::invalid_argument("convex contact cell is not closed");
  center/=double(3*triangles.size());
  for (const auto &face : triangles) {
    Eigen::Vector3d normal=(face[1]-face[0]).cross(face[2]-face[0]).normalized();
    if (normal.dot(center-face[0])>0.) normal=-normal;
    for (const auto &vertex : vertices) {
      const auto &p=vertex.first;
      if (normal.dot(Eigen::Vector3d(p[0],p[1],p[2])-face[0])>convex_surface_rounding_m)
        throw std::invalid_argument("nonconvex contact cell; hull would change CAD");
    }
  }
}
struct Callback final : btCollisionWorld::ContactResultCallback {
  MeshContactResult result;
  Eigen::Isometry3d world_second;
  const btCollisionObject *first;
  unsigned limit;
  Callback(const Eigen::Isometry3d &pose, const btCollisionObject *object, unsigned maximum)
      : world_second(pose), first(object), limit(maximum) {}
  btScalar addSingleResult(btManifoldPoint &point,
      const btCollisionObjectWrapper *a, int, int,
      const btCollisionObjectWrapper *, int, int) override {
    const double depth=-point.getDistance()/units_per_metre;
    const Eigen::Vector3d position=.5*(vector(point.getPositionWorldOnA())+
        vector(point.getPositionWorldOnB()))/units_per_metre;
    const Eigen::Vector3d normal=(a->getCollisionObject()==first ? 1. : -1.)*
        vector(point.m_normalWorldOnB);
    if (!std::isfinite(depth) || !position.allFinite() || !normal.allFinite() ||
        std::abs(normal.norm()-1.)>1e-4) { result.valid=false; return 0.; }
    if (depth<=0. || result.count>=limit) return 0.;
    auto &output=result.points[result.count++];
    output.position_world_m=world_second*position;
    output.normal_toward_first=world_second.linear()*normal;
    output.depth_m=depth;
    return 0.;
  }
};
class QueryContactResult final : public btManifoldResult {
public:
  QueryContactResult(const btCollisionObjectWrapper &a,const btCollisionObjectWrapper &b,
      Callback &output):btManifoldResult(&a,&b),output_(output) {}
  void addContactPoint(const btVector3 &normal,const btVector3 &point,btScalar distance) override {
    btManifoldPoint contact(btVector3(0,0,0),btVector3(0,0,0),normal,distance);
    contact.m_positionWorldOnA=point+normal*distance;contact.m_positionWorldOnB=point;
    const bool swapped=getPersistentManifold()->getBody0()!=getBody0Wrap()->getCollisionObject();
    output_.addSingleResult(contact,swapped ? getBody1Wrap() : getBody0Wrap(),-1,-1,
        swapped ? getBody0Wrap() : getBody1Wrap(),-1,-1);
  }
private:
  Callback &output_;
};
// Immutable triangle SAT data. This preserves the official triangle face
// order/planes and clipFaceAgainstHull path, avoiding per-query reconstruction.
struct TriangleFeature {
  btVector3 points[3];
  btConvexPolyhedron polyhedron;
};
// Own one fresh manifold per cell/mesh query; no stored contact history.
class CachedTriangleQuery final : public btConvexTriangleCallback {
public:
  CachedTriangleQuery(btCollisionDispatcher &dispatcher,const btCollisionObjectWrapper &first,
      const btCollisionObjectWrapper &second,bool reverse,const btDispatcherInfo &info,
      btManifoldResult &result,const std::vector<std::unique_ptr<TriangleFeature>> &features):
      btConvexTriangleCallback(&dispatcher,&first,&second,reverse),
      convex_(reverse ? second : first),mesh_(reverse ? first : second),result_(result),features_(features) {
    result_.setPersistentManifold(m_manifoldPtr);
    setTimeStepAndCounters(0.,info,&convex_,&mesh_,&result_);
  }
  void processTriangle(btVector3 *,int part,int index) override {
    if(part!=0 || index<0 || unsigned(index)>=features_.size())throw std::runtime_error("triangle cache index invalid");
    const auto &triangle=*features_[index];
    const auto *poly=static_cast<const btPolyhedralConvexShape *>(convex_.getCollisionShape())->getConvexPolyhedron();
    btVector3 normal;
    if(!btPolyhedralContactClipping::findSeparatingAxis(*poly,triangle.polyhedron,
        convex_.getWorldTransform(),mesh_.getWorldTransform(),normal,result_))return;
    btVertexArray vertices,clipped;
    for(const auto &point:triangle.points)vertices.push_back(mesh_.getWorldTransform()*point);
    // Same threshold and triangle-face clipping as Bullet3.05; caching must
    // not silently select its different hull/hull clipping branch.
    const btScalar threshold=m_manifoldPtr->getContactBreakingThreshold();
    btPolyhedralContactClipping::clipFaceAgainstHull(normal,*poly,convex_.getWorldTransform(),
        vertices,clipped,btScalar(-1e30)-threshold,threshold,result_);
  }
private:
  const btCollisionObjectWrapper &convex_,&mesh_;
  btManifoldResult &result_;
  const std::vector<std::unique_ptr<TriangleFeature>> &features_;
};
} // namespace
struct MeshContactGeometry::Impl {
  // Shape refers to mesh storage: destroy shape before storage.
  std::unique_ptr<btTriangleMesh> mesh;
  std::unique_ptr<btCollisionShape> shape;
  std::vector<std::unique_ptr<TriangleFeature>> triangles;
};
MeshContactGeometry::MeshContactGeometry(const std::vector<Triangle> &triangles, bool convex)
    : impl_(std::make_unique<Impl>()) {
  validate(triangles, convex);
  if (convex) {
    auto shape=std::make_unique<btConvexHullShape>(); shape->setMargin(0.);
    for (const auto &triangle : triangles) for (const auto &p : triangle)
      shape->addPoint(internal(p), false);
    shape->recalcLocalAabb(); shape->optimizeConvexHull();
    // Exact polyhedral face/edge clipping supplies a supporting manifold.
    // A single GJK witness per CAD triangle leaves a flat loaded face with
    // only two diagonal points and creates spurious ODE rocking/impulses.
    if(!shape->initializePolyhedralFeatures())
      throw std::invalid_argument("convex CAD cell has no valid polyhedral features");
    impl_->shape=std::move(shape);
  } else {
    impl_->mesh=std::make_unique<btTriangleMesh>();
    for (const auto &triangle : triangles)
      impl_->mesh->addTriangle(internal(triangle[0]),internal(triangle[1]),internal(triangle[2]),false);
    for(const auto &triangle:triangles) {
      auto feature=std::make_unique<TriangleFeature>();
      for(unsigned i=0;i<3;++i)feature->points[i]=internal(triangle[i]);
      auto &poly=feature->polyhedron;
      for(unsigned i:{2u,0u,1u})poly.m_vertices.push_back(feature->points[i]);
      btVector3 edges[3];
      for(unsigned i=0;i<3;++i)edges[i]=(feature->points[(i+1)%3]-feature->points[i]).normalized();
      const auto normal=edges[0].cross(edges[1]).normalized();
      for(unsigned reverse=0;reverse<2;++reverse) {
        const auto direction=reverse ? -normal : normal;
        btFace face;
        for(unsigned i:{0u,reverse ? 2u : 1u,reverse ? 1u : 2u})face.m_indices.push_back(i);
        face.m_plane[0]=direction.x();face.m_plane[1]=direction.y();face.m_plane[2]=direction.z();
        btScalar plane=btScalar(1e30);
        for(const auto &point:feature->points)plane=btMin(plane,point.dot(direction));
        face.m_plane[3]=-plane;poly.m_faces.push_back(face);
      }
      for(const auto &edge:edges)poly.m_uniqueEdges.push_back(edge);
      poly.initialize2();impl_->triangles.push_back(std::move(feature));
    }
    impl_->shape=std::make_unique<btBvhTriangleMeshShape>(impl_->mesh.get(),true);
    impl_->shape->setMargin(0.);
  }
}
MeshContactGeometry::~MeshContactGeometry()=default;
struct MeshContactKernel::Impl {
  btDefaultCollisionConfiguration config;
  btCollisionDispatcher dispatcher{&config};
  btDbvtBroadphase broad;
  btCollisionWorld world{&dispatcher,&broad,&config};
  Impl() {world.getDispatchInfo().m_enableSatConvex=true;}
};
MeshContactKernel::MeshContactKernel() : impl_(std::make_unique<Impl>()) {}
MeshContactKernel::~MeshContactKernel()=default;
MeshContactResult MeshContactKernel::collide(const MeshContactGeometry &first,
    const Eigen::Isometry3d &world_first, const MeshContactGeometry &second,
    const Eigen::Isometry3d &world_second, unsigned limit) {
  if (!limit || limit>250 || !rigid(world_first) || !rigid(world_second))
    return MeshContactResult{{},0,false};
  // Subtract world coordinates in double precision before converting to
  // Bullet's local millimetre frame, preserving micrometre contact at any base.
  const Eigen::Isometry3d relative=world_second.inverse()*world_first;
  btTransform transform; transform.setIdentity(); transform.setOrigin(internal(relative.translation()));
  const Eigen::Matrix3d rotation=relative.linear();
  transform.setBasis(btMatrix3x3(rotation(0,0),rotation(0,1),rotation(0,2),
      rotation(1,0),rotation(1,1),rotation(1,2),rotation(2,0),rotation(2,1),rotation(2,2)));
  btCollisionObject a,b;
  a.setCollisionFlags(0); a.setCollisionShape(first.impl_->shape.get()); a.setWorldTransform(transform);
  b.setCollisionShape(second.impl_->shape.get()); b.setWorldTransform(btTransform::getIdentity());
  Callback callback(world_second,&a,limit);
  const bool reverse=a.getCollisionShape()->isConcave() && b.getCollisionShape()->isConvex();
  if(reverse || (a.getCollisionShape()->isConvex() && b.getCollisionShape()->isConcave())) {
    btCollisionObjectWrapper wa(nullptr,a.getCollisionShape(),&a,a.getWorldTransform(),-1,-1);
    btCollisionObjectWrapper wb(nullptr,b.getCollisionShape(),&b,b.getWorldTransform(),-1,-1);
    QueryContactResult result(wa,wb,callback);
    const auto &mesh=reverse ? first : second;
    CachedTriangleQuery query(impl_->dispatcher,wa,wb,reverse,impl_->world.getDispatchInfo(),result,mesh.impl_->triangles);
    static_cast<const btConcaveShape *>(mesh.impl_->shape.get())->processAllTriangles(&query,query.getAabbMin(),query.getAabbMax());
    query.clearWrapperData();
  } else impl_->world.contactPairTest(&a,&b,callback);
  return callback.result;
}
} // namespace fr10_simulation
