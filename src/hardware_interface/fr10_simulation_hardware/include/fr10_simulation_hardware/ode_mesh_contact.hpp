#pragma once

#include "fr10_simulation_hardware/mesh_contact_kernel.hpp"
#include <gazebo/ode/ode.h>

namespace fr10_simulation {

// Per-world collision cache, independent of body dynamics and joint commands.
class OdeMeshContactContext {
public:
  OdeMeshContactContext();
  ~OdeMeshContactContext();
  // Geometry preparation only, under the caller's world-update lock. Complete
  // expensive BVH/SAT allocation before controller/FT feedback is active.
  void prepareNativeMesh(dGeomID geometry);
private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
  friend dGeomID createOdeConvexMeshContact(const std::vector<Triangle> &,
      const std::shared_ptr<OdeMeshContactContext> &);
};

// Returned placeable geom owns its CAD cell. The caller owns dGeomDestroy.
// Native primitives/rays retain their original ODE triangle-mesh collider;
// mesh pairs use the verified convex/CAD kernel, never a concave CAD hull.
dGeomID createOdeConvexMeshContact(const std::vector<Triangle> &triangles,
    const std::shared_ptr<OdeMeshContactContext> &context);

// Read original ODE triangle vertices into the geometry's local frame.
std::vector<Triangle> odeMeshTriangles(dGeomID geom);

} // namespace fr10_simulation
