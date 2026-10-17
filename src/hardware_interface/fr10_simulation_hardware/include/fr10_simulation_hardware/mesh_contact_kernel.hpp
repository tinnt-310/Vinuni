#pragma once

#include <Eigen/Geometry>
#include <array>
#include <memory>
#include <vector>

namespace fr10_simulation {

using Triangle = std::array<Eigen::Vector3d, 3>;
struct MeshContactPoint {
  Eigen::Vector3d position_world_m{Eigen::Vector3d::Zero()};
  Eigen::Vector3d normal_toward_first{Eigen::Vector3d::Zero()};
  double depth_m{0.};
};
struct MeshContactResult {
  std::array<MeshContactPoint, 250> points;
  unsigned count{0};
  bool valid{true};
};

// Collision geometry only: no body integration, actuator or joint ownership.
// Convex cells must be closed and actually convex; the concave surface retains
// every supplied CAD triangle and its holes. Internal millimetre units resolve
// Bullet's fixed single-precision GJK/EPA tolerances; public quantities are SI.
class MeshContactGeometry {
public:
  MeshContactGeometry(const std::vector<Triangle> &triangles, bool convex);
  ~MeshContactGeometry();
  MeshContactGeometry(const MeshContactGeometry &) = delete;
  MeshContactGeometry &operator=(const MeshContactGeometry &) = delete;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  friend class MeshContactKernel;
};

class MeshContactKernel {
public:
  MeshContactKernel();
  ~MeshContactKernel();
  MeshContactResult collide(const MeshContactGeometry &first,
      const Eigen::Isometry3d &world_first, const MeshContactGeometry &second,
      const Eigen::Isometry3d &world_second, unsigned limit = 250);
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace fr10_simulation
