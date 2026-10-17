#pragma once

#include <array>
#include <limits>
#include <Eigen/Geometry>
#include <Eigen/LU>

namespace fr10_simulation {

// Inputs are world-frame wrenches about the physical tool body's COM.
// Backend-specific feedback slots/frames are decoded before reaching this sum.
// Report the tool-on-environment resultant about the physical TCP, without
// gravity or internal tool loads. This is an ideal external-contact source,
// distinct from a raw load cell and never an actuator.
class ContactWrenchAccumulator {
public:
  void invalidate() { valid_ = false; }
  void add(const Eigen::Vector3d &force_on_tool, const Eigen::Vector3d &moment_about_com,
      const Eigen::Vector3d &world_com, const Eigen::Vector3d &world_tcp) {
    if (!force_on_tool.allFinite() || !moment_about_com.allFinite() ||
        !world_com.allFinite() || !world_tcp.allFinite()) { valid_ = false; return; }
    force_ -= force_on_tool;
    moment_ -= moment_about_com + (world_com-world_tcp).cross(force_on_tool);
  }
  std::array<double, 6> inFrame(const Eigen::Matrix3d &world_from_tcp) const {
    std::array<double, 6> result;
    if (!valid_ || !world_from_tcp.allFinite() ||
        (world_from_tcp.transpose()*world_from_tcp-Eigen::Matrix3d::Identity()).norm() > 1e-8 ||
        world_from_tcp.determinant() <= 0.) {
      result.fill(std::numeric_limits<double>::quiet_NaN()); return result;
    }
    const Eigen::Vector3d force = world_from_tcp.transpose()*force_;
    const Eigen::Vector3d moment = world_from_tcp.transpose()*moment_;
    for (std::size_t i=0; i<3; ++i) { result[i]=force[i]; result[i+3]=moment[i]; }
    return result;
  }
private:
  Eigen::Vector3d force_{Eigen::Vector3d::Zero()}, moment_{Eigen::Vector3d::Zero()};
  bool valid_{true};
};

} // namespace fr10_simulation
