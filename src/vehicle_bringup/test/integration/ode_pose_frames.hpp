#pragma once

#include <gazebo/msgs/msgs.hh>
#include <ignition/math/Pose3.hh>
#include <cmath>
#include <string>
#include <vector>

// Gazebo Classic World::ProcessMessages publishes model and link RelativePose,
// including every parent model in the same PosesStamped. Never combine clocks
// or silently treat a missing parent as an identity world transform.
inline bool resolveWorldPose(const gazebo::msgs::PosesStamped &message,
                             const std::string &name,
                             ignition::math::Pose3d &world) {
  std::vector<ignition::math::Pose3d> chain;
  std::string scope = name;
  while (!scope.empty()) {
    const gazebo::msgs::Pose *found = nullptr;
    for (const auto &pose : message.pose())
      if (pose.name() == scope) { found = &pose; break; }
    if (!found) return false;
    const auto &p = found->position();
    const auto &q = found->orientation();
    const double norm = q.w()*q.w()+q.x()*q.x()+q.y()*q.y()+q.z()*q.z();
    if (!std::isfinite(p.x()) || !std::isfinite(p.y()) ||
        !std::isfinite(p.z()) || !std::isfinite(norm) || norm < 1e-12)
      return false;
    ignition::math::Quaterniond rotation(q.w(), q.x(), q.y(), q.z());
    rotation.Normalize();
    chain.emplace_back(ignition::math::Vector3d(p.x(),p.y(),p.z()), rotation);
    const auto separator = scope.rfind("::");
    scope = separator == std::string::npos ? "" : scope.substr(0, separator);
  }
  world = ignition::math::Pose3d::Zero;
  for (auto it = chain.rbegin(); it != chain.rend(); ++it) world = world * *it;
  return !chain.empty();
}
