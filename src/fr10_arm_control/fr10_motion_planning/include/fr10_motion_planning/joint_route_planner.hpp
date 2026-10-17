#pragma once

#include "fr10_motion_core/cartesian_route_planner.hpp"

namespace fr10_motion_planning {

// Managed-worker planner. EndpointGuard validates the insertion segment before
// any approach is selected. The core materializes every accepted route again.
class JointRoutePlanner {
public:
  using EndpointGuard = std::function<bool(const fr10_motion::KinematicState &,
      const fr10_motion::CartesianRoutePlanner::CancelCheck &)>;
  static fr10_motion::CartesianRoutePlan plan(
      const fr10_motion::Pose &goal, const fr10_motion::KinematicState &seed,
      fr10_motion::IKProvider &provider,
      const fr10_motion::CartesianRoutePlannerConfig &config,
      const EndpointGuard &endpoint_guard = {},
      const fr10_motion::CartesianRoutePlanner::CancelCheck &cancel = {},
      double search_timeout_s = 5.0);
};

} // namespace fr10_motion_planning
