#pragma once

// Temporary source-compatibility shim. New code must include the planning
// package header and use fr10_motion_planning::CollisionValidatedIKProvider.
#include "fr10_motion_planning/collision_validated_ik_provider.hpp"

namespace fr10_pose_controller {
using CollisionValidatedIKProvider =
    fr10_motion_planning::CollisionValidatedIKProvider;
} // namespace fr10_pose_controller
