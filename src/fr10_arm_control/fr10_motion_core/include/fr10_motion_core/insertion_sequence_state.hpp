#pragma once

#include <cmath>
#include <cstdint>
#include <mutex>
#include <optional>

#include "fr10_motion_core/core_types.hpp"
#include "fr10_motion_core/math_utils.hpp"

namespace fr10_motion {

struct PendingInsertionSequence {
  CartesianTarget insertion_target;
  Pose pre_insert_pose;
  uint64_t staging_generation{0};
};

// Owns only the pending/retained insertion targets. Geometry and planning
// remain with their dedicated planners; this state store has no ROS types.
class InsertionSequenceState {
public:
  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.reset();
    last_target_.reset();
  }

  void rejectStaging(uint64_t generation) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_ && pending_->staging_generation == generation) {
      pending_.reset();
    }
  }

  void markDirect(const CartesianTarget &target) {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.reset();
    last_target_ = target;
  }

  bool isDuplicate(const CartesianTarget &target, double position_tolerance_m,
                   double angular_tolerance_rad) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const CartesianTarget *reference = pending_
                                           ? &pending_->insertion_target
                                           : (last_target_ ? &*last_target_ : nullptr);
    if (!reference) {
      return false;
    }
    double squared_change = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const double delta = target.position_m[axis] - reference->position_m[axis];
      squared_change += delta * delta;
    }
    const double angular_change = math::quaternionAngularDistance(
        target.orientation_quat, reference->orientation_quat);
    return std::sqrt(squared_change) <= position_tolerance_m &&
           angular_change <= angular_tolerance_rad;
  }

  void arm(const CartesianTarget &target, const Pose &pre_insert_pose,
           uint64_t staging_generation) {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_ = PendingInsertionSequence{target, pre_insert_pose,
                                        staging_generation};
  }

  std::optional<PendingInsertionSequence>
  pendingForGeneration(uint64_t generation) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_ || pending_->staging_generation != generation) {
      return std::nullopt;
    }
    return pending_;
  }

  bool takeIfGeneration(uint64_t generation) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_ || pending_->staging_generation != generation) {
      return false;
    }
    pending_.reset();
    return true;
  }

  void retain(const CartesianTarget &target) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_target_ = target;
  }

  std::optional<CartesianTarget> beginRetract() {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.reset();
    return last_target_;
  }

private:
  mutable std::mutex mutex_;
  std::optional<PendingInsertionSequence> pending_;
  std::optional<CartesianTarget> last_target_;
};

} // namespace fr10_motion
