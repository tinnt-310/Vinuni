#include "fr10_motion_core/motion_coordinator.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "fr10_motion_core/math_utils.hpp"

namespace fr10_motion {
MotionCoordinator::MotionCoordinator(double jt, double pt, double at)
    : joint_tolerance_(jt), position_tolerance_(pt), angular_tolerance_(at) {
  if (!std::isfinite(jt) || !std::isfinite(pt) || !std::isfinite(at) ||
      jt < 0 || pt < 0 || at < 0) throw std::invalid_argument("invalid command tolerances");
}
bool MotionCoordinator::equivalent(const CommandIntent &a, const CommandIntent &b) const {
  if (a.mode != b.mode || a.policy != b.policy || a.session_id != b.session_id) return false;
  if (a.mode == ControlMode::JOINT) {
    for (std::size_t i = 0; i < kJointCount; ++i) {
      if (!std::isfinite(a.joint.position_rad[i]) ||
          std::abs(a.joint.position_rad[i] - b.joint.position_rad[i]) > joint_tolerance_)
        return false;
    }
    return a.joint.velocity_scale == b.joint.velocity_scale &&
           a.joint.acceleration_scale == b.joint.acceleration_scale &&
           a.joint.jerk_scale == b.joint.jerk_scale;
  }
  double squared = 0;
  for (std::size_t i = 0; i < 3; ++i) {
    const double delta = a.pose.position_m[i] - b.pose.position_m[i];
    if (!std::isfinite(delta)) return false;
    squared += delta * delta;
  }
  return std::sqrt(squared) <= position_tolerance_ &&
      math::quaternionAngularDistance(a.pose.orientation_quat, b.pose.orientation_quat) <= angular_tolerance_ &&
      a.pose.linear_speed_scale == b.pose.linear_speed_scale &&
      a.pose.angular_speed_scale == b.pose.angular_speed_scale;
}
void MotionCoordinator::emit(const CommandReport &report) {
  if (count_ == events_.size()) {
    read_ = (read_ + 1) % events_.size(); --count_; ++dropped_;
  }
  events_[(read_ + count_) % events_.size()] = report; ++count_;
}
CommandDecision MotionCoordinator::request(const CommandIntent &intent, uint64_t scene,
                                           uint64_t boundary, bool busy) {
  std::lock_guard<std::mutex> lock(mutex_);
  CommandReport incoming{++next_id_, generation_, scene, boundary, intent.mode};
  if (busy) {
    incoming.phase = CommandPhase::REJECTED; incoming.reason = CommandReason::BUSY;
    emit(incoming); return {incoming, false, false};
  }
  if (intent_ && current_ && equivalent(intent, *intent_) &&
      current_->phase != CommandPhase::STOPPED && current_->phase != CommandPhase::STOPPING &&
      current_->phase != CommandPhase::REJECTED) {
    incoming.generation = current_->generation;
    incoming.phase = CommandPhase::COALESCED; incoming.reason = CommandReason::DUPLICATE;
    emit(incoming); return {incoming, false, true};
  }
  if (current_ && current_->phase != CommandPhase::SUCCEEDED &&
      current_->phase != CommandPhase::REJECTED && current_->phase != CommandPhase::STOPPED) {
    auto prior = *current_; prior.phase = CommandPhase::SUPERSEDED; emit(prior);
  }
  incoming.generation = ++generation_;
  intent_ = intent; current_ = incoming; emit(incoming);
  return {incoming, true, false};
}
CommandReport MotionCoordinator::reject(ControlMode mode, CommandReason reason) {
  std::lock_guard<std::mutex> lock(mutex_);
  CommandReport report{++next_id_, generation_, 0, 0, mode, CommandPhase::REJECTED, reason};
  emit(report); return report;
}
bool MotionCoordinator::transition(uint64_t id, CommandPhase phase, CommandReason reason) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!current_ || current_->id != id || current_->phase == CommandPhase::STOPPED ||
      current_->phase == CommandPhase::REJECTED) return false;
  if (current_->phase == phase && current_->reason == reason) return true;
  current_->phase = phase; current_->reason = reason; emit(*current_);
  if (phase == CommandPhase::STOPPED || phase == CommandPhase::REJECTED) intent_.reset();
  return true;
}
bool MotionCoordinator::owns(uint64_t id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return current_ && current_->id == id && current_->phase != CommandPhase::STOPPED &&
         current_->phase != CommandPhase::REJECTED;
}
void MotionCoordinator::stop(CommandReason reason, bool retain) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (current_) { current_->phase = CommandPhase::STOPPING; current_->reason = reason; emit(*current_); }
  if (!retain) intent_.reset();
}
std::optional<CommandReport> MotionCoordinator::current() const {
  std::lock_guard<std::mutex> lock(mutex_); return current_;
}
std::optional<CommandReport> MotionCoordinator::nextEvent() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!count_) return std::nullopt;
  const auto event = events_[read_]; read_ = (read_ + 1) % events_.size(); --count_;
  return event;
}
uint64_t MotionCoordinator::droppedEvents() const {
  std::lock_guard<std::mutex> lock(mutex_); return dropped_;
}
} // namespace fr10_motion
