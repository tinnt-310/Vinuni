#ifndef SWERVE_VEHICLE_CONTROL__TEMPORAL_VALIDITY_HPP_
#define SWERVE_VEHICLE_CONTROL__TEMPORAL_VALIDITY_HPP_

#include <atomic>
#include <cstdint>
#include <string>

namespace swerve_vehicle_control
{

inline int64_t source_stamp_ns(int32_t seconds, uint32_t nanoseconds) noexcept
{
  if (seconds < 0 || nanoseconds >= 1'000'000'000U) return 0;
  return static_cast<int64_t>(seconds) * 1'000'000'000LL + nanoseconds;
}

// The message has no publisher epoch or sequence number. A clock rewind is
// therefore ambiguous with a delayed pre-reset command at the same source
// stamp. Latch stop until the controller is reactivated and its input is
// explicitly cleared; do not silently re-arm when simulation time catches up.
class CommandTimeGate
{
public:
  void reset() noexcept
  {
    last_source_ns_.store(0, std::memory_order_release);
    paused_source_floor_ns_.store(0, std::memory_order_release);
    last_update_ns_.store(0, std::memory_order_release);
    last_update_steady_ns_.store(0, std::memory_order_release);
    reset_latched_.store(false, std::memory_order_release);
  }

  bool observe_update(int64_t now_ns, int64_t steady_ns,
                      int64_t timeout_ns) noexcept
  {
    if (now_ns <= 0 || steady_ns <= 0 || timeout_ns <= 0) return false;
    const int64_t previous = last_update_ns_.exchange(now_ns, std::memory_order_acq_rel);
    const int64_t previous_steady =
      last_update_steady_ns_.exchange(steady_ns, std::memory_order_acq_rel);
    if ((previous > 0 && now_ns < previous) ||
        (previous_steady > 0 && steady_ns < previous_steady)) {
      reset_latched_.store(true, std::memory_order_release);
    }
    const bool paused = previous > 0 && now_ns == previous;
    const bool wall_gap = previous_steady > 0 && steady_ns >= previous_steady &&
      steady_ns - previous_steady > timeout_ns;
    if (paused || wall_gap) {
      // With Gazebo paused, update() may stop entirely. The existing command
      // timeout also bounds the wall gap. Invalidate the current source epoch
      // on the first resumed tick and require a later-stamped command.
      paused_source_floor_ns_.store(now_ns, std::memory_order_release);
    }
    return !paused && !wall_gap &&
           !reset_latched_.load(std::memory_order_acquire);
  }

  bool accept(const std::string & frame, int64_t source_ns,
              int64_t receive_ns, int64_t timeout_ns) noexcept
  {
    if (reset_latched_.load(std::memory_order_acquire) || frame != "base_footprint" ||
        source_ns <= paused_source_floor_ns_.load(std::memory_order_acquire) ||
        !age_valid(source_ns, receive_ns, timeout_ns)) return false;
    int64_t previous = last_source_ns_.load(std::memory_order_acquire);
    while (source_ns > previous) {
      if (last_source_ns_.compare_exchange_weak(previous, source_ns,
            std::memory_order_acq_rel, std::memory_order_acquire)) {
        return !reset_latched_.load(std::memory_order_acquire);
      }
    }
    return false;
  }

  bool usable(int64_t source_ns, int64_t receive_ns,
              int64_t now_ns, int64_t timeout_ns) const noexcept
  {
    return !reset_latched_.load(std::memory_order_acquire) &&
           source_ns > paused_source_floor_ns_.load(std::memory_order_acquire) &&
           age_valid(source_ns, now_ns, timeout_ns) &&
           age_valid(receive_ns, now_ns, timeout_ns);
  }

  bool reset_latched() const noexcept
  {
    return reset_latched_.load(std::memory_order_acquire);
  }

private:
  static bool age_valid(int64_t event_ns, int64_t now_ns, int64_t timeout_ns) noexcept
  {
    return event_ns > 0 && now_ns > 0 && timeout_ns > 0 &&
           event_ns <= now_ns && now_ns - event_ns <= timeout_ns;
  }

  std::atomic<int64_t> last_source_ns_{0};
  std::atomic<int64_t> paused_source_floor_ns_{0};
  std::atomic<int64_t> last_update_ns_{0};
  std::atomic<int64_t> last_update_steady_ns_{0};
  std::atomic<bool> reset_latched_{false};
};

// A wall timer may publish only a new controller-update snapshot. The
// timestamp is the update's source time, never the timer's wall time.
class OdomSampleGate
{
public:
  void reset() noexcept
  {
    last_sequence_.store(0, std::memory_order_release);
    last_stamp_ns_.store(0, std::memory_order_release);
  }

  bool should_publish(uint64_t sequence, int64_t stamp_ns) noexcept
  {
    if (sequence == 0 || stamp_ns <= 0 ||
        sequence <= last_sequence_.load(std::memory_order_acquire) ||
        stamp_ns <= last_stamp_ns_.load(std::memory_order_acquire)) return false;
    last_stamp_ns_.store(stamp_ns, std::memory_order_release);
    last_sequence_.store(sequence, std::memory_order_release);
    return true;
  }

private:
  std::atomic<uint64_t> last_sequence_{0};
  std::atomic<int64_t> last_stamp_ns_{0};
};

}  // namespace swerve_vehicle_control

#endif  // SWERVE_VEHICLE_CONTROL__TEMPORAL_VALIDITY_HPP_
