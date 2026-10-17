#pragma once

#include <cstdint>
#include <limits>

namespace swerve_vehicle_control {

struct ParkingRequest {
  uint64_t owner_id{0}, sequence{0};
  int64_t receipt_ns{0}, duration_ns{0};
  bool release{false};
};

// Owned exclusively by the actuator update. Expiry does not start the vehicle:
// loss of the client leaves a latched zero-drive hold until authorized release.
class ParkingLease {
public:
  bool apply(const ParkingRequest &request, int64_t now_ns) noexcept {
    if (request.owner_id == 0 || request.sequence == 0 ||
        request.receipt_ns <= 0 || now_ns < request.receipt_ns ||
        (!request.release && (request.duration_ns <= 0 ||
         request.duration_ns > 1000000000 ||
         request.receipt_ns > std::numeric_limits<int64_t>::max()-request.duration_ns)))
      return false;
    if (owner_id_ != 0 && owner_id_ != request.owner_id) return false;
    if (last_owner_id_ == request.owner_id && request.sequence <= sequence_) return false;
    if (request.release && owner_id_ != request.owner_id) return false;
    last_owner_id_ = request.owner_id;
    sequence_ = request.sequence;
    if (request.release) {
      owner_id_ = 0;
      expiry_ns_ = 0;
      release_ns_ = now_ns;
    } else {
      owner_id_ = request.owner_id;
      expiry_ns_ = request.receipt_ns + request.duration_ns;
    }
    return true;
  }
  bool inhibited() const noexcept { return owner_id_ != 0; }
  bool expired(int64_t now_ns) const noexcept {
    return inhibited() && now_ns >= expiry_ns_;
  }
  bool commandAllowed(int64_t command_receipt_ns) const noexcept {
    return !inhibited() && command_receipt_ns > release_ns_;
  }
  uint64_t owner() const noexcept { return owner_id_; }
  uint64_t sequence() const noexcept { return sequence_; }

private:
  uint64_t owner_id_{0}, last_owner_id_{0}, sequence_{0};
  int64_t expiry_ns_{0}, release_ns_{0};
};

} // namespace swerve_vehicle_control
