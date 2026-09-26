#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <utility>

namespace fr10_motion {

// Single-writer, multi-reader latest-value buffer. Readers pin the active
// slot while copying it; the writer never waits and drops an update only when
// the inactive slot is still pinned by a slow reader. No heap allocation or
// mutex acquisition occurs in read() or write().
template <typename T> class RealtimeBuffer {
public:
  RealtimeBuffer() = default;
  RealtimeBuffer(const RealtimeBuffer &) = delete;
  RealtimeBuffer &operator=(const RealtimeBuffer &) = delete;

  bool write(const T &value) { return writeImpl(value); }
  bool write(T &&value) { return writeImpl(std::move(value)); }

  bool read(T &value) const {
    if (!valid_.load(std::memory_order_acquire)) {
      return false;
    }
    for (std::size_t attempt = 0; attempt < 4; ++attempt) {
      const std::size_t index = active_.load(std::memory_order_seq_cst);
      readers_[index].fetch_add(1, std::memory_order_seq_cst);
      if (index != active_.load(std::memory_order_seq_cst)) {
        readers_[index].fetch_sub(1, std::memory_order_seq_cst);
        continue;
      }
      value = slots_[index];
      readers_[index].fetch_sub(1, std::memory_order_seq_cst);
      return true;
    }
    return false;
  }

  bool valid() const { return valid_.load(std::memory_order_acquire); }

  uint64_t droppedWrites() const {
    return dropped_writes_.load(std::memory_order_relaxed);
  }

private:
  template <typename U> bool writeImpl(U &&value) {
    const std::size_t inactive =
        1U - active_.load(std::memory_order_seq_cst);
    if (readers_[inactive].load(std::memory_order_seq_cst) != 0U) {
      dropped_writes_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    slots_[inactive] = std::forward<U>(value);
    active_.store(inactive, std::memory_order_seq_cst);
    valid_.store(true, std::memory_order_release);
    return true;
  }

  std::array<T, 2> slots_{};
  mutable std::array<std::atomic<uint32_t>, 2> readers_{};
  std::atomic<std::size_t> active_{0};
  std::atomic<bool> valid_{false};
  std::atomic<uint64_t> dropped_writes_{0};
};

} // namespace fr10_motion
