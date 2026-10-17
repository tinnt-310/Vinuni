#include <atomic>
#include <cstdint>
#include <iostream>
#include <thread>

#include "fr10_motion_core/realtime_buffer.hpp"
#include "test_support.hpp"

namespace {
struct Snapshot {
  uint64_t generation{0};
  uint64_t first{0};
  uint64_t second{0};
  uint64_t checksum{0};
};

// The destination used by the reader can deliberately pause inside its copy
// assignment. Buffer-owned slots leave pause_on_assignment=false, so writes
// remain non-blocking. This makes the pinned-slot/drop path deterministic
// instead of relying on scheduler timing.
struct BlockingSnapshot {
  uint64_t generation{0};
  bool pause_on_assignment{false};

  BlockingSnapshot() = default;
  explicit BlockingSnapshot(uint64_t value) : generation(value) {}

  BlockingSnapshot &operator=(const BlockingSnapshot &other) {
    if (pause_on_assignment) {
      reader_entered.store(true, std::memory_order_release);
      while (!release_reader.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
    }
    generation = other.generation;
    return *this;
  }

  static std::atomic<bool> reader_entered;
  static std::atomic<bool> release_reader;
};

std::atomic<bool> BlockingSnapshot::reader_entered{false};
std::atomic<bool> BlockingSnapshot::release_reader{false};
}

int main() {
  fr10_motion_test::Check check;
  fr10_motion::RealtimeBuffer<Snapshot> buffer;
  Snapshot snapshot;
  check.require(!buffer.read(snapshot), "RT_BUFFER_INITIAL_EMPTY",
                "an unwritten buffer reported valid data");
  check.require(buffer.write(Snapshot{1, 11, 17, 1 ^ 11 ^ 17}) &&
                    buffer.read(snapshot) && snapshot.generation == 1,
                "RT_BUFFER_ROUND_TRIP", "basic write/read failed");

  std::atomic<bool> done{false};
  std::atomic<uint64_t> torn_reads{0};
  Snapshot first_torn;
  std::thread reader([&]() {
    Snapshot value;
    while (!done.load(std::memory_order_acquire)) {
      if (buffer.read(value) &&
          value.checksum !=
              (value.generation ^ value.first ^ value.second)) {
        if (torn_reads.fetch_add(1, std::memory_order_relaxed) == 0) {
          first_torn = value;
        }
      }
    }
  });
  uint64_t accepted = 0;
  for (uint64_t generation = 2; generation < 200000; ++generation) {
    const Snapshot value{generation, generation * 3, generation * 7,
                         generation ^ (generation * 3) ^ (generation * 7)};
    accepted += buffer.write(value) ? 1U : 0U;
  }
  done.store(true, std::memory_order_release);
  reader.join();
  check.require(torn_reads.load() == 0 && accepted > 0,
                "RT_BUFFER_CONCURRENT_COHERENCE",
                "reader observed a torn snapshot");

  fr10_motion::RealtimeBuffer<Snapshot> consumer_fast_buffer;
  check.require(consumer_fast_buffer.write(
                    Snapshot{10, 30, 70, 10 ^ 30 ^ 70}),
                "RT_BUFFER_CONSUMER_FAST_WRITE",
                "initial latest-value write failed");
  bool repeated_latest_valid = true;
  for (std::size_t read_index = 0; read_index < 10000; ++read_index) {
    Snapshot value;
    repeated_latest_valid =
        repeated_latest_valid && consumer_fast_buffer.read(value) &&
        value.generation == 10 &&
        value.checksum == (value.generation ^ value.first ^ value.second);
  }
  check.require(repeated_latest_valid &&
                    consumer_fast_buffer.droppedWrites() == 0,
                "RT_BUFFER_CONSUMER_FAST_LATEST_STABLE",
                "fast consumer did not observe a coherent stable latest value");
  check.require(consumer_fast_buffer.write(
                    Snapshot{11, 33, 77, 11 ^ 33 ^ 77}) &&
                    consumer_fast_buffer.read(snapshot) &&
                    snapshot.generation == 11,
                "RT_BUFFER_CONSUMER_FAST_ADVANCES",
                "consumer did not advance to the next published generation");

  fr10_motion::RealtimeBuffer<BlockingSnapshot> producer_fast_buffer;
  check.require(producer_fast_buffer.write(BlockingSnapshot{1}),
                "RT_BUFFER_PRODUCER_FAST_INITIAL_WRITE",
                "initial blocking-buffer write failed");
  BlockingSnapshot::reader_entered.store(false, std::memory_order_release);
  BlockingSnapshot::release_reader.store(false, std::memory_order_release);
  BlockingSnapshot pinned_value;
  pinned_value.pause_on_assignment = true;
  bool pinned_read_succeeded = false;
  std::thread slow_reader([&]() {
    pinned_read_succeeded = producer_fast_buffer.read(pinned_value);
  });
  for (std::size_t attempt = 0;
       attempt < 1000000 &&
       !BlockingSnapshot::reader_entered.load(std::memory_order_acquire);
       ++attempt) {
    std::this_thread::yield();
  }
  const bool reader_was_pinned =
      BlockingSnapshot::reader_entered.load(std::memory_order_acquire);
  const bool next_write_accepted =
      reader_was_pinned && producer_fast_buffer.write(BlockingSnapshot{2});
  const bool pinned_slot_write_dropped =
      reader_was_pinned && !producer_fast_buffer.write(BlockingSnapshot{3});
  BlockingSnapshot::release_reader.store(true, std::memory_order_release);
  slow_reader.join();
  BlockingSnapshot latest_value;
  const bool latest_read = producer_fast_buffer.read(latest_value);
  check.require(reader_was_pinned && pinned_read_succeeded &&
                    next_write_accepted && pinned_slot_write_dropped &&
                    producer_fast_buffer.droppedWrites() == 1,
                "RT_BUFFER_PRODUCER_FAST_BOUNDED_DROP",
                "pinned inactive slot did not produce one bounded dropped write");
  check.require(latest_read && latest_value.generation == 2,
                "RT_BUFFER_DROPPED_VALUE_NOT_VISIBLE",
                "a dropped generation became visible to a consumer");
  check.require(!slow_reader.joinable(), "RT_BUFFER_DETERMINISTIC_SHUTDOWN",
                "reader thread did not join deterministically");

  std::cout << "METRIC realtime_buffer_accepted_writes=" << accepted
            << " dropped_writes=" << buffer.droppedWrites()
            << " torn_reads=" << torn_reads.load()
            << " first_torn=" << first_torn.generation << ","
            << first_torn.first << "," << first_torn.second << ","
            << first_torn.checksum
            << " producer_fast_dropped_writes="
            << producer_fast_buffer.droppedWrites()
            << " producer_fast_latest_generation=" << latest_value.generation
            << " consumer_fast_reads=10000\n";
  return check.result();
}
