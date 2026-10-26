#pragma once

#include <array>
#include <atomic>
#include <concepts>
#include <cstddef>

namespace sim::runtime {

// Hands the latest value from one producer thread to one consumer thread without
// either ever blocking. The consumer always sees the most recently published
// value; values published in between two reads are skipped.
//
// Three slots: the producer writes the back slot, the consumer reads the front
// slot, and the middle slot is swapped atomically between them.
template <std::copyable T>
class TripleBuffer {
 public:
  // Producer only.
  void publish(const T& value) {
    slots_[back_] = value;
    const unsigned previous = middle_.exchange(back_ | fresh_bit, std::memory_order_acq_rel);
    back_ = previous & index_mask;
  }

  // Consumer only. The reference stays valid until the next call.
  [[nodiscard]] const T& latest() {
    if ((middle_.load(std::memory_order_relaxed) & fresh_bit) != 0) {
      front_ = middle_.exchange(front_, std::memory_order_acq_rel) & index_mask;
    }
    return slots_[front_];
  }

 private:
  static constexpr unsigned index_mask = 0b011;
  static constexpr unsigned fresh_bit = 0b100;  // Set when the middle slot holds an unread value.
  // Keeps the producer's and consumer's indices on separate cache lines. (Not
  // std::hardware_destructive_interference_size, whose value isn't ABI-stable.)
  static constexpr std::size_t cache_line = 64;

  std::array<T, 3> slots_{};
  alignas(cache_line) std::atomic<unsigned> middle_{1};
  alignas(cache_line) unsigned back_ = 0;  // Owned by the producer.
  alignas(cache_line) unsigned front_ = 2;  // Owned by the consumer.
};

}  // namespace sim::runtime
