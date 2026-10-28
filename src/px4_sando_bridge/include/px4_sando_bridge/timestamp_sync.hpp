// Copyright 2026, SANDO Lyrical / PX4 integration
// SPDX-License-Identifier: MIT

#ifndef PX4_SANDO_BRIDGE__TIMESTAMP_SYNC_HPP_
#define PX4_SANDO_BRIDGE__TIMESTAMP_SYNC_HPP_

#include <chrono>
#include <cstdint>
#include <mutex>

namespace px4_sando_bridge
{

/// Tracks the offset between the host monotonic clock, the ROS/system clock,
/// and the PX4 autopilot clock observed in incoming uORB messages.
class TimestampSynchronizer
{
public:
  using Clock = std::chrono::steady_clock;

  TimestampSynchronizer();

  /// Sample the system and monotonic clocks. Called periodically.
  void updateClocks();

  /// Observe a PX4 uORB timestamp (microseconds since autopilot boot).
  void updatePx4Timestamp(uint64_t px4_timestamp_us);

  /// System clock in microseconds.
  uint64_t systemTimeUs() const;

  /// Monotonic clock in microseconds.
  uint64_t monotonicTimeUs() const;

  /// system_now - monotonic_now, in microseconds.
  int64_t systemMinusMonotonicUs() const;

  /// system_now - px4_boot_us. Zero until a PX4 timestamp is observed.
  int64_t px4MinusSystemUs() const;

  /// Convert a system timestamp to the estimated PX4 time base.
  uint64_t toPx4TimeUs(uint64_t system_time_us) const;

  /// Current time in the estimated PX4 time base.
  uint64_t px4NowUs() const;

  bool hasPx4Sample() const;

  /// Recent filter residual magnitude, used to log clock jitter.
  int64_t lastPx4ResidualUs() const;

private:
  mutable std::mutex mutex_;

  int64_t system_minus_monotonic_us_{0};
  bool have_clock_sample_{false};

  int64_t px4_minus_system_us_{0};
  bool have_px4_sample_{false};
  int64_t last_px4_residual_us_{0};
};

}  // namespace px4_sando_bridge

#endif  // PX4_SANDO_BRIDGE__TIMESTAMP_SYNC_HPP_
