// Copyright 2026, SANDO Lyrical / PX4 integration
// SPDX-License-Identifier: MIT

#include "px4_sando_bridge/timestamp_sync.hpp"

#include <algorithm>
#include <cmath>

namespace px4_sando_bridge
{
namespace
{

constexpr double kOffsetFilterAlpha = 0.05;
constexpr int64_t kClockStepThresholdUs = 1000000;  // 1 s

int64_t systemNowUs()
{
  return std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

int64_t monotonicNowUs()
{
  return std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

TimestampSynchronizer::TimestampSynchronizer()
{
  updateClocks();
}

void TimestampSynchronizer::updateClocks()
{
  const int64_t system_us = systemNowUs();
  const int64_t monotonic_us = monotonicNowUs();
  const int64_t observed_offset = system_us - monotonic_us;

  std::lock_guard<std::mutex> lock(mutex_);
  if (!have_clock_sample_) {
    system_minus_monotonic_us_ = observed_offset;
    have_clock_sample_ = true;
    return;
  }

  const int64_t residual = observed_offset - system_minus_monotonic_us_;
  if (std::llabs(residual) > kClockStepThresholdUs) {
    system_minus_monotonic_us_ = observed_offset;
  } else {
    system_minus_monotonic_us_ += static_cast<int64_t>(
      std::llround(kOffsetFilterAlpha * static_cast<double>(residual)));
  }
}

void TimestampSynchronizer::updatePx4Timestamp(uint64_t px4_timestamp_us)
{
  if (px4_timestamp_us == 0) {
    return;
  }

  const int64_t system_us = systemNowUs();
  const int64_t observed_offset =
    static_cast<int64_t>(px4_timestamp_us) - system_us;

  std::lock_guard<std::mutex> lock(mutex_);
  if (!have_px4_sample_) {
    px4_minus_system_us_ = observed_offset;
    have_px4_sample_ = true;
    last_px4_residual_us_ = 0;
    return;
  }

  last_px4_residual_us_ = observed_offset - px4_minus_system_us_;
  if (std::llabs(last_px4_residual_us_) > kClockStepThresholdUs) {
    px4_minus_system_us_ = observed_offset;
  } else {
    px4_minus_system_us_ += static_cast<int64_t>(
      std::llround(kOffsetFilterAlpha * static_cast<double>(last_px4_residual_us_)));
  }
}

uint64_t TimestampSynchronizer::systemTimeUs() const
{
  return static_cast<uint64_t>(std::max<int64_t>(0, systemNowUs()));
}

uint64_t TimestampSynchronizer::monotonicTimeUs() const
{
  return static_cast<uint64_t>(std::max<int64_t>(0, monotonicNowUs()));
}

int64_t TimestampSynchronizer::systemMinusMonotonicUs() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return system_minus_monotonic_us_;
}

int64_t TimestampSynchronizer::px4MinusSystemUs() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return px4_minus_system_us_;
}

uint64_t TimestampSynchronizer::toPx4TimeUs(uint64_t system_time_us) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!have_px4_sample_) {
    return system_time_us;
  }

  const int64_t px4_us =
    static_cast<int64_t>(system_time_us) + px4_minus_system_us_;
  return static_cast<uint64_t>(std::max<int64_t>(0, px4_us));
}

uint64_t TimestampSynchronizer::px4NowUs() const
{
  return toPx4TimeUs(systemTimeUs());
}

bool TimestampSynchronizer::hasPx4Sample() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return have_px4_sample_;
}

int64_t TimestampSynchronizer::lastPx4ResidualUs() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_px4_residual_us_;
}

}  // namespace px4_sando_bridge
