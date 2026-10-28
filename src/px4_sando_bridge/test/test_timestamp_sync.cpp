// Copyright 2026, SANDO Lyrical / PX4 integration
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <cstdint>
#include <cstdlib>

#include "px4_sando_bridge/timestamp_sync.hpp"

namespace
{

constexpr int64_t kToleranceUs = 50 * 1000;  // 50 ms

}  // namespace

TEST(TimestampSynchronizer, StartsWithoutPx4Sample)
{
  px4_sando_bridge::TimestampSynchronizer sync;

  EXPECT_FALSE(sync.hasPx4Sample());
  EXPECT_EQ(sync.px4MinusSystemUs(), 0);
  EXPECT_LE(std::llabs(sync.px4NowUs() - sync.systemTimeUs()), 1000);
  EXPECT_GT(sync.systemMinusMonotonicUs(), static_cast<int64_t>(1e12));
}

TEST(TimestampSynchronizer, FirstSampleDefinesOffset)
{
  px4_sando_bridge::TimestampSynchronizer sync;

  const int64_t system_us = static_cast<int64_t>(sync.systemTimeUs());
  const int64_t px4_us = system_us - 3'600'000'000;  // PX4 up 1 h

  sync.updatePx4Timestamp(static_cast<uint64_t>(px4_us));

  EXPECT_TRUE(sync.hasPx4Sample());
  EXPECT_NEAR(
    static_cast<double>(sync.px4NowUs() - static_cast<uint64_t>(px4_us)),
    0.0, static_cast<double>(kToleranceUs));

  const int64_t expected = static_cast<int64_t>(sync.systemTimeUs()) + px4_us -
    system_us;
  EXPECT_NEAR(
    static_cast<double>(static_cast<int64_t>(sync.px4NowUs()) - expected),
    0.0, static_cast<double>(kToleranceUs));
}

TEST(TimestampSynchronizer, ZeroTimestampIsIgnored)
{
  px4_sando_bridge::TimestampSynchronizer sync;
  sync.updatePx4Timestamp(0);
  EXPECT_FALSE(sync.hasPx4Sample());
}

TEST(TimestampSynchronizer, OffsetFilterTracksStableClock)
{
  px4_sando_bridge::TimestampSynchronizer sync;

  const int64_t base_system_us = static_cast<int64_t>(sync.systemTimeUs());
  const int64_t px4_offset_us = -7'200'000'000;  // PX4 up 2 h

  for (int i = 0; i < 200; ++i) {
    const int64_t system_us =
      static_cast<int64_t>(sync.systemTimeUs()) + i * 100;
    sync.updatePx4Timestamp(static_cast<uint64_t>(system_us + px4_offset_us));
  }

  EXPECT_LT(std::llabs(sync.lastPx4ResidualUs()), kToleranceUs);
  EXPECT_NEAR(
    static_cast<double>(
      sync.px4MinusSystemUs() - px4_offset_us),
    0.0, static_cast<double>(kToleranceUs));

  const uint64_t early = sync.toPx4TimeUs(static_cast<uint64_t>(base_system_us));
  const uint64_t late = sync.toPx4TimeUs(base_system_us + 1'000'000);
  EXPECT_GT(late, early);
}
