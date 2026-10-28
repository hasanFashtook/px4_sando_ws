// Copyright 2026, SANDO Lyrical / PX4 integration
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "px4_sando_bridge/watchdog.hpp"

using namespace std::chrono_literals;

TEST(Watchdog, NeverFedIsExpired)
{
  px4_sando_bridge::Watchdog watchdog(250ms);

  EXPECT_FALSE(watchdog.hasBeenFed());
  EXPECT_TRUE(watchdog.expired());
  EXPECT_EQ(watchdog.timeout(), 250ms);
  EXPECT_EQ(watchdog.timeSinceLastFeed(), 250ms);
}

TEST(Watchdog, FreshFeedClearsExpiry)
{
  px4_sando_bridge::Watchdog watchdog(250ms);

  watchdog.feed();
  EXPECT_TRUE(watchdog.hasBeenFed());
  EXPECT_FALSE(watchdog.expired());
  EXPECT_LT(watchdog.timeSinceLastFeed(), 250ms);
}

TEST(Watchdog, ExpiresAfterTimeout)
{
  px4_sando_bridge::Watchdog watchdog(50ms);

  watchdog.feed();
  EXPECT_FALSE(watchdog.expired());

  std::this_thread::sleep_for(200ms);
  EXPECT_TRUE(watchdog.expired());
  EXPECT_GE(watchdog.timeSinceLastFeed(), 50ms);
}

TEST(Watchdog, ResetForgetsFeed)
{
  px4_sando_bridge::Watchdog watchdog(250ms);

  watchdog.feed();
  watchdog.reset();

  EXPECT_FALSE(watchdog.hasBeenFed());
  EXPECT_TRUE(watchdog.expired());
}

TEST(Watchdog, TimeoutClampedAndAdjustable)
{
  px4_sando_bridge::Watchdog watchdog(0ms);
  EXPECT_GE(watchdog.timeout().count(), 1);

  watchdog.setTimeout(500ms);
  EXPECT_EQ(watchdog.timeout(), 500ms);

  watchdog.feed();
  std::this_thread::sleep_for(120ms);
  EXPECT_FALSE(watchdog.expired());

  watchdog.setTimeout(20ms);
  EXPECT_TRUE(watchdog.expired());
}
