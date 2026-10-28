// Copyright 2026, SANDO Lyrical / PX4 integration
// SPDX-License-Identifier: MIT

#include "px4_sando_bridge/watchdog.hpp"

#include <algorithm>

namespace px4_sando_bridge
{

Watchdog::Watchdog(std::chrono::milliseconds timeout)
: timeout_(std::max(timeout, std::chrono::milliseconds(1)))
{
}

void Watchdog::feed()
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_feed_ = Clock::now();
  fed_ = true;
}

void Watchdog::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  fed_ = false;
}

bool Watchdog::expired() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!fed_) {
    return true;
  }
  return (Clock::now() - last_feed_) > timeout_;
}

bool Watchdog::hasBeenFed() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return fed_;
}

std::chrono::milliseconds Watchdog::timeout() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return timeout_;
}

void Watchdog::setTimeout(std::chrono::milliseconds timeout)
{
  std::lock_guard<std::mutex> lock(mutex_);
  timeout_ = std::max(timeout, std::chrono::milliseconds(1));
}

std::chrono::milliseconds Watchdog::timeSinceLastFeed() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!fed_) {
    return timeout_;
  }
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    Clock::now() - last_feed_);
}

}  // namespace px4_sando_bridge
