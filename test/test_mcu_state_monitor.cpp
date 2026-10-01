// Copyright 2026 Gray Lin
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <chrono>
#include <optional>

#include "motor_control_g4dual/mcu_state_monitor.hpp"

namespace motor_control_g4dual
{
namespace
{
using Clock = McuStateMonitor::Clock;
using Event = McuStateMonitor::Event;

McuStatusReport report(std::uint32_t uptime_s, std::uint8_t flags)
{
  McuStatusReport value;
  value.uptime_s = uptime_s;
  value.flags = flags;
  return value;
}

TEST(McuStateMonitor, FirstReportThenNormalReports)
{
  McuStateMonitor monitor;
  const auto start = Clock::now();
  EXPECT_FALSE(monitor.supported());
  EXPECT_EQ(monitor.observe(report(100U, 0U), start, std::nullopt), Event::kFirstReport);
  EXPECT_TRUE(monitor.supported());
  EXPECT_EQ(
    monitor.observe(report(101U, 0U), start + std::chrono::seconds(1), std::nullopt),
    Event::kNone);
  EXPECT_EQ(monitor.reset_count(), 0U);
}

TEST(McuStateMonitor, DetectsUptimeGoingBackwards)
{
  McuStateMonitor monitor;
  const auto start = Clock::now();
  monitor.observe(report(5000U, McuStatusReport::kFlagRosHostMode), start, start);
  EXPECT_EQ(
    monitor.observe(report(2U, 0U), start + std::chrono::seconds(1), start),
    Event::kResetDetected);
  EXPECT_EQ(monitor.reset_count(), 1U);
}

TEST(McuStateMonitor, DetectsLostEnableOnlyAfterGracePeriod)
{
  McuStateMonitor monitor;
  const auto start = Clock::now();
  monitor.observe(report(500U, 0U), start, std::nullopt);
  // A status frame sent just before the enable was processed must not trigger.
  EXPECT_EQ(
    monitor.observe(report(501U, 0U), start + std::chrono::milliseconds(1000), start),
    Event::kNone);
  EXPECT_EQ(
    monitor.observe(report(502U, 0U), start + std::chrono::milliseconds(2500), start),
    Event::kStateMismatch);
}

TEST(McuStateMonitor, NoMismatchWhenHostModeIsSetOrMotionDisabled)
{
  McuStateMonitor monitor;
  const auto start = Clock::now();
  monitor.observe(report(10U, McuStatusReport::kFlagRosHostMode), start, std::nullopt);
  EXPECT_EQ(
    monitor.observe(
      report(11U, McuStatusReport::kFlagRosHostMode), start + std::chrono::seconds(10), start),
    Event::kNone);
  EXPECT_EQ(
    monitor.observe(report(12U, 0U), start + std::chrono::seconds(11), std::nullopt),
    Event::kNone);
}

TEST(McuStateMonitor, StaleOnlyAfterReportsWereSeen)
{
  McuStateMonitor monitor;
  const auto start = Clock::now();
  EXPECT_FALSE(monitor.stale(start + std::chrono::hours(1)));
  monitor.observe(report(1U, 0U), start, std::nullopt);
  EXPECT_FALSE(monitor.stale(start + std::chrono::seconds(2)));
  EXPECT_TRUE(monitor.stale(start + std::chrono::seconds(4)));
}

}  // namespace
}  // namespace motor_control_g4dual
