// Copyright 2026 Gray Lin
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include "motor_control_g4dual/control_reply_tracker.hpp"

namespace motor_control_g4dual
{
namespace
{
using namespace std::chrono_literals;

TEST(ControlReplyTracker, DefaultsToUnavailableForUnacknowledgedProtocol)
{
  ControlReplyTracker tracker;
  const auto now = ControlReplyTracker::Clock::now();
  tracker.begin(3U, now);
  tracker.observe(0x6040U, 0U, false, 0U, now);
  tracker.expire(now + 1s);
  tracker.interrupt();
  tracker.transmission_failed();
  EXPECT_EQ(tracker.state(), "unavailable");
}

TEST(ControlReplyTracker, RequiresAllEnableAcknowledgementsAndMatchingObject)
{
  ControlReplyTracker tracker(true, 500ms);
  const auto now = ControlReplyTracker::Clock::now();
  tracker.begin(3U, now);
  tracker.observe(0x6060U, 0U, false, 0U, now);
  tracker.observe(0x6040U, 1U, false, 0U, now);
  EXPECT_EQ(tracker.remaining(), 3U);
  tracker.observe(0x6040U, 0U, false, 0U, now);
  tracker.observe(0x6040U, 0U, false, 0U, now);
  EXPECT_EQ(tracker.state(), "pending");
  tracker.observe(0x6040U, 0U, false, 0U, now);
  EXPECT_EQ(tracker.state(), "acknowledged");
  tracker.begin(1U, now + 10ms);
  tracker.observe(0x6040U, 0U, false, 0U, now + 20ms);
  EXPECT_EQ(tracker.state(), "acknowledged");
}

TEST(ControlReplyTracker, PreservesAbortAndDoesNotConfirmRemainingEnableStages)
{
  ControlReplyTracker tracker(true, 500ms);
  const auto now = ControlReplyTracker::Clock::now();
  tracker.begin(3U, now);
  tracker.observe(0x6040U, 0U, true, 0x06020000U, now);
  tracker.observe(0x6040U, 0U, false, 0U, now);
  EXPECT_EQ(tracker.state(), "rejected");
  EXPECT_EQ(tracker.abort_code(), 0x06020000U);
  tracker.begin(1U, now + 1ms);
  EXPECT_EQ(tracker.state(), "ambiguous");
}

TEST(ControlReplyTracker, TimesOutAndNeverAssignsLateReplyToNewCommand)
{
  ControlReplyTracker tracker(true, 500ms);
  const auto now = ControlReplyTracker::Clock::now();
  tracker.begin(1U, now);
  tracker.expire(now + 499ms);
  EXPECT_EQ(tracker.state(), "pending");
  tracker.observe(0x6040U, 0U, false, 0U, now + 500ms);
  EXPECT_EQ(tracker.state(), "timed_out");
  tracker.begin(1U, now + 501ms);
  tracker.observe(0x6040U, 0U, false, 0U, now + 502ms);
  EXPECT_EQ(tracker.state(), "ambiguous");
}

TEST(ControlReplyTracker, OverlappingCommandsRemainAmbiguous)
{
  ControlReplyTracker tracker(true, 500ms);
  const auto now = ControlReplyTracker::Clock::now();
  tracker.begin(3U, now);
  tracker.begin(1U, now + 1ms);
  tracker.observe(0x6040U, 0U, false, 0U, now + 2ms);
  EXPECT_EQ(tracker.state(), "ambiguous");
  tracker.begin(1U, now + 1s);
  EXPECT_EQ(tracker.state(), "ambiguous");
}

TEST(ControlReplyTracker, FailedTransmissionAndSafetyStopInvalidateCorrelation)
{
  const auto now = ControlReplyTracker::Clock::now();
  for (const bool safety_stop : {false, true}) {
    ControlReplyTracker tracker(true, 500ms);
    tracker.begin(3U, now);
    if (safety_stop) {
      tracker.interrupt();
      EXPECT_EQ(tracker.state(), "ambiguous");
    } else {
      tracker.transmission_failed();
      EXPECT_EQ(tracker.state(), "transmission_failed");
    }
    tracker.begin(1U, now + 1ms);
    tracker.observe(0x6040U, 0U, false, 0U, now + 2ms);
    EXPECT_EQ(tracker.state(), "ambiguous");
  }
}

}  // namespace
}  // namespace motor_control_g4dual
