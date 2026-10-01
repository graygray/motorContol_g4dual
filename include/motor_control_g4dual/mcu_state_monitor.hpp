// Copyright 2026 Gray Lin
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

#include "motor_control_g4dual/motor_can_protocol.hpp"

namespace motor_control_g4dual
{

// Detects a motor-controller reset from the periodic status report (CAN 0x281).
// A reset is declared when the controller uptime goes backwards, or when the
// host believes motion is enabled for longer than a grace period but the
// controller reports that it is not in ROS-host mode (it forgot the enable).
// Controllers that never send status reports are never flagged.
class McuStateMonitor
{
public:
  using Clock = std::chrono::steady_clock;

  enum class Event
  {
    kNone,
    kFirstReport,
    kResetDetected,
    kStateMismatch,
  };

  explicit McuStateMonitor(
    std::chrono::milliseconds enable_grace = std::chrono::milliseconds(2000),
    std::chrono::milliseconds stale_timeout = std::chrono::milliseconds(3000))
  : enable_grace_(enable_grace), stale_timeout_(stale_timeout) {}

  // `motion_enabled_since` is when the host last enabled motion, if it currently
  // believes motion is enabled.
  Event observe(
    const McuStatusReport & report, Clock::time_point now,
    std::optional<Clock::time_point> motion_enabled_since)
  {
    const bool first = !supported_;
    const bool uptime_went_back = !first && report.uptime_s < last_report_.uptime_s;
    supported_ = true;
    last_seen_ = now;
    last_report_ = report;

    if (uptime_went_back) {
      ++reset_count_;
      return Event::kResetDetected;
    }
    if (motion_enabled_since &&
      now - *motion_enabled_since >= enable_grace_ &&
      (report.flags & McuStatusReport::kFlagRosHostMode) == 0U)
    {
      ++reset_count_;
      return Event::kStateMismatch;
    }
    return first ? Event::kFirstReport : Event::kNone;
  }

  // True when reports were seen before but have stopped arriving.
  bool stale(Clock::time_point now) const
  {
    return supported_ && now - last_seen_ > stale_timeout_;
  }

  bool supported() const {return supported_;}
  std::uint32_t reset_count() const {return reset_count_;}
  const McuStatusReport & last_report() const {return last_report_;}

private:
  std::chrono::milliseconds enable_grace_;
  std::chrono::milliseconds stale_timeout_;
  bool supported_{false};
  std::uint32_t reset_count_{0U};
  McuStatusReport last_report_{};
  Clock::time_point last_seen_{};
};

}  // namespace motor_control_g4dual
