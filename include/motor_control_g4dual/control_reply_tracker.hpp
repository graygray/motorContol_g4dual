// Copyright 2026 Gray Lin
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

namespace motor_control_g4dual
{

// Optional diagnostics for controllers that acknowledge each 0x6040:00 write.
// The wire response has no command value or transaction ID. This requires a
// single client and ordered, nonduplicated responses; it never enables motion.
class ControlReplyTracker
{
public:
  using Clock = std::chrono::steady_clock;

  explicit ControlReplyTracker(
    bool enabled = false, std::chrono::milliseconds timeout = std::chrono::milliseconds(500))
  : enabled_(enabled), timeout_(timeout), state_(enabled ? "idle" : "unavailable") {}

  void begin(std::size_t expected, Clock::time_point now)
  {
    if (!enabled_) {
      return;
    }
    expire(now);
    // Outstanding/late replies cannot be assigned to a subsequent command.
    if (state_ == "pending") {
      uncertain_ = true;
    }
    remaining_ = expected;
    deadline_ = now + timeout_;
    state_ = uncertain_ ? "ambiguous" : "pending";
    abort_code_ = 0U;
  }

  void expire(Clock::time_point now)
  {
    if (state_ == "pending" && now >= deadline_) {
      state_ = "timed_out";
      uncertain_ = true;
    }
  }

  void observe(
    std::uint16_t index, std::uint8_t subindex, bool aborted,
    std::uint32_t code, Clock::time_point now)
  {
    expire(now);
    if (index != 0x6040U || subindex != 0U || state_ != "pending") {
      return;
    }
    if (aborted) {
      state_ = "rejected";
      abort_code_ = code;
      // Other stages in the same enable burst may still have replies in flight.
      uncertain_ = remaining_ > 1U;
    } else if (--remaining_ == 0U) {
      state_ = "acknowledged";
    }
  }

  void transmission_failed()
  {
    if (enabled_) {
      uncertain_ = uncertain_ || state_ == "pending";
      state_ = "transmission_failed";
    }
  }

  void interrupt()
  {
    if (enabled_) {
      // An internal safety command uses the same object as user control events.
      uncertain_ = true;
      state_ = "ambiguous";
    }
  }

  const std::string & state() const {return state_;}
  std::size_t remaining() const {return remaining_;}
  std::uint32_t abort_code() const {return abort_code_;}

private:
  bool enabled_;
  std::chrono::milliseconds timeout_;
  std::string state_;
  bool uncertain_{false};
  std::size_t remaining_{0U};
  std::uint32_t abort_code_{0U};
  Clock::time_point deadline_{};
};

}  // namespace motor_control_g4dual
