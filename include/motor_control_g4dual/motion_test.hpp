// Copyright 2026 Gray Lin
// SPDX-License-Identifier: MIT
#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace motor_control_g4dual
{
// Internal state machine owned by MotorControlNode; this is not a ROS node.
// All times are monotonic seconds, yaw is continuous (not wrapped at +/- pi).
class MotionTest
{
public:
  struct Config
  {
    double distance{1.0}, angle{1.5707963267948966};
    double linear_speed{0.1}, angular_speed{0.2};
    double linear_accel{0.1}, angular_accel{0.2};
    double distance_tolerance{0.01}, angle_tolerance{0.02};
    double settle_time{0.5}, stop_rpm{0.5}, segment_timeout{60.0};
    double stall_timeout{5.0}, feedback_timeout{0.5};
    int repetitions{3};
  };
  struct Sample
  {
    double time{}, x{}, y{}, yaw{}, left_rpm{}, right_rpm{};
    double odom_age{}, speed_age{};
  };
  struct Command {double linear{}, angular{};};

  static void validate(const Config & c)
  {
    for (double v : {c.distance, c.angle, c.linear_speed, c.angular_speed,
      c.linear_accel, c.angular_accel, c.distance_tolerance, c.angle_tolerance,
      c.settle_time, c.stop_rpm, c.segment_timeout, c.stall_timeout, c.feedback_timeout})
    {
      if (!std::isfinite(v) || v <= 0.0) {
        throw std::invalid_argument("motion_test parameters must be finite and positive");
      }
    }
    if (c.repetitions < 1 || c.repetitions > 100 || c.distance <= c.distance_tolerance ||
      c.angle <= c.angle_tolerance || c.segment_timeout <= c.settle_time ||
      c.stall_timeout >= c.segment_timeout)
    {
      throw std::invalid_argument("Invalid motion_test repetitions, tolerance or timeout");
    }
  }

  void start(const std::string & kind, const Config & config, const Sample & s)
  {
    validate(config);
    if (active() || (kind != "straight" && kind != "rotate")) {
      throw std::invalid_argument("Test is active or kind is not straight/rotate");
    }
    config_ = config;
    kind_ = kind;
    state_ = "settling";
    reason_ = "waiting for initial standstill";
    segment_ = 0;
    initial_ = origin_ = s;
    phase_started_ = last_time_ = last_progress_time_ = s.time;
    still_since_ = -1.0;
    speed_ = best_progress_ = last_segment_progress_ = 0.0;
  }

  bool active() const {return state_ == "running" || state_ == "settling";}
  const std::string & state() const {return state_;}
  const std::string & reason() const {return reason_;}
  const std::string & kind() const {return kind_;}
  int segment() const {return segment_;}
  double last_segment_progress() const {return last_segment_progress_;}
  const Sample & initial() const {return initial_;}
  double progress(const Sample & s) const
  {
    const double delta = kind_ == "rotate" ? s.yaw - origin_.yaw :
      (s.x - origin_.x) * std::cos(origin_.yaw) +
      (s.y - origin_.y) * std::sin(origin_.yaw);
    return direction() * delta;
  }
  void abort(const std::string & reason)
  {
    if (active()) {state_ = "aborted"; reason_ = reason; speed_ = 0.0;}
  }
  Command update(const Sample & s)
  {
    if (!active()) {return {};}
    if (s.odom_age > config_.feedback_timeout || s.speed_age > config_.feedback_timeout) {
      abort("stale odometry or wheel speed");
      return {};
    }
    const double dt = s.time - last_time_;
    last_time_ = s.time;
    if (dt < 0.0 || dt > config_.feedback_timeout) {
      abort("control loop deadline missed");
      return {};
    }
    if (s.time - phase_started_ > config_.segment_timeout) {
      abort("segment or standstill timeout");
      return {};
    }
    if (state_ == "settling") {
      if (std::abs(s.left_rpm) <= config_.stop_rpm &&
        std::abs(s.right_rpm) <= config_.stop_rpm)
      {
        if (still_since_ < 0.0) {still_since_ = s.time;}
        if (s.time - still_since_ >= config_.settle_time) {
          if (segment_ > 0) {last_segment_progress_ = progress(s);}
          if (segment_ == 2 * config_.repetitions) {
            state_ = "completed";
            reason_ = "sequence completed; wheel odometry is not ground truth";
          } else {
            ++segment_;
            origin_ = s;
            phase_started_ = last_progress_time_ = s.time;
            speed_ = best_progress_ = 0.0;
            state_ = "running";
            reason_ = "executing segment";
          }
        }
      } else {still_since_ = -1.0;}
      return {};
    }
    const bool rotate = kind_ == "rotate";
    const double tolerance = rotate ? config_.angle_tolerance : config_.distance_tolerance;
    const double remaining = (rotate ? config_.angle : config_.distance) - progress(s);
    if (remaining <= tolerance) {
      const double accel = rotate ? config_.angular_accel : config_.linear_accel;
      speed_ = std::max(0.0, speed_ - accel * dt);
      if (speed_ > 0.0) {
        return rotate ? Command{0.0, direction() * speed_} :
               Command{direction() * speed_, 0.0};
      }
      state_ = "settling";
      reason_ = "waiting for standstill";
      phase_started_ = s.time;
      still_since_ = -1.0;
      speed_ = 0.0;
      return {};
    }
    if (progress(s) > best_progress_ + tolerance * 0.25) {
      best_progress_ = progress(s);
      last_progress_time_ = s.time;
    }
    if (s.time - last_progress_time_ > config_.stall_timeout) {
      abort("no forward progress in commanded direction");
      return {};
    }
    const double accel = rotate ? config_.angular_accel : config_.linear_accel;
    const double target = std::min(rotate ? config_.angular_speed : config_.linear_speed,
      std::sqrt(2.0 * accel * std::max(0.0, remaining - tolerance)));
    speed_ += std::clamp(target - speed_, -accel * dt, accel * dt);
    return rotate ? Command{0.0, direction() * speed_} :
           Command{direction() * speed_, 0.0};
  }

private:
  double direction() const {return segment_ % 2 == 1 ? 1.0 : -1.0;}
  Config config_;
  Sample initial_, origin_;
  std::string state_{"idle"}, reason_, kind_;
  int segment_{};
  double phase_started_{}, last_time_{}, still_since_{-1.0};
  double speed_{}, best_progress_{}, last_progress_time_{}, last_segment_progress_{};
};
}  // namespace motor_control_g4dual
