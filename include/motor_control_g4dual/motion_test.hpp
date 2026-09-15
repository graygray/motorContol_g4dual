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
    double circle_radius{1.0}, square_side{1.0}, circle_timeout{120.0};
    double s_length{4.0}, s_radius{1.0};
    double s_heading_gain{1.0}, s_lateral_gain{1.0};
    bool clockwise{false};
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
      c.settle_time, c.stop_rpm, c.segment_timeout, c.stall_timeout, c.feedback_timeout,
      c.circle_radius, c.square_side, c.circle_timeout, c.s_length, c.s_radius})
    {
      if (!std::isfinite(v) || v <= 0.0) {
        throw std::invalid_argument("motion_test parameters must be finite and positive");
      }
    }
    if (!std::isfinite(c.s_heading_gain) || !std::isfinite(c.s_lateral_gain) ||
      c.s_heading_gain < 0.0 || c.s_lateral_gain < 0.0 || c.s_length <= c.distance_tolerance)
    {
      throw std::invalid_argument("Invalid S length or correction gain");
    }
    if (c.repetitions < 1 || c.repetitions > 100 || c.distance <= c.distance_tolerance ||
      c.angle <= c.angle_tolerance || c.segment_timeout <= c.settle_time ||
      c.stall_timeout >= c.segment_timeout || c.stall_timeout >= c.circle_timeout ||
      c.square_side <= c.distance_tolerance || c.angle_tolerance >= 1.5707963267948966)
    {
      throw std::invalid_argument("Invalid motion_test repetitions, tolerance or timeout");
    }
  }

  static bool supported(const std::string & kind)
  {
    return kind == "straight" || kind == "rotate" || kind == "circle" || kind == "square" ||
           kind == "s";
  }
  static double maximum_wheel_speed(const std::string & kind, const Config & c, double separation)
  {
    if (kind == "s") {
      return std::min(c.linear_speed, c.angular_speed * c.s_radius) +
             c.angular_speed * separation / 2.0;
    }
    if (kind == "circle") {
      return std::min(c.angular_speed, c.linear_speed / c.circle_radius) *
             (c.circle_radius + separation / 2.0);
    }
    if (kind == "square") {
      return std::max(c.linear_speed, c.angular_speed * separation / 2.0);
    }
    return kind == "straight" ? c.linear_speed : c.angular_speed * separation / 2.0;
  }

  void start(const std::string & kind, const Config & config, const Sample & s)
  {
    validate(config);
    if (active() || !supported(kind)) {
      throw std::invalid_argument("Test is active or pattern is unsupported");
    }
    config_ = config;
    kind_ = kind;
    state_ = "settling";
    reason_ = "waiting for initial standstill";
    segment_ = 0;
    initial_ = origin_ = s;
    phase_started_ = last_time_ = last_progress_time_ = s.time;
    still_since_ = -1.0;
    speed_ = best_progress_ = last_segment_progress_ = last_segment_target_ = 0.0;
    last_segment_type_ = "none";
    s_distance_ = s_reference_x_ = s_reference_y_ = s_angular_ = 0.0;
    s_last_pose_ = s;
    s_finishing_ = false;
  }

  bool active() const {return state_ == "running" || state_ == "settling";}
  const std::string & state() const {return state_;}
  const std::string & reason() const {return reason_;}
  const std::string & kind() const {return kind_;}
  int segment() const {return segment_;}
  double last_segment_progress() const {return last_segment_progress_;}
  std::string segment_type() const
  {
    if (segment_ == 0) {return "standstill";}
    if (kind_ == "circle") {return "arc";}
    if (kind_ == "s") {return "s_curve";}
    return angular_segment() ? "turn" : "line";
  }
  const std::string & last_segment_type() const {return last_segment_type_;}
  double last_segment_target() const {return last_segment_target_;}
  double segment_target() const
  {
    if (kind_ == "circle") {return 6.28318530717958647692;}
    if (kind_ == "s") {return config_.s_length;}
    if (kind_ == "square") {
      return angular_segment() ? 1.5707963267948966 : config_.square_side;
    }
    return angular_segment() ? config_.angle : config_.distance;
  }
  const char * progress_unit() const {return angular_segment() ? "rad" : "m";}
  double reference_heading() const
  {
    if (kind_ != "s") {return origin_.yaw;}
    return origin_.yaw + s_heading_at(s_distance_);
  }
  double heading_error(const Sample & s) const
  {
    return std::atan2(std::sin(reference_heading() - s.yaw),
      std::cos(reference_heading() - s.yaw));
  }
  double path_error(const Sample & s) const
  {
    const double dx = s.x - origin_.x, dy = s.y - origin_.y;
    if (kind_ == "s") {
      const double local_x = dx * std::cos(origin_.yaw) + dy * std::sin(origin_.yaw);
      const double local_y = -dx * std::sin(origin_.yaw) + dy * std::cos(origin_.yaw);
      const double heading = s_heading_at(s_distance_);
      return -(local_x - s_reference_x_) * std::sin(heading) +
             (local_y - s_reference_y_) * std::cos(heading);
    }
    if (kind_ == "circle") {
      const double cx = -direction() * config_.circle_radius * std::sin(origin_.yaw);
      const double cy = direction() * config_.circle_radius * std::cos(origin_.yaw);
      return std::hypot(dx - cx, dy - cy) - config_.circle_radius;
    }
    return angular_segment() ? std::hypot(dx, dy) :
           -dx * std::sin(origin_.yaw) + dy * std::cos(origin_.yaw);
  }
  const Sample & initial() const {return initial_;}
  double progress(const Sample & s) const
  {
    if (kind_ == "s") {return s_distance_;}
    const double delta = angular_segment() ? s.yaw - origin_.yaw :
      (s.x - origin_.x) * std::cos(origin_.yaw) +
      (s.y - origin_.y) * std::sin(origin_.yaw);
    return direction() * delta;
  }
  void abort(const std::string & reason)
  {
    if (active()) {state_ = "aborted"; reason_ = reason; speed_ = s_angular_ = 0.0;}
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
    const double deadline = kind_ == "circle" && state_ == "running" ?
      config_.circle_timeout : config_.segment_timeout;
    if (s.time - phase_started_ > deadline) {
      abort("segment or standstill timeout");
      return {};
    }
    if (kind_ == "s" && segment_ > 0) {update_s_progress(s);}
    if (state_ == "settling") {
      if (std::abs(s.left_rpm) <= config_.stop_rpm &&
        std::abs(s.right_rpm) <= config_.stop_rpm)
      {
        if (still_since_ < 0.0) {still_since_ = s.time;}
        if (s.time - still_since_ >= config_.settle_time) {
          if (segment_ > 0) {
            last_segment_progress_ = progress(s);
            last_segment_target_ = segment_target();
            last_segment_type_ = segment_type();
          }
          if (segment_ == segment_count()) {
            state_ = "completed";
            reason_ = "sequence completed; wheel odometry is not ground truth";
          } else {
            ++segment_;
            origin_ = s;
            s_last_pose_ = s;
            s_distance_ = s_reference_x_ = s_reference_y_ = s_angular_ = 0.0;
            s_finishing_ = false;
            phase_started_ = last_progress_time_ = s.time;
            speed_ = best_progress_ = 0.0;
            state_ = "running";
            reason_ = "executing segment";
          }
        }
      } else {still_since_ = -1.0;}
      return {};
    }
    if (kind_ == "s") {return update_s_command(s, dt);}
    const bool rotate = angular_segment();
    const double tolerance = rotate ? config_.angle_tolerance : config_.distance_tolerance;
    const double remaining = segment_target() - progress(s);
    if (remaining <= tolerance) {
      const double accel = segment_acceleration();
      speed_ = std::max(0.0, speed_ - accel * dt);
      if (speed_ > 0.0) {
        return velocity_command();
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
    const double accel = segment_acceleration();
    const double target = std::min(segment_speed(),
      std::sqrt(2.0 * accel * std::max(0.0, remaining - tolerance)));
    speed_ += std::clamp(target - speed_, -accel * dt, accel * dt);
    return velocity_command();
  }

private:
  double s_heading_at(double distance) const
  {
    const double phase = 6.28318530717958647692 *
      std::clamp(distance / config_.s_length, 0.0, 1.0);
    return (config_.clockwise ? -1.0 : 1.0) * config_.s_length /
           (6.28318530717958647692 * config_.s_radius) * (1.0 - std::cos(phase));
  }
  void update_s_progress(const Sample & s)
  {
    // Signed odometry travel; reversing or spinning cannot complete the S.
    const double middle_yaw = (s.yaw + s_last_pose_.yaw) / 2.0;
    const double ds = (s.x - s_last_pose_.x) * std::cos(middle_yaw) +
      (s.y - s_last_pose_.y) * std::sin(middle_yaw);
    const int steps = static_cast<int>(std::clamp(std::ceil(std::abs(ds) / 0.01), 1.0, 128.0));
    for (int i = 0; i < steps; ++i) {
      const double heading = s_heading_at(s_distance_ + ds * (i + 0.5) / steps);
      s_reference_x_ += ds / steps * std::cos(heading);
      s_reference_y_ += ds / steps * std::sin(heading);
    }
    s_distance_ += ds;
    s_last_pose_ = s;
  }
  Command update_s_command(const Sample & s, double dt)
  {
    const double remaining = config_.s_length - s_distance_;
    if (remaining <= config_.distance_tolerance) {s_finishing_ = true;}
    if (!s_finishing_) {
      if (s_distance_ > best_progress_ + config_.distance_tolerance * 0.25) {
        best_progress_ = s_distance_;
        last_progress_time_ = s.time;
      }
      if (s.time - last_progress_time_ > config_.stall_timeout) {
        abort("no forward progress in commanded direction");
        return {};
      }
    }
    const double target_speed = s_finishing_ ? 0.0 : std::min(
      std::min(config_.linear_speed, config_.angular_speed * config_.s_radius),
      std::sqrt(2.0 * config_.linear_accel *
      std::max(0.0, remaining - config_.distance_tolerance)));
    speed_ += std::clamp(target_speed - speed_, -config_.linear_accel * dt,
      config_.linear_accel * dt);
    const double phase = 6.28318530717958647692 *
      std::clamp(s_distance_ / config_.s_length, 0.0, 1.0);
    const double curvature = (config_.clockwise ? -1.0 : 1.0) * std::sin(phase) / config_.s_radius;
    const double target_angular = s_finishing_ ? 0.0 : std::clamp(
      speed_ * curvature + config_.s_heading_gain * heading_error(s) -
      config_.s_lateral_gain * speed_ * path_error(s),
      -config_.angular_speed, config_.angular_speed);
    s_angular_ += std::clamp(target_angular - s_angular_, -config_.angular_accel * dt,
      config_.angular_accel * dt);
    if (s_finishing_ && speed_ == 0.0 && s_angular_ == 0.0) {
      state_ = "settling";
      reason_ = "waiting for standstill";
      phase_started_ = s.time;
      still_since_ = -1.0;
    }
    return {speed_, s_angular_};
  }
  int segment_count() const
  {
    return config_.repetitions * (kind_ == "square" ? 8 : ((kind_ == "circle" || kind_ == "s") ? 1 : 2));
  }
  bool angular_segment() const
  {
    return kind_ == "rotate" || kind_ == "circle" ||
           (kind_ == "square" && segment_ > 0 && segment_ % 2 == 0);
  }
  double direction() const
  {
    if (kind_ == "circle" || (kind_ == "square" && angular_segment())) {
      return config_.clockwise ? -1.0 : 1.0;
    }
    if (kind_ == "square") {return 1.0;}
    return segment_ % 2 == 1 ? 1.0 : -1.0;
  }
  double segment_speed() const
  {
    if (kind_ == "circle") {
      return std::min(config_.angular_speed, config_.linear_speed / config_.circle_radius);
    }
    return angular_segment() ? config_.angular_speed : config_.linear_speed;
  }
  double segment_acceleration() const
  {
    if (kind_ == "circle") {
      return std::min(config_.angular_accel, config_.linear_accel / config_.circle_radius);
    }
    return angular_segment() ? config_.angular_accel : config_.linear_accel;
  }
  Command velocity_command() const
  {
    if (kind_ == "circle") {return {config_.circle_radius * speed_, direction() * speed_};}
    return angular_segment() ? Command{0.0, direction() * speed_} :
           Command{direction() * speed_, 0.0};
  }
  double s_distance_{}, s_reference_x_{}, s_reference_y_{}, s_angular_{};
  bool s_finishing_{};
  Sample s_last_pose_;
  Config config_;
  Sample initial_, origin_;
  std::string state_{"idle"}, reason_, kind_;
  std::string last_segment_type_{"none"};
  double last_segment_target_{};
  int segment_{};
  double phase_started_{}, last_time_{}, still_since_{-1.0};
  double speed_{}, best_progress_{}, last_progress_time_{}, last_segment_progress_{};
};
}  // namespace motor_control_g4dual
