// Copyright 2026 Gray Lin
// SPDX-License-Identifier: MIT

#include "motor_control_g4dual/differential_drive_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace motor_control_g4dual
{
namespace
{
constexpr double kSecondsPerMinute = 60.0;
constexpr double kTwoPi = 6.28318530717958647692;
}

DifferentialDriveKinematics::DifferentialDriveKinematics(
  double wheel_radius_m, double wheel_separation_m, double gear_ratio,
  double max_motor_speed_rpm, bool invert_left_motor, bool invert_right_motor)
: wheel_radius_m_(wheel_radius_m),
  wheel_separation_m_(wheel_separation_m),
  gear_ratio_(gear_ratio),
  max_motor_speed_rpm_(max_motor_speed_rpm),
  invert_left_motor_(invert_left_motor),
  invert_right_motor_(invert_right_motor)
{
  if (!std::isfinite(wheel_radius_m_) || !std::isfinite(wheel_separation_m_) ||
    !std::isfinite(gear_ratio_) || !std::isfinite(max_motor_speed_rpm_) ||
    wheel_radius_m_ <= 0.0 || wheel_separation_m_ <= 0.0 || gear_ratio_ <= 0.0 ||
    max_motor_speed_rpm_ <= 0.0)
  {
    throw std::invalid_argument("Differential-drive parameters must be finite and positive");
  }
}

WheelPair DifferentialDriveKinematics::twist_to_motor_rpm(
  double linear_velocity_mps, double angular_velocity_radps) const
{
  if (!std::isfinite(linear_velocity_mps) || !std::isfinite(angular_velocity_radps)) {
    throw std::invalid_argument("Velocity command must be finite");
  }

  const double left_velocity_mps =
    linear_velocity_mps - angular_velocity_radps * wheel_separation_m_ / 2.0;
  const double right_velocity_mps =
    linear_velocity_mps + angular_velocity_radps * wheel_separation_m_ / 2.0;
  const double rpm_per_mps = gear_ratio_ * kSecondsPerMinute / (kTwoPi * wheel_radius_m_);

  double left_rpm = left_velocity_mps * rpm_per_mps;
  double right_rpm = right_velocity_mps * rpm_per_mps;
  const double largest_magnitude = std::max(std::abs(left_rpm), std::abs(right_rpm));
  if (largest_magnitude > max_motor_speed_rpm_) {
    const double scale = max_motor_speed_rpm_ / largest_magnitude;
    left_rpm *= scale;
    right_rpm *= scale;
  }
  if (invert_left_motor_) {
    left_rpm = -left_rpm;
  }
  if (invert_right_motor_) {
    right_rpm = -right_rpm;
  }
  return {left_rpm, right_rpm};
}

WheelPair DifferentialDriveKinematics::motor_values_to_wheel(
  double m1_value, double m2_value) const
{
  return {
    invert_left_motor_ ? -m1_value : m1_value,
    invert_right_motor_ ? -m2_value : m2_value};
}

std::int64_t DifferentialDriveKinematics::wrapped_encoder_delta(
  std::int32_t current, std::int32_t previous)
{
  const auto raw_delta = static_cast<std::uint32_t>(current) -
    static_cast<std::uint32_t>(previous);
  if (raw_delta <= static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max())) {
    return static_cast<std::int64_t>(raw_delta);
  }
  return static_cast<std::int64_t>(raw_delta) - 0x100000000LL;
}

}  // namespace motor_control_g4dual
