// Copyright 2026 Gray Lin
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>

namespace motor_control_g4dual
{

struct WheelPair
{
  double left{0.0};
  double right{0.0};
};

class DifferentialDriveKinematics
{
public:
  DifferentialDriveKinematics(
    double wheel_radius_m, double wheel_separation_m, double gear_ratio,
    double max_motor_speed_rpm, bool invert_left_motor, bool invert_right_motor);

  WheelPair twist_to_motor_rpm(double linear_velocity_mps, double angular_velocity_radps) const;
  WheelPair motor_values_to_wheel(double m1_value, double m2_value) const;

  static std::int64_t wrapped_encoder_delta(std::int32_t current, std::int32_t previous);

private:
  double wheel_radius_m_;
  double wheel_separation_m_;
  double gear_ratio_;
  double max_motor_speed_rpm_;
  bool invert_left_motor_;
  bool invert_right_motor_;
};

}  // namespace motor_control_g4dual

