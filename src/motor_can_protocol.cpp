// Copyright 2026 Gray Lin
// SPDX-License-Identifier: MIT

#include "motor_control_g4dual/motor_can_protocol.hpp"

#include <cmath>
#include <type_traits>

namespace motor_control_g4dual
{
namespace
{
constexpr std::uint8_t kMotorBoth = 0x03U;

template<typename EnumT>
constexpr std::uint8_t to_byte(EnumT value)
{
  static_assert(std::is_enum<EnumT>::value, "to_byte requires an enum");
  return static_cast<std::uint8_t>(value);
}

void write_int16_little_endian(
  std::array<std::uint8_t, 8U> & data, std::size_t offset, std::int16_t value)
{
  const auto raw_value = static_cast<std::uint16_t>(value);
  data[offset] = static_cast<std::uint8_t>(raw_value & 0xFFU);
  data[offset + 1U] = static_cast<std::uint8_t>((raw_value >> 8U) & 0xFFU);
}
}  // namespace

std::optional<CanFrame> MotorCanProtocol::encode_motor_speed(
  MotorSelector motor, double speed_rpm)
{
  if (motor != MotorSelector::kM1 && motor != MotorSelector::kM2) {
    return std::nullopt;
  }

  const auto speed_units = encode_speed_units(speed_rpm);
  if (!speed_units) {
    return std::nullopt;
  }

  std::array<std::uint8_t, 8U> data{0x23U, 0xFFU, 0x60U, to_byte(motor)};
  write_int16_little_endian(data, 4U, *speed_units);
  return make_frame(data);
}

std::optional<CanFrame> MotorCanProtocol::encode_wheel_speeds(
  double m1_speed_rpm, double m2_speed_rpm)
{
  const auto m1_speed_units = encode_speed_units(m1_speed_rpm);
  const auto m2_speed_units = encode_speed_units(m2_speed_rpm);
  if (!m1_speed_units || !m2_speed_units) {
    return std::nullopt;
  }

  std::array<std::uint8_t, 8U> data{0x23U, 0xFFU, 0x60U, kMotorBoth};
  write_int16_little_endian(data, 4U, *m1_speed_units);
  write_int16_little_endian(data, 6U, *m2_speed_units);
  return make_frame(data);
}

std::optional<CanFrame> MotorCanProtocol::encode_jerk_limit_profile(
  JerkLimitProfile profile)
{
  if (to_byte(profile) > to_byte(JerkLimitProfile::kBoost)) {
    return std::nullopt;
  }

  return make_frame({0x2FU, 0xF0U, 0x20U, 0x00U, to_byte(profile), 0U, 0U, 0U});
}

std::optional<CanFrame> MotorCanProtocol::encode_movement_method(MovementMethod method)
{
  if (to_byte(method) > to_byte(MovementMethod::kJerkLimit)) {
    return std::nullopt;
  }

  return make_frame({0x2FU, 0xF3U, 0x20U, 0x00U, to_byte(method), 0U, 0U, 0U});
}

std::optional<CanFrame> MotorCanProtocol::encode_pid_set(PidSet pid_set)
{
  if (to_byte(pid_set) > to_byte(PidSet::kRpm20_40_0p4V2)) {
    return std::nullopt;
  }

  return make_frame({0x2FU, 0xF4U, 0x20U, 0x00U, to_byte(pid_set), 0U, 0U, 0U});
}

std::optional<CanFrame> MotorCanProtocol::encode_lower_test_case(LowerTestCase test_case)
{
  if (test_case != LowerTestCase::kPid && test_case != LowerTestCase::kEncoderPush) {
    return std::nullopt;
  }

  return make_frame({0x2FU, 0xF1U, 0x20U, 0x00U, to_byte(test_case), 0U, 0U, 0U});
}

std::optional<CanFrame> MotorCanProtocol::encode_control(ControlCommand command)
{
  switch (command) {
    case ControlCommand::kStop:
    case ControlCommand::kEmergencyStop:
    case ControlCommand::kEmergencyStopRamp:
    case ControlCommand::kEnableStage1:
    case ControlCommand::kEnableStage2:
    case ControlCommand::kResetFaults:
    case ControlCommand::kEnableOperation:
      return make_frame({0x2BU, 0x40U, 0x60U, 0x00U, to_byte(command), 0U, 0U, 0U});
    default:
      return std::nullopt;
  }
}

std::optional<CanFrame> MotorCanProtocol::encode_s_curve_duration(
  SCurveDirection direction, std::uint16_t duration_ms)
{
  if (direction != SCurveDirection::kAcceleration &&
    direction != SCurveDirection::kDeceleration)
  {
    return std::nullopt;
  }

  return make_frame(
    {0x23U, to_byte(direction), 0x60U, 0x00U,
      static_cast<std::uint8_t>(duration_ms & 0xFFU),
      static_cast<std::uint8_t>((duration_ms >> 8U) & 0xFFU), 0U, 0U});
}

CanFrame MotorCanProtocol::encode_toggle_lower_run_test()
{
  return make_frame({0x2FU, 0xF2U, 0x20U, 0x00U, 0x01U, 0U, 0U, 0U});
}

CanFrame MotorCanProtocol::encode_firmware_version_request()
{
  return make_frame({0x40U, 0x31U, 0x20U, 0x00U, 0U, 0U, 0U, 0U});
}

CanFrame MotorCanProtocol::encode_wheel_speeds_request()
{
  return make_frame({0x43U, 0x6CU, 0x60U, kMotorBoth, 0U, 0U, 0U, 0U});
}

CanFrame MotorCanProtocol::encode_encoder_deltas_request()
{
  return make_frame({0x43U, 0x64U, 0x60U, kMotorBoth, 0U, 0U, 0U, 0U});
}

std::optional<std::int16_t> MotorCanProtocol::encode_speed_units(double speed_rpm)
{
  if (!std::isfinite(speed_rpm) || speed_rpm < -kMaxSpeedRpm || speed_rpm > kMaxSpeedRpm) {
    return std::nullopt;
  }

  return static_cast<std::int16_t>(std::round(speed_rpm * kSpeedUnitsPerRpm));
}

CanFrame MotorCanProtocol::make_frame(const std::array<std::uint8_t, 8U> & data)
{
  return CanFrame{kCommandId, static_cast<std::uint8_t>(data.size()), data};
}

}  // namespace motor_control_g4dual

