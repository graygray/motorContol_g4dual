// Copyright 2026 Gray Lin
// SPDX-License-Identifier: MIT

#include "motor_control_g4dual/motor_can_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
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

std::int16_t read_int16_little_endian(
  const std::array<std::uint8_t, 8U> & data, std::size_t offset)
{
  const auto value = static_cast<std::uint16_t>(data[offset]) |
    static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[offset + 1U]) << 8U);
  if (value <= static_cast<std::uint16_t>(std::numeric_limits<std::int16_t>::max())) {
    return static_cast<std::int16_t>(value);
  }
  return static_cast<std::int16_t>(static_cast<std::int32_t>(value) - 0x10000);
}

std::int32_t read_int32_little_endian(
  const std::array<std::uint8_t, 8U> & data, std::size_t offset)
{
  const auto value = static_cast<std::uint32_t>(data[offset]) |
    (static_cast<std::uint32_t>(data[offset + 1U]) << 8U) |
    (static_cast<std::uint32_t>(data[offset + 2U]) << 16U) |
    (static_cast<std::uint32_t>(data[offset + 3U]) << 24U);
  if (value <= static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max())) {
    return static_cast<std::int32_t>(value);
  }
  return static_cast<std::int32_t>(static_cast<std::int64_t>(value) - 0x100000000LL);
}

DecodeResult decode_reply(
  const CanFrame & frame, double rpm_resolution, bool firmware_request_pending)
{
  const auto index = static_cast<std::uint16_t>(
    frame.data[1] | (static_cast<std::uint16_t>(frame.data[2]) << 8U));
  if (frame.data[0] == 0x60U) {
    if (!std::all_of(frame.data.begin() + 4, frame.data.end(),
      [](std::uint8_t byte) {return byte == 0U;}))
    {
      return {DecodeStatus::kInvalidPayload, std::nullopt};
    }
    return {DecodeStatus::kSuccess, WriteAcknowledgement{index, frame.data[3]}};
  }
  if (frame.data[0] == 0x80U) {
    const auto code = static_cast<std::uint32_t>(frame.data[4]) |
      (static_cast<std::uint32_t>(frame.data[5]) << 8U) |
      (static_cast<std::uint32_t>(frame.data[6]) << 16U) |
      (static_cast<std::uint32_t>(frame.data[7]) << 24U);
    return {DecodeStatus::kSuccess, AbortReply{index, frame.data[3], code}};
  }
  if (frame.data[0] == 0x43U && frame.data[1] == 0x6CU && frame.data[2] == 0x60U) {
    return {
      DecodeStatus::kSuccess,
      WheelSpeedsReply{
        static_cast<double>(read_int16_little_endian(frame.data, 4U)) * rpm_resolution,
        static_cast<double>(read_int16_little_endian(frame.data, 6U)) * rpm_resolution}};
  }

  if (frame.data[0] == 0x00U && frame.data[1] == 0x64U && frame.data[2] == 0x60U) {
    return {
      DecodeStatus::kSuccess,
      EncoderDeltasReply{
        read_int16_little_endian(frame.data, 4U),
        read_int16_little_endian(frame.data, 6U)}};
  }

  std::string identifier;
  bool null_seen = false;
  for (const auto byte : frame.data) {
    if (byte == 0U) {
      null_seen = true;
      continue;
    }
    if (null_seen || byte < 0x20U || byte > 0x7EU) {
      return {DecodeStatus::kInvalidPayload, std::nullopt};
    }
    identifier.push_back(static_cast<char>(byte));
  }
  if (identifier.empty()) {
    return {DecodeStatus::kInvalidPayload, std::nullopt};
  }
  if (!firmware_request_pending) {
    return {DecodeStatus::kUnexpectedReply, std::nullopt};
  }
  return {DecodeStatus::kSuccess, FirmwareReply{identifier}};
}
}  // namespace

std::optional<CanFrame> MotorCanProtocol::encode_motor_speed(
  MotorSelector motor, double speed_rpm, double rpm_resolution)
{
  if (motor != MotorSelector::kM1 && motor != MotorSelector::kM2) {
    return std::nullopt;
  }

  const auto speed_units = encode_speed_units(speed_rpm, rpm_resolution);
  if (!speed_units) {
    return std::nullopt;
  }

  std::array<std::uint8_t, 8U> data{0x23U, 0xFFU, 0x60U, to_byte(motor)};
  write_int16_little_endian(data, 4U, *speed_units);
  return make_frame(data);
}

std::optional<CanFrame> MotorCanProtocol::encode_wheel_speeds(
  double m1_speed_rpm, double m2_speed_rpm, double rpm_resolution)
{
  const auto m1_speed_units = encode_speed_units(m1_speed_rpm, rpm_resolution);
  const auto m2_speed_units = encode_speed_units(m2_speed_rpm, rpm_resolution);
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

DecodeResult MotorCanProtocol::decode(
  const CanFrame & frame, double rpm_resolution, bool firmware_request_pending)
{
  if (!is_valid_rpm_resolution(rpm_resolution)) {
    return {DecodeStatus::kInvalidPayload, std::nullopt};
  }
  if (frame.id != kReplyId && frame.id != kEncoderReportId && frame.id != kFaultReportId) {
    return {DecodeStatus::kUnsupportedId, std::nullopt};
  }
  if (frame.length != frame.data.size()) {
    return {DecodeStatus::kInvalidLength, std::nullopt};
  }

  if (frame.id == kReplyId) {
    return decode_reply(frame, rpm_resolution, firmware_request_pending);
  }
  if (frame.id == kEncoderReportId) {
    return {
      DecodeStatus::kSuccess,
      EncoderPositionReport{
        read_int32_little_endian(frame.data, 0U),
        read_int32_little_endian(frame.data, 4U)}};
  }

  if ((frame.data[0] != to_byte(MotorSelector::kM1) &&
    frame.data[0] != to_byte(MotorSelector::kM2)) ||
    frame.data[3] != 0U || frame.data[4] != 0U || frame.data[5] != 0U ||
    frame.data[6] != 0U || frame.data[7] != 0U)
  {
    return {DecodeStatus::kInvalidPayload, std::nullopt};
  }

  const auto fault_mask = static_cast<std::uint16_t>(
    static_cast<std::uint16_t>(frame.data[1]) |
    static_cast<std::uint16_t>(static_cast<std::uint16_t>(frame.data[2]) << 8U));
  return {
    DecodeStatus::kSuccess,
    MotorFaultReport{static_cast<MotorSelector>(frame.data[0]), fault_mask}};
}

const char * MotorCanProtocol::decode_status_name(DecodeStatus status)
{
  switch (status) {
    case DecodeStatus::kSuccess:
      return "success";
    case DecodeStatus::kUnsupportedId:
      return "unsupported CAN ID";
    case DecodeStatus::kInvalidLength:
      return "expected eight-byte payload";
    case DecodeStatus::kInvalidPayload:
      return "invalid or unsupported payload";
    case DecodeStatus::kUnexpectedReply:
      return "unsolicited firmware/text reply";
    default:
      return "unknown decode status";
  }
}

std::string MotorCanProtocol::format_frame(const CanFrame & frame)
{
  std::ostringstream stream;
  stream << "id=0x" << std::hex << std::uppercase << std::setfill('0') <<
    std::setw(3) << frame.id << std::dec << " length=" <<
    static_cast<unsigned int>(frame.length) << " data=" << std::hex;
  const auto size = std::min(static_cast<std::size_t>(frame.length), frame.data.size());
  for (std::size_t index = 0; index < size; ++index) {
    if (index != 0U) {
      stream << ' ';
    }
    stream << std::setw(2) << static_cast<unsigned int>(frame.data[index]);
  }
  return stream.str();
}

bool MotorCanProtocol::is_valid_rpm_resolution(double rpm_resolution)
{
  return rpm_resolution == 1.0 || rpm_resolution == 0.1;
}

std::optional<std::int16_t> MotorCanProtocol::encode_speed_units(
  double speed_rpm, double rpm_resolution)
{
  if (!is_valid_rpm_resolution(rpm_resolution) || !std::isfinite(speed_rpm) ||
    speed_rpm < -kMaxSpeedRpm || speed_rpm > kMaxSpeedRpm)
  {
    return std::nullopt;
  }

  return static_cast<std::int16_t>(std::round(speed_rpm * (1.0 / rpm_resolution)));
}

CanFrame MotorCanProtocol::make_frame(const std::array<std::uint8_t, 8U> & data)
{
  return CanFrame{kCommandId, static_cast<std::uint8_t>(data.size()), data};
}

}  // namespace motor_control_g4dual
