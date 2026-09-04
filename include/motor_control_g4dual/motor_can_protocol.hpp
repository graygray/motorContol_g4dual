// Copyright 2026 Gray Lin
// SPDX-License-Identifier: MIT

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <variant>

namespace motor_control_g4dual
{

struct CanFrame
{
  std::uint16_t id{0U};
  std::uint8_t length{0U};
  std::array<std::uint8_t, 8U> data{};
};

enum class MotorSelector : std::uint8_t
{
  kM1 = 0x01U,
  kM2 = 0x02U,
};

enum class JerkLimitProfile : std::uint8_t
{
  kComfort = 0U,
  kBalanced = 1U,
  kSport = 2U,
  kAggressive = 3U,
  kPerformance = 4U,
  kBoost = 5U,
};

enum class MovementMethod : std::uint8_t
{
  kSCurve = 0U,
  kQuintic = 1U,
  kJerkLimit = 2U,
};

enum class PidSet : std::uint8_t
{
  kMcSdkDefault = 0U,
  kRpm20_40_0p4 = 1U,
  kRpm20_40_0p4V2 = 2U,
};

enum class LowerTestCase : std::uint8_t
{
  kPid = 0U,
  // Value 1 is reserved by the lower controller's CAN test-case ABI.
  kEncoderPush = 2U,
};

enum class ControlCommand : std::uint8_t
{
  kStop = 0x00U,
  kEmergencyStopRamped = 0x02U,
  kEmergencyStopImmediate = 0x03U,
  kEnableStage1 = 0x06U,
  kEnableStage2 = 0x07U,
  kResetFaults = 0x08U,
  kEnableOperation = 0x0FU,
};

enum class SCurveDirection : std::uint8_t
{
  kAcceleration = 0x83U,
  kDeceleration = 0x84U,
};

struct FirmwareReply
{
  std::string identifier;
};

struct WheelSpeedsReply
{
  double m1_speed_rpm{0.0};
  double m2_speed_rpm{0.0};
};

struct EncoderDeltasReply
{
  std::int16_t m1_delta{0};
  std::int16_t m2_delta{0};
};

struct EncoderPositionReport
{
  // Quadrature-scaled wire counts; the controller reports 16,384 per revolution.
  std::int32_t m1_position{0};
  std::int32_t m2_position{0};
};

struct MotorFaultReport
{
  MotorSelector motor{MotorSelector::kM1};
  std::uint16_t fault_mask{0U};
};

using DecodedCanMessage = std::variant<
  FirmwareReply,
  WheelSpeedsReply,
  EncoderDeltasReply,
  EncoderPositionReport,
  MotorFaultReport>;

enum class DecodeStatus
{
  kSuccess,
  kUnsupportedId,
  kInvalidLength,
  kInvalidPayload,
};

struct DecodeResult
{
  DecodeStatus status{DecodeStatus::kInvalidPayload};
  std::optional<DecodedCanMessage> message;

  explicit operator bool() const noexcept
  {
    return status == DecodeStatus::kSuccess && message.has_value();
  }
};

class MotorCanProtocol
{
public:
  static constexpr std::uint16_t kCommandId = 0x601U;
  static constexpr std::uint16_t kReplyId = 0x581U;
  static constexpr std::uint16_t kEncoderReportId = 0x481U;
  static constexpr std::uint16_t kFaultReportId = 0x381U;
  static constexpr double kMaxSpeedRpm = 135.0;
  static constexpr double kSpeedUnitsPerRpm = 10.0;
  static constexpr double kEncoderCountsPerRevolution = 16384.0;

  static std::optional<CanFrame> encode_motor_speed(
    MotorSelector motor, double speed_rpm);
  static std::optional<CanFrame> encode_wheel_speeds(
    double m1_speed_rpm, double m2_speed_rpm);
  static std::optional<CanFrame> encode_jerk_limit_profile(JerkLimitProfile profile);
  static std::optional<CanFrame> encode_movement_method(MovementMethod method);
  static std::optional<CanFrame> encode_pid_set(PidSet pid_set);
  static std::optional<CanFrame> encode_lower_test_case(LowerTestCase test_case);
  static std::optional<CanFrame> encode_control(ControlCommand command);
  static std::optional<CanFrame> encode_s_curve_duration(
    SCurveDirection direction, std::uint16_t duration_ms);

  static CanFrame encode_toggle_lower_run_test();
  static CanFrame encode_firmware_version_request();
  static CanFrame encode_wheel_speeds_request();
  static CanFrame encode_encoder_deltas_request();
  static DecodeResult decode(const CanFrame & frame);

private:
  static std::optional<std::int16_t> encode_speed_units(double speed_rpm);
  static CanFrame make_frame(const std::array<std::uint8_t, 8U> & data);
};

}  // namespace motor_control_g4dual
