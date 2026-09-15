// Copyright 2026 Gray Lin
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <limits>
#include <variant>

#include "motor_control_g4dual/motor_can_protocol.hpp"

namespace motor_control_g4dual
{
namespace
{
TEST(MotorCanProtocol, EncodesDualWheelSpeeds)
{
  const auto frame = MotorCanProtocol::encode_wheel_speeds(12.3, -45.6);
  ASSERT_TRUE(frame);
  EXPECT_EQ(frame->id, 0x601U);
  EXPECT_EQ(frame->length, 8U);
  EXPECT_EQ(
    frame->data,
    (std::array<std::uint8_t, 8U>{
      0x23U, 0xFFU, 0x60U, 0x03U, 0x7BU, 0x00U, 0x38U, 0xFEU}));
}

TEST(MotorCanProtocol, RejectsInvalidSpeeds)
{
  EXPECT_FALSE(MotorCanProtocol::encode_wheel_speeds(135.1, 0.0));
  EXPECT_FALSE(MotorCanProtocol::encode_wheel_speeds(
      std::numeric_limits<double>::quiet_NaN(), 0.0));
  EXPECT_TRUE(MotorCanProtocol::encode_wheel_speeds(-135.0, 135.0));
}

TEST(MotorCanProtocol, EncodesWholeRpmAndRoundsSignedSpeeds)
{
  const auto frame = MotorCanProtocol::encode_wheel_speeds(12.5, -45.5, 1.0);
  ASSERT_TRUE(frame);
  EXPECT_EQ(
    frame->data,
    (std::array<std::uint8_t, 8U>{
      0x23U, 0xFFU, 0x60U, 0x03U, 0x0DU, 0U, 0xD2U, 0xFFU}));
  const auto single = MotorCanProtocol::encode_motor_speed(MotorSelector::kM2, -12.5, 1.0);
  ASSERT_TRUE(single);
  EXPECT_EQ(single->data[3], 0x02U);
  EXPECT_EQ(single->data[4], 0xF3U);
  EXPECT_EQ(single->data[5], 0xFFU);
}

TEST(MotorCanProtocol, AppliesResolutionToFeedbackOnlyForSpeeds)
{
  const CanFrame frame{MotorCanProtocol::kReplyId, 8U,
    {0x43U, 0x6CU, 0x60U, 0U, 0x7BU, 0U, 0x85U, 0xFFU}};
  for (const double resolution : {1.0, 0.1}) {
    const auto decoded = MotorCanProtocol::decode(frame, resolution);
    ASSERT_TRUE(decoded);
    const auto speeds = std::get<WheelSpeedsReply>(*decoded.message);
    EXPECT_DOUBLE_EQ(speeds.m1_speed_rpm, 123.0 * resolution);
    EXPECT_DOUBLE_EQ(speeds.m2_speed_rpm, -123.0 * resolution);
    const auto deltas = MotorCanProtocol::decode(
      {MotorCanProtocol::kReplyId, 8U,
        {0U, 0x64U, 0x60U, 0U, 0x7BU, 0U, 0x85U, 0xFFU}}, resolution);
    ASSERT_TRUE(deltas);
    EXPECT_EQ(std::get<EncoderDeltasReply>(*deltas.message).m1_delta, 123);
    EXPECT_EQ(std::get<EncoderDeltasReply>(*deltas.message).m2_delta, -123);
  }
}

TEST(MotorCanProtocol, PreservesLimitsAtBothResolutions)
{
  for (const double resolution : {1.0, 0.1}) {
    EXPECT_TRUE(MotorCanProtocol::encode_wheel_speeds(-135.0, 135.0, resolution));
    EXPECT_FALSE(MotorCanProtocol::encode_wheel_speeds(-135.1, 0.0, resolution));
    EXPECT_FALSE(MotorCanProtocol::encode_wheel_speeds(0.0, 135.1, resolution));
    EXPECT_FALSE(MotorCanProtocol::encode_wheel_speeds(
        0.0, std::numeric_limits<double>::infinity(), resolution));
  }
}

TEST(MotorCanProtocol, RejectsInvalidResolutions)
{
  for (const double resolution : {0.0, -0.1, 0.01, 0.5, 10.0,
    std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
  {
    EXPECT_FALSE(MotorCanProtocol::is_valid_rpm_resolution(resolution));
    EXPECT_FALSE(MotorCanProtocol::encode_wheel_speeds(12.0, -12.0, resolution));
    EXPECT_FALSE(MotorCanProtocol::encode_motor_speed(MotorSelector::kM1, 12.0, resolution));
    EXPECT_FALSE(MotorCanProtocol::decode(
        {MotorCanProtocol::kReplyId, 8U,
          {0x43U, 0x6CU, 0x60U, 0U, 0U, 0U, 0U, 0U}}, resolution));
  }
}

TEST(MotorCanProtocol, EncodesSafetyControls)
{
  const auto emergency_stop =
    MotorCanProtocol::encode_control(ControlCommand::kEmergencyStop);
  ASSERT_TRUE(emergency_stop);
  EXPECT_EQ(
    emergency_stop->data,
    (std::array<std::uint8_t, 8U>{0x2BU, 0x40U, 0x60U, 0U, 0x02U, 0U, 0U, 0U}));

  const auto emergency_stop_ramp =
    MotorCanProtocol::encode_control(ControlCommand::kEmergencyStopRamp);
  ASSERT_TRUE(emergency_stop_ramp);
  EXPECT_EQ(
    emergency_stop_ramp->data,
    (std::array<std::uint8_t, 8U>{0x2BU, 0x40U, 0x60U, 0U, 0x03U, 0U, 0U, 0U}));

  const auto stop = MotorCanProtocol::encode_control(ControlCommand::kStop);
  ASSERT_TRUE(stop);
  EXPECT_EQ(stop->data[4], 0x00U);
}

TEST(MotorCanProtocol, EncodesConfigurationAndReadRequests)
{
  const auto duration = MotorCanProtocol::encode_s_curve_duration(
    SCurveDirection::kDeceleration, 1000U);
  ASSERT_TRUE(duration);
  EXPECT_EQ(
    duration->data,
    (std::array<std::uint8_t, 8U>{0x23U, 0x84U, 0x60U, 0U, 0xE8U, 0x03U, 0U, 0U}));

  const auto profile = MotorCanProtocol::encode_jerk_limit_profile(JerkLimitProfile::kBoost);
  ASSERT_TRUE(profile);
  EXPECT_EQ(profile->data[1], 0xF0U);
  EXPECT_EQ(profile->data[4], 0x05U);

  EXPECT_EQ(MotorCanProtocol::encode_firmware_version_request().data[1], 0x31U);
  EXPECT_EQ(MotorCanProtocol::encode_wheel_speeds_request().data[1], 0x6CU);
  EXPECT_EQ(MotorCanProtocol::encode_encoder_deltas_request().data[1], 0x64U);
}

TEST(MotorCanProtocol, DecodesReplyPayloads)
{
  const auto firmware = MotorCanProtocol::decode(
    {MotorCanProtocol::kReplyId, 8U, {'p', 'r', 'i', 'm', 'a', 'x', 0U, 0U}}, 0.1, true);
  ASSERT_TRUE(firmware);
  EXPECT_EQ(std::get<FirmwareReply>(*firmware.message).identifier, "primax");

  const auto speeds = MotorCanProtocol::decode(
    {MotorCanProtocol::kReplyId, 8U,
      {0x43U, 0x6CU, 0x60U, 0U, 0x58U, 0x02U, 0xA8U, 0xFDU}});
  ASSERT_TRUE(speeds);
  const auto speed_values = std::get<WheelSpeedsReply>(*speeds.message);
  EXPECT_DOUBLE_EQ(speed_values.m1_speed_rpm, 60.0);
  EXPECT_DOUBLE_EQ(speed_values.m2_speed_rpm, -60.0);

  const auto deltas = MotorCanProtocol::decode(
    {MotorCanProtocol::kReplyId, 8U,
      {0U, 0x64U, 0x60U, 0U, 0x64U, 0U, 0xCEU, 0xFFU}});
  ASSERT_TRUE(deltas);
  EXPECT_EQ(std::get<EncoderDeltasReply>(*deltas.message).m1_delta, 100);
  EXPECT_EQ(std::get<EncoderDeltasReply>(*deltas.message).m2_delta, -50);
}

TEST(MotorCanProtocol, RecognizesAcknowledgementsBeforeFirmwareText)
{
  for (const bool pending : {false, true}) {
    // Includes both printable replies observed on hardware and a binary index.
    for (const std::uint16_t index : {0x6040U, 0x6060U, 0x60FFU}) {
      const auto decoded = MotorCanProtocol::decode(
        {0x581U, 8U, {0x60U, static_cast<std::uint8_t>(index & 0xFFU),
            static_cast<std::uint8_t>(index >> 8U), 0U, 0U, 0U, 0U, 0U}}, 0.1, pending);
      ASSERT_TRUE(decoded);
      ASSERT_TRUE(std::holds_alternative<WriteAcknowledgement>(*decoded.message));
      EXPECT_EQ(std::get<WriteAcknowledgement>(*decoded.message).index, index);
      EXPECT_EQ(std::get<WriteAcknowledgement>(*decoded.message).subindex, 0U);
    }
  }
}

TEST(MotorCanProtocol, DecodesAbortObjectAndUnsignedCode)
{
  const auto decoded = MotorCanProtocol::decode(
    {0x581U, 8U, {0x80U, 0x40U, 0x60U, 0x02U, 0x78U, 0x56U, 0x34U, 0xF2U}});
  ASSERT_TRUE(decoded);
  const auto reply = std::get<AbortReply>(*decoded.message);
  EXPECT_EQ(reply.index, 0x6040U);
  EXPECT_EQ(reply.subindex, 2U);
  EXPECT_EQ(reply.code, 0xF2345678U);
}

TEST(MotorCanProtocol, RequiresPendingQueryForFirmware)
{
  const CanFrame firmware{0x581U, 8U, {'p', 'r', 'i', 'm', 'a', 'x', 0U, 0U}};
  EXPECT_EQ(MotorCanProtocol::decode(firmware).status, DecodeStatus::kUnexpectedReply);
  ASSERT_TRUE(MotorCanProtocol::decode(firmware, 0.1, true));
  const CanFrame full{0x581U, 8U, {'f', 'i', 'r', 'm', 'w', 'a', 'r', 'e'}};
  ASSERT_TRUE(MotorCanProtocol::decode(full, 0.1, true));
  EXPECT_EQ(std::get<FirmwareReply>(
      *MotorCanProtocol::decode(full, 0.1, true).message).identifier, "firmware");
  for (const CanFrame invalid : {
      CanFrame{0x581U, 8U, {}},
      CanFrame{0x581U, 8U, {'o', 'k', 0U, 'x', 0U, 0U, 0U, 0U}},
      CanFrame{0x581U, 8U, {'x', 0xFFU, 0U, 0U, 0U, 0U, 0U, 0U}}})
  {
    EXPECT_EQ(MotorCanProtocol::decode(invalid, 0.1, true).status,
      DecodeStatus::kInvalidPayload);
  }
}

TEST(MotorCanProtocol, RejectsMalformedAcknowledgementsAndFormatsRawFrame)
{
  const CanFrame invalid{0x581U, 8U, {0x60U, 0x40U, 0x60U, 0U, 1U, 0U, 0U, 0U}};
  EXPECT_EQ(MotorCanProtocol::decode(invalid, 0.1, true).status, DecodeStatus::kInvalidPayload);
  EXPECT_EQ(MotorCanProtocol::format_frame(invalid),
    "id=0x581 length=8 data=60 40 60 00 01 00 00 00");
  EXPECT_EQ(MotorCanProtocol::decode({0x581U, 3U, {0x60U, 0x40U, 0x60U}}).status,
    DecodeStatus::kInvalidLength);
  EXPECT_EQ(MotorCanProtocol::format_frame({0x581U, 3U, {0x60U, 0x40U, 0x60U}}),
    "id=0x581 length=3 data=60 40 60");
  EXPECT_EQ(MotorCanProtocol::format_frame({0x581U, 255U, {}}),
    "id=0x581 length=255 data=00 00 00 00 00 00 00 00");
}

TEST(MotorCanProtocol, DecodesEncoderAndFaultReports)
{
  const auto positions = MotorCanProtocol::decode(
    {MotorCanProtocol::kEncoderReportId, 8U,
      {0x90U, 0x01U, 0U, 0U, 0x38U, 0xFFU, 0xFFU, 0xFFU}});
  ASSERT_TRUE(positions);
  EXPECT_EQ(std::get<EncoderPositionReport>(*positions.message).m1_position, 400);
  EXPECT_EQ(std::get<EncoderPositionReport>(*positions.message).m2_position, -200);

  const auto fault = MotorCanProtocol::decode(
    {MotorCanProtocol::kFaultReportId, 8U,
      {0x02U, 0x40U, 0U, 0U, 0U, 0U, 0U, 0U}});
  ASSERT_TRUE(fault);
  const auto fault_value = std::get<MotorFaultReport>(*fault.message);
  EXPECT_EQ(fault_value.motor, MotorSelector::kM2);
  EXPECT_EQ(fault_value.fault_mask, 0x0040U);
}

TEST(MotorCanProtocol, RejectsMalformedFrames)
{
  EXPECT_EQ(
    MotorCanProtocol::decode({0x123U, 8U, {}}).status,
    DecodeStatus::kUnsupportedId);
  EXPECT_EQ(
    MotorCanProtocol::decode({MotorCanProtocol::kReplyId, 7U, {}}).status,
    DecodeStatus::kInvalidLength);
  EXPECT_EQ(
    MotorCanProtocol::decode(
      {MotorCanProtocol::kFaultReportId, 8U, {0x03U, 1U, 0U, 0U, 0U, 0U, 0U, 0U}}).status,
    DecodeStatus::kInvalidPayload);
  EXPECT_EQ(
    MotorCanProtocol::decode(
      {MotorCanProtocol::kReplyId, 8U, {'o', 'k', 0U, 'x', 0U, 0U, 0U, 0U}}).status,
    DecodeStatus::kInvalidPayload);
}

}  // namespace
}  // namespace motor_control_g4dual
