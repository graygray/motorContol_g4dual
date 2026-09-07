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
    {MotorCanProtocol::kReplyId, 8U, {'p', 'r', 'i', 'm', 'a', 'x', 0U, 0U}});
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
