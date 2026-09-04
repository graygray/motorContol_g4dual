// Copyright 2026 Gray Lin
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include "motor_control_g4dual/differential_drive_kinematics.hpp"

namespace motor_control_g4dual
{
namespace
{
constexpr double kTolerance = 1.0e-9;

TEST(DifferentialDriveKinematics, ConvertsStraightAndTurningCommands)
{
  const DifferentialDriveKinematics kinematics(0.1, 0.5, 1.0, 200.0, false, false);

  const auto straight = kinematics.twist_to_motor_rpm(1.0, 0.0);
  EXPECT_NEAR(straight.left, 95.4929658551372, kTolerance);
  EXPECT_NEAR(straight.right, 95.4929658551372, kTolerance);

  const auto turn = kinematics.twist_to_motor_rpm(0.0, 1.0);
  EXPECT_NEAR(turn.left, -23.8732414637843, kTolerance);
  EXPECT_NEAR(turn.right, 23.8732414637843, kTolerance);
}

TEST(DifferentialDriveKinematics, AppliesGearRatioInversionAndClamp)
{
  const DifferentialDriveKinematics kinematics(0.1, 0.5, 2.0, 135.0, true, false);
  const auto command = kinematics.twist_to_motor_rpm(1.0, 0.0);

  EXPECT_DOUBLE_EQ(command.left, -135.0);
  EXPECT_DOUBLE_EQ(command.right, 135.0);

  const auto feedback = kinematics.motor_values_to_wheel(20.0, -30.0);
  EXPECT_DOUBLE_EQ(feedback.left, -20.0);
  EXPECT_DOUBLE_EQ(feedback.right, -30.0);
}

TEST(DifferentialDriveKinematics, PreservesCurvatureWhenLimited)
{
  const DifferentialDriveKinematics kinematics(0.1, 0.5, 1.0, 100.0, false, false);
  const auto command = kinematics.twist_to_motor_rpm(1.0, 2.0);

  EXPECT_NEAR(command.right, 100.0, kTolerance);
  EXPECT_NEAR(command.left / command.right, 1.0 / 3.0, kTolerance);
}

TEST(DifferentialDriveKinematics, HandlesSignedEncoderRollover)
{
  EXPECT_EQ(
    DifferentialDriveKinematics::wrapped_encoder_delta(
      std::numeric_limits<std::int32_t>::min(), std::numeric_limits<std::int32_t>::max()),
    1);
  EXPECT_EQ(
    DifferentialDriveKinematics::wrapped_encoder_delta(
      std::numeric_limits<std::int32_t>::max(), std::numeric_limits<std::int32_t>::min()),
    -1);
}

TEST(DifferentialDriveKinematics, RejectsInvalidParametersAndCommands)
{
  EXPECT_THROW(
    DifferentialDriveKinematics(0.0, 0.5, 1.0, 135.0, false, false),
    std::invalid_argument);
  const DifferentialDriveKinematics kinematics(0.1, 0.5, 1.0, 135.0, false, false);
  EXPECT_THROW(
    kinematics.twist_to_motor_rpm(std::numeric_limits<double>::quiet_NaN(), 0.0),
    std::invalid_argument);
}

}  // namespace
}  // namespace motor_control_g4dual
