// Copyright 2026 Gray Lin
// SPDX-License-Identifier: MIT

#include <memory>

#include "motor_control_g4dual/motor_control_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<motor_control_g4dual::MotorControlNode>());
  rclcpp::shutdown();
  return 0;
}
