// Copyright 2026 Gray Lin
// SPDX-License-Identifier: MIT

#include <algorithm>
#include <cctype>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "motor_control_g4dual/motor_control_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
bool parse_bool(std::string value, bool & result)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char character) {return static_cast<char>(std::tolower(character));});
  if (value == "true" || value == "1" || value == "on") {
    result = true;
    return true;
  }
  if (value == "false" || value == "0" || value == "off") {
    result = false;
    return true;
  }
  return false;
}
}  // namespace

int main(int argc, char * argv[])
{
  bool log_enabled = false;
  std::vector<char *> ros_arguments;
  ros_arguments.reserve(static_cast<std::size_t>(argc));
  ros_arguments.push_back(argv[0]);

  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--log") {
      log_enabled = true;
      if (index + 1 < argc) {
        bool requested_value = false;
        if (parse_bool(argv[index + 1], requested_value)) {
          log_enabled = requested_value;
          ++index;
        }
      }
      continue;
    }
    constexpr char log_prefix[] = "--log=";
    if (argument.rfind(log_prefix, 0) == 0) {
      if (!parse_bool(argument.substr(sizeof(log_prefix) - 1U), log_enabled)) {
        std::cerr << "Invalid --log value; use true/false, 1/0, or on/off" << std::endl;
        return 2;
      }
      continue;
    }
    ros_arguments.push_back(argv[index]);
  }

  int ros_argc = static_cast<int>(ros_arguments.size());
  rclcpp::init(ros_argc, ros_arguments.data());
  rclcpp::spin(
    std::make_shared<motor_control_g4dual::MotorControlNode>(rclcpp::NodeOptions(), log_enabled));
  rclcpp::shutdown();
  return 0;
}
