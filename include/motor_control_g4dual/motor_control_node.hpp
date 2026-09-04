// Copyright 2026 Gray Lin
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace motor_control_g4dual
{

class MotorControlNode : public rclcpp::Node
{
public:
  explicit MotorControlNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void command_callback(const geometry_msgs::msg::Twist::SharedPtr message);
  void control_callback();
  void publish_motor_rpm(double left_rpm, double right_rpm);

  std::string command_topic_;
  std::string motor_rpm_topic_;
  double wheel_radius_m_;
  double wheel_separation_m_;
  double gear_ratio_;
  double max_motor_speed_rpm_;
  bool invert_left_motor_;
  bool invert_right_motor_;
  std::chrono::milliseconds command_timeout_;

  double linear_velocity_mps_{0.0};
  double angular_velocity_radps_{0.0};
  bool command_received_{false};
  bool watchdog_stopped_{false};
  std::chrono::steady_clock::time_point last_command_time_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_rpm_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace motor_control_g4dual
