// Copyright 2026 Gray Lin
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "motor_control_g4dual/socket_can_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace motor_control_g4dual
{

class MotorControlNode : public rclcpp::Node
{
public:
  explicit MotorControlNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MotorControlNode() override;

private:
  void command_callback(const geometry_msgs::msg::Twist::SharedPtr message);
  void control_callback();
  void receive_can_frames();
  void publish_motor_rpm(double left_rpm, double right_rpm);
  void enable_motors(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void stop_motors(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void reset_faults(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void set_emergency_stop(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  bool send_control_command(ControlCommand command, std::string & error_message);
  void check_feedback_timeout(std::chrono::steady_clock::time_point now);
  void latch_safety_stop(const char * reason);

  std::string command_topic_;
  std::string motor_rpm_topic_;
  std::string can_interface_;
  bool enable_can_;
  double wheel_radius_m_;
  double wheel_separation_m_;
  double gear_ratio_;
  double max_motor_speed_rpm_;
  bool invert_left_motor_;
  bool invert_right_motor_;
  std::chrono::milliseconds command_timeout_;
  std::chrono::milliseconds feedback_timeout_;

  double linear_velocity_mps_{0.0};
  double angular_velocity_radps_{0.0};
  bool command_received_{false};
  bool watchdog_stopped_{false};
  bool motion_commands_enabled_{false};
  bool fault_latched_{false};
  bool feedback_timeout_latched_{false};
  std::chrono::steady_clock::time_point last_command_time_;
  std::chrono::steady_clock::time_point last_feedback_time_;
  std::unique_ptr<SocketCanTransport> can_transport_;
  std::optional<WheelSpeedsReply> latest_wheel_speeds_;
  std::optional<EncoderDeltasReply> latest_encoder_deltas_;
  std::optional<EncoderPositionReport> latest_encoder_positions_;
  std::optional<MotorFaultReport> latest_motor_fault_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_rpm_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr can_receive_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_faults_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr emergency_stop_service_;
};

}  // namespace motor_control_g4dual
