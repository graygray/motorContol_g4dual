// Copyright 2026 Gray Lin
// SPDX-License-Identifier: MIT

#include "motor_control_g4dual/motor_control_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

#include "motor_control_g4dual/motor_can_protocol.hpp"

namespace motor_control_g4dual
{
namespace
{
constexpr double kSecondsPerMinute = 60.0;
constexpr double kTwoPi = 6.28318530717958647692;
}

MotorControlNode::MotorControlNode(const rclcpp::NodeOptions & options)
: Node("motor_control", options),
  last_command_time_(std::chrono::steady_clock::now()),
  last_feedback_time_(std::chrono::steady_clock::now())
{
  command_topic_ = declare_parameter<std::string>("command_topic", "cmd_vel");
  motor_rpm_topic_ = declare_parameter<std::string>("motor_rpm_topic", "motor_rpm_command");
  can_interface_ = declare_parameter<std::string>("can_interface", "can0");
  enable_can_ = declare_parameter<bool>("enable_can", false);
  wheel_radius_m_ = declare_parameter<double>("wheel_radius_m", 0.1);
  wheel_separation_m_ = declare_parameter<double>("wheel_separation_m", 0.5);
  gear_ratio_ = declare_parameter<double>("gear_ratio", 1.0);
  max_motor_speed_rpm_ = declare_parameter<double>("max_motor_speed_rpm", 135.0);
  invert_left_motor_ = declare_parameter<bool>("invert_left_motor", false);
  invert_right_motor_ = declare_parameter<bool>("invert_right_motor", false);

  const auto command_timeout_ms = declare_parameter<int>("command_timeout_ms", 500);
  const auto control_period_ms = declare_parameter<int>("control_period_ms", 50);
  const auto can_receive_poll_ms = declare_parameter<int>("can_receive_poll_ms", 10);
  const auto feedback_timeout_ms = declare_parameter<int>("feedback_timeout_ms", 500);

  if (wheel_radius_m_ <= 0.0 || wheel_separation_m_ <= 0.0 || gear_ratio_ <= 0.0 ||
    max_motor_speed_rpm_ <= 0.0 || command_timeout_ms <= 0 || control_period_ms <= 0 ||
    can_receive_poll_ms <= 0 || feedback_timeout_ms <= 0)
  {
    throw std::invalid_argument("Motor geometry, limits, and timing parameters must be positive");
  }
  if (max_motor_speed_rpm_ > MotorCanProtocol::kMaxSpeedRpm) {
    throw std::invalid_argument("max_motor_speed_rpm exceeds the CAN protocol limit of 135 RPM");
  }

  command_timeout_ = std::chrono::milliseconds(command_timeout_ms);
  feedback_timeout_ = std::chrono::milliseconds(feedback_timeout_ms);

  if (enable_can_) {
    can_transport_ = std::make_unique<SocketCanTransport>(can_interface_);
    std::string error_message;
    if (!can_transport_->open(error_message)) {
      throw std::runtime_error(
        "Failed to open SocketCAN interface '" + can_interface_ + "': " + error_message);
    }
    RCLCPP_INFO(get_logger(), "SocketCAN transmission enabled on '%s'", can_interface_.c_str());
    can_receive_timer_ = create_wall_timer(
      std::chrono::milliseconds(can_receive_poll_ms),
      std::bind(&MotorControlNode::receive_can_frames, this));
  } else {
    RCLCPP_WARN(get_logger(), "SocketCAN transmission is disabled; running in dry-run mode");
  }

  motor_rpm_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(motor_rpm_topic_, 10);
  command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    command_topic_, 10,
    std::bind(&MotorControlNode::command_callback, this, std::placeholders::_1));
  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(control_period_ms),
    std::bind(&MotorControlNode::control_callback, this));
  enable_service_ = create_service<std_srvs::srv::Trigger>(
    "enable_motors",
    std::bind(
      &MotorControlNode::enable_motors, this, std::placeholders::_1, std::placeholders::_2));
  stop_service_ = create_service<std_srvs::srv::Trigger>(
    "stop_motors",
    std::bind(
      &MotorControlNode::stop_motors, this, std::placeholders::_1, std::placeholders::_2));
  reset_faults_service_ = create_service<std_srvs::srv::Trigger>(
    "reset_faults",
    std::bind(
      &MotorControlNode::reset_faults, this, std::placeholders::_1, std::placeholders::_2));
  emergency_stop_service_ = create_service<std_srvs::srv::SetBool>(
    "emergency_stop",
    std::bind(
      &MotorControlNode::set_emergency_stop, this,
      std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(
    get_logger(),
    "Motor-control node ready: subscribing to '%s', publishing RPM targets on '%s'",
    command_topic_.c_str(), motor_rpm_topic_.c_str());
}

MotorControlNode::~MotorControlNode()
{
  if (!can_transport_ || !can_transport_->is_open()) {
    return;
  }

  std::string ignored_error;
  const auto zero_speed = MotorCanProtocol::encode_wheel_speeds(0.0, 0.0);
  if (zero_speed) {
    can_transport_->send_command(*zero_speed, ignored_error);
  }
  const auto stop = MotorCanProtocol::encode_control(ControlCommand::kStop);
  if (stop) {
    can_transport_->send_command(*stop, ignored_error);
  }
  can_transport_->close();
}

void MotorControlNode::command_callback(const geometry_msgs::msg::Twist::SharedPtr message)
{
  if (!std::isfinite(message->linear.x) || !std::isfinite(message->angular.z)) {
    RCLCPP_WARN(get_logger(), "Ignoring cmd_vel containing a non-finite value");
    return;
  }

  linear_velocity_mps_ = message->linear.x;
  angular_velocity_radps_ = message->angular.z;
  last_command_time_ = std::chrono::steady_clock::now();
  command_received_ = true;
  watchdog_stopped_ = false;
}

void MotorControlNode::control_callback()
{
  const auto now = std::chrono::steady_clock::now();
  check_feedback_timeout(now);
  if (!command_received_ || now - last_command_time_ > command_timeout_) {
    if (!watchdog_stopped_) {
      publish_motor_rpm(0.0, 0.0);
      watchdog_stopped_ = true;
      if (command_received_) {
        RCLCPP_WARN(get_logger(), "Command watchdog expired; requesting zero motor speed");
      }
    }
    return;
  }

  const double left_velocity_mps =
    linear_velocity_mps_ - angular_velocity_radps_ * wheel_separation_m_ / 2.0;
  const double right_velocity_mps =
    linear_velocity_mps_ + angular_velocity_radps_ * wheel_separation_m_ / 2.0;
  const double rpm_per_mps = gear_ratio_ * kSecondsPerMinute / (kTwoPi * wheel_radius_m_);

  double left_rpm = std::clamp(
    left_velocity_mps * rpm_per_mps, -max_motor_speed_rpm_, max_motor_speed_rpm_);
  double right_rpm = std::clamp(
    right_velocity_mps * rpm_per_mps, -max_motor_speed_rpm_, max_motor_speed_rpm_);

  if (invert_left_motor_) {
    left_rpm = -left_rpm;
  }
  if (invert_right_motor_) {
    right_rpm = -right_rpm;
  }

  publish_motor_rpm(left_rpm, right_rpm);
}

void MotorControlNode::publish_motor_rpm(double left_rpm, double right_rpm)
{
  std_msgs::msg::Float64MultiArray message;
  message.data = {left_rpm, right_rpm};
  motor_rpm_publisher_->publish(std::move(message));

  if (!enable_can_ || !motion_commands_enabled_) {
    return;
  }

  const auto frame = MotorCanProtocol::encode_wheel_speeds(left_rpm, right_rpm);
  if (!frame) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Cannot encode motor RPM command (left=%.3f, right=%.3f)", left_rpm, right_rpm);
    return;
  }

  std::string error_message;
  if (!can_transport_->send_command(*frame, error_message)) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000, "CAN command transmission failed: %s",
      error_message.c_str());
  }
}

void MotorControlNode::receive_can_frames()
{
  constexpr std::size_t kMaximumFramesPerPoll = 64U;
  for (std::size_t index = 0U; index < kMaximumFramesPerPoll; ++index) {
    CanFrame frame;
    std::string error_message;
    const auto receive_status = can_transport_->receive(frame, error_message);
    if (receive_status == ReceiveStatus::kNoData) {
      return;
    }
    if (receive_status == ReceiveStatus::kError) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "CAN receive failed: %s", error_message.c_str());
      return;
    }

    const auto decoded = MotorCanProtocol::decode(frame);
    if (!decoded) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Ignoring malformed CAN frame 0x%03X (decode status %u)", frame.id,
        static_cast<unsigned int>(decoded.status));
      continue;
    }

    if (const auto * reply = std::get_if<FirmwareReply>(&*decoded.message)) {
      RCLCPP_INFO(
        get_logger(), "Motor-controller firmware identifier: %s",
        reply->identifier.c_str());
    } else if (const auto * reply = std::get_if<WheelSpeedsReply>(&*decoded.message)) {
      latest_wheel_speeds_ = *reply;
      last_feedback_time_ = std::chrono::steady_clock::now();
    } else if (const auto * reply = std::get_if<EncoderDeltasReply>(&*decoded.message)) {
      latest_encoder_deltas_ = *reply;
    } else if (const auto * report = std::get_if<EncoderPositionReport>(&*decoded.message)) {
      latest_encoder_positions_ = *report;
      last_feedback_time_ = std::chrono::steady_clock::now();
    } else if (const auto * report = std::get_if<MotorFaultReport>(&*decoded.message)) {
      latest_motor_fault_ = *report;
      fault_latched_ = true;
      RCLCPP_ERROR(
        get_logger(), "Motor %u reported fault mask 0x%04X",
        static_cast<unsigned int>(report->motor),
        static_cast<unsigned int>(report->fault_mask));
      latch_safety_stop("motor fault report");
    }
  }

  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "CAN receive backlog exceeded %zu frames in one poll", kMaximumFramesPerPoll);
}

void MotorControlNode::enable_motors(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  static_cast<void>(request);
  if (!enable_can_) {
    response->success = false;
    response->message = "SocketCAN is disabled";
    return;
  }
  if (fault_latched_ || feedback_timeout_latched_) {
    response->success = false;
    response->message = "A safety condition is latched; call reset_faults before enabling";
    return;
  }

  std::string error_message;
  const ControlCommand sequence[] = {
    ControlCommand::kEnableStage1,
    ControlCommand::kEnableStage2,
    ControlCommand::kEnableOperation,
  };
  for (const auto command : sequence) {
    if (!send_control_command(command, error_message)) {
      motion_commands_enabled_ = false;
      std::string stop_error;
      send_control_command(ControlCommand::kEmergencyStopImmediate, stop_error);
      response->success = false;
      response->message = "Enable sequence failed: " + error_message;
      return;
    }
  }

  const auto zero_speed = MotorCanProtocol::encode_wheel_speeds(0.0, 0.0);
  if (!zero_speed || !can_transport_->send_command(*zero_speed, error_message)) {
    motion_commands_enabled_ = false;
    std::string stop_error;
    send_control_command(ControlCommand::kEmergencyStopImmediate, stop_error);
    response->success = false;
    response->message = "Could not send initial zero-speed command: " + error_message;
    return;
  }

  last_feedback_time_ = std::chrono::steady_clock::now();
  command_received_ = false;
  watchdog_stopped_ = false;
  motion_commands_enabled_ = true;
  response->success = true;
  response->message = "Enable sequence transmitted; controller acknowledgement is unavailable";
}

void MotorControlNode::stop_motors(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  static_cast<void>(request);
  motion_commands_enabled_ = false;
  command_received_ = false;
  std::string error_message;
  response->success = send_control_command(ControlCommand::kStop, error_message);
  response->message = response->success ?
    "Stop command transmitted; controller acknowledgement is unavailable" : error_message;
}

void MotorControlNode::reset_faults(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  static_cast<void>(request);
  motion_commands_enabled_ = false;
  command_received_ = false;
  std::string error_message;
  response->success = send_control_command(ControlCommand::kResetFaults, error_message);
  if (!response->success) {
    response->message = error_message;
    return;
  }

  fault_latched_ = false;
  feedback_timeout_latched_ = false;
  latest_motor_fault_.reset();
  last_feedback_time_ = std::chrono::steady_clock::now();
  response->message =
    "Fault reset transmitted; controller acknowledgement is unavailable; enable is still required";
}

void MotorControlNode::set_emergency_stop(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  motion_commands_enabled_ = false;
  command_received_ = false;
  const auto command = request->data ?
    ControlCommand::kEmergencyStopImmediate : ControlCommand::kEnableOperation;
  std::string error_message;
  response->success = send_control_command(command, error_message);
  if (!response->success) {
    response->message = error_message;
    return;
  }

  response->message = request->data ?
    "Immediate emergency-stop command transmitted" :
    "Emergency-stop release transmitted; enable_motors is still required";
}

bool MotorControlNode::send_control_command(
  ControlCommand command, std::string & error_message)
{
  if (!enable_can_ || !can_transport_ || !can_transport_->is_open()) {
    error_message = "SocketCAN is disabled or unavailable";
    return false;
  }
  const auto frame = MotorCanProtocol::encode_control(command);
  if (!frame) {
    error_message = "Invalid motor-control command";
    return false;
  }
  return can_transport_->send_command(*frame, error_message);
}

void MotorControlNode::check_feedback_timeout(std::chrono::steady_clock::time_point now)
{
  if (!enable_can_ || !motion_commands_enabled_ ||
    now - last_feedback_time_ <= feedback_timeout_)
  {
    return;
  }

  feedback_timeout_latched_ = true;
  RCLCPP_ERROR(get_logger(), "Motor feedback timed out; latching an emergency stop");
  latch_safety_stop("feedback timeout");
}

void MotorControlNode::latch_safety_stop(const char * reason)
{
  const bool was_enabled = motion_commands_enabled_;
  motion_commands_enabled_ = false;
  command_received_ = false;
  if (!was_enabled) {
    return;
  }

  std::string error_message;
  if (!send_control_command(ControlCommand::kEmergencyStopImmediate, error_message)) {
    RCLCPP_ERROR(
      get_logger(), "Failed to transmit emergency stop after %s: %s", reason,
      error_message.c_str());
  }
}

}  // namespace motor_control_g4dual
