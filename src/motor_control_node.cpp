// Copyright 2026 Gray Lin
// SPDX-License-Identifier: MIT

#include "motor_control_g4dual/motor_control_node.hpp"

#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "motor_control_g4dual/motor_can_protocol.hpp"

namespace motor_control_g4dual
{
namespace
{
constexpr double kSecondsPerMinute = 60.0;
constexpr double kTwoPi = 6.28318530717958647692;

const char * control_command_name(ControlCommand command)
{
  switch (command) {
    case ControlCommand::kEnableStage1:
      return "enable stage 1";
    case ControlCommand::kEnableStage2:
      return "enable stage 2";
    case ControlCommand::kEnableOperation:
      return "enable operation";
    case ControlCommand::kStop:
      return "stop";
    case ControlCommand::kEmergencyStop:
      return "emergency stop";
    case ControlCommand::kEmergencyStopRamp:
      return "emergency ramp stop";
    case ControlCommand::kResetFaults:
      return "reset faults";
    default:
      return "unknown";
  }
}
}

MotorControlNode::MotorControlNode(const rclcpp::NodeOptions & options, bool info_enabled)
: Node("motor_control", options),
  last_command_time_(std::chrono::steady_clock::now()),
  last_feedback_time_(std::chrono::steady_clock::now())
{
  declare_parameter<bool>("info", info_enabled);
  command_topic_ = declare_parameter<std::string>("command_topic", "cmd_vel");
  motor_rpm_topic_ = declare_parameter<std::string>("motor_rpm_topic", "motor_rpm_command");
  can_interface_ = declare_parameter<std::string>("can_interface", "can0");
  odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
  base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");
  left_joint_name_ = declare_parameter<std::string>("left_joint_name", "left_wheel_joint");
  right_joint_name_ = declare_parameter<std::string>("right_joint_name", "right_wheel_joint");
  enable_can_ = declare_parameter<bool>("enable_can", false);
  publish_odom_tf_ = declare_parameter<bool>("publish_odom_tf", true);
  wheel_radius_m_ = declare_parameter<double>("wheel_radius_m", 0.085);
  wheel_separation_m_ = declare_parameter<double>("wheel_separation_m", 0.4762);
  gear_ratio_ = declare_parameter<double>("gear_ratio", 1.0);
  max_motor_speed_rpm_ = declare_parameter<double>("max_motor_speed_rpm", 135.0);
  invert_left_motor_ = declare_parameter<bool>("invert_left_motor", false);
  invert_right_motor_ = declare_parameter<bool>("invert_right_motor", true);

  rcl_interfaces::msg::ParameterDescriptor rpm_resolution_descriptor;
  rpm_resolution_descriptor.description =
    "CAN speed resolution in RPM per unit: 1.0 or 0.1; must match controller firmware";
  rpm_resolution_descriptor.read_only = true;
  rpm_resolution_ = declare_parameter<double>(
    "rpm_resolution", MotorCanProtocol::kDefaultRpmResolution, rpm_resolution_descriptor);
  if (!MotorCanProtocol::is_valid_rpm_resolution(rpm_resolution_)) {
    throw std::invalid_argument("rpm_resolution must be 1.0 or 0.1 RPM per CAN unit");
  }

  rcl_interfaces::msg::ParameterDescriptor startup_descriptor;
  startup_descriptor.read_only = true;
  startup_descriptor.description = "Expect one acknowledgement per control write; diagnostics only";
  const bool expect_control_ack = declare_parameter<bool>(
    "expect_control_ack", false, startup_descriptor);
  startup_descriptor.description = "Reply deadline in milliseconds";
  const int reply_timeout_ms = declare_parameter<int>(
    "reply_timeout_ms", 500, startup_descriptor);
  startup_descriptor.description = "Optional expected raw firmware identifier";
  expected_firmware_identifier_ = declare_parameter<std::string>(
    "expected_firmware_identifier", "", startup_descriptor);
  if (reply_timeout_ms <= 0) {
    throw std::invalid_argument("reply_timeout_ms must be positive");
  }
  control_replies_ = ControlReplyTracker(
    enable_can_ && expect_control_ack, std::chrono::milliseconds(reply_timeout_ms));

  const auto command_timeout_ms = declare_parameter<int>("command_timeout_ms", 500);
  const auto control_period_ms = declare_parameter<int>("control_period_ms", 50);
  const auto can_receive_poll_ms = declare_parameter<int>("can_receive_poll_ms", 50);
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
  kinematics_ = std::make_unique<DifferentialDriveKinematics>(
    wheel_radius_m_, wheel_separation_m_, gear_ratio_, max_motor_speed_rpm_,
    invert_left_motor_, invert_right_motor_);

  if (enable_can_) {
    can_transport_ = std::make_unique<SocketCanTransport>(can_interface_);
    std::string error_message;
    if (!can_transport_->open(error_message)) {
      throw std::runtime_error(
        "Failed to open SocketCAN interface '" + can_interface_ + "': " + error_message);
    }
    if (get_parameter("info").as_bool()) {
      RCLCPP_INFO(get_logger(), "SocketCAN transmission enabled on '%s'", can_interface_.c_str());
    }
    can_receive_timer_ = create_wall_timer(
      std::chrono::milliseconds(can_receive_poll_ms),
      std::bind(&MotorControlNode::receive_can_frames, this));
  } else {
    RCLCPP_WARN(get_logger(), "SocketCAN transmission is disabled; running in dry-run mode");
  }

  motor_rpm_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(motor_rpm_topic_, 10);
  wheel_speed_publisher_ =
    create_publisher<std_msgs::msg::Float64MultiArray>("wheel_speed_feedback", 10);
  encoder_delta_publisher_ =
    create_publisher<std_msgs::msg::Int32MultiArray>("encoder_delta_feedback", 10);
  motor_fault_publisher_ =
    create_publisher<std_msgs::msg::UInt32MultiArray>("motor_fault", 10);
  joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  diagnostics_publisher_ =
    create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);
  if (publish_odom_tf_) {
    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }
  command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    command_topic_, 10,
    std::bind(&MotorControlNode::command_callback, this, std::placeholders::_1));
  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(control_period_ms),
    std::bind(&MotorControlNode::control_callback, this));
  diagnostics_timer_ = create_wall_timer(
    std::chrono::seconds(1), std::bind(&MotorControlNode::publish_diagnostics, this));
  // Control events must not be replayed to a restarted node.
  const auto control_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  enable_subscription_ = create_subscription<std_msgs::msg::Empty>(
    "enable_motors", control_qos,
    std::bind(&MotorControlNode::enable_motors, this, std::placeholders::_1));
  stop_subscription_ = create_subscription<std_msgs::msg::Empty>(
    "stop_motors", control_qos,
    std::bind(&MotorControlNode::stop_motors, this, std::placeholders::_1));
  reset_faults_subscription_ = create_subscription<std_msgs::msg::Empty>(
    "reset_faults", control_qos,
    std::bind(&MotorControlNode::reset_faults, this, std::placeholders::_1));
  emergency_stop_subscription_ = create_subscription<std_msgs::msg::Bool>(
    "emergency_stop", control_qos,
    std::bind(&MotorControlNode::set_emergency_stop, this, std::placeholders::_1));

  configure_motion_test();

  if (enable_can_) {
    std::string error_message;
    if (can_transport_->send_command(
        MotorCanProtocol::encode_firmware_version_request(), error_message))
    {
      firmware_query_state_ = "pending";
      firmware_deadline_ = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(reply_timeout_ms);
    } else {
      firmware_query_state_ = "transmission_failed";
      RCLCPP_WARN(get_logger(), "Firmware query transmission failed: %s", error_message.c_str());
    }
    RCLCPP_INFO(
      get_logger(), "Configured CAN speed resolution: %.1f RPM/unit (not verified by firmware)",
      rpm_resolution_);
  }

  if (get_parameter("info").as_bool()) {
    RCLCPP_INFO(
      get_logger(),
      "Node ready: cmd='%s', rpm='%s', CAN=%s, control=%d ms, watchdog=%d ms, RPM resolution=%.1f",
      command_topic_.c_str(), motor_rpm_topic_.c_str(), enable_can_ ? "enabled" : "disabled",
      control_period_ms, command_timeout_ms, rpm_resolution_);
  }
}

MotorControlNode::~MotorControlNode()
{
  try {
    abort_motion_test("node shutdown");
  } catch (const std::exception & error) {
    // ROS publishers may already be invalid after context shutdown. Always
    // continue to the direct CAN zero/stop sequence below.
    RCLCPP_WARN(get_logger(), "Could not finalize motion test at shutdown: %s", error.what());
  }
  if (!can_transport_ || !can_transport_->is_open()) {
    if (get_parameter("info").as_bool()) {
      RCLCPP_INFO(get_logger(), "Node shutting down; no open CAN transport to stop");
    }
    return;
  }

  if (get_parameter("info").as_bool()) {
    RCLCPP_INFO(get_logger(), "Node shutting down; sending zero speed and stop");
  }

  std::string ignored_error;
  const auto zero_speed = MotorCanProtocol::encode_wheel_speeds(0.0, 0.0, rpm_resolution_);
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
  if (motion_test_.active()) {
    abort_motion_test("external cmd_vel received; command discarded");
    return;
  }
  if (!std::isfinite(message->linear.x) || !std::isfinite(message->angular.z)) {
    RCLCPP_WARN(get_logger(), "Ignoring cmd_vel containing a non-finite value");
    return;
  }

  linear_velocity_mps_ = message->linear.x;
  angular_velocity_radps_ = message->angular.z;
  last_command_time_ = std::chrono::steady_clock::now();
  command_received_ = true;
  watchdog_stopped_ = false;
  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 1000, "cmd_vel received: linear=%.3f m/s, angular=%.3f rad/s",
    linear_velocity_mps_, angular_velocity_radps_);
}

void MotorControlNode::control_callback()
{
  const auto now = std::chrono::steady_clock::now();
  check_reply_timeouts();
  check_feedback_timeout(now);
  if (motion_test_.active()) {
    update_motion_test();
    return;
  }
  if (!command_received_ || now - last_command_time_ > command_timeout_) {
    if (!watchdog_stopped_) {
      publish_motor_rpm(0.0, 0.0);
      watchdog_stopped_ = true;
      if (command_received_) {
        ++command_timeout_count_;
        const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - last_command_time_).count();
        RCLCPP_WARN(
          get_logger(),
          "Command watchdog expired: age=%lld ms, limit=%lld ms; requesting zero speed",
          static_cast<long long>(age), static_cast<long long>(command_timeout_.count()));
        publish_diagnostics();
      }
    }
    return;
  }

  const auto motor_rpm =
    kinematics_->twist_to_motor_rpm(linear_velocity_mps_, angular_velocity_radps_);
  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 1000, "Control update: target left=%.2f RPM, right=%.2f RPM",
    motor_rpm.left, motor_rpm.right);
  publish_motor_rpm(motor_rpm.left, motor_rpm.right);
}

void MotorControlNode::publish_motor_rpm(double left_rpm, double right_rpm)
{
  std_msgs::msg::Float64MultiArray message;
  message.data = {left_rpm, right_rpm};
  motor_rpm_publisher_->publish(std::move(message));
  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 1000, "Published RPM target: left=%.2f, right=%.2f",
    left_rpm, right_rpm);

  if (!enable_can_ || !motion_commands_enabled_) {
    return;
  }

  const auto frame = MotorCanProtocol::encode_wheel_speeds(left_rpm, right_rpm, rpm_resolution_);
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
    abort_motion_test("CAN speed transmission failed");
  } else {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000, "CAN speed command transmitted on 0x%03X", frame->id);
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
      abort_motion_test("CAN receive failed");
      return;
    }

    check_reply_timeouts();
    const auto decoded = MotorCanProtocol::decode(
      frame, rpm_resolution_, firmware_query_state_ == "pending");
    if (!decoded) {
      ++rejected_frame_count_;
      last_rejected_frame_ = MotorCanProtocol::format_frame(frame);
      last_rejected_reason_ = MotorCanProtocol::decode_status_name(decoded.status);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Ignoring CAN frame: %s; %s (rejected=%llu)",
        last_rejected_frame_.c_str(), last_rejected_reason_.c_str(),
        static_cast<unsigned long long>(rejected_frame_count_));
      continue;
    }

    const auto stamp = now();
    if (const auto * reply = std::get_if<WriteAcknowledgement>(&*decoded.message)) {
      ++acknowledgement_count_;
      last_acknowledgement_ = MotorCanProtocol::format_frame(frame);
      const auto previous_state = control_replies_.state();
      control_replies_.observe(
        reply->index, reply->subindex, false, 0U, std::chrono::steady_clock::now());
      if (previous_state != control_replies_.state()) {
        publish_diagnostics();
      }
    } else if (const auto * reply = std::get_if<AbortReply>(&*decoded.message)) {
      ++abort_reply_count_;
      last_abort_reply_ = MotorCanProtocol::format_frame(frame);
      const auto previous_control_state = control_replies_.state();
      const auto previous_firmware_state = firmware_query_state_;
      control_replies_.observe(
        reply->index, reply->subindex, true, reply->code, std::chrono::steady_clock::now());
      if (reply->index == 0x2031U && reply->subindex == 0U &&
        firmware_query_state_ == "pending")
      {
        firmware_query_state_ = "rejected";
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Controller abort: object=0x%04X:%02X code=0x%08X; %s",
        static_cast<unsigned int>(reply->index), static_cast<unsigned int>(reply->subindex),
        static_cast<unsigned int>(reply->code), last_abort_reply_.c_str());
      if (previous_control_state != control_replies_.state() ||
        previous_firmware_state != firmware_query_state_)
      {
        publish_diagnostics();
      }
    } else if (const auto * reply = std::get_if<FirmwareReply>(&*decoded.message)) {
      firmware_query_state_ = "received";
      firmware_identifier_ = reply->identifier;
      if (!expected_firmware_identifier_.empty() &&
        firmware_identifier_ != expected_firmware_identifier_)
      {
        RCLCPP_WARN(
          get_logger(), "Firmware identifier mismatch: expected '%s', received '%s'",
          expected_firmware_identifier_.c_str(), firmware_identifier_.c_str());
      }
      publish_diagnostics();
      if (get_parameter("info").as_bool()) {
        RCLCPP_INFO(
          get_logger(), "Motor-controller firmware identifier: %s",
          reply->identifier.c_str());
      }
    } else if (const auto * reply = std::get_if<WheelSpeedsReply>(&*decoded.message)) {
      latest_wheel_speeds_ = *reply;
      last_speed_time_ = std::chrono::steady_clock::now();
      last_feedback_time_ = std::chrono::steady_clock::now();
      feedback_received_ = true;
      publish_wheel_speeds(*reply, stamp);
    } else if (const auto * reply = std::get_if<EncoderDeltasReply>(&*decoded.message)) {
      latest_encoder_deltas_ = *reply;
      last_feedback_time_ = std::chrono::steady_clock::now();
      feedback_received_ = true;
      publish_encoder_deltas(*reply);
    } else if (const auto * report = std::get_if<EncoderPositionReport>(&*decoded.message)) {
      latest_encoder_positions_ = *report;
      last_feedback_time_ = std::chrono::steady_clock::now();
      feedback_received_ = true;
      update_encoder_odometry(*report, stamp);
    } else if (const auto * report = std::get_if<MotorFaultReport>(&*decoded.message)) {
      latest_motor_fault_ = *report;
      fault_latched_ = true;
      RCLCPP_ERROR(
        get_logger(), "Motor %u reported fault mask 0x%04X",
        static_cast<unsigned int>(report->motor),
        static_cast<unsigned int>(report->fault_mask));
      latch_safety_stop("motor fault report");
      publish_motor_fault(*report);
      publish_diagnostics();
    }
  }

  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "CAN receive backlog exceeded %zu frames in one poll", kMaximumFramesPerPoll);
}

void MotorControlNode::enable_motors(const std_msgs::msg::Empty::SharedPtr message)
{
  static_cast<void>(message);
  if (motion_test_.active()) {
    abort_motion_test("enable event during test");
    report_control_result("enable_motors", false, "Test interrupted; enable again explicitly");
    return;
  }
  if (!enable_can_) {
    report_control_result("enable_motors", false, "SocketCAN is disabled");
    return;
  }
  if (fault_latched_ || feedback_timeout_latched_) {
    report_control_result(
      "enable_motors", false,
      "A safety condition is latched; publish reset_faults before enabling");
    return;
  }

  std::string error_message;
  control_replies_.begin(3U, std::chrono::steady_clock::now());
  const ControlCommand sequence[] = {
    ControlCommand::kEnableStage1,
    ControlCommand::kEnableStage2,
    ControlCommand::kEnableOperation,
  };
  for (const auto command : sequence) {
    if (!send_control_command(command, error_message)) {
      motion_commands_enabled_ = false;
      std::string stop_error;
      send_control_command(ControlCommand::kEmergencyStop, stop_error);
      report_control_result("enable_motors", false, "Enable sequence failed: " + error_message);
      return;
    }
  }

  const auto zero_speed = MotorCanProtocol::encode_wheel_speeds(0.0, 0.0, rpm_resolution_);
  if (!zero_speed || !can_transport_->send_command(*zero_speed, error_message)) {
    motion_commands_enabled_ = false;
    std::string stop_error;
    send_control_command(ControlCommand::kEmergencyStop, stop_error);
    report_control_result(
      "enable_motors", false, "Could not send initial zero-speed command: " + error_message);
    return;
  }

  last_feedback_time_ = std::chrono::steady_clock::now();
  command_received_ = false;
  watchdog_stopped_ = false;
  motion_commands_enabled_ = true;
  report_control_result(
    "enable_motors", true,
    "Enable sequence transmitted; see last_control_reply_state for controller reply status");
}

void MotorControlNode::stop_motors(const std_msgs::msg::Empty::SharedPtr message)
{
  abort_motion_test("stop_motors event");
  static_cast<void>(message);
  motion_commands_enabled_ = false;
  command_received_ = false;
  std::string error_message;
  control_replies_.begin(1U, std::chrono::steady_clock::now());
  const bool success = send_control_command(ControlCommand::kStop, error_message);
  report_control_result(
    "stop_motors", success, success ?
    "Stop command transmitted; see last_control_reply_state for controller reply status" :
    error_message);
}

void MotorControlNode::reset_faults(const std_msgs::msg::Empty::SharedPtr message)
{
  abort_motion_test("reset_faults event");
  static_cast<void>(message);
  motion_commands_enabled_ = false;
  command_received_ = false;
  std::string error_message;
  control_replies_.begin(1U, std::chrono::steady_clock::now());
  if (!send_control_command(ControlCommand::kResetFaults, error_message)) {
    report_control_result("reset_faults", false, error_message);
    return;
  }

  fault_latched_ = false;
  feedback_timeout_latched_ = false;
  latest_motor_fault_.reset();
  last_feedback_time_ = std::chrono::steady_clock::now();
  report_control_result(
    "reset_faults", true,
    "Fault reset transmitted; see last_control_reply_state for controller reply status; "
    "enable is still required");
}

void MotorControlNode::set_emergency_stop(const std_msgs::msg::Bool::SharedPtr message)
{
  abort_motion_test("emergency_stop event");
  motion_commands_enabled_ = false;
  command_received_ = false;
  const auto command = message->data ?
    ControlCommand::kEmergencyStop : ControlCommand::kEnableOperation;
  std::string error_message;
  control_replies_.begin(1U, std::chrono::steady_clock::now());
  if (!send_control_command(command, error_message)) {
    report_control_result("emergency_stop", false, error_message);
    return;
  }

  report_control_result(
    "emergency_stop", true, message->data ?
    "Emergency-stop command transmitted (ramp-stop behavior)" :
    "Emergency-stop release transmitted; enable_motors is still required");
}

void MotorControlNode::report_control_result(
  const char * command, bool success, const std::string & message)
{
  last_control_command_ = command;
  last_control_success_ = success;
  last_control_message_ = message;
  if (!success) {
    control_replies_.transmission_failed();
    RCLCPP_ERROR(get_logger(), "%s failed: %s", command, message.c_str());
  } else if (get_parameter("info").as_bool()) {
    RCLCPP_INFO(get_logger(), "%s: %s", command, message.c_str());
  }
  publish_diagnostics();
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
  const bool sent = can_transport_->send_command(*frame, error_message);
  if (!sent) {
    RCLCPP_ERROR(
      get_logger(), "CAN control command failed (%s): %s", control_command_name(command),
      error_message.c_str());
  } else if (get_parameter("info").as_bool()) {
    RCLCPP_INFO(get_logger(), "CAN control command sent: %s", control_command_name(command));
  }
  return sent;
}

void MotorControlNode::check_feedback_timeout(std::chrono::steady_clock::time_point now)
{
  if (!enable_can_ || !motion_commands_enabled_ ||
    now - last_feedback_time_ <= feedback_timeout_)
  {
    return;
  }

  feedback_timeout_latched_ = true;
  const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
    now - last_feedback_time_).count();
  RCLCPP_ERROR(
    get_logger(), "Motor feedback timed out: age=%lld ms, limit=%lld ms; latching emergency stop",
    static_cast<long long>(age), static_cast<long long>(feedback_timeout_.count()));
  latch_safety_stop("feedback timeout");
  publish_diagnostics();
}

void MotorControlNode::latch_safety_stop(const char * reason)
{
  abort_motion_test(reason);
  const bool was_enabled = motion_commands_enabled_;
  motion_commands_enabled_ = false;
  command_received_ = false;
  if (!was_enabled) {
    if (get_parameter("info").as_bool()) {
      RCLCPP_INFO(get_logger(), "Safety stop already gated (%s)", reason);
    }
    return;
  }

  RCLCPP_WARN(get_logger(), "Latching safety stop: %s", reason);
  control_replies_.interrupt();

  std::string error_message;
  if (!send_control_command(ControlCommand::kEmergencyStop, error_message)) {
    RCLCPP_ERROR(
      get_logger(), "Failed to transmit emergency stop after %s: %s", reason,
      error_message.c_str());
  }
}

void MotorControlNode::publish_wheel_speeds(
  const WheelSpeedsReply & reply, const rclcpp::Time & stamp)
{
  const auto wheel_rpm =
    kinematics_->motor_values_to_wheel(reply.m1_speed_rpm, reply.m2_speed_rpm);

  std_msgs::msg::Float64MultiArray speed_message;
  speed_message.data = {wheel_rpm.left, wheel_rpm.right};
  wheel_speed_publisher_->publish(std::move(speed_message));
  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 1000, "Wheel-speed feedback: left=%.2f RPM, right=%.2f RPM",
    wheel_rpm.left, wheel_rpm.right);

  sensor_msgs::msg::JointState joint_message;
  joint_message.header.stamp = stamp;
  joint_message.name = {left_joint_name_, right_joint_name_};
  joint_message.position = {left_joint_position_rad_, right_joint_position_rad_};
  joint_message.velocity = {
    wheel_rpm.left * kTwoPi / kSecondsPerMinute,
    wheel_rpm.right * kTwoPi / kSecondsPerMinute};
  joint_state_publisher_->publish(std::move(joint_message));
}

void MotorControlNode::publish_encoder_deltas(const EncoderDeltasReply & reply)
{
  const auto wheel_delta = kinematics_->motor_values_to_wheel(reply.m1_delta, reply.m2_delta);

  std_msgs::msg::Int32MultiArray message;
  message.data = {
    static_cast<std::int32_t>(wheel_delta.left),
    static_cast<std::int32_t>(wheel_delta.right)};
  encoder_delta_publisher_->publish(std::move(message));
  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 1000, "Encoder delta: left=%d, right=%d counts",
    static_cast<std::int32_t>(wheel_delta.left),
    static_cast<std::int32_t>(wheel_delta.right));
}

void MotorControlNode::update_encoder_odometry(
  const EncoderPositionReport & report, const rclcpp::Time & stamp)
{
  const double radians_per_count = kTwoPi / MotorCanProtocol::kEncoderCountsPerRevolution;
  if (!encoder_initialized_) {
    previous_m1_position_ = report.m1_position;
    previous_m2_position_ = report.m2_position;
    const auto wheel_position = kinematics_->motor_values_to_wheel(
      static_cast<double>(report.m1_position), static_cast<double>(report.m2_position));
    left_joint_position_rad_ = wheel_position.left * radians_per_count;
    right_joint_position_rad_ = wheel_position.right * radians_per_count;
    encoder_initialized_ = true;
    if (get_parameter("info").as_bool()) {
      RCLCPP_INFO(
        get_logger(), "Odometry initialized from encoder positions: M1=%d, M2=%d",
        report.m1_position, report.m2_position);
    }
  } else {
    const auto m1_delta = DifferentialDriveKinematics::wrapped_encoder_delta(
      report.m1_position, previous_m1_position_);
    const auto m2_delta = DifferentialDriveKinematics::wrapped_encoder_delta(
      report.m2_position, previous_m2_position_);
    previous_m1_position_ = report.m1_position;
    previous_m2_position_ = report.m2_position;

    const auto wheel_delta = kinematics_->motor_values_to_wheel(
      static_cast<double>(m1_delta), static_cast<double>(m2_delta));

    const double maximum_plausible_delta =
      MotorCanProtocol::kMaxSpeedRpm * MotorCanProtocol::kEncoderCountsPerRevolution *
      static_cast<double>(feedback_timeout_.count()) / 60000.0 * 2.0;
    if (std::abs(wheel_delta.left) > maximum_plausible_delta ||
      std::abs(wheel_delta.right) > maximum_plausible_delta)
    {
      RCLCPP_WARN(
        get_logger(),
        "Rebasing after implausible encoder jump (left=%lld, right=%lld counts)",
        static_cast<long long>(wheel_delta.left),
        static_cast<long long>(wheel_delta.right));
      abort_motion_test("implausible encoder jump");
      return;
    }

    const double left_delta_rad = wheel_delta.left * radians_per_count;
    const double right_delta_rad = wheel_delta.right * radians_per_count;
    left_joint_position_rad_ += left_delta_rad;
    right_joint_position_rad_ += right_delta_rad;

    const double left_distance_m = left_delta_rad * wheel_radius_m_;
    const double right_distance_m = right_delta_rad * wheel_radius_m_;
    const double distance_m = (left_distance_m + right_distance_m) / 2.0;
    const double yaw_delta = (right_distance_m - left_distance_m) / wheel_separation_m_;
    odom_continuous_yaw_ += yaw_delta;
    odom_x_m_ += distance_m * std::cos(odom_yaw_rad_ + yaw_delta / 2.0);
    odom_y_m_ += distance_m * std::sin(odom_yaw_rad_ + yaw_delta / 2.0);
    odom_yaw_rad_ = std::atan2(
      std::sin(odom_yaw_rad_ + yaw_delta), std::cos(odom_yaw_rad_ + yaw_delta));
  }

  last_odom_time_ = std::chrono::steady_clock::now();
  WheelPair wheel_rpm;
  if (latest_wheel_speeds_) {
    wheel_rpm = kinematics_->motor_values_to_wheel(
      latest_wheel_speeds_->m1_speed_rpm, latest_wheel_speeds_->m2_speed_rpm);
  }
  const double left_velocity_mps =
    wheel_rpm.left * kTwoPi * wheel_radius_m_ / kSecondsPerMinute;
  const double right_velocity_mps =
    wheel_rpm.right * kTwoPi * wheel_radius_m_ / kSecondsPerMinute;

  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = stamp;
  odometry.header.frame_id = odom_frame_id_;
  odometry.child_frame_id = base_frame_id_;
  odometry.pose.pose.position.x = odom_x_m_;
  odometry.pose.pose.position.y = odom_y_m_;
  odometry.pose.pose.orientation.z = std::sin(odom_yaw_rad_ / 2.0);
  odometry.pose.pose.orientation.w = std::cos(odom_yaw_rad_ / 2.0);
  odometry.twist.twist.linear.x = (left_velocity_mps + right_velocity_mps) / 2.0;
  odometry.twist.twist.angular.z =
    (right_velocity_mps - left_velocity_mps) / wheel_separation_m_;
  odometry.pose.covariance.fill(0.0);
  odometry.pose.covariance[0] = 0.01;
  odometry.pose.covariance[7] = 0.01;
  odometry.pose.covariance[14] = 1000000.0;
  odometry.pose.covariance[21] = 1000000.0;
  odometry.pose.covariance[28] = 1000000.0;
  odometry.pose.covariance[35] = 0.05;
  odometry.twist.covariance = odometry.pose.covariance;
  odometry_publisher_->publish(odometry);
  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "Odometry: x=%.3f m, y=%.3f m, yaw=%.3f rad, linear=%.3f m/s, angular=%.3f rad/s",
    odom_x_m_, odom_y_m_, odom_yaw_rad_, odometry.twist.twist.linear.x,
    odometry.twist.twist.angular.z);

  sensor_msgs::msg::JointState joint_message;
  joint_message.header.stamp = stamp;
  joint_message.name = {left_joint_name_, right_joint_name_};
  joint_message.position = {left_joint_position_rad_, right_joint_position_rad_};
  joint_message.velocity = {
    wheel_rpm.left * kTwoPi / kSecondsPerMinute,
    wheel_rpm.right * kTwoPi / kSecondsPerMinute};
  joint_state_publisher_->publish(std::move(joint_message));

  if (transform_broadcaster_) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odometry.header;
    transform.child_frame_id = base_frame_id_;
    transform.transform.translation.x = odom_x_m_;
    transform.transform.translation.y = odom_y_m_;
    transform.transform.rotation = odometry.pose.pose.orientation;
    transform_broadcaster_->sendTransform(transform);
  }
}

void MotorControlNode::publish_motor_fault(const MotorFaultReport & report)
{
  std_msgs::msg::UInt32MultiArray message;
  message.data = {
    static_cast<std::uint32_t>(report.motor),
    static_cast<std::uint32_t>(report.fault_mask)};
  motor_fault_publisher_->publish(std::move(message));
  if (get_parameter("info").as_bool()) {
    RCLCPP_INFO(
      get_logger(), "Published motor fault: motor=%u, mask=0x%04X",
      static_cast<unsigned int>(report.motor), static_cast<unsigned int>(report.fault_mask));
  }
}

void MotorControlNode::check_reply_timeouts()
{
  const auto current = std::chrono::steady_clock::now();
  control_replies_.expire(current);
  if (firmware_query_state_ == "pending" && current >= firmware_deadline_) {
    firmware_query_state_ = "timed_out";
    RCLCPP_WARN(get_logger(), "Firmware query timed out; controller identity remains unknown");
  }
}

void MotorControlNode::publish_diagnostics()
{
  check_reply_timeouts();
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = get_fully_qualified_name() + std::string(": motor controller");
  status.hardware_id = can_interface_;

  if (fault_latched_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "Motor fault latched";
  } else if (feedback_timeout_latched_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "Motor feedback timeout latched";
  } else if (!enable_can_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "Dry-run mode; SocketCAN disabled";
  } else if (!motion_commands_enabled_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "Motor commands safely gated";
  } else if (command_received_ && watchdog_stopped_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "Command input timed out; zero speed requested";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "Motor control enabled";
  }
  const bool firmware_mismatch = !expected_firmware_identifier_.empty() &&
    !firmware_identifier_.empty() && firmware_identifier_ != expected_firmware_identifier_;
  const auto & reply_state = control_replies_.state();
  if (status.level == diagnostic_msgs::msg::DiagnosticStatus::OK &&
    (firmware_mismatch || reply_state == "rejected" || reply_state == "timed_out" ||
    reply_state == "ambiguous" || reply_state == "transmission_failed"))
  {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = firmware_mismatch ? "Firmware identifier mismatch" :
      "Controller reply status: " + reply_state;
  }

  const auto add_value =
    [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = value;
      status.values.push_back(std::move(item));
    };
  if (last_control_success_.has_value()) {
    add_value("last_control_command", last_control_command_);
    add_value("last_control_success", *last_control_success_ ? "true" : "false");
    add_value("last_control_message", last_control_message_);
  }
  add_value("motion_test_state", motion_test_.state());
  add_value("motion_test_kind", motion_test_.kind());
  add_value("motion_test_segment", std::to_string(motion_test_.segment()));
  add_value("motion_test_reason", motion_test_.reason());
  add_value("motion_test_log", motion_test_log_path_);
  add_value("last_control_reply_state", reply_state);
  add_value("control_replies_remaining", std::to_string(control_replies_.remaining()));
  add_value("control_abort_code", std::to_string(control_replies_.abort_code()));
  add_value("firmware_query_state", firmware_query_state_);
  add_value("firmware_identifier", firmware_identifier_.empty() ? "unknown" : firmware_identifier_);
  add_value("expected_firmware_identifier", expected_firmware_identifier_);
  add_value("firmware_identity_matches", expected_firmware_identifier_.empty() ||
    firmware_identifier_.empty() ? "unknown" : (firmware_mismatch ? "false" : "true"));
  add_value("rpm_resolution_verified", "false");
  add_value("rejected_frame_count", std::to_string(rejected_frame_count_));
  add_value("acknowledgement_count", std::to_string(acknowledgement_count_));
  add_value("abort_reply_count", std::to_string(abort_reply_count_));
  add_value("last_rejected_frame", last_rejected_frame_);
  add_value("last_rejected_reason", last_rejected_reason_);
  add_value("last_acknowledgement", last_acknowledgement_);
  add_value("last_abort_reply", last_abort_reply_);
  add_value("command_received", command_received_ ? "true" : "false");
  add_value("command_watchdog_expired", command_received_ && watchdog_stopped_ ? "true" : "false");
  add_value("command_timeout_count", std::to_string(command_timeout_count_));
  add_value("command_timeout_ms", std::to_string(command_timeout_.count()));
  add_value("feedback_timeout_ms", std::to_string(feedback_timeout_.count()));
  add_value("command_age_ms", command_received_ ? std::to_string(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - last_command_time_).count()) : "never");
  add_value("can_interface", can_interface_);
  add_value("rpm_resolution", std::to_string(rpm_resolution_));
  add_value("can_enabled", enable_can_ ? "true" : "false");
  add_value("motion_commands_enabled", motion_commands_enabled_ ? "true" : "false");
  add_value("feedback_received", feedback_received_ ? "true" : "false");
  add_value("fault_latched", fault_latched_ ? "true" : "false");
  add_value("feedback_timeout_latched", feedback_timeout_latched_ ? "true" : "false");
  if (feedback_received_) {
    const auto feedback_age = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - last_feedback_time_);
    add_value("feedback_age_ms", std::to_string(feedback_age.count()));
  } else {
    add_value("feedback_age_ms", "never");
  }
  if (latest_motor_fault_) {
    add_value("fault_motor", std::to_string(static_cast<unsigned int>(latest_motor_fault_->motor)));
    add_value("fault_mask", std::to_string(latest_motor_fault_->fault_mask));
  }

  array.status.push_back(std::move(status));
  diagnostics_publisher_->publish(std::move(array));
  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "Diagnostics: CAN=%s, motion=%s, feedback=%s, fault=%s, timeout=%s",
    enable_can_ ? "on" : "off", motion_commands_enabled_ ? "enabled" : "gated",
    feedback_received_ ? "received" : "none", fault_latched_ ? "latched" : "clear",
    feedback_timeout_latched_ ? "latched" : "clear");
}

}  // namespace motor_control_g4dual
