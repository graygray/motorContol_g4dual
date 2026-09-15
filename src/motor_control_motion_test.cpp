// Copyright 2026 Gray Lin
// SPDX-License-Identifier: MIT

#include "motor_control_g4dual/motor_control_node.hpp"

#include <filesystem>
#include <functional>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace motor_control_g4dual
{
void MotorControlNode::configure_motion_test()
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  descriptor.description = "Built-in motion test setting; restart to change";
  motion_test_allowed_ = declare_parameter<bool>("motion_test.enabled", true, descriptor);
  motion_test_log_directory_ = declare_parameter<std::string>(
    "motion_test.log_directory", "/tmp/motor_control_tests", descriptor);
  const auto number = [this, &descriptor](const char * key, double value) {
      return declare_parameter<double>(std::string("motion_test.") + key, value, descriptor);
    };
  auto & c = motion_test_config_;
  c.distance = number("distance_m", c.distance);
  c.angle = number("angle_deg", 90.0) * 3.14159265358979323846 / 180.0;
  c.linear_speed = number("linear_speed_mps", c.linear_speed);
  c.angular_speed = number("angular_speed_radps", c.angular_speed);
  c.linear_accel = number("linear_accel_mps2", c.linear_accel);
  c.angular_accel = number("angular_accel_radps2", c.angular_accel);
  c.distance_tolerance = number("distance_tolerance_m", c.distance_tolerance);
  c.angle_tolerance = number("angle_tolerance_rad", c.angle_tolerance);
  c.settle_time = number("settle_time_s", c.settle_time);
  c.stop_rpm = number("stop_rpm", c.stop_rpm);
  c.segment_timeout = number("segment_timeout_s", c.segment_timeout);
  c.stall_timeout = number("stall_timeout_s", c.stall_timeout);
  c.feedback_timeout = static_cast<double>(feedback_timeout_.count()) / 1000.0;
  c.repetitions = declare_parameter<int>("motion_test.repetitions", c.repetitions, descriptor);
  MotionTest::validate(c);
  motion_test_status_publisher_ = create_publisher<std_msgs::msg::String>(
    "motion_test/status", rclcpp::QoS(1).reliable().transient_local());
  motion_test_subscription_ = create_subscription<std_msgs::msg::String>(
    "motion_test/command", rclcpp::QoS(10).reliable().durability_volatile(),
    std::bind(&MotorControlNode::motion_test_command, this, std::placeholders::_1));
  publish_motion_test_status("ready");
}

MotionTest::Sample MotorControlNode::motion_test_sample() const
{
  const auto current = std::chrono::steady_clock::now();
  MotionTest::Sample sample;
  sample.time = std::chrono::duration<double>(current.time_since_epoch()).count();
  sample.x = odom_x_m_;
  sample.y = odom_y_m_;
  sample.yaw = odom_continuous_yaw_;
  sample.odom_age = std::chrono::duration<double>(current - last_odom_time_).count();
  sample.speed_age = std::chrono::duration<double>(current - last_speed_time_).count();
  if (latest_wheel_speeds_) {
    const auto rpm = kinematics_->motor_values_to_wheel(
      latest_wheel_speeds_->m1_speed_rpm, latest_wheel_speeds_->m2_speed_rpm);
    sample.left_rpm = rpm.left;
    sample.right_rpm = rpm.right;
  }
  return sample;
}

void MotorControlNode::motion_test_command(const std_msgs::msg::String::SharedPtr message)
{
  const auto & kind = message->data;
  if (kind == "cancel") {
    abort_motion_test("cancel requested");
    publish_motion_test_status("cancel processed");
    return;
  }
  const auto reject = [this](const char * reason) {
      RCLCPP_WARN(get_logger(), "Motion test rejected: %s", reason);
      publish_motion_test_status(std::string("rejected: ") + reason);
    };
  if (kind != "straight" && kind != "rotate") {
    reject("expected straight, rotate or cancel");
    return;
  }
  if (motion_test_.active()) {reject("a test is already active"); return;}
  if (!motion_test_allowed_) {reject("motion_test.enabled is false"); return;}
  const double rpm_per_mps = gear_ratio_ * 60.0 / (6.28318530717958647692 * wheel_radius_m_);
  const double wheel_speed = kind == "straight" ? motion_test_config_.linear_speed :
    motion_test_config_.angular_speed * wheel_separation_m_ / 2.0;
  if (wheel_speed * rpm_per_mps > max_motor_speed_rpm_ || motion_test_log_directory_.empty()) {
    reject("selected test exceeds RPM limit or has no log directory");
    return;
  }
  if (!enable_can_ || !motion_commands_enabled_ || fault_latched_ || feedback_timeout_latched_) {
    reject("CAN and explicitly enabled motors without safety latches are required");
    return;
  }
  const auto sample = motion_test_sample();
  if (!encoder_initialized_ || !latest_wheel_speeds_ ||
    sample.odom_age > motion_test_config_.feedback_timeout ||
    sample.speed_age > motion_test_config_.feedback_timeout)
  {
    reject("fresh absolute encoder and wheel-speed feedback are required");
    return;
  }
  if (std::abs(sample.left_rpm) > motion_test_config_.stop_rpm ||
    std::abs(sample.right_rpm) > motion_test_config_.stop_rpm ||
    (command_received_ && std::chrono::steady_clock::now() - last_command_time_ <=
    command_timeout_))
  {
    reject("stop external cmd_vel and wait for command timeout and standstill");
    return;
  }
  try {
    std::filesystem::create_directories(motion_test_log_directory_);
    const auto id = std::chrono::system_clock::now().time_since_epoch().count();
    motion_test_log_path_ = (std::filesystem::path(motion_test_log_directory_) /
      ("motion_test_" + std::to_string(id) + "_" + kind + ".csv")).string();
    motion_test_log_.clear();
    motion_test_log_.open(motion_test_log_path_, std::ios::out);
    const auto & c = motion_test_config_;
    motion_test_log_ << "# kind=" << kind << "; distance_m=" << c.distance <<
      "; angle_rad=" << c.angle << "; repetitions=" << c.repetitions <<
      "; linear_speed_mps=" << c.linear_speed << "; angular_speed_radps=" << c.angular_speed <<
      "; linear_accel_mps2=" << c.linear_accel << "; angular_accel_radps2=" << c.angular_accel <<
      "; wheel_radius_m=" << wheel_radius_m_ << "; wheel_separation_m=" << wheel_separation_m_ <<
      "; gear_ratio=" << gear_ratio_ << "; rpm_resolution=" << rpm_resolution_ << '\n';
    motion_test_log_ << "elapsed_s,state,segment,progress,last_stopped_segment_progress,"
      "relative_x_m,relative_y_m,"
      "relative_yaw_rad,linear_command_mps,angular_command_radps,left_target_rpm,"
      "right_target_rpm,left_measured_rpm,right_measured_rpm,odom_age_s,speed_age_s,reason\n";
    motion_test_log_.flush();
    if (!motion_test_log_) {throw std::runtime_error("cannot create CSV log");}
    motion_test_.start(kind, motion_test_config_, sample);
  } catch (const std::exception & error) {
    motion_test_log_.close();
    reject(error.what());
    return;
  }
  command_received_ = false;
  linear_velocity_mps_ = angular_velocity_radps_ = 0.0;
  publish_motor_rpm(0.0, 0.0);
  if (motion_test_.active()) {publish_motion_test_status("started");}
}

void MotorControlNode::update_motion_test()
{
  if (!motion_commands_enabled_) {abort_motion_test("motion gate closed"); return;}
  const auto previous_state = motion_test_.state();
  const auto previous_segment = motion_test_.segment();
  const auto command = motion_test_.update(motion_test_sample());
  linear_velocity_mps_ = command.linear;
  angular_velocity_radps_ = command.angular;
  if (!motion_test_.active()) {finish_motion_test(); return;}
  const auto rpm = kinematics_->twist_to_motor_rpm(command.linear, command.angular);
  publish_motor_rpm(rpm.left, rpm.right);
  if (!motion_test_.active()) {return;}  // Transmission failure already finalized the run.
  record_motion_test();
  if (!motion_test_log_) {abort_motion_test("CSV write failed"); return;}
  if (previous_state != motion_test_.state() || previous_segment != motion_test_.segment()) {
    publish_motion_test_status("phase changed");
  }
}

void MotorControlNode::record_motion_test()
{
  const auto s = motion_test_sample();
  const auto & origin = motion_test_.initial();
  const double dx = s.x - origin.x, dy = s.y - origin.y;
  const auto raw_target = kinematics_->twist_to_motor_rpm(
    linear_velocity_mps_, angular_velocity_radps_);
  const auto target = kinematics_->motor_values_to_wheel(raw_target.left, raw_target.right);
  motion_test_log_ << std::setprecision(12) << s.time - origin.time << ',' <<
    motion_test_.state() << ',' << motion_test_.segment() << ',' <<
    motion_test_.progress(s) << ',' << motion_test_.last_segment_progress() << ',' <<
    dx * std::cos(origin.yaw) + dy * std::sin(origin.yaw) <<
    ',' << -dx * std::sin(origin.yaw) + dy * std::cos(origin.yaw) << ',' <<
    s.yaw - origin.yaw << ',' << linear_velocity_mps_ << ',' << angular_velocity_radps_ <<
    ',' << target.left << ',' << target.right << ',' << s.left_rpm << ',' << s.right_rpm <<
    ',' << s.odom_age << ',' << s.speed_age << ',' << motion_test_.reason() << '\n';
  motion_test_log_.flush();
}

void MotorControlNode::abort_motion_test(const std::string & reason)
{
  if (!motion_test_.active()) {return;}
  motion_test_.abort(reason);
  finish_motion_test();
}

void MotorControlNode::finish_motion_test(bool keep_enabled)
{
  command_received_ = false;
  watchdog_stopped_ = false;
  linear_velocity_mps_ = angular_velocity_radps_ = 0.0;
  // The state machine is already terminal, so stop callbacks cannot re-enter finalization.
  publish_motor_rpm(0.0, 0.0);
  // Normal completion and cmd_vel takeover keep the existing motor enable state.
  // Faults, explicit cancellation and stop events still close the motion gate.
  if (!keep_enabled && motion_test_.state() != "completed") {
    latch_safety_stop(motion_test_.reason().c_str());
  }
  record_motion_test();
  const bool logged = static_cast<bool>(motion_test_log_);
  motion_test_log_.close();
  publish_motion_test_status(logged ? "finished" : "finished; CSV write failed");
  publish_diagnostics();
}

void MotorControlNode::publish_motion_test_status(const std::string & event)
{
  const auto s = motion_test_sample();
  const auto & origin = motion_test_.initial();
  std::ostringstream output;
  output << event << "; state=" << motion_test_.state() << "; kind=" << motion_test_.kind() <<
    "; segment=" << motion_test_.segment() << "; reason=" << motion_test_.reason() <<
    "; last_stopped_segment_progress=" << motion_test_.last_segment_progress() <<
    "; displacement_m=" << std::hypot(s.x - origin.x, s.y - origin.y) <<
    "; yaw_change_rad=" << s.yaw - origin.yaw << "; csv=" << motion_test_log_path_;
  std_msgs::msg::String message;
  message.data = output.str();
  motion_test_status_publisher_->publish(message);
  RCLCPP_INFO(get_logger(), "Motion test: %s", message.data.c_str());
}
}  // namespace motor_control_g4dual
