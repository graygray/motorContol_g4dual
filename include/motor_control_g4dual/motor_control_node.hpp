// Copyright 2026 Gray Lin
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <fstream>
#include <memory>
#include <optional>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "motor_control_g4dual/control_reply_tracker.hpp"
#include "motor_control_g4dual/differential_drive_kinematics.hpp"
#include "motor_control_g4dual/socket_can_transport.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "motor_control_g4dual/motion_test.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/u_int32_multi_array.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace motor_control_g4dual
{

class MotorControlNode : public rclcpp::Node
{
public:
  explicit MotorControlNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(), bool info_enabled = false);
  ~MotorControlNode() override;

private:
  friend struct MotorControlNodeTestPeer;
  void command_callback(const geometry_msgs::msg::Twist::SharedPtr message);
  void control_callback();
  void receive_can_frames();
  void publish_motor_rpm(double left_rpm, double right_rpm);
  void enable_motors(const std_msgs::msg::Empty::SharedPtr message);
  void stop_motors(const std_msgs::msg::Empty::SharedPtr message);
  void reset_faults(const std_msgs::msg::Empty::SharedPtr message);
  void set_emergency_stop(const std_msgs::msg::Bool::SharedPtr message);
  void report_control_result(const char * command, bool success, const std::string & message);
  bool send_control_command(ControlCommand command, std::string & error_message);
  void check_feedback_timeout(std::chrono::steady_clock::time_point now);
  void latch_safety_stop(const char * reason);
  void publish_wheel_speeds(const WheelSpeedsReply & reply, const rclcpp::Time & stamp);
  void publish_encoder_deltas(const EncoderDeltasReply & reply);
  void update_encoder_odometry(
    const EncoderPositionReport & report, const rclcpp::Time & stamp);
  void publish_motor_fault(const MotorFaultReport & report);
  void publish_diagnostics();
  void check_reply_timeouts();

  void configure_motion_test();
  void motion_test_command(const std_msgs::msg::String::SharedPtr message);
  MotionTest::Sample motion_test_sample() const;
  void update_motion_test();
  void abort_motion_test(const std::string & reason);
  void finish_motion_test(bool keep_enabled = false);
  void record_motion_test();
  void publish_motion_test_status(const std::string & event);

  MotionTest motion_test_;
  MotionTest::Config motion_test_config_;
  bool motion_test_allowed_{true};
  std::string motion_test_log_directory_, motion_test_log_path_;
  std::ofstream motion_test_log_;
  double odom_continuous_yaw_{0.0};
  std::chrono::steady_clock::time_point last_odom_time_{}, last_speed_time_{};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motion_test_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motion_test_status_publisher_;

  std::string command_topic_;
  std::string motor_rpm_topic_;
  std::string can_interface_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string left_joint_name_;
  std::string right_joint_name_;
  bool enable_can_;
  bool publish_odom_tf_;
  double wheel_radius_m_;
  double wheel_separation_m_;
  double gear_ratio_;
  double max_motor_speed_rpm_;
  double rpm_resolution_;
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
  bool feedback_received_{false};
  bool encoder_initialized_{false};
  std::chrono::steady_clock::time_point last_command_time_;
  std::chrono::steady_clock::time_point last_feedback_time_;
  std::int32_t previous_m1_position_{0};
  std::int32_t previous_m2_position_{0};
  double left_joint_position_rad_{0.0};
  double right_joint_position_rad_{0.0};
  double odom_x_m_{0.0};
  double odom_y_m_{0.0};
  double odom_yaw_rad_{0.0};
  std::unique_ptr<SocketCanTransport> can_transport_;
  std::unique_ptr<DifferentialDriveKinematics> kinematics_;
  std::optional<WheelSpeedsReply> latest_wheel_speeds_;
  std::optional<EncoderDeltasReply> latest_encoder_deltas_;
  std::optional<EncoderPositionReport> latest_encoder_positions_;
  std::optional<MotorFaultReport> latest_motor_fault_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_rpm_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_speed_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_delta_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr motor_fault_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr can_receive_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr enable_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_faults_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_subscription_;
  std::string last_control_command_;
  std::optional<bool> last_control_success_;
  std::string last_control_message_;
  ControlReplyTracker control_replies_;
  std::string firmware_query_state_{"disabled"};
  std::string firmware_identifier_;
  std::string expected_firmware_identifier_;
  std::chrono::steady_clock::time_point firmware_deadline_{};
  std::uint64_t rejected_frame_count_{0U};
  std::uint64_t acknowledgement_count_{0U};
  std::uint64_t abort_reply_count_{0U};
  std::uint64_t command_timeout_count_{0U};
  std::string last_rejected_frame_;
  std::string last_rejected_reason_;
  std::string last_acknowledgement_;
  std::string last_abort_reply_;
};

}  // namespace motor_control_g4dual
