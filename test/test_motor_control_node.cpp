// Copyright 2026 Gray Lin
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include "motor_control_g4dual/motor_control_node.hpp"

namespace motor_control_g4dual
{
namespace
{
class MotorControlTopicsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("enable_can", false),
      rclcpp::Parameter("publish_odom_tf", false)});
    motor_ = std::make_shared<MotorControlNode>(options);
    client_ = std::make_shared<rclcpp::Node>("motor_control_topics_test");
    executor_->add_node(motor_);
    executor_->add_node(client_);
    diagnostics_ = client_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "diagnostics", 10,
      [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {
        for (const auto & status : message->status) {
          for (const auto & value : status.values) {
            values_[value.key] = value.value;
          }
        }
      });
  }

  void TearDown() override
  {
    executor_->remove_node(client_);
    executor_->remove_node(motor_);
    diagnostics_.reset();
    client_.reset();
    motor_.reset();
    executor_.reset();
    rclcpp::shutdown();
  }

  bool spin_until(const std::function<bool()> & condition)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      if (condition()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }

  template<typename Message>
  void expect_disabled_result(const std::string & topic, const Message & message)
  {
    auto publisher = client_->create_publisher<Message>(
      topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
    ASSERT_TRUE(spin_until([&]() {
      return publisher->get_subscription_count() == 1U &&
             diagnostics_->get_publisher_count() == 1U;
    }));
    // Wait for periodic diagnostics to establish delivery, then publish only once.
    ASSERT_TRUE(spin_until([&]() {return values_.count("can_enabled") != 0U;}));
    values_.clear();
    publisher->publish(message);
    ASSERT_TRUE(spin_until([&]() {return values_["last_control_command"] == topic;}));
    EXPECT_EQ(values_["last_control_success"], "false");
    EXPECT_NE(values_["last_control_message"].find("SocketCAN is disabled"), std::string::npos);
    EXPECT_EQ(values_["motion_commands_enabled"], "false");
    EXPECT_EQ(values_["fault_latched"], "false");
    EXPECT_EQ(values_["feedback_timeout_latched"], "false");
  }

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<MotorControlNode> motor_;
  rclcpp::Node::SharedPtr client_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  std::map<std::string, std::string> values_;
};

TEST_F(MotorControlTopicsTest, EmptyCommandsReportFailureWithoutCan)
{
  expect_disabled_result("enable_motors", std_msgs::msg::Empty{});
  expect_disabled_result("stop_motors", std_msgs::msg::Empty{});
  expect_disabled_result("reset_faults", std_msgs::msg::Empty{});
  const auto services = motor_->get_service_names_and_types();
  EXPECT_EQ(services.count("/enable_motors"), 0U);
  EXPECT_EQ(services.count("/stop_motors"), 0U);
  EXPECT_EQ(services.count("/reset_faults"), 0U);
  EXPECT_EQ(services.count("/emergency_stop"), 0U);
}

TEST_F(MotorControlTopicsTest, EmergencyStopEngageReportsFailureWithoutCan)
{
  std_msgs::msg::Bool message;
  message.data = true;
  expect_disabled_result("emergency_stop", message);
}

TEST_F(MotorControlTopicsTest, EmergencyStopReleaseKeepsMotionDisabled)
{
  std_msgs::msg::Bool message;
  message.data = false;
  expect_disabled_result("emergency_stop", message);
}

TEST_F(MotorControlTopicsTest, DiagnosticsKeepUnknownFirmwareAndScaleExplicit)
{
  ASSERT_TRUE(spin_until([&]() {return values_.count("firmware_query_state") != 0U;}));
  EXPECT_EQ(values_["firmware_query_state"], "disabled");
  EXPECT_EQ(values_["firmware_identifier"], "unknown");
  EXPECT_EQ(values_["firmware_identity_matches"], "unknown");
  EXPECT_EQ(values_["rpm_resolution_verified"], "false");
  EXPECT_EQ(values_["last_control_reply_state"], "unavailable");
  EXPECT_EQ(values_["rejected_frame_count"], "0");
  EXPECT_EQ(values_["acknowledgement_count"], "0");
  EXPECT_EQ(values_["abort_reply_count"], "0");
  EXPECT_EQ(values_["command_age_ms"], "never");
  EXPECT_EQ(values_["feedback_age_ms"], "never");
  const auto results = motor_->set_parameters({
      rclcpp::Parameter("expect_control_ack", true),
      rclcpp::Parameter("reply_timeout_ms", 100),
      rclcpp::Parameter("expected_firmware_identifier", "primax")});
  ASSERT_EQ(results.size(), 3U);
  for (const auto & result : results) {
    EXPECT_FALSE(result.successful);
  }
}

TEST_F(MotorControlTopicsTest, BuiltInTestRejectsDisabledStartAndKeepsMotionGated)
{
  std::string status;
  auto subscription = client_->create_subscription<std_msgs::msg::String>(
    "motion_test/status", rclcpp::QoS(1).reliable().transient_local(),
    [&status](const std_msgs::msg::String::SharedPtr message) {status = message->data;});
  auto publisher = client_->create_publisher<std_msgs::msg::String>(
    "motion_test/command", rclcpp::QoS(10).reliable().durability_volatile());
  ASSERT_TRUE(spin_until([&]() {
    return publisher->get_subscription_count() == 1U && !status.empty();
  }));
  for (const auto * kind : {"straight", "rotate"}) {
    status.clear();
    std_msgs::msg::String command;
    command.data = kind;
    publisher->publish(command);
    ASSERT_TRUE(spin_until([&]() {
      return status.find("rejected: motion_test.enabled is false") != std::string::npos;
    }));
    EXPECT_NE(status.find("state=idle"), std::string::npos);
  }
  const auto result = motor_->set_parameters({rclcpp::Parameter("motion_test.enabled", true)});
  ASSERT_EQ(result.size(), 1U);
  EXPECT_FALSE(result.front().successful);
  ASSERT_TRUE(spin_until([&]() {return values_.count("motion_test_state") != 0U;}));
  EXPECT_EQ(values_["motion_test_state"], "idle");
  EXPECT_EQ(values_["motion_commands_enabled"], "false");
}

TEST_F(MotorControlTopicsTest, BuiltInTestOptInStillRequiresPhysicalCan)
{
  executor_->remove_node(motor_);
  motor_.reset();
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("enable_can", false),
    rclcpp::Parameter("publish_odom_tf", false),
    rclcpp::Parameter("motion_test.enabled", true)});
  motor_ = std::make_shared<MotorControlNode>(options);
  executor_->add_node(motor_);
  std::string status;
  auto subscription = client_->create_subscription<std_msgs::msg::String>(
    "motion_test/status", rclcpp::QoS(1).reliable().transient_local(),
    [&status](const std_msgs::msg::String::SharedPtr message) {status = message->data;});
  auto publisher = client_->create_publisher<std_msgs::msg::String>("motion_test/command", 10);
  ASSERT_TRUE(spin_until([&]() {
    return publisher->get_subscription_count() == 1U && !status.empty();
  }));
  std_msgs::msg::String command;
  command.data = "straight";
  publisher->publish(command);
  ASSERT_TRUE(spin_until([&]() {
    return status.find("rejected: CAN and explicitly enabled motors") != std::string::npos;
  }));
  EXPECT_NE(status.find("state=idle"), std::string::npos);
}

TEST_F(MotorControlTopicsTest, CommandWatchdogReportsInputGapSeparatelyFromFeedback)
{
  auto publisher = client_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  ASSERT_TRUE(spin_until([&]() {return publisher->get_subscription_count() == 1U;}));
  publisher->publish(geometry_msgs::msg::Twist{});
  ASSERT_TRUE(spin_until([&]() {return values_["command_watchdog_expired"] == "true";}));
  EXPECT_EQ(values_["command_timeout_count"], "1");
  EXPECT_EQ(values_["command_timeout_ms"], "500");
  EXPECT_GE(std::stoll(values_["command_age_ms"]), 500);
  EXPECT_EQ(values_["feedback_age_ms"], "never");
  EXPECT_EQ(values_["feedback_timeout_latched"], "false");

  // Continuous fresh input recovers the command watchdog without changing the
  // feedback watchdog or incrementing the expiration counter again.
  auto timer = client_->create_wall_timer(std::chrono::milliseconds(50), [&]() {
      publisher->publish(geometry_msgs::msg::Twist{});
    });
  ASSERT_TRUE(spin_until([&]() {return values_["command_watchdog_expired"] == "false";}));
  EXPECT_EQ(values_["command_timeout_count"], "1");
  EXPECT_EQ(values_["feedback_timeout_latched"], "false");
}

}  // namespace
}  // namespace motor_control_g4dual
