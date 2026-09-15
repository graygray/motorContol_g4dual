// Copyright 2026 Gray Lin
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <map>
#include <limits>
#include <memory>
#include <string>
#include <thread>

#include "motor_control_g4dual/motor_control_node.hpp"

namespace motor_control_g4dual
{
// Exercise command ownership without enabling a physical CAN transport.
struct MotorControlNodeTestPeer
{
  static void start(MotorControlNode & node, const std::string & kind = "straight")
  {
    MotionTest::Config config;
    config.repetitions = 1;
    config.distance = 0.02;
    config.settle_time = 0.01;
    node.motion_test_.start(kind, config, node.motion_test_sample());
    node.motion_test_log_.open("/dev/null");
    node.motion_commands_enabled_ = true;
  }
  static void command(MotorControlNode & node, double linear, double angular)
  {
    auto message = std::make_shared<geometry_msgs::msg::Twist>();
    message->linear.x = linear;
    message->angular.z = angular;
    node.command_callback(message);
  }
  static void complete(MotorControlNode & node)
  {
    auto sample = node.motion_test_sample();
    for (int i = 0; i < 1000 && node.motion_test_.active(); ++i) {
      sample.time += 0.05;
      sample.odom_age = sample.speed_age = 0.0;
      const auto command = node.motion_test_.update(sample);
      sample.x += command.linear * 0.05;
    }
    ASSERT_EQ(node.motion_test_.state(), "completed");
    node.finish_motion_test();
  }
  static void cancel(MotorControlNode & node)
  {
    auto message = std::make_shared<std_msgs::msg::String>();
    message->data = "cancel";
    node.motion_test_command(message);
  }
  static bool enabled(const MotorControlNode & node) {return node.motion_commands_enabled_;}
  static bool received(const MotorControlNode & node) {return node.command_received_;}
  static double linear(const MotorControlNode & node) {return node.linear_velocity_mps_;}
  static double angular(const MotorControlNode & node) {return node.angular_velocity_radps_;}
  static const MotionTest & test(const MotorControlNode & node) {return node.motion_test_;}
};
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
  executor_->remove_node(motor_);
  motor_.reset();
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("enable_can", false),
    rclcpp::Parameter("publish_odom_tf", false),
    rclcpp::Parameter("motion_test.enabled", false)});
  motor_ = std::make_shared<MotorControlNode>(options);
  executor_->add_node(motor_);
  std::string status;
  auto subscription = client_->create_subscription<std_msgs::msg::String>(
    "motion_test/status", rclcpp::QoS(1).reliable().transient_local(),
    [&status](const std_msgs::msg::String::SharedPtr message) {status = message->data;});
  auto publisher = client_->create_publisher<std_msgs::msg::String>(
    "motion_test/command", rclcpp::QoS(10).reliable().durability_volatile());
  ASSERT_TRUE(spin_until([&]() {
    return publisher->get_subscription_count() == 1U && !status.empty();
  }));
  for (const auto * kind : {"straight", "rotate", "circle", "square", "s"}) {
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

TEST_F(MotorControlTopicsTest, BuiltInTestDefaultsEnabledButStillRequiresPhysicalCan)
{
  EXPECT_TRUE(motor_->get_parameter("motion_test.enabled").as_bool());
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

TEST_F(MotorControlTopicsTest, ValidNormalCommandTakesOverTestWithoutClosingGate)
{
  for (const auto * kind : {"straight", "rotate", "circle", "square", "s"}) {
    MotorControlNodeTestPeer::start(*motor_, kind);
    MotorControlNodeTestPeer::command(*motor_, 0.12, -0.15);
    EXPECT_EQ(MotorControlNodeTestPeer::test(*motor_).state(), "aborted");
    EXPECT_EQ(MotorControlNodeTestPeer::test(*motor_).reason(), "external cmd_vel took control");
    EXPECT_TRUE(MotorControlNodeTestPeer::enabled(*motor_));
    EXPECT_TRUE(MotorControlNodeTestPeer::received(*motor_));
    EXPECT_DOUBLE_EQ(MotorControlNodeTestPeer::linear(*motor_), 0.12);
    EXPECT_DOUBLE_EQ(MotorControlNodeTestPeer::angular(*motor_), -0.15);
  }
}

TEST_F(MotorControlTopicsTest, InvalidCommandKeepsTestRunningAndZeroCommandTakesOver)
{
  MotorControlNodeTestPeer::start(*motor_);
  MotorControlNodeTestPeer::command(*motor_, std::numeric_limits<double>::quiet_NaN(), 0.0);
  EXPECT_TRUE(MotorControlNodeTestPeer::test(*motor_).active());
  MotorControlNodeTestPeer::command(*motor_, 0.0, 0.0);
  EXPECT_FALSE(MotorControlNodeTestPeer::test(*motor_).active());
  EXPECT_TRUE(MotorControlNodeTestPeer::enabled(*motor_));
  EXPECT_TRUE(MotorControlNodeTestPeer::received(*motor_));
  EXPECT_DOUBLE_EQ(MotorControlNodeTestPeer::linear(*motor_), 0.0);
}

TEST_F(MotorControlTopicsTest, CompletionAllowsNextCommandWithoutReenable)
{
  MotorControlNodeTestPeer::start(*motor_);
  MotorControlNodeTestPeer::complete(*motor_);
  EXPECT_TRUE(MotorControlNodeTestPeer::enabled(*motor_));
  EXPECT_FALSE(MotorControlNodeTestPeer::received(*motor_));
  EXPECT_DOUBLE_EQ(MotorControlNodeTestPeer::linear(*motor_), 0.0);
  MotorControlNodeTestPeer::command(*motor_, -0.1, 0.0);
  EXPECT_TRUE(MotorControlNodeTestPeer::enabled(*motor_));
  EXPECT_DOUBLE_EQ(MotorControlNodeTestPeer::linear(*motor_), -0.1);
}

TEST_F(MotorControlTopicsTest, ExplicitCancelStillClosesMotionGate)
{
  MotorControlNodeTestPeer::start(*motor_);
  MotorControlNodeTestPeer::cancel(*motor_);
  EXPECT_EQ(MotorControlNodeTestPeer::test(*motor_).state(), "aborted");
  EXPECT_FALSE(MotorControlNodeTestPeer::enabled(*motor_));
  EXPECT_FALSE(MotorControlNodeTestPeer::received(*motor_));
}

TEST_F(MotorControlTopicsTest, LowMotorLimitDoesNotPreventNormalNodeStartup)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("enable_can", false),
    rclcpp::Parameter("publish_odom_tf", false),
    rclcpp::Parameter("max_motor_speed_rpm", 1.0)});
  EXPECT_NO_THROW(std::make_shared<MotorControlNode>(options));
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
