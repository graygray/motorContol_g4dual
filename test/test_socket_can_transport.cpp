// Copyright 2026 Gray Lin
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <variant>

#include "motor_control_g4dual/motor_can_protocol.hpp"
#include "motor_control_g4dual/socket_can_transport.hpp"

namespace motor_control_g4dual
{
namespace
{
class FileDescriptor
{
public:
  explicit FileDescriptor(int value)
  : value_(value) {}

  ~FileDescriptor()
  {
    if (value_ >= 0) {
      ::close(value_);
    }
  }

  int get() const
  {
    return value_;
  }

private:
  int value_;
};

TEST(SocketCanTransport, ExchangesFramesOnVirtualCan)
{
  const char * interface_environment = std::getenv("MOTOR_CONTROL_VCAN_INTERFACE");
  if (interface_environment == nullptr || std::strlen(interface_environment) == 0U) {
    GTEST_SKIP() << "Set MOTOR_CONTROL_VCAN_INTERFACE to run the vcan integration test";
  }
  const std::string interface_name(interface_environment);

  SocketCanTransport transport(interface_name);
  std::string error_message;
  ASSERT_TRUE(transport.open(error_message)) << error_message;

  FileDescriptor peer(::socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW));
  ASSERT_GE(peer.get(), 0);
  const unsigned int interface_index = if_nametoindex(interface_name.c_str());
  ASSERT_NE(interface_index, 0U);

  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = static_cast<int>(interface_index);
  ASSERT_EQ(
    ::bind(peer.get(), reinterpret_cast<const sockaddr *>(&address), sizeof(address)), 0);

  const auto command = MotorCanProtocol::encode_wheel_speeds(12.3, -45.6);
  ASSERT_TRUE(command);
  ASSERT_TRUE(transport.send_command(*command, error_message)) << error_message;

  pollfd descriptor{peer.get(), POLLIN, 0};
  ASSERT_GT(::poll(&descriptor, 1U, 1000), 0);
  can_frame received_command{};
  ASSERT_EQ(::read(peer.get(), &received_command, sizeof(received_command)),
    static_cast<ssize_t>(sizeof(received_command)));
  EXPECT_EQ(received_command.can_id & CAN_SFF_MASK, MotorCanProtocol::kCommandId);
  EXPECT_EQ(received_command.can_dlc, 8U);
  EXPECT_EQ(received_command.data[4], 0x7BU);
  EXPECT_EQ(received_command.data[6], 0x38U);

  can_frame reply{};
  reply.can_id = MotorCanProtocol::kReplyId;
  reply.can_dlc = 8U;
  const std::uint8_t reply_data[8U] = {
    0x43U, 0x6CU, 0x60U, 0U, 0x58U, 0x02U, 0xA8U, 0xFDU};
  std::memcpy(reply.data, reply_data, sizeof(reply_data));
  ASSERT_EQ(::write(peer.get(), &reply, sizeof(reply)), static_cast<ssize_t>(sizeof(reply)));

  CanFrame received_reply;
  ReceiveStatus receive_status = ReceiveStatus::kNoData;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (receive_status == ReceiveStatus::kNoData && std::chrono::steady_clock::now() < deadline) {
    receive_status = transport.receive(received_reply, error_message);
    if (receive_status == ReceiveStatus::kNoData) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  ASSERT_EQ(receive_status, ReceiveStatus::kFrameReceived) << error_message;
  const auto decoded = MotorCanProtocol::decode(received_reply);
  ASSERT_TRUE(decoded);
  EXPECT_DOUBLE_EQ(std::get<WheelSpeedsReply>(*decoded.message).m1_speed_rpm, 60.0);
  EXPECT_DOUBLE_EQ(std::get<WheelSpeedsReply>(*decoded.message).m2_speed_rpm, -60.0);
}

}  // namespace
}  // namespace motor_control_g4dual
