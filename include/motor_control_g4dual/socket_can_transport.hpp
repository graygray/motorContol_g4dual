// Copyright 2026 Gray Lin
// SPDX-License-Identifier: MIT

#pragma once

#include <mutex>
#include <string>

#include "motor_control_g4dual/motor_can_protocol.hpp"

namespace motor_control_g4dual
{

enum class ReceiveStatus
{
  kFrameReceived,
  kNoData,
  kError,
  // Kernel error frame: bus-off, controller state change or driver restart.
  // The socket stays open; the error message describes the condition.
  kBusFault,
  // The network interface disappeared or went down. The transport closed its
  // socket; call open() again once the interface is back.
  kLinkDown,
};

class SocketCanTransport
{
public:
  explicit SocketCanTransport(std::string interface_name);
  ~SocketCanTransport();

  SocketCanTransport(const SocketCanTransport &) = delete;
  SocketCanTransport & operator=(const SocketCanTransport &) = delete;

  bool open(std::string & error_message);
  void close() noexcept;
  bool is_open() const noexcept;
  bool send_command(const CanFrame & frame, std::string & error_message);
  ReceiveStatus receive(CanFrame & frame, std::string & error_message);

  const std::string & interface_name() const noexcept;

private:
  void close_locked() noexcept;

  std::string interface_name_;
  int socket_fd_{-1};
  mutable std::mutex mutex_;
};

}  // namespace motor_control_g4dual
