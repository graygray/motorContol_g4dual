// Copyright 2026 Gray Lin
// SPDX-License-Identifier: MIT

#include "motor_control_g4dual/socket_can_transport.hpp"

#include <cerrno>
#include <cstring>
#include <utility>

#ifdef __linux__
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#endif

namespace motor_control_g4dual
{
namespace
{
#ifdef __linux__
std::string system_error(const char * operation)
{
  return std::string(operation) + ": " + std::strerror(errno);
}
#endif
}  // namespace

SocketCanTransport::SocketCanTransport(std::string interface_name)
: interface_name_(std::move(interface_name))
{
}

SocketCanTransport::~SocketCanTransport()
{
  close();
}

bool SocketCanTransport::open(std::string & error_message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  error_message.clear();

#ifdef __linux__
  if (socket_fd_ >= 0) {
    return true;
  }
  if (interface_name_.empty() || interface_name_.size() >= IFNAMSIZ) {
    error_message = "SocketCAN interface name is empty or too long";
    return false;
  }

  errno = 0;
  const unsigned int interface_index = if_nametoindex(interface_name_.c_str());
  if (interface_index == 0U) {
    error_message = system_error("Cannot resolve SocketCAN interface");
    return false;
  }

  const int candidate_fd = ::socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
  if (candidate_fd < 0) {
    error_message = system_error("Cannot create SocketCAN socket");
    return false;
  }

  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = static_cast<int>(interface_index);
  if (::bind(
      candidate_fd, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) < 0)
  {
    error_message = system_error("Cannot bind SocketCAN socket");
    ::close(candidate_fd);
    return false;
  }

  socket_fd_ = candidate_fd;
  return true;
#else
  error_message = "SocketCAN is only supported on Linux";
  return false;
#endif
}

void SocketCanTransport::close() noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);

#ifdef __linux__
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
#else
  socket_fd_ = -1;
#endif
}

bool SocketCanTransport::is_open() const noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  return socket_fd_ >= 0;
}

bool SocketCanTransport::send_command(
  const CanFrame & frame, std::string & error_message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  error_message.clear();

  if (frame.id != MotorCanProtocol::kCommandId) {
    error_message = "Refusing to transmit a frame whose ID is not 0x601";
    return false;
  }
  if (frame.length != frame.data.size()) {
    error_message = "A motor command must contain exactly eight data bytes";
    return false;
  }

#ifdef __linux__
  if (socket_fd_ < 0) {
    error_message = "SocketCAN transport is not open";
    return false;
  }

  can_frame socket_frame{};
  socket_frame.can_id = frame.id & CAN_SFF_MASK;
  socket_frame.can_dlc = frame.length;
  std::copy_n(frame.data.begin(), frame.length, socket_frame.data);

  ssize_t bytes_written;
  do {
    bytes_written = ::write(socket_fd_, &socket_frame, sizeof(socket_frame));
  } while (bytes_written < 0 && errno == EINTR);

  if (bytes_written < 0) {
    error_message = system_error("SocketCAN write failed");
    return false;
  }
  if (bytes_written != static_cast<ssize_t>(sizeof(socket_frame))) {
    error_message = "SocketCAN write returned an incomplete frame";
    return false;
  }
  return true;
#else
  error_message = "SocketCAN is only supported on Linux";
  return false;
#endif
}

const std::string & SocketCanTransport::interface_name() const noexcept
{
  return interface_name_;
}

}  // namespace motor_control_g4dual
