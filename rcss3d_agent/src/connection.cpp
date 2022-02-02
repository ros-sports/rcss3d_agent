// Copyright 2019 Bold Hearts
// Copyright 2021 Kenji Brameld
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <netinet/tcp.h>
#include <string>
#include <algorithm>
#include "connection.hpp"
#include "rclcpp/logger.hpp"

namespace rcss3d_agent
{

Connection::Connection()
: logger(rclcpp::get_logger("connection")),
  socket_(PF_INET, SOCK_STREAM, 0),
  socket_address_()
{
}

void Connection::initialise(const std::string & host, int port)
{
  initSocket(host, port);
  connect();
}

void Connection::send(std::string msg)
{
  RCLCPP_DEBUG(logger, (std::string{"Sending: "} + msg).c_str());

  auto len = htonl(msg.length());

  auto prefix = std::string{reinterpret_cast<const char *>(&len), sizeof(unsigned int)};
  auto data = prefix + msg;

  socket_.writeExactly(data.c_str(), data.length());
}

void Connection::initSocket(std::string const & host, int port)
{
  socket_address_ = SocketAddress(PF_INET, port, host);
}

void Connection::connect()
{
  try {
    socket_.connect(socket_address_);
    RCLCPP_INFO(logger, "Connected to server");
    initConnection();
  } catch (std::runtime_error &) {
    RCLCPP_ERROR(
      logger, "Failed connecting to server. "
      "Please ensure that the simulation server is running.");
  }
}

void Connection::initConnection()
{
  socket_.setBlocking(true);
  socket_.setsockopt(IPPROTO_TCP, TCP_NODELAY, true);
}

std::string Connection::receive()
{
  // Read new message
  RCLCPP_DEBUG(logger, "Starting receive");
  auto len = receive_();
  RCLCPP_DEBUG(logger, ("Received: " + std::to_string(len)).c_str());
  if (len == 0) {
    return "";
  }
  auto msg = std::string{buffer_.data()};
  RCLCPP_DEBUG(logger, (std::string{"Received: "} + msg).c_str());
  return msg;
}


uint32_t Connection::receive_()
{
  buffer_.reserve(4);
  auto len = socket_.readExactly(buffer_.data(), 4);
  if (len != 4) {
    RCLCPP_ERROR(
      logger, "Disconnected from the simulator. Please restart this node.");
    return 0;
  }

  auto prefix = int32_t{};
  std::copy(buffer_.begin(), std::next(buffer_.begin(), 4), reinterpret_cast<char *>(&prefix));
  prefix = ntohl(prefix);

  buffer_.reserve(prefix + 1);
  len = socket_.readExactly(buffer_.data(), prefix);
  if (len != uint64_t(prefix)) {
    RCLCPP_ERROR(
      logger, "Disconnected from the simulator. Please restart this node.");
    return 0;
  }
  // Ensure string is 0-terminated
  buffer_[prefix] = 0;
  return prefix;
}

}  // namespace rcss3d_agent
