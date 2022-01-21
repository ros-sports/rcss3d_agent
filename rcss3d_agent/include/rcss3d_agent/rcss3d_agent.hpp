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

#ifndef RCSS3D_AGENT__RCSS3D_AGENT_HPP_
#define RCSS3D_AGENT__RCSS3D_AGENT_HPP_

#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcss3d_agent/connection.hpp"
#include "rcss3d_agent_msgs/msg/hinge_joint.hpp"
#include "rcss3d_agent_msgs/msg/universal_joint.hpp"
#include "rcss3d_agent_msgs/msg/beam.hpp"
#include "rcss3d_agent_msgs/msg/percept.hpp"
#include "rcss3d_agent_msgs/msg/say.hpp"

namespace rcss3d_agent
{
class Params;

class Rcss3dAgent
{
public:
  explicit Rcss3dAgent(const Params & p);
  virtual ~Rcss3dAgent();

  // Functions to call to send message
  void sendHingeJoint(const rcss3d_agent_msgs::msg::HingeJoint & j);
  void sendUniversalJoint(const rcss3d_agent_msgs::msg::UniversalJoint & j);
  // void sendSynchronize();
  void sendBeam(const rcss3d_agent_msgs::msg::Beam & b);
  void sendSay(const rcss3d_agent_msgs::msg::Say & s);

  // Register callback methods
  void registerPerceptCallback(
    std::function<void(const rcss3d_agent_msgs::msg::Percept &)> callback);

private:
  using Percept = rcss3d_agent_msgs::msg::Percept;
  using HingeJoint = rcss3d_agent_msgs::msg::HingeJoint;
  using Beam = rcss3d_agent_msgs::msg::Beam;
  using UniversalJoint = rcss3d_agent_msgs::msg::UniversalJoint;
  using Say = rcss3d_agent_msgs::msg::Say;

  Connection connection;
  std::thread receive_thread_;
  rclcpp::Logger logger;

  void handle(std::string const & msg);
  void logParametersToRclcppDebug(
    std::string rcss3d_host, int rcss3d_port, std::string team, int unum);

  // Registered Callbacks
  std::vector<std::function<void(const Percept &)>> callbacksPercept;
};

class Params
{
public:
  Params(std::string rcss3d_host, int rcss3d_port, std::string team, int unum)
  : rcss3d_host(rcss3d_host), rcss3d_port(rcss3d_port), team(team), unum(unum) {}
  std::string rcss3d_host;
  int rcss3d_port;
  std::string team;
  int unum;
};

}  // namespace rcss3d_agent

#endif  // RCSS3D_AGENT__RCSS3D_AGENT_HPP_
