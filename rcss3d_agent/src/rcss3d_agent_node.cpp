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

#include "rcss3d_agent/rcss3d_agent_node.hpp"

#include <netinet/tcp.h>
#include <cmath>
#include <string>
#include <utility>
#include <algorithm>
#include <memory>

#include "rcss3d_agent/sexp_creator.hpp"
#include "rcss3d_agent/sexp_parser.hpp"

using namespace std::chrono_literals;

namespace rcss3d_agent
{

Rcss3dAgentNode::Rcss3dAgentNode(const rclcpp::NodeOptions & options)
: rclcpp::Node{"rcss3d_agent", options}
{
  // Declare parameters
  RCLCPP_DEBUG(get_logger(), "Declare parameters");
  std::string rcss3d_host = this->declare_parameter<std::string>("rcss3d/host", "127.0.0.1");
  int rcss3d_port = this->declare_parameter<int>("rcss3d/port", 3100);
  std::string team = this->declare_parameter<std::string>("team", "Anonymous");
  int unum = this->declare_parameter<int>("unum", 0);

  // Create Rcss3dAgent
  params = std::make_unique<rcss3d_agent::Params>(rcss3d_host, rcss3d_port, team, unum);
  rcss3dAgent = std::make_unique<Rcss3dAgent>(*params);

  // Create publisher
  percept_pub_ =
    create_publisher<Percept>("percept", 10);

  // Register callback
  rcss3dAgent->registerPerceptCallback(
    [this](const Percept & percept) {
      percept_pub_->publish(std::make_unique<Percept>(percept));
    });

  // Subscriptions
  hingeJointSub =
    create_subscription<rcss3d_agent_msgs::msg::HingeJoint>(
    "/effectors/hinge_joint", 10,
    [this](rcss3d_agent_msgs::msg::HingeJoint::SharedPtr cmd) {
      rcss3dAgent->sendHingeJoint(*cmd);
    });

  universalJointSub =
    create_subscription<rcss3d_agent_msgs::msg::UniversalJoint>(
    "/effectors/universal_joint", 10,
    [this](rcss3d_agent_msgs::msg::UniversalJoint::SharedPtr cmd) {
      rcss3dAgent->sendUniversalJoint(*cmd);
    });

  beamSub =
    create_subscription<rcss3d_agent_msgs::msg::Beam>(
    "/effectors/beam", 10,
    [this](rcss3d_agent_msgs::msg::Beam::SharedPtr cmd) {
      rcss3dAgent->sendBeam(*cmd);
    });

  saySub =
    create_subscription<rcss3d_agent_msgs::msg::Say>(
    "/effectors/say", 10,
    [this](rcss3d_agent_msgs::msg::Say::SharedPtr cmd) {
      rcss3dAgent->sendSay(*cmd);
    });
}

Rcss3dAgentNode::~Rcss3dAgentNode()
{
}

}  // namespace rcss3d_agent
