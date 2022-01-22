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

#ifndef RCSS3D_AGENT_NODE__RCSS3D_AGENT_NODE_HPP_
#define RCSS3D_AGENT_NODE__RCSS3D_AGENT_NODE_HPP_

#include <memory>
#include <vector>
#include <string>

#include "rclcpp/node.hpp"
#include "rcss3d_agent/rcss3d_agent.hpp"
#include "rcss3d_agent_msgs/msg/beam.hpp"

namespace rcss3d_agent_node
{

class Rcss3dAgentNode : public rclcpp::Node
{
public:
  explicit Rcss3dAgentNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  virtual ~Rcss3dAgentNode();

private:
  std::unique_ptr<rcss3d_agent::Params> params;
  std::unique_ptr<rcss3d_agent::Rcss3dAgent> rcss3dAgent;

  rclcpp::Publisher<rcss3d_agent_msgs::msg::Percept>::SharedPtr perceptPub;

  rclcpp::Subscription<rcss3d_agent_msgs::msg::HingeJointVel>::SharedPtr hingeJointSub;
  rclcpp::Subscription<rcss3d_agent_msgs::msg::UniversalJointVel>::SharedPtr universalJointSub;
  rclcpp::Subscription<rcss3d_agent_msgs::msg::Beam>::SharedPtr beamSub;
  rclcpp::Subscription<rcss3d_agent_msgs::msg::Say>::SharedPtr saySub;
};

}  // namespace rcss3d_agent_node

#endif  // RCSS3D_AGENT_NODE__RCSS3D_AGENT_NODE_HPP_
