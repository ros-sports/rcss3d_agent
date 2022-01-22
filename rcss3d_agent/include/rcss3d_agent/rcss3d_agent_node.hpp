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

#ifndef RCSS3D_AGENT__RCSS3D_AGENT_NODE_HPP_
#define RCSS3D_AGENT__RCSS3D_AGENT_NODE_HPP_

#include <memory>
#include <vector>
#include <string>

#include "rclcpp/node.hpp"
#include "rcss3d_agent/rcss3d_agent.hpp"
#include "rcss3d_agent_msgs/msg/ball.hpp"
#include "rcss3d_agent_msgs/msg/beam.hpp"

namespace rcss3d_agent
{

class Rcss3dAgentNode : public rclcpp::Node
{
public:
  explicit Rcss3dAgentNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  virtual ~Rcss3dAgentNode();

private:
  using Ball = rcss3d_agent_msgs::msg::Ball;
  using Beam = rcss3d_agent_msgs::msg::Beam;
  using GyroRate = rcss3d_agent_msgs::msg::GyroRate;
  using HingeJoint = rcss3d_agent_msgs::msg::HingeJoint;
  using UniversalJoint = rcss3d_agent_msgs::msg::UniversalJoint;
  using ForceResistance = rcss3d_agent_msgs::msg::ForceResistance;
  using Accelerometer = rcss3d_agent_msgs::msg::Accelerometer;
  using Vision = rcss3d_agent_msgs::msg::Vision;
  using GameState = rcss3d_agent_msgs::msg::GameState;
  using AgentState = rcss3d_agent_msgs::msg::AgentState;
  using Hear = rcss3d_agent_msgs::msg::Hear;
  using Percept = rcss3d_agent_msgs::msg::Percept;

  std::unique_ptr<rcss3d_agent::Params> params;
  std::unique_ptr<Rcss3dAgent> rcss3dAgent;

  rclcpp::Publisher<Percept>::SharedPtr percept_pub_;

  rclcpp::Subscription<rcss3d_agent_msgs::msg::HingeJoint>::SharedPtr hingeJointSub;
  rclcpp::Subscription<rcss3d_agent_msgs::msg::UniversalJoint>::SharedPtr universalJointSub;
  rclcpp::Subscription<rcss3d_agent_msgs::msg::Beam>::SharedPtr beamSub;
  rclcpp::Subscription<rcss3d_agent_msgs::msg::Say>::SharedPtr saySub;
};

}  // namespace rcss3d_agent

#endif  // RCSS3D_AGENT__RCSS3D_AGENT_NODE_HPP_
