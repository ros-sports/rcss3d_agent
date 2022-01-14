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

#ifndef RCSS3D_AGENT__RCSS3D_AGENT_HPP_
#define RCSS3D_AGENT__RCSS3D_AGENT_HPP_

#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcss3d_agent/visibility_control.h"
#include "rcss3d_agent/connection.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "rcss3d_agent_msgs/msg/joint_command.hpp"
#include "rcss3d_agent_msgs/msg/ball.hpp"
#include "rcss3d_agent_msgs/msg/goalpost_array.hpp"
#include "rcss3d_agent_msgs/msg/field_line_array.hpp"
#include "rcss3d_agent_msgs/msg/robot_array.hpp"
#include "rcss3d_agent_msgs/msg/flag_array.hpp"
#include "rcss3d_agent_msgs/msg/beam.hpp"

namespace rcss3d_agent
{

class Rcss3dAgent : public rclcpp::Node
{
public:
  explicit Rcss3dAgent(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

  virtual ~Rcss3dAgent();

private:
  using Clock = rosgraph_msgs::msg::Clock;
  using Imu = sensor_msgs::msg::Imu;
  using JointState = sensor_msgs::msg::JointState;
  using JointCommand = rcss3d_agent_msgs::msg::JointCommand;
  using Ball = rcss3d_agent_msgs::msg::Ball;
  using GoalpostArray = rcss3d_agent_msgs::msg::GoalpostArray;
  using FieldLineArray = rcss3d_agent_msgs::msg::FieldLineArray;
  using RobotArray = rcss3d_agent_msgs::msg::RobotArray;
  using FlagArray = rcss3d_agent_msgs::msg::FlagArray;
  using Beam = rcss3d_agent_msgs::msg::Beam;

  Connection connection;

  std::thread receive_thread_;

  rclcpp::Publisher<Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<Ball>::SharedPtr ball_pub_;
  rclcpp::Publisher<GoalpostArray>::SharedPtr posts_pub_;
  rclcpp::Publisher<FieldLineArray>::SharedPtr lines_pub_;
  rclcpp::Publisher<RobotArray>::SharedPtr robots_pub_;
  rclcpp::Publisher<FlagArray>::SharedPtr flags_pub_;

  rclcpp::Subscription<JointCommand>::SharedPtr joint_command_sub_;
  rclcpp::Subscription<Beam>::SharedPtr beam_sub_;

  void handle(
    std::string const & msg, std::string const & imu_frame,
    std::string const & camera_frame);
  void logParametersToRclcppDebug(
    std::string rcss3d_host, int rcss3d_port, std::string team, int unum, std::string imu_frame,
    std::string camera_frame);
};

}  // namespace rcss3d_agent

#endif  // RCSS3D_AGENT__RCSS3D_AGENT_HPP_
