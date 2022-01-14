// Copyright 2019 Bold Hearts
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

#ifndef RCSS3D_AGENT__RCSS3D_JOINT_CONTROLLER_HPP_
#define RCSS3D_AGENT__RCSS3D_JOINT_CONTROLLER_HPP_

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rcss3d_agent_msgs/msg/joint_command.hpp"
#include "rcss3d_agent_msgs/msg/joint_position_command.hpp"

namespace rcss3d_agent
{

class Rcss3DJointController : public rclcpp::Node
{
public:
  explicit Rcss3DJointController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using JointState = sensor_msgs::msg::JointState;
  using JointCommand = rcss3d_agent_msgs::msg::JointCommand;
  using JointPositionCommand = rcss3d_agent_msgs::msg::JointPositionCommand;

  rclcpp::Subscription<JointPositionCommand>::SharedPtr joint_position_sub_;
  rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<JointCommand>::SharedPtr joint_command_pub_;

  JointPositionCommand::UniquePtr joint_positions_;

  static inline double normalize_angle_positive(double angle)
  {
    return fmod(fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
  }

  static inline double normalize_angle(double angle)
  {
    double a = normalize_angle_positive(angle);
    if (a > M_PI) {
      a -= 2.0 * M_PI;
    }
    return a;
  }
};

}  // namespace rcss3d_agent

#endif  // RCSS3D_AGENT__RCSS3D_JOINT_CONTROLLER_HPP_
