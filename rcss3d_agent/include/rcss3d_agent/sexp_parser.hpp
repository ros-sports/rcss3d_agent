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

#ifndef RCSS3D_AGENT__SEXP_PARSER_HPP_
#define RCSS3D_AGENT__SEXP_PARSER_HPP_

#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "rcss3d_agent_msgs/msg/ball.hpp"
#include "rcss3d_agent_msgs/msg/goalpost_array.hpp"
#include "rcss3d_agent_msgs/msg/field_line_array.hpp"
#include "rcss3d_agent_msgs/msg/robot_array.hpp"
#include "rcss3d_agent_msgs/msg/flag_array.hpp"

#define SEXPRESSO_OPT_OUT_PIKESTYLE
#include "sexpresso/sexpresso.hpp"

namespace rcss3d_agent
{

class SexpParser
{
public:
  explicit SexpParser(std::string msg);

  rosgraph_msgs::msg::Clock getClock();
  std::optional<sensor_msgs::msg::Imu> getImu();
  sensor_msgs::msg::JointState getJoints();
  std::optional<rcss3d_agent_msgs::msg::Ball> getBall();
  std::optional<rcss3d_agent_msgs::msg::GoalpostArray> getGoalposts();
  std::optional<rcss3d_agent_msgs::msg::FieldLineArray> getFieldLines();
  std::optional<rcss3d_agent_msgs::msg::RobotArray> getRobots();
  std::optional<rcss3d_agent_msgs::msg::FlagArray> getFlags();

private:
  sexpresso::Sexp sexp;
  rclcpp::Logger logger;

  rosgraph_msgs::msg::Clock clock;
};


}  // namespace rcss3d_agent

#endif  // RCSS3D_AGENT__SEXP_PARSER_HPP_
