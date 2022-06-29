// Copyright 2022 Kenji Brameld
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

#ifndef RCSS3D_AGENT_MSGS_TO_SOCCER_INTERFACES__CONVERSION_HPP_
#define RCSS3D_AGENT_MSGS_TO_SOCCER_INTERFACES__CONVERSION_HPP_

#include <optional>
#include <vector>
#include "soccer_vision_3d_msgs/msg/ball_array.hpp"
#include "soccer_vision_3d_msgs/msg/goalpost_array.hpp"
#include "soccer_vision_3d_msgs/msg/marking_array.hpp"
#include "soccer_vision_3d_msgs/msg/robot_array.hpp"
#include "rcss3d_agent_msgs/msg/ball.hpp"
#include "rcss3d_agent_msgs/msg/goalpost.hpp"
#include "rcss3d_agent_msgs/msg/field_line.hpp"
#include "rcss3d_agent_msgs/msg/player.hpp"

namespace rcss3d_agent_msgs_to_soccer_interfaces
{

soccer_vision_3d_msgs::msg::BallArray getBallArray(
  const std::optional<rcss3d_agent_msgs::msg::Ball> & ball);

soccer_vision_3d_msgs::msg::GoalpostArray getGoalpostArray(
  const std::vector<rcss3d_agent_msgs::msg::Goalpost> & goalpost);

soccer_vision_3d_msgs::msg::MarkingArray getMarkingArray(
  const std::vector<rcss3d_agent_msgs::msg::FieldLine> & fieldLines);

soccer_vision_3d_msgs::msg::RobotArray getRobotArray(
  const std::vector<rcss3d_agent_msgs::msg::Player> & players);

}  // namespace rcss3d_agent_msgs_to_soccer_interfaces

#endif  // RCSS3D_AGENT_MSGS_TO_SOCCER_INTERFACES__CONVERSION_HPP_
