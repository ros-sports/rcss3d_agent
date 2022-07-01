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

#include <vector>
#include "rcss3d_agent_msgs_to_soccer_interfaces/conversion.hpp"
#include "polar_to_point.hpp"
#include "soccer_vision_3d_msgs/msg/ball.hpp"
#include "deg2rad.hpp"

namespace rcss3d_agent_msgs_to_soccer_interfaces
{

soccer_vision_3d_msgs::msg::BallArray getBallArray(
  const std::optional<rcss3d_agent_msgs::msg::Ball> & ball)
{
  soccer_vision_3d_msgs::msg::BallArray ballArray;
  ballArray.header.frame_id = "CameraTop_frame";
  if (ball.has_value()) {
    auto ballV = ball.value();
    auto converted = rcss3d_agent_msgs_to_soccer_interfaces::polar_to_point(
      ballV.center.r, deg2rad(ballV.center.phi), deg2rad(ballV.center.theta));

    soccer_vision_3d_msgs::msg::Ball ball;
    ball.center = converted;
    ballArray.balls.push_back(ball);
  }
  return ballArray;
}

soccer_vision_3d_msgs::msg::GoalpostArray getGoalpostArray(
  const std::vector<rcss3d_agent_msgs::msg::Goalpost> & goalposts)
{
  soccer_vision_3d_msgs::msg::GoalpostArray goalpostArray;
  goalpostArray.header.frame_id = "CameraTop_frame";
  for (auto & goalpost : goalposts) {
    auto converted = rcss3d_agent_msgs_to_soccer_interfaces::polar_to_point(
      goalpost.top.r, deg2rad(goalpost.top.phi), deg2rad(goalpost.top.theta));

    soccer_vision_3d_msgs::msg::Goalpost goalpostConverted;
    goalpostConverted.bb.center.position.x = converted.x;
    goalpostConverted.bb.center.position.y = converted.y;
    // Must subtract 0.4m (half of the goalpost height) to get the center's z-coordinate
    // because the top of the goalpost is observed. Note that this calculation doesn't take into
    // account the orientation of the goalpost, but it seems like this is the best we can do for
    // now.
    // 0.7071 - 0.4 = 0.3031m
    goalpostConverted.bb.center.position.z = converted.z - 0.4;
    // Diameter of the goalpost is 0.1m (this is an estimate, based of SPL rules)
    goalpostConverted.bb.size.x = 0.1;
    goalpostConverted.bb.size.y = 0.1;
    // Height of the goalpost is 0.8m
    goalpostConverted.bb.size.z = 0.8;

    goalpostArray.posts.push_back(goalpostConverted);
  }
  return goalpostArray;
}

soccer_vision_3d_msgs::msg::MarkingArray getMarkingArray(
  const std::vector<rcss3d_agent_msgs::msg::FieldLine> & fieldLines)
{
  soccer_vision_3d_msgs::msg::MarkingArray markingArray;
  markingArray.header.frame_id = "CameraTop_frame";
  for (auto & fieldLine : fieldLines) {
    soccer_vision_3d_msgs::msg::MarkingSegment markingSegment;
    markingSegment.start = rcss3d_agent_msgs_to_soccer_interfaces::polar_to_point(
      fieldLine.start.r, deg2rad(fieldLine.start.phi), deg2rad(fieldLine.start.theta));
    markingSegment.end = rcss3d_agent_msgs_to_soccer_interfaces::polar_to_point(
      fieldLine.end.r, deg2rad(fieldLine.end.phi), deg2rad(fieldLine.end.theta));

    markingArray.segments.push_back(markingSegment);
  }
  return markingArray;
}

soccer_vision_3d_msgs::msg::RobotArray getRobotArray(
  const std::vector<rcss3d_agent_msgs::msg::Player> & players, std::string nameTeamOwn)
{
  soccer_vision_3d_msgs::msg::RobotArray robotArray;
  robotArray.header.frame_id = "CameraTop_frame";
  for (auto & player : players) {
    // Only take into account robots that have a head reported, just because this simplifies the
    // logic. Future improvements to use other body parts are welcome.
    if (player.head.size() > 0) {
      soccer_vision_3d_msgs::msg::Robot robot;

      robot.bb.center.position = rcss3d_agent_msgs_to_soccer_interfaces::polar_to_point(
        player.head[0].r, deg2rad(player.head[0].phi), deg2rad(player.head[0].theta));

      // Diameter of the robot is 0.3m (estimate)
      robot.bb.size.x = 0.3;
      robot.bb.size.y = 0.3;

      // Height of the robot is 0.6m (estimate)
      robot.bb.size.z = 0.6;

      if (nameTeamOwn == "") {
        robot.attributes.team = soccer_vision_attribute_msgs::msg::Robot::TEAM_UNKNOWN;
      } else if (nameTeamOwn == player.team) {
        robot.attributes.team = soccer_vision_attribute_msgs::msg::Robot::TEAM_OWN;
      } else {
        robot.attributes.team = soccer_vision_attribute_msgs::msg::Robot::TEAM_OPPONENT;
      }

      robotArray.robots.push_back(robot);
    }
  }
  return robotArray;
}

}  // namespace rcss3d_agent_msgs_to_soccer_interfaces
