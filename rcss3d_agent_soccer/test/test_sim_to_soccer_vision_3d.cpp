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

#include <gtest/gtest.h>
#include <optional>
#include "rcss3d_agent_msgs/msg/ball.hpp"
#include "rcss3d_agent_msgs/msg/spherical.hpp"
#include "rcss3d_agent_soccer/sim_to_soccer_vision_3d.hpp"
#include "../src/polar_to_point.hpp"

TEST(SimToSoccerVision3D, TestBallArrayNoBall)
{
  auto ballArray = rcss3d_agent_soccer::sim_to_soccer_vision_3d::getBallArray(std::nullopt);
  EXPECT_EQ(ballArray.balls.size(), 0u);
}

TEST(SimToSoccerVision3D, TestBallArrayOneBall)
{
  rcss3d_agent_msgs::msg::Ball ball;
  ball.center.r = 1.0;
  ball.center.phi = 45;
  ball.center.theta = 45;

  auto ballArray = rcss3d_agent_soccer::sim_to_soccer_vision_3d::getBallArray(ball);

  EXPECT_EQ(ballArray.header.frame_id, "CameraTop_frame");
  EXPECT_EQ(ballArray.balls.size(), 1u);
  EXPECT_NEAR(ballArray.balls[0].center.x, 0.5, 0.01);
  EXPECT_NEAR(ballArray.balls[0].center.y, 0.5, 0.01);
  EXPECT_NEAR(ballArray.balls[0].center.z, 0.7071, 0.01);
}

TEST(SimToSoccerVision3D, TestGoalpostArrayNoBall)
{
  auto goalpostArray = rcss3d_agent_soccer::sim_to_soccer_vision_3d::getGoalpostArray({});
  EXPECT_EQ(goalpostArray.posts.size(), 0u);
}

TEST(SimToSoccerVision3D, TestGoalpostArrayOneGoalpost)
{
  rcss3d_agent_msgs::msg::Goalpost goalpost;
  goalpost.top.r = 1.0;
  goalpost.top.phi = 45;
  goalpost.top.theta = 45;

  auto goalpostArray = rcss3d_agent_soccer::sim_to_soccer_vision_3d::getGoalpostArray({goalpost});

  EXPECT_EQ(goalpostArray.header.frame_id, "CameraTop_frame");
  EXPECT_EQ(goalpostArray.posts.size(), 1u);

  EXPECT_NEAR(goalpostArray.posts[0].bb.center.position.x, 0.5, 0.01);
  EXPECT_NEAR(goalpostArray.posts[0].bb.center.position.y, 0.5, 0.01);
  // Must subtract 0.4m (half of the goalpost height) to get the center's z-coordinate
  // because the top of the goalpost is observed. Note that this calculation doesn't take into
  // account the orientation of the goalpost, but it seems like this is the best we can do for now.
  // 0.7071 - 0.4 = 0.3031m
  EXPECT_NEAR(goalpostArray.posts[0].bb.center.position.z, 0.3071, 0.01);

  // Diameter of the goalpost is 0.1m (this is an estimate, based of SPL rules)
  EXPECT_NEAR(goalpostArray.posts[0].bb.size.x, 0.1, 0.01);
  EXPECT_NEAR(goalpostArray.posts[0].bb.size.y, 0.1, 0.01);

  // Height of the goalpost is 0.8m
  EXPECT_NEAR(goalpostArray.posts[0].bb.size.z, 0.8, 0.01);
}

TEST(SimToSoccerVision3D, TestGoalpostArrayMultipleGoalposts)
{
  std::vector<rcss3d_agent_msgs::msg::Goalpost> goalposts(2);

  auto goalpostArray = rcss3d_agent_soccer::sim_to_soccer_vision_3d::getGoalpostArray(goalposts);
  EXPECT_EQ(goalpostArray.posts.size(), 2u);
}

TEST(SimToSoccerVision3D, TestGoalpostArrayNoFieldLines)
{
  auto markingArray = rcss3d_agent_soccer::sim_to_soccer_vision_3d::getMarkingArray({});
  EXPECT_EQ(markingArray.segments.size(), 0u);
}

TEST(SimToSoccerVision3D, TestGoalpostArrayOneFieldLine)
{
  rcss3d_agent_msgs::msg::FieldLine fieldLine;
  fieldLine.start.r = 1.0;
  fieldLine.start.phi = 45;
  fieldLine.start.theta = 45;
  fieldLine.end.r = 1.0;
  fieldLine.end.phi = -45;
  fieldLine.end.theta = 45;

  auto markingArray = rcss3d_agent_soccer::sim_to_soccer_vision_3d::getMarkingArray({fieldLine});
  EXPECT_EQ(markingArray.header.frame_id, "CameraTop_frame");
  EXPECT_EQ(markingArray.segments.size(), 1u);
  EXPECT_NEAR(markingArray.segments[0].start.x, 0.5, 0.01);
  EXPECT_NEAR(markingArray.segments[0].start.y, 0.5, 0.01);
  EXPECT_NEAR(markingArray.segments[0].start.z, 0.7071, 0.01);
  EXPECT_NEAR(markingArray.segments[0].end.x, 0.5, 0.01);
  EXPECT_NEAR(markingArray.segments[0].end.y, -0.5, 0.01);
  EXPECT_NEAR(markingArray.segments[0].end.z, 0.7071, 0.01);
}

TEST(SimToSoccerVision3D, TestGoalpostArrayMultipleFieldLines)
{
  std::vector<rcss3d_agent_msgs::msg::FieldLine> fieldLines(2);

  auto markingArray = rcss3d_agent_soccer::sim_to_soccer_vision_3d::getMarkingArray(fieldLines);
  EXPECT_EQ(markingArray.segments.size(), 2u);
}

TEST(SimToSoccerVision3D, TestRobotArrayNoRobot)
{
  auto robotArray = rcss3d_agent_soccer::sim_to_soccer_vision_3d::getRobotArray({});
  EXPECT_EQ(robotArray.robots.size(), 0u);
}

TEST(SimToSoccerVision3D, TestRobotArrayOneRobotWithHead)
{
  rcss3d_agent_msgs::msg::Player player;
  rcss3d_agent_msgs::msg::Spherical head;
  head.r = 1.0;
  head.phi = 45;
  head.theta = 45;
  player.head.push_back(head);

  auto robotArray = rcss3d_agent_soccer::sim_to_soccer_vision_3d::getRobotArray({player});

  EXPECT_EQ(robotArray.header.frame_id, "CameraTop_frame");
  EXPECT_EQ(robotArray.robots.size(), 1u);

  EXPECT_NEAR(robotArray.robots[0].bb.center.position.x, 0.5, 0.01);
  EXPECT_NEAR(robotArray.robots[0].bb.center.position.y, 0.5, 0.01);
  EXPECT_NEAR(robotArray.robots[0].bb.center.position.z, 0.7071, 0.01);

  // Diameter of the robot is 0.3m (estimate)
  EXPECT_NEAR(robotArray.robots[0].bb.size.x, 0.3, 0.01);
  EXPECT_NEAR(robotArray.robots[0].bb.size.y, 0.3, 0.01);

  // Height of the robot is 0.6m (estimate)
  EXPECT_NEAR(robotArray.robots[0].bb.size.z, 0.6, 0.01);
}

TEST(SimToSoccerVision3D, TestRobotArrayRobotWithoutHeadShouldntBeCounted)
{
  rcss3d_agent_msgs::msg::Spherical spherical;
  spherical.r = 1.0;
  spherical.phi = 45;
  spherical.theta = 45;

  rcss3d_agent_msgs::msg::Player player;
  player.rlowerarm.push_back(spherical);
  player.llowerarm.push_back(spherical);
  player.rfoot.push_back(spherical);
  player.lfoot.push_back(spherical);

  auto robotArray = rcss3d_agent_soccer::sim_to_soccer_vision_3d::getRobotArray({player});

  EXPECT_EQ(robotArray.header.frame_id, "CameraTop_frame");
  EXPECT_EQ(robotArray.robots.size(), 0u);
}

TEST(SimToSoccerVision3D, TestRobotArrayMultipleRobots)
{
  std::vector<rcss3d_agent_msgs::msg::Player> players;
  rcss3d_agent_msgs::msg::Player player;
  player.head.push_back(rcss3d_agent_msgs::msg::Spherical{});

  players.push_back(player);
  players.push_back(player);

  auto robotArray = rcss3d_agent_soccer::sim_to_soccer_vision_3d::getRobotArray(players);
  EXPECT_EQ(robotArray.robots.size(), 2u);
}
