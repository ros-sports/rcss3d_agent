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

#include <gtest/gtest.h>
#include <iostream>
#include "rcss3d_agent/sexp_parser.hpp"
#include "rcss3d_agent/angle_conversion.hpp"

// Taken from https://gitlab.com/robocup-sim/SimSpark/-/wikis/Perceptors#vision-perceptors
// There is one extra backet (typo) in the original wiki text which has been fixed here
static const char * sexp =
  "(time (now 104.87))"
  "(See (G2R (pol 17.55 -3.33 4.31)) "
  "(G1R (pol 17.52 3.27 4.07)) "
  "(F1R (pol 18.52 18.94 1.54)) "
  "(F2R (pol 18.52 -18.91 1.52)) "
  "(B (pol 8.51 -0.21 -0.17)) "
  "(P (team teamRed) (id 1) "
  "(head (pol 16.98 -0.21 3.19)) "
  "(rlowerarm (pol 16.83 -0.06 2.80)) "
  "(llowerarm (pol 16.86 -0.36 3.10)) "
  "(rfoot (pol 17.00 0.29 1.68)) "
  "(lfoot (pol 16.95 -0.51 1.32))) "
  "(P (team teamBlue) (id 3) "
  "(rlowerarm (pol 0.18 -33.55 -20.16)) "
  "(llowerarm (pol 0.18 34.29 -19.80))) "
  "(L (pol 12.11 -40.77 -2.40) (pol 12.95 -37.76 -2.41)) "
  "(L (pol 12.97 -37.56 -2.24) (pol 13.32 -32.98 -2.20)))";

static const char * sexp_empty = "(time (now 104.87))(See )";
static const char * sexp_none = "(time (now 104.87))";

TEST(TestGoalposts, TestHasGoalposts)
{
  rcss3d_agent::SexpParser parser(sexp);
  auto goalposts_optional = parser.getGoalposts();
  ASSERT_EQ(goalposts_optional.has_value(), true);

  auto goalposts = goalposts_optional.value();
  ASSERT_EQ(goalposts.posts.size(), 2u);

  // Checks in order of: G1L, G1R, G2L, G2R

  // (G1R (pol 17.52 3.27 4.07))
  rcss3d_agent_msgs::msg::Goalpost & post1 = goalposts.posts.at(0);
  EXPECT_NEAR(post1.top.r, 17.52, 0.01);
  EXPECT_NEAR(post1.top.theta, rcss3d_agent::angle_conversion::deg2rad(3.27), 0.01);
  EXPECT_NEAR(post1.top.phi, rcss3d_agent::angle_conversion::deg2rad(4.07), 0.01);

  // (G2R (pol 17.55 -3.33 4.31))
  rcss3d_agent_msgs::msg::Goalpost & post2 = goalposts.posts.at(1);
  EXPECT_NEAR(post2.top.r, 17.55, 0.01);
  EXPECT_NEAR(post2.top.theta, rcss3d_agent::angle_conversion::deg2rad(-3.33), 0.01);
  EXPECT_NEAR(post2.top.phi, rcss3d_agent::angle_conversion::deg2rad(4.31), 0.01);
}

TEST(TestGoalposts, TestNoGoalposts)
{
  rcss3d_agent::SexpParser parser(sexp_empty);
  ASSERT_EQ(parser.getGoalposts().has_value(), false);
}

TEST(TestGoalposts, TestNoVisionData)
{
  rcss3d_agent::SexpParser parser(sexp_none);
  ASSERT_EQ(parser.getGoalposts().has_value(), false);
}

TEST(TestBall, TestHasBall)
{
  rcss3d_agent::SexpParser parser(sexp);
  auto ball_optional = parser.getBall();
  ASSERT_EQ(ball_optional.has_value(), true);

  // (B (pol 8.51 -0.21 -0.17))
  auto ball = ball_optional.value();
  EXPECT_NEAR(ball.center.r, 8.51, 0.01);
  EXPECT_NEAR(ball.center.theta, rcss3d_agent::angle_conversion::deg2rad(-0.21), 0.01);
  EXPECT_NEAR(ball.center.phi, rcss3d_agent::angle_conversion::deg2rad(-0.17), 0.01);
}

TEST(TestBall, TestNoBall)
{
  rcss3d_agent::SexpParser parser(sexp_empty);
  ASSERT_EQ(parser.getBall().has_value(), false);
}

TEST(TestBall, TestNoVisionData)
{
  rcss3d_agent::SexpParser parser(sexp_none);
  ASSERT_EQ(parser.getBall().has_value(), false);
}

TEST(TestFieldLines, TestHasFieldLines)
{
  rcss3d_agent::SexpParser parser(sexp);
  auto lines_optional = parser.getFieldLines();
  ASSERT_EQ(lines_optional.has_value(), true);

  auto lines = lines_optional.value();
  ASSERT_EQ(lines.lines.size(), 2u);

  // (L (pol 12.11 -40.77 -2.40) (pol 12.95 -37.76 -2.41))
  rcss3d_agent_msgs::msg::FieldLine & line1 = lines.lines.at(0);
  EXPECT_NEAR(line1.start.r, 12.11, 0.01);
  EXPECT_NEAR(line1.start.theta, rcss3d_agent::angle_conversion::deg2rad(-40.77), 0.01);
  EXPECT_NEAR(line1.start.phi, rcss3d_agent::angle_conversion::deg2rad(-2.40), 0.01);
  EXPECT_NEAR(line1.end.r, 12.95, 0.01);
  EXPECT_NEAR(line1.end.theta, rcss3d_agent::angle_conversion::deg2rad(-37.76), 0.01);
  EXPECT_NEAR(line1.end.phi, rcss3d_agent::angle_conversion::deg2rad(-2.41), 0.01);

  // (L (pol 12.97 -37.56 -2.24) (pol 13.32 -32.98 -2.20))
  rcss3d_agent_msgs::msg::FieldLine & line2 = lines.lines.at(1);
  EXPECT_NEAR(line2.start.r, 12.97, 0.01);
  EXPECT_NEAR(line2.start.theta, rcss3d_agent::angle_conversion::deg2rad(-37.56), 0.01);
  EXPECT_NEAR(line2.start.phi, rcss3d_agent::angle_conversion::deg2rad(-2.24), 0.01);
  EXPECT_NEAR(line2.end.r, 13.32, 0.01);
  EXPECT_NEAR(line2.end.theta, rcss3d_agent::angle_conversion::deg2rad(-32.98), 0.01);
  EXPECT_NEAR(line2.end.phi, rcss3d_agent::angle_conversion::deg2rad(-2.20), 0.01);
}

TEST(TestFieldLines, TestNoFieldLines)
{
  rcss3d_agent::SexpParser parser(sexp_empty);
  ASSERT_EQ(parser.getFieldLines().has_value(), false);
}

TEST(TestFieldLines, TestNoVisionData)
{
  rcss3d_agent::SexpParser parser(sexp_none);
  ASSERT_EQ(parser.getFieldLines().has_value(), false);
}

TEST(TestRobots, TestHasRobots)
{
  // Currently, WE ONLY DETECT THE "HEAD" of the robot
  // If a limb is sent, we ignore it. That's why the
  // second robot is not counted as a robot in thie test.
  rcss3d_agent::SexpParser parser(sexp);
  auto robots_optional = parser.getRobots();
  ASSERT_EQ(robots_optional.has_value(), true);

  auto robots = robots_optional.value();
  ASSERT_EQ(robots.robots.size(), 1u);

  // (head (pol 16.98 -0.21 3.19))
  rcss3d_agent_msgs::msg::Robot & robot1 = robots.robots.at(0);
  EXPECT_EQ(robot1.team, "teamRed");
  EXPECT_EQ(robot1.id, 1);
  EXPECT_NEAR(robot1.head.r, 16.98, 0.01);
  EXPECT_NEAR(robot1.head.theta, rcss3d_agent::angle_conversion::deg2rad(-0.21), 0.01);
  EXPECT_NEAR(robot1.head.phi, rcss3d_agent::angle_conversion::deg2rad(3.19), 0.01);
}

TEST(TestRobots, TestNoRobots)
{
  rcss3d_agent::SexpParser parser(sexp_empty);
  ASSERT_EQ(parser.getRobots().has_value(), false);
}

TEST(TestRobots, TestNoVisionData)
{
  rcss3d_agent::SexpParser parser(sexp_none);
  ASSERT_EQ(parser.getRobots().has_value(), false);
}

TEST(TestFlags, TestHasFlags)
{
  rcss3d_agent::SexpParser parser(sexp);
  auto flags_optional = parser.getFlags();
  ASSERT_EQ(flags_optional.has_value(), true);

  auto flags = flags_optional.value();
  ASSERT_EQ(flags.flags.size(), 2u);

  // Checks in order of: F1L, F1R, F2L, F2R

  // "(F1R (pol 18.52 18.94 1.54)) "
  rcss3d_agent_msgs::msg::Flag & flag1 = flags.flags.at(0);
  EXPECT_NEAR(flag1.base.r, 18.52, 0.01);
  EXPECT_NEAR(flag1.base.theta, rcss3d_agent::angle_conversion::deg2rad(18.94), 0.01);
  EXPECT_NEAR(flag1.base.phi, rcss3d_agent::angle_conversion::deg2rad(1.54), 0.01);

  // "(F2R (pol 18.52 -18.91 1.52)) "
  rcss3d_agent_msgs::msg::Flag & flag2 = flags.flags.at(1);
  EXPECT_NEAR(flag2.base.r, 18.52, 0.01);
  EXPECT_NEAR(flag2.base.theta, rcss3d_agent::angle_conversion::deg2rad(-18.91), 0.01);
  EXPECT_NEAR(flag2.base.phi, rcss3d_agent::angle_conversion::deg2rad(1.52), 0.01);
}

TEST(TestFlags, TestNoFlags)
{
  rcss3d_agent::SexpParser parser(sexp_empty);
  ASSERT_EQ(parser.getFlags().has_value(), false);
}

TEST(TestFlags, TestNoVisionData)
{
  rcss3d_agent::SexpParser parser(sexp_none);
  ASSERT_EQ(parser.getFlags().has_value(), false);
}
