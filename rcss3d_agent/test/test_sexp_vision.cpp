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
#include "../src/sexp_vision.hpp"

#define SEXPRESSO_OPT_OUT_PIKESTYLE
#include "sexpresso/sexpresso.hpp"

// Taken from https://gitlab.com/robocup-sim/SimSpark/-/wikis/Perceptors#vision-perceptors
// There is one extra backet (typo) in the original wiki text which has been fixed here
static const char * sexpEmpty = "See";
static sexpresso::Sexp seeSexpEmpty = sexpresso::parse(sexpEmpty);

static const char * sexp =
  "See "
  "(G2R (pol 17.55 -3.33 4.31)) "
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
  "(L (pol 12.97 -37.56 -2.24) (pol 13.32 -32.98 -2.20))";
static sexpresso::Sexp seeSexp = sexpresso::parse(sexp);

TEST(TestBall, TestNoBall)
{
  ASSERT_FALSE(rcss3d_agent::sexp_vision::getBall(seeSexpEmpty).has_value());
}

TEST(TestBall, TestHasBall)
{
  auto ball_optional = rcss3d_agent::sexp_vision::getBall(seeSexp);
  ASSERT_TRUE(ball_optional.has_value());

  // (B (pol 8.51 -0.21 -0.17))
  rcss3d_agent_msgs::msg::Ball & ball = ball_optional.value();
  EXPECT_NEAR(ball.center.r, 8.51, 0.01);
  EXPECT_NEAR(ball.center.theta, -0.21, 0.01);
  EXPECT_NEAR(ball.center.phi, -0.17, 0.01);
}

TEST(TestFieldLines, TestNoFieldLines)
{
  ASSERT_EQ(rcss3d_agent::sexp_vision::getFieldLines(seeSexpEmpty).size(), 0u);
}

TEST(TestFieldLines, TestFieldLines)
{
  auto fieldLines = rcss3d_agent::sexp_vision::getFieldLines(seeSexp);
  ASSERT_EQ(fieldLines.size(), 2u);

  // (L (pol 12.11 -40.77 -2.40) (pol 12.95 -37.76 -2.41))
  rcss3d_agent_msgs::msg::FieldLine & line1 = fieldLines.at(0);
  EXPECT_NEAR(line1.start.r, 12.11, 0.01);
  EXPECT_NEAR(line1.start.theta, -40.77, 0.01);
  EXPECT_NEAR(line1.start.phi, -2.40, 0.01);
  EXPECT_NEAR(line1.end.r, 12.95, 0.01);
  EXPECT_NEAR(line1.end.theta, -37.76, 0.01);
  EXPECT_NEAR(line1.end.phi, -2.41, 0.01);

  // (L (pol 12.97 -37.56 -2.24) (pol 13.32 -32.98 -2.20))
  rcss3d_agent_msgs::msg::FieldLine & line2 = fieldLines.at(1);
  EXPECT_NEAR(line2.start.r, 12.97, 0.01);
  EXPECT_NEAR(line2.start.theta, -37.56, 0.01);
  EXPECT_NEAR(line2.start.phi, -2.24, 0.01);
  EXPECT_NEAR(line2.end.r, 13.32, 0.01);
  EXPECT_NEAR(line2.end.theta, -32.98, 0.01);
  EXPECT_NEAR(line2.end.phi, -2.20, 0.01);
}


TEST(TestGoalposts, TestNoGoalposts)
{
  ASSERT_EQ(rcss3d_agent::sexp_vision::getGoalposts(seeSexpEmpty).size(), 0u);
}

TEST(TestGoalposts, TestGoalposts)
{
  auto goalposts = rcss3d_agent::sexp_vision::getGoalposts(seeSexp);
  ASSERT_EQ(goalposts.size(), 2u);

  // Checks in order of: G1L, G1R, G2L, G2R

  // (G1R (pol 17.52 3.27 4.07))
  rcss3d_agent_msgs::msg::Goalpost & post1 = goalposts.at(0);
  EXPECT_EQ(post1.name, "G1R");
  EXPECT_NEAR(post1.top.r, 17.52, 0.01);
  EXPECT_NEAR(post1.top.theta, 3.27, 0.01);
  EXPECT_NEAR(post1.top.phi, 4.07, 0.01);

  // (G2R (pol 17.55 -3.33 4.31))
  rcss3d_agent_msgs::msg::Goalpost & post2 = goalposts.at(1);
  EXPECT_EQ(post2.name, "G2R");
  EXPECT_NEAR(post2.top.r, 17.55, 0.01);
  EXPECT_NEAR(post2.top.theta, -3.33, 0.01);
  EXPECT_NEAR(post2.top.phi, 4.31, 0.01);
}

TEST(TestFlags, TestNoFlags)
{
  ASSERT_EQ(rcss3d_agent::sexp_vision::getFlags(seeSexpEmpty).size(), 0u);
}

TEST(TestFlags, TestFlags)
{
  auto flags = rcss3d_agent::sexp_vision::getFlags(seeSexp);
  ASSERT_EQ(flags.size(), 2u);

  // Checks in order of: F1L, F1R, F2L, F2R

  // "(F1R (pol 18.52 18.94 1.54)) "
  rcss3d_agent_msgs::msg::Flag & flag1 = flags.at(0);
  EXPECT_NEAR(flag1.base.r, 18.52, 0.01);
  EXPECT_NEAR(flag1.base.theta, 18.94, 0.01);
  EXPECT_NEAR(flag1.base.phi, 1.54, 0.01);

  // "(F2R (pol 18.52 -18.91 1.52)) "
  rcss3d_agent_msgs::msg::Flag & flag2 = flags.at(1);
  EXPECT_NEAR(flag2.base.r, 18.52, 0.01);
  EXPECT_NEAR(flag2.base.theta, -18.91, 0.01);
  EXPECT_NEAR(flag2.base.phi, 1.52, 0.01);
}

TEST(TestPlayers, TestNoPlayers)
{
  ASSERT_EQ(rcss3d_agent::sexp_vision::getPlayers(seeSexpEmpty).size(), 0u);
}

TEST(TestPlayers, TestPlayers)
{
  auto players = rcss3d_agent::sexp_vision::getPlayers(seeSexp);
  ASSERT_EQ(players.size(), 2u);

  // Player1
  rcss3d_agent_msgs::msg::Player & player1 = players.at(0);

  // "(P (team teamRed) (id 1) ..... )"
  EXPECT_EQ(player1.team, "teamRed");
  EXPECT_EQ(player1.id, 1);

  // "(head (pol 16.98 -0.21 3.19)) "
  ASSERT_EQ(player1.head.size(), 1u);
  EXPECT_NEAR(player1.head.at(0).r, 16.98, 0.01);
  EXPECT_NEAR(player1.head.at(0).theta, -0.21, 0.01);
  EXPECT_NEAR(player1.head.at(0).phi, 3.19, 0.01);

  // "(rlowerarm (pol 16.83 -0.06 2.80)) "
  ASSERT_EQ(player1.rlowerarm.size(), 1u);
  EXPECT_NEAR(player1.rlowerarm.at(0).r, 16.83, 0.01);
  EXPECT_NEAR(player1.rlowerarm.at(0).theta, -0.06, 0.01);
  EXPECT_NEAR(player1.rlowerarm.at(0).phi, 2.80, 0.01);

  // "(llowerarm (pol 16.86 -0.36 3.10)) "
  ASSERT_EQ(player1.llowerarm.size(), 1u);
  EXPECT_NEAR(player1.llowerarm.at(0).r, 16.86, 0.01);
  EXPECT_NEAR(player1.llowerarm.at(0).theta, -0.36, 0.01);
  EXPECT_NEAR(player1.llowerarm.at(0).phi, 3.10, 0.01);

  // "(rfoot (pol 17.00 0.29 1.68)) "
  ASSERT_EQ(player1.rfoot.size(), 1u);
  EXPECT_NEAR(player1.rfoot.at(0).r, 17.00, 0.01);
  EXPECT_NEAR(player1.rfoot.at(0).theta, 0.29, 0.01);
  EXPECT_NEAR(player1.rfoot.at(0).phi, 1.68, 0.01);

  // "(lfoot (pol 16.95 -0.51 1.32))) "
  ASSERT_EQ(player1.lfoot.size(), 1u);
  EXPECT_NEAR(player1.lfoot.at(0).r, 16.95, 0.01);
  EXPECT_NEAR(player1.lfoot.at(0).theta, -0.51, 0.01);
  EXPECT_NEAR(player1.lfoot.at(0).phi, 1.32, 0.01);

  // Player2
  rcss3d_agent_msgs::msg::Player & player2 = players.at(1);

  // "(P (team teamBlue) (id 3) ..... )"
  EXPECT_EQ(player2.team, "teamBlue");
  EXPECT_EQ(player2.id, 3);

  // "(rlowerarm (pol 0.18 -33.55 -20.16)) "
  ASSERT_EQ(player2.rlowerarm.size(), 1u);
  EXPECT_NEAR(player2.rlowerarm.at(0).r, 0.18, 0.01);
  EXPECT_NEAR(player2.rlowerarm.at(0).theta, -33.55, 0.01);
  EXPECT_NEAR(player2.rlowerarm.at(0).phi, -20.16, 0.01);

  // "(llowerarm (pol 0.18 34.29 -19.80))) "
  ASSERT_EQ(player2.llowerarm.size(), 1u);
  EXPECT_NEAR(player2.llowerarm.at(0).r, 0.18, 0.01);
  EXPECT_NEAR(player2.llowerarm.at(0).theta, 34.29, 0.01);
  EXPECT_NEAR(player2.llowerarm.at(0).phi, -19.80, 0.01);
}
