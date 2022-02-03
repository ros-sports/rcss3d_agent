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

#include <string>
#include "gtest/gtest.h"
#include "../src/sexp_creator.hpp"

TEST(TestSexpCreator, TestCreateCreateMessage)
{
  std::string m = "rsg/agent/soccerbotcomp.rsg";
  std::string msg = rcss3d_agent::sexp_creator::createCreateMessage(m);
  EXPECT_EQ(msg, "(scene rsg/agent/soccerbotcomp.rsg)");
}

TEST(TestSexpCreator, TestCreateBeamMessage)
{
  rcss3d_agent_msgs::msg::Beam b;
  b.x = 10.0;
  b.y = -10.0;
  b.rot = 0.1;
  std::string msg = rcss3d_agent::sexp_creator::createBeamMessage(b);
  EXPECT_EQ(msg, "(beam 10.000000 -10.000000 0.100000)");
}

TEST(TestSexpCreator, TestCreateSayMessage)
{
  rcss3d_agent_msgs::msg::Say s;
  s.message = "helloworld";
  std::string msg = rcss3d_agent::sexp_creator::createSayMessage(s);
  EXPECT_EQ(msg, "(say helloworld)");
}

TEST(TestSexpCreator, TestCreateHingeJointVelMessage)
{
  rcss3d_agent_msgs::msg::HingeJointVel j;
  j.name = "lae3";
  j.ax = 5.3;
  std::string msg = rcss3d_agent::sexp_creator::createHingeJointVelMessage(j);
  EXPECT_EQ(msg, "(lae3 5.300000)");
}

TEST(TestSexpCreator, TestCreateUniversalJointVelMessage)
{
  rcss3d_agent_msgs::msg::UniversalJointVel j;
  j.name = "lae1";
  j.ax1 = -2.3;
  j.ax2 = 1.2;
  std::string msg = rcss3d_agent::sexp_creator::createUniversalJointVelMessage(j);
  EXPECT_EQ(msg, "(lae1 -2.300000 1.200000)");
}

TEST(TestSexpCreator, TestCreateSynchronizeMessage)
{
  EXPECT_EQ(rcss3d_agent::sexp_creator::createSynchronizeMessage(), "(syn)");
}
