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
#include "../src/sexp_parser.hpp"

TEST(TestGyroRates, TestNoGyroRates)
{
  rcss3d_agent::SexpParser parser("");
  auto gyroRates = parser.getGyroRates();
  EXPECT_EQ(gyroRates.size(), 0u);
}

TEST(TestGyroRates, TestOneGyroRate)
{
  rcss3d_agent::SexpParser parser("(GYR (n torso) (rt 0.01 0.07 0.46))");
  auto gyroRates = parser.getGyroRates();
  ASSERT_EQ(gyroRates.size(), 1u);
  auto gyroRate = gyroRates.at(0);
  EXPECT_EQ(gyroRate.name, "torso");
  EXPECT_NEAR(gyroRate.x, 0.01, 0.00001);
  EXPECT_NEAR(gyroRate.y, 0.07, 0.00001);
  EXPECT_NEAR(gyroRate.z, 0.46, 0.00001);
}

TEST(TestGyroRates, TestOneGyroRateDifferentOrder)
{
  rcss3d_agent::SexpParser parser("(GYR (rt 0.01 0.07 0.46) (n torso))");
  auto gyroRates = parser.getGyroRates();
  ASSERT_EQ(gyroRates.size(), 1u);
  auto gyroRate = gyroRates.at(0);
  EXPECT_EQ(gyroRate.name, "torso");
  EXPECT_NEAR(gyroRate.x, 0.01, 0.00001);
  EXPECT_NEAR(gyroRate.y, 0.07, 0.00001);
  EXPECT_NEAR(gyroRate.z, 0.46, 0.00001);
}

TEST(TestGyroRates, TestMultipleGyroRates)
{
  rcss3d_agent::SexpParser parser(
    "(GYR (n torso) (rt 0.01 0.07 0.46))(GYR (n head) (rt 0.02 0.08 0.47))");
  auto gyroRates = parser.getGyroRates();
  EXPECT_EQ(gyroRates.size(), 2u);
}

TEST(TestHingeJointPos, TestNoHingeJointPos)
{
  rcss3d_agent::SexpParser parser("");
  auto hingeJoints = parser.getHingeJointPos();
  EXPECT_EQ(hingeJoints.size(), 0u);
}

TEST(TestHingeJointPos, TestOneHingeJointPos)
{
  rcss3d_agent::SexpParser parser("(HJ (n laj3) (ax -1.02))");
  auto hingeJoints = parser.getHingeJointPos();
  ASSERT_EQ(hingeJoints.size(), 1u);
  auto hingeJoint = hingeJoints.at(0);
  EXPECT_EQ(hingeJoint.name, "laj3");
  EXPECT_NEAR(hingeJoint.ax, -1.02, 0.00001);
}

TEST(TestHingeJointPos, TestOneHingeJointPosDifferentOrder)
{
  rcss3d_agent::SexpParser parser("(HJ (ax -1.02) (n laj3))");
  auto hingeJoints = parser.getHingeJointPos();
  ASSERT_EQ(hingeJoints.size(), 1u);
  auto hingeJoint = hingeJoints.at(0);
  EXPECT_EQ(hingeJoint.name, "laj3");
  EXPECT_NEAR(hingeJoint.ax, -1.02, 0.00001);
}

TEST(TestHingeJointPos, TestMultipleHingeJointPos)
{
  rcss3d_agent::SexpParser parser("(HJ (n laj3) (ax -1.02))(HJ (n laj4) (ax -1.03))");
  auto hingeJoints = parser.getHingeJointPos();
  EXPECT_EQ(hingeJoints.size(), 2u);
}

TEST(TestUniversalJointPos, TestNoUniversalJointPos)
{
  rcss3d_agent::SexpParser parser("");
  auto universalJoints = parser.getUniversalJointPos();
  EXPECT_EQ(universalJoints.size(), 0u);
}

TEST(TestUniversalJointPos, TestOneUniversalJointPos)
{
  rcss3d_agent::SexpParser parser("(UJ (n laj1) (ax1 -1.32) (ax2 2.00))");
  auto universalJoints = parser.getUniversalJointPos();
  ASSERT_EQ(universalJoints.size(), 1u);
  auto universalJoint = universalJoints.at(0);
  EXPECT_EQ(universalJoint.name, "laj1");
  EXPECT_NEAR(universalJoint.ax1, -1.32, 0.00001);
  EXPECT_NEAR(universalJoint.ax2, 2.00, 0.00001);
}

TEST(TestUniversalJointPos, TestOneUniversalJointPosDifferentOrder)
{
  rcss3d_agent::SexpParser parser("(UJ (ax2 2.00) (ax1 -1.32) (n laj1))");
  auto universalJoints = parser.getUniversalJointPos();
  ASSERT_EQ(universalJoints.size(), 1u);
  auto universalJoint = universalJoints.at(0);
  EXPECT_EQ(universalJoint.name, "laj1");
  EXPECT_NEAR(universalJoint.ax1, -1.32, 0.00001);
  EXPECT_NEAR(universalJoint.ax2, 2.00, 0.00001);
}

TEST(TestUniversalJointPos, TestMultipleUniversalJointPos)
{
  rcss3d_agent::SexpParser parser(
    "(UJ (n laj1) (ax1 -1.32) (ax2 2.00))(UJ (n laj2) (ax1 -1.32) (ax2 2.00))");
  auto universalJoints = parser.getUniversalJointPos();
  EXPECT_EQ(universalJoints.size(), 2u);
}

TEST(TestForceResistances, TestNoForceResistances)
{
  rcss3d_agent::SexpParser parser("");
  auto forceResistances = parser.getForceResistances();
  EXPECT_EQ(forceResistances.size(), 0u);
}

TEST(TestForceResistances, TestOneForceResistance)
{
  rcss3d_agent::SexpParser parser("(FRP (n lf) (c -0.14 0.08 -0.05) (f 1.12 -0.26 13.07))");
  auto forceResistances = parser.getForceResistances();
  ASSERT_EQ(forceResistances.size(), 1u);
  auto forceResistance = forceResistances.at(0);
  EXPECT_EQ(forceResistance.name, "lf");
  EXPECT_NEAR(forceResistance.px, -0.14, 0.00001);
  EXPECT_NEAR(forceResistance.py, 0.08, 0.00001);
  EXPECT_NEAR(forceResistance.pz, -0.05, 0.00001);
  EXPECT_NEAR(forceResistance.fx, 1.12, 0.00001);
  EXPECT_NEAR(forceResistance.fy, -0.26, 0.00001);
  EXPECT_NEAR(forceResistance.fz, 13.07, 0.00001);
}

TEST(TestForceResistances, TestOneForceResistanceDifferentOrder)
{
  rcss3d_agent::SexpParser parser("(FRP (f 1.12 -0.26 13.07) (c -0.14 0.08 -0.05) (n lf))");
  auto forceResistances = parser.getForceResistances();
  ASSERT_EQ(forceResistances.size(), 1u);
  auto forceResistance = forceResistances.at(0);
  EXPECT_EQ(forceResistance.name, "lf");
  EXPECT_NEAR(forceResistance.px, -0.14, 0.00001);
  EXPECT_NEAR(forceResistance.py, 0.08, 0.00001);
  EXPECT_NEAR(forceResistance.pz, -0.05, 0.00001);
  EXPECT_NEAR(forceResistance.fx, 1.12, 0.00001);
  EXPECT_NEAR(forceResistance.fy, -0.26, 0.00001);
  EXPECT_NEAR(forceResistance.fz, 13.07, 0.00001);
}

TEST(TestForceResistances, TestMultipleForceResistances)
{
  rcss3d_agent::SexpParser parser(
    "(FRP (n lf) (c -0.14 0.08 -0.05) (f 1.12 -0.26 13.07))"
    "(FRP (n rf) (c -0.14 0.08 -0.05) (f 1.12 -0.26 13.07))");
  auto forceResistances = parser.getForceResistances();
  EXPECT_EQ(forceResistances.size(), 2u);
}

TEST(TestAccelerometers, TestNoAccelerometers)
{
  rcss3d_agent::SexpParser parser("");
  auto accelerometers = parser.getAccelerometers();
  EXPECT_EQ(accelerometers.size(), 0u);
}

TEST(TestAccelerometers, TestOneAccelerometer)
{
  rcss3d_agent::SexpParser parser("(ACC (n torso) (a 0.01 -0.01 9.81))");
  auto accelerometers = parser.getAccelerometers();
  ASSERT_EQ(accelerometers.size(), 1u);
  auto accelerometer = accelerometers.at(0);
  EXPECT_EQ(accelerometer.name, "torso");
  EXPECT_NEAR(accelerometer.x, 0.01, 0.00001);
  EXPECT_NEAR(accelerometer.y, -0.01, 0.00001);
  EXPECT_NEAR(accelerometer.z, 9.81, 0.00001);
}

TEST(TestAccelerometers, TestOneAccelerometerDifferentOrder)
{
  rcss3d_agent::SexpParser parser("(ACC (a 0.01 -0.01 9.81) (n torso))");
  auto accelerometers = parser.getAccelerometers();
  ASSERT_EQ(accelerometers.size(), 1u);
  auto accelerometer = accelerometers.at(0);
  EXPECT_EQ(accelerometer.name, "torso");
  EXPECT_NEAR(accelerometer.x, 0.01, 0.00001);
  EXPECT_NEAR(accelerometer.y, -0.01, 0.00001);
  EXPECT_NEAR(accelerometer.z, 9.81, 0.00001);
}

TEST(TestAccelerometers, TestMultipleAccelerometers)
{
  rcss3d_agent::SexpParser parser(
    "(ACC (n torso) (a 0.00 0.00 9.81))(ACC (n head) (a 0.00 0.00 9.81))");
  auto accelerometers = parser.getAccelerometers();
  EXPECT_EQ(accelerometers.size(), 2u);
}

TEST(TestVisions, TestNoVision)
{
  rcss3d_agent::SexpParser parser("");
  auto vision = parser.getVision();
  EXPECT_FALSE(vision.has_value());
}

TEST(TestVisions, TestHasVision)
{
  rcss3d_agent::SexpParser parser("(See)");
  auto vision = parser.getVision();
  EXPECT_TRUE(vision.has_value());
}

TEST(TestGameState, TestGameState)
{
  rcss3d_agent::SexpParser parser("(GS (sl 1) (sr 2) (t 2.50) (pm BeforeKickOff))");
  auto gameState = parser.getGameState();
  EXPECT_EQ(gameState.time, 2.50);
  EXPECT_EQ(gameState.playmode, "BeforeKickOff");
  EXPECT_EQ(gameState.score_left, 1);
  EXPECT_EQ(gameState.score_right, 2);
}

TEST(TestGameState, TestGameStateDifferentOrder)
{
  rcss3d_agent::SexpParser parser("(GS (pm BeforeKickOff) (t 2.50) (sr 2) (sl 1))");
  auto gameState = parser.getGameState();
  EXPECT_EQ(gameState.time, 2.50);
  EXPECT_EQ(gameState.playmode, "BeforeKickOff");
  EXPECT_EQ(gameState.score_left, 1);
  EXPECT_EQ(gameState.score_right, 2);
}

TEST(TestAgentState, TestNoAgentState)
{
  rcss3d_agent::SexpParser parser("");
  auto agentState = parser.getAgentState();
  EXPECT_FALSE(agentState.has_value());
}

TEST(TestAgentState, TestAgentState)
{
  rcss3d_agent::SexpParser parser("(AgentState (temp 48) (battery 75))");
  auto agentState = parser.getAgentState();
  ASSERT_TRUE(agentState.has_value());
  EXPECT_EQ(agentState.value().temp, 48);
  EXPECT_EQ(agentState.value().battery, 75);
}

TEST(TestAgentState, TestAgentStateDifferentOrder)
{
  rcss3d_agent::SexpParser parser("(AgentState (battery 75) (temp 48))");
  auto agentState = parser.getAgentState();
  ASSERT_TRUE(agentState.has_value());
  EXPECT_EQ(agentState.value().temp, 48);
  EXPECT_EQ(agentState.value().battery, 75);
}

TEST(TestHears, TestNoHears)
{
  rcss3d_agent::SexpParser parser("");
  auto hears = parser.getHears();
  EXPECT_EQ(hears.size(), 0u);
}

TEST(TestHears, TestHearSelf)
{
  rcss3d_agent::SexpParser parser("(hear teamFoo 12.3 self helloyourself)");
  auto hears = parser.getHears();
  ASSERT_EQ(hears.size(), 1u);
  auto hear = hears.at(0);
  EXPECT_EQ(hear.team, "teamFoo");
  EXPECT_NEAR(hear.time, 12.3, 0.001);
  EXPECT_TRUE(hear.self);
  EXPECT_EQ(hear.direction.size(), 0u);
  EXPECT_EQ(hear.message, "helloyourself");
}

TEST(TestHears, TestHearDirection)
{
  rcss3d_agent::SexpParser parser("(hear teamBar 12.3 -12.7 helloworld)");
  auto hears = parser.getHears();
  ASSERT_EQ(hears.size(), 1u);
  auto hear = hears.at(0);
  EXPECT_EQ(hear.team, "teamBar");
  EXPECT_NEAR(hear.time, 12.3, 0.001);
  EXPECT_FALSE(hear.self);
  ASSERT_EQ(hear.direction.size(), 1u);
  EXPECT_NEAR(hear.direction.at(0), -12.7, 0.001);
  EXPECT_EQ(hear.message, "helloworld");
}

TEST(TestHears, TestMultipleHears)
{
  rcss3d_agent::SexpParser parser(
    "(hear teamFoo 12.3 self helloyourself)"
    "(hear teamBar 12.3 -12.7 helloworld)");
  auto hears = parser.getHears();
  EXPECT_EQ(hears.size(), 2u);
}
