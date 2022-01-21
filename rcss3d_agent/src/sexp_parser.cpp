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
#include <vector>

#include "rcss3d_agent/sexp_parser.hpp"
#include "./sexp_vision.hpp"

namespace rcss3d_agent
{

SexpParser::SexpParser(std::string msg)
: sexp(sexpresso::parse("()" + msg)),
  // "()" tricks sexpresso into making everything a sexp argument. Without this, the first
  // perceptor (eg. "(time (now 104.87))") becomes the sexp parent, which is not what we want.
  logger(rclcpp::get_logger("sexp_parser"))
{
}

std::vector<rcss3d_agent_msgs::msg::GyroRate> SexpParser::getGyroRates()
{
  std::vector<rcss3d_agent_msgs::msg::GyroRate> gyroRates;
  for (auto const & arg : sexp.arguments()) {
    // Gyro expressions have form: (GYR (n <name>) (rt <x> <y> <z>))
    auto const & s = arg.value.sexp;
    if (s.at(0).value.str == "GYR") {
      rcss3d_agent_msgs::msg::GyroRate gyroRate;
      gyroRate.name = s.at(1).value.sexp.at(1).value.str;
      gyroRate.x = std::stof(s.at(2).value.sexp.at(1).value.str);
      gyroRate.y = std::stof(s.at(2).value.sexp.at(2).value.str);
      gyroRate.z = std::stof(s.at(2).value.sexp.at(3).value.str);
      gyroRates.push_back(gyroRate);
    }
  }
  return gyroRates;
}

std::vector<rcss3d_agent_msgs::msg::HingeJoint> SexpParser::getHingeJoints()
{
  std::vector<rcss3d_agent_msgs::msg::HingeJoint> hingeJoints;
  for (auto const & arg : sexp.arguments()) {
    // Joint expressions have form: (HJ (n <name>) (ax <ax>))
    auto const & s = arg.value.sexp;
    if (s.at(0).value.str == "HJ") {
      rcss3d_agent_msgs::msg::HingeJoint hingeJoint;
      hingeJoint.name = s.at(1).value.sexp.at(1).value.str;
      hingeJoint.ax = std::stof(s.at(2).value.sexp.at(1).value.str);
      hingeJoints.push_back(hingeJoint);
    }
  }
  return hingeJoints;
}

std::vector<rcss3d_agent_msgs::msg::UniversalJoint> SexpParser::getUniversalJoints()
{
  std::vector<rcss3d_agent_msgs::msg::UniversalJoint> universalJoints;
  for (auto const & arg : sexp.arguments()) {
    // Joint expressions have form: (UJ (n <name>) (ax1 <ax1>) (ax2 <ax2>))
    auto const & s = arg.value.sexp;
    if (s.at(0).value.str == "UJ") {
      rcss3d_agent_msgs::msg::UniversalJoint universalJoint;
      universalJoint.name = s.at(1).value.sexp.at(1).value.str;
      universalJoint.ax1 = std::stof(s.at(2).value.sexp.at(1).value.str);
      universalJoint.ax2 = std::stof(s.at(3).value.sexp.at(1).value.str);
      universalJoints.push_back(universalJoint);
    }
  }
  return universalJoints;
}

std::vector<rcss3d_agent_msgs::msg::ForceResistance> SexpParser::getForceResistances()
{
  std::vector<rcss3d_agent_msgs::msg::ForceResistance> forceResistances;
  for (auto const & arg : sexp.arguments()) {
    // Joint expressions have form: (FRP (n <name>) (c <px> <py> <pz>) (f <fx> <fy> <fz>))
    auto const & s = arg.value.sexp;
    if (s.at(0).value.str == "FRP") {
      rcss3d_agent_msgs::msg::ForceResistance forceResistance;
      forceResistance.name = s.at(1).value.sexp.at(1).value.str;
      forceResistance.px = std::stof(s.at(2).value.sexp.at(1).value.str);
      forceResistance.py = std::stof(s.at(2).value.sexp.at(2).value.str);
      forceResistance.pz = std::stof(s.at(2).value.sexp.at(3).value.str);
      forceResistance.fx = std::stof(s.at(3).value.sexp.at(1).value.str);
      forceResistance.fy = std::stof(s.at(3).value.sexp.at(2).value.str);
      forceResistance.fz = std::stof(s.at(3).value.sexp.at(3).value.str);
      forceResistances.push_back(forceResistance);
    }
  }
  return forceResistances;
}

std::vector<rcss3d_agent_msgs::msg::Accelerometer> SexpParser::getAccelerometers()
{
  std::vector<rcss3d_agent_msgs::msg::Accelerometer> accelerometers;
  for (auto const & arg : sexp.arguments()) {
    // Joint expressions have form: (ACC (n <name>) (a <x> <y> <z>))
    auto const & s = arg.value.sexp;
    if (s.at(0).value.str == "ACC") {
      rcss3d_agent_msgs::msg::Accelerometer accelerometer;
      accelerometer.name = s.at(1).value.sexp.at(1).value.str;
      accelerometer.x = std::stof(s.at(2).value.sexp.at(1).value.str);
      accelerometer.y = std::stof(s.at(2).value.sexp.at(2).value.str);
      accelerometer.z = std::stof(s.at(2).value.sexp.at(3).value.str);
      accelerometers.push_back(accelerometer);
    }
  }
  return accelerometers;
}

std::optional<rcss3d_agent_msgs::msg::Vision> SexpParser::getVision()
{
  auto * seeSexp = sexp.getChildByPath("See");
  if (seeSexp != nullptr) {
    rcss3d_agent_msgs::msg::Vision vision;
    if (auto ball = sexp_vision::getBall(*seeSexp); ball.has_value()) {
      vision.ball.push_back(ball.value());
    }
    return vision;
  }
  return std::nullopt;
}

rcss3d_agent_msgs::msg::GameState SexpParser::getGameState()
{
  rcss3d_agent_msgs::msg::GameState gameState;
  if (auto * gameStateSexp = sexp.getChildByPath("GS"); gameStateSexp != nullptr) {
    gameState.time = std::stof(gameStateSexp->value.sexp.at(1).value.sexp.at(1).value.str);
    gameState.playmode = gameStateSexp->value.sexp.at(2).value.sexp.at(1).value.str;
  } else {
    RCLCPP_ERROR(logger, "Can't find GameState in message received from rcssserver3d");
  }

  return gameState;
}

std::optional<rcss3d_agent_msgs::msg::AgentState> SexpParser::getAgentState()
{
  if (auto * agentStateSexp = sexp.getChildByPath("AgentState"); agentStateSexp != nullptr) {
    rcss3d_agent_msgs::msg::AgentState agentState;
    agentState.temp = std::stof(agentStateSexp->value.sexp.at(1).value.sexp.at(1).value.str);
    agentState.battery = std::stof(agentStateSexp->value.sexp.at(2).value.sexp.at(1).value.str);
    return agentState;
  }
  return std::nullopt;
}

std::vector<rcss3d_agent_msgs::msg::Hear> SexpParser::getHears()
{
  std::vector<rcss3d_agent_msgs::msg::Hear> hears;
  for (auto const & arg : sexp.arguments()) {
    // Hears have form: (hear <time> self/<direction> <message>)
    auto const & s = arg.value.sexp;
    if (s.at(0).value.str == "hear") {
      rcss3d_agent_msgs::msg::Hear hear;
      hear.time = std::stof(s.at(1).value.str);
      if (std::string arg2 = s.at(2).value.str; arg2 == "self") {
        hear.self = true;
      } else {
        hear.direction.push_back(std::stof(arg2));
      }
      hear.message = s.at(3).value.str;
      hears.push_back(hear);
    }
  }
  return hears;
}

}  // namespace rcss3d_agent
