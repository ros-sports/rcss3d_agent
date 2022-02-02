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

#include "sexp_parser.hpp"
#include "sexp_vision.hpp"

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
  for (auto & arg : sexp.arguments()) {
    // Gyro expressions have form: (GYR (n <name>) (rt <x> <y> <z>))
    if (arg.value.sexp.at(0).value.str == "GYR") {
      auto * nSexp = arg.getChildByPath("n");
      auto * rtSexp = arg.getChildByPath("rt");
      if (nSexp && rtSexp) {
        rcss3d_agent_msgs::msg::GyroRate gyroRate;
        gyroRate.name = nSexp->value.sexp.at(1).value.str;
        gyroRate.x = std::stof(rtSexp->value.sexp.at(1).value.str);
        gyroRate.y = std::stof(rtSexp->value.sexp.at(2).value.str);
        gyroRate.z = std::stof(rtSexp->value.sexp.at(3).value.str);
        gyroRates.push_back(gyroRate);
      } else {
        RCLCPP_ERROR(logger, "Ignoring corrupted gyroscope.");
      }
    }
  }
  return gyroRates;
}

std::vector<rcss3d_agent_msgs::msg::HingeJointPos> SexpParser::getHingeJointPos()
{
  std::vector<rcss3d_agent_msgs::msg::HingeJointPos> hingeJoints;
  for (auto & arg : sexp.arguments()) {
    // Joint expressions have form: (HJ (n <name>) (ax <ax>))
    if (arg.value.sexp.at(0).value.str == "HJ") {
      auto * nSexp = arg.getChildByPath("n");
      auto * axSexp = arg.getChildByPath("ax");
      if (nSexp && axSexp) {
        rcss3d_agent_msgs::msg::HingeJointPos hingeJoint;
        hingeJoint.name = nSexp->value.sexp.at(1).value.str;
        hingeJoint.ax = std::stof(axSexp->value.sexp.at(1).value.str);
        hingeJoints.push_back(hingeJoint);
      } else {
        RCLCPP_ERROR(logger, "Ignoring corrupted hinge joint.");
      }
    }
  }
  return hingeJoints;
}

std::vector<rcss3d_agent_msgs::msg::UniversalJointPos> SexpParser::getUniversalJointPos()
{
  std::vector<rcss3d_agent_msgs::msg::UniversalJointPos> universalJoints;
  for (auto & arg : sexp.arguments()) {
    // Joint expressions have form: (UJ (n <name>) (ax1 <ax1>) (ax2 <ax2>))
    if (arg.value.sexp.at(0).value.str == "UJ") {
      auto * nSexp = arg.getChildByPath("n");
      auto * ax1Sexp = arg.getChildByPath("ax1");
      auto * ax2Sexp = arg.getChildByPath("ax2");
      if (nSexp && ax1Sexp && ax2Sexp) {
        rcss3d_agent_msgs::msg::UniversalJointPos universalJoint;
        universalJoint.name = nSexp->value.sexp.at(1).value.str;
        universalJoint.ax1 = std::stof(ax1Sexp->value.sexp.at(1).value.str);
        universalJoint.ax2 = std::stof(ax2Sexp->value.sexp.at(1).value.str);
        universalJoints.push_back(universalJoint);
      } else {
        RCLCPP_ERROR(logger, "Ignoring corrupted universal joint.");
      }
    }
  }
  return universalJoints;
}

std::vector<rcss3d_agent_msgs::msg::ForceResistance> SexpParser::getForceResistances()
{
  std::vector<rcss3d_agent_msgs::msg::ForceResistance> forceResistances;
  for (auto & arg : sexp.arguments()) {
    // Joint expressions have form: (FRP (n <name>) (c <px> <py> <pz>) (f <fx> <fy> <fz>))
    if (arg.value.sexp.at(0).value.str == "FRP") {
      auto * nSexp = arg.getChildByPath("n");
      auto * cSexp = arg.getChildByPath("c");
      auto * fSexp = arg.getChildByPath("f");
      if (nSexp && cSexp && fSexp) {
        rcss3d_agent_msgs::msg::ForceResistance forceResistance;
        forceResistance.name = nSexp->value.sexp.at(1).value.str;
        forceResistance.px = std::stof(cSexp->value.sexp.at(1).value.str);
        forceResistance.py = std::stof(cSexp->value.sexp.at(2).value.str);
        forceResistance.pz = std::stof(cSexp->value.sexp.at(3).value.str);
        forceResistance.fx = std::stof(fSexp->value.sexp.at(1).value.str);
        forceResistance.fy = std::stof(fSexp->value.sexp.at(2).value.str);
        forceResistance.fz = std::stof(fSexp->value.sexp.at(3).value.str);
        forceResistances.push_back(forceResistance);
      } else {
        RCLCPP_ERROR(logger, "Ignoring corrupted force resistance.");
      }
    }
  }
  return forceResistances;
}

std::vector<rcss3d_agent_msgs::msg::Accelerometer> SexpParser::getAccelerometers()
{
  std::vector<rcss3d_agent_msgs::msg::Accelerometer> accelerometers;
  for (auto & arg : sexp.arguments()) {
    // Joint expressions have form: (ACC (n <name>) (a <x> <y> <z>))
    if (arg.value.sexp.at(0).value.str == "ACC") {
      auto * nSexp = arg.getChildByPath("n");
      auto * aSexp = arg.getChildByPath("a");
      if (nSexp && aSexp) {
        rcss3d_agent_msgs::msg::Accelerometer accelerometer;
        accelerometer.name = nSexp->value.sexp.at(1).value.str;
        accelerometer.x = std::stof(aSexp->value.sexp.at(1).value.str);
        accelerometer.y = std::stof(aSexp->value.sexp.at(2).value.str);
        accelerometer.z = std::stof(aSexp->value.sexp.at(3).value.str);
        accelerometers.push_back(accelerometer);
      } else {
        RCLCPP_ERROR(logger, "Ignoring corrupted accleerometer.");
      }
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

  if (auto * tSexp = sexp.getChildByPath("GS/t"); tSexp != nullptr) {
    gameState.time = std::stof(tSexp->value.sexp.at(1).value.str);
  } else {
    RCLCPP_ERROR(logger, "Can't find GameState time in message received from simulator");
  }

  if (auto * pmSexp = sexp.getChildByPath("GS/pm"); pmSexp != nullptr) {
    gameState.playmode = pmSexp->value.sexp.at(1).value.str;
  } else {
    RCLCPP_ERROR(logger, "Can't find GameState playmode in message received from simulator");
  }

  if (auto * slSexp = sexp.getChildByPath("GS/sl"); slSexp != nullptr) {
    gameState.score_left = std::stod(slSexp->value.sexp.at(1).value.str);
  } else {
    RCLCPP_ERROR(logger, "Can't find GameState score left in message received from simulator");
  }

  if (auto * srSexp = sexp.getChildByPath("GS/sr"); srSexp != nullptr) {
    gameState.score_right = std::stod(srSexp->value.sexp.at(1).value.str);
  } else {
    RCLCPP_ERROR(logger, "Can't find GameState score right in message received from simulator");
  }

  return gameState;
}

std::optional<rcss3d_agent_msgs::msg::AgentState> SexpParser::getAgentState()
{
  if (auto * agentStateSexp = sexp.getChildByPath("AgentState"); agentStateSexp != nullptr) {
    auto * tempSexp = agentStateSexp->getChildByPath("temp");
    auto * batterySexp = agentStateSexp->getChildByPath("battery");
    if (tempSexp && batterySexp) {
      rcss3d_agent_msgs::msg::AgentState agentState;
      agentState.temp = std::stof(tempSexp->value.sexp.at(1).value.str);
      agentState.battery = std::stof(batterySexp->value.sexp.at(1).value.str);
      return agentState;
    } else {
      RCLCPP_ERROR(logger, "Ignoring corrupted agent state.");
    }
  }
  return std::nullopt;
}

std::vector<rcss3d_agent_msgs::msg::Hear> SexpParser::getHears()
{
  std::vector<rcss3d_agent_msgs::msg::Hear> hears;
  for (auto const & arg : sexp.arguments()) {
    // Hears have form: (hear <team> <time> self/<direction> <message>)
    auto const & s = arg.value.sexp;
    if (s.at(0).value.str == "hear") {
      rcss3d_agent_msgs::msg::Hear hear;
      hear.team = s.at(1).value.str;
      hear.time = std::stof(s.at(2).value.str);
      if (std::string arg2 = s.at(3).value.str; arg2 == "self") {
        hear.self = true;
      } else {
        hear.direction.push_back(std::stof(arg2));
      }
      hear.message = s.at(4).value.str;
      hears.push_back(hear);
    }
  }
  return hears;
}

}  // namespace rcss3d_agent
