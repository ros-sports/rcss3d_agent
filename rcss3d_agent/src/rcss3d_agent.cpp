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

#include "rcss3d_agent/rcss3d_agent.hpp"

#include <netinet/tcp.h>
#include <cmath>
#include <string>
#include <utility>
#include <algorithm>
#include <memory>

#include "sexp_creator.hpp"
#include "sexp_parser.hpp"
#include "connection.hpp"

using namespace std::chrono_literals;

namespace rcss3d_agent
{

Rcss3dAgent::Rcss3dAgent(const Params & p)
: connection(std::make_unique<Connection>()),
  logger(rclcpp::get_logger("Rcss3dAgent"))
{
  // Declare parameters
  RCLCPP_DEBUG(logger, "Declare parameters");

  // Log parameters for debugging
  logParametersToRclcppDebug(p.model, p.rcss3d_host, p.rcss3d_port, p.team, p.unum);

  // Initialise connection
  connection->initialise(p.rcss3d_host, p.rcss3d_port);

  // Create the robot
  connection->send(sexp_creator::createCreateMessage(p.model));

  // Receive, this is needed for the init message to be sent next
  connection->receive();

  // Send init
  connection->send(sexp_creator::createInitMessage(p.team, p.unum));

  // Start receive loop
  receive_thread_ = std::thread(
    [this]() {
      while (rclcpp::ok()) {
        std::string recv = connection->receive();
        if (!recv.empty()) {
          RCLCPP_DEBUG(this->logger, ("Received: " + recv).c_str());
          handle(recv);
        } else {
          // Simulation connection broken, log to ERROR is done by connection.cpp
          break;
        }
      }
    });
}

Rcss3dAgent::~Rcss3dAgent()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
}

void Rcss3dAgent::handle(std::string const & msg)
{
  SexpParser parsed(msg);

  rcss3d_agent_msgs::msg::Percept percept;
  percept.gyro_rates = parsed.getGyroRates();
  percept.hinge_joints = parsed.getHingeJointPos();
  percept.universal_joints = parsed.getUniversalJointPos();
  percept.force_resistances = parsed.getForceResistances();
  percept.accelerometers = parsed.getAccelerometers();
  if (auto vision = parsed.getVision(); vision.has_value()) {
    percept.vision.push_back(vision.value());
  }
  percept.game_state = parsed.getGameState();
  if (auto agent_state = parsed.getAgentState(); agent_state.has_value()) {
    percept.agent_state.push_back(agent_state.value());
  }
  percept.hears = parsed.getHears();

  for (auto callbackPercept : callbacksPercept) {
    callbackPercept(percept);
  }
}

void Rcss3dAgent::sendHingeJointVel(const rcss3d_agent_msgs::msg::HingeJointVel & j)
{
  connection->send(sexp_creator::createHingeJointVelMessage(j));
}

void Rcss3dAgent::sendUniversalJointVel(const rcss3d_agent_msgs::msg::UniversalJointVel & j)
{
  connection->send(sexp_creator::createUniversalJointVelMessage(j));
}

void Rcss3dAgent::sendBeam(const rcss3d_agent_msgs::msg::Beam & b)
{
  connection->send(sexp_creator::createBeamMessage(b));
}

void Rcss3dAgent::sendSay(const rcss3d_agent_msgs::msg::Say & s)
{
  if (!s.message.empty()) {
    connection->send(sexp_creator::createSayMessage(s));
  } else {
    RCLCPP_ERROR(
      logger,
      "Say message was not sent as it was empty. Sending an empty Say message is prohibited "
      "as it may cause undefined behaviour on the receiver end.");
  }
}

void Rcss3dAgent::sendSynchronize()
{
  connection->send(sexp_creator::createSynchronizeMessage());
}

void Rcss3dAgent::registerPerceptCallback(
  std::function<void(const rcss3d_agent_msgs::msg::Percept &)> callback)
{
  callbacksPercept.push_back(callback);
}

void Rcss3dAgent::logParametersToRclcppDebug(
  std::string model, std::string rcss3d_host, int rcss3d_port, std::string team, int unum)
{
  RCLCPP_DEBUG(logger, "Parameters: ");
  RCLCPP_DEBUG(logger, "  model: %s", model.c_str());
  RCLCPP_DEBUG(logger, "  rcss3d/host: %s", rcss3d_host.c_str());
  RCLCPP_DEBUG(logger, "  rcss3d/port: %d", rcss3d_port);
  RCLCPP_DEBUG(logger, "  team: %s", team.c_str());
  RCLCPP_DEBUG(logger, "  unum: %d", unum);
}

}  // namespace rcss3d_agent
