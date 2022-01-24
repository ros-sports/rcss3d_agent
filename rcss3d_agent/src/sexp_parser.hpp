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

#ifndef SEXP_PARSER_HPP_
#define SEXP_PARSER_HPP_

#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcss3d_agent_msgs/msg/gyro_rate.hpp"
#include "rcss3d_agent_msgs/msg/hinge_joint_pos.hpp"
#include "rcss3d_agent_msgs/msg/universal_joint_pos.hpp"
#include "rcss3d_agent_msgs/msg/force_resistance.hpp"
#include "rcss3d_agent_msgs/msg/accelerometer.hpp"
#include "rcss3d_agent_msgs/msg/vision.hpp"
#include "rcss3d_agent_msgs/msg/game_state.hpp"
#include "rcss3d_agent_msgs/msg/agent_state.hpp"
#include "rcss3d_agent_msgs/msg/hear.hpp"
#include "rcss3d_agent_msgs/msg/beam.hpp"
#include "rcss3d_agent_msgs/msg/say.hpp"
#include "rcss3d_agent_msgs/msg/percept.hpp"

#define SEXPRESSO_OPT_OUT_PIKESTYLE
#include "sexpresso/sexpresso.hpp"

namespace rcss3d_agent
{

class SexpParser
{
public:
  explicit SexpParser(std::string msg);

  std::vector<rcss3d_agent_msgs::msg::GyroRate> getGyroRates();
  std::vector<rcss3d_agent_msgs::msg::HingeJointPos> getHingeJointPos();
  std::vector<rcss3d_agent_msgs::msg::UniversalJointPos> getUniversalJointPos();
  std::vector<rcss3d_agent_msgs::msg::ForceResistance> getForceResistances();
  std::vector<rcss3d_agent_msgs::msg::Accelerometer> getAccelerometers();
  std::optional<rcss3d_agent_msgs::msg::Vision> getVision();
  rcss3d_agent_msgs::msg::GameState getGameState();
  std::optional<rcss3d_agent_msgs::msg::AgentState> getAgentState();
  std::vector<rcss3d_agent_msgs::msg::Hear> getHears();

private:
  sexpresso::Sexp sexp;
  rclcpp::Logger logger;
};


}  // namespace rcss3d_agent

#endif  // SEXP_PARSER_HPP_
