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

#ifndef RCSS3D_AGENT__SEXP_CREATOR_HPP_
#define RCSS3D_AGENT__SEXP_CREATOR_HPP_

#include <string>
#include <vector>
#include "rcss3d_agent_msgs/msg/beam.hpp"
#include "rcss3d_agent_msgs/msg/hinge_joint.hpp"
#include "rcss3d_agent_msgs/msg/universal_joint.hpp"
#include "rcss3d_agent_msgs/msg/say.hpp"

namespace rcss3d_agent
{

namespace sexp_creator
{
std::string createCreateMessage();
std::string createInitMessage(std::string const & team_name, int number);
std::string createJointMessage(std::vector<std::string> names, std::vector<float> speeds);
std::string createBeamMessage(const rcss3d_agent_msgs::msg::Beam & b);
std::string createSayMessage(const rcss3d_agent_msgs::msg::Say & s);
std::string createHingeJointMessage(const rcss3d_agent_msgs::msg::HingeJoint & j);
std::string createUniversalJointMessage(const rcss3d_agent_msgs::msg::UniversalJoint & j);
}

}  // namespace rcss3d_agent

#endif  // RCSS3D_AGENT__SEXP_CREATOR_HPP_
