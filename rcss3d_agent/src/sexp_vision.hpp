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

#ifndef SEXP_VISION_HPP_
#define SEXP_VISION_HPP_

#include <optional>
#include <vector>
#include "rcss3d_agent_msgs/msg/ball.hpp"
#include "rcss3d_agent_msgs/msg/field_line.hpp"
#include "rcss3d_agent_msgs/msg/flag.hpp"
#include "rcss3d_agent_msgs/msg/goalpost.hpp"
#include "rcss3d_agent_msgs/msg/player.hpp"

namespace sexpresso {class Sexp;}

namespace rcss3d_agent
{
namespace sexp_vision
{

std::optional<rcss3d_agent_msgs::msg::Ball> getBall(sexpresso::Sexp & seeSexp);
std::vector<rcss3d_agent_msgs::msg::FieldLine> getFieldLines(sexpresso::Sexp & seeSexp);
std::vector<rcss3d_agent_msgs::msg::Flag> getFlags(sexpresso::Sexp & seeSexp);
std::vector<rcss3d_agent_msgs::msg::Goalpost> getGoalposts(sexpresso::Sexp & seeSexp);
std::vector<rcss3d_agent_msgs::msg::Player> getPlayers(sexpresso::Sexp & seeSexp);

}  // namespace sexp_vision
}  // namespace rcss3d_agent

#endif  // SEXP_VISION_HPP_
