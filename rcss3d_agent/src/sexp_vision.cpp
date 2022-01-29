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

#include <iostream>
#include <vector>
#include <string>
#include "sexp_vision.hpp"

#define SEXPRESSO_OPT_OUT_PIKESTYLE
#include "sexpresso/sexpresso.hpp"

namespace rcss3d_agent
{
namespace sexp_vision
{

std::optional<rcss3d_agent_msgs::msg::Ball> getBall(sexpresso::Sexp & seeSexp)
{
  // Ball has form: (B (pol <distance> <angle1> <angle2>))
  auto const * ballSexp = seeSexp.getChildByPath("B/pol");
  if (ballSexp != nullptr) {
    rcss3d_agent_msgs::msg::Ball ball;
    ball.center.r = std::stof(ballSexp->value.sexp.at(1).value.str);
    ball.center.theta = std::stof(ballSexp->value.sexp.at(2).value.str);
    ball.center.phi = std::stof(ballSexp->value.sexp.at(3).value.str);
    return std::make_optional(ball);
  }
  return std::nullopt;
}
std::vector<rcss3d_agent_msgs::msg::FieldLine> getFieldLines(sexpresso::Sexp & seeSexp)
{
  std::vector<rcss3d_agent_msgs::msg::FieldLine> fieldLines;
  for (auto const & arg : seeSexp.arguments()) {
    // Lines have form: (L (pol <distance> <angle1> <angle2>) (pol <distance> <angle1> <angle2>))
    auto const & s = arg.value.sexp;
    if (s.at(0).value.str == "L") {
      rcss3d_agent_msgs::msg::FieldLine fieldLine;
      fieldLine.start.r = std::stof(s.at(1).value.sexp.at(1).value.str);
      fieldLine.start.theta = std::stof(s.at(1).value.sexp.at(2).value.str);
      fieldLine.start.phi = std::stof(s.at(1).value.sexp.at(3).value.str);
      fieldLine.end.r = std::stof(s.at(2).value.sexp.at(1).value.str);
      fieldLine.end.theta = std::stof(s.at(2).value.sexp.at(2).value.str);
      fieldLine.end.phi = std::stof(s.at(2).value.sexp.at(3).value.str);
      fieldLines.push_back(fieldLine);
    }
  }
  return fieldLines;
}
std::vector<rcss3d_agent_msgs::msg::Flag> getFlags(sexpresso::Sexp & seeSexp)
{
  std::vector<rcss3d_agent_msgs::msg::Flag> flags;
  for (std::string & flagName :
    std::vector<std::string>{"F1L", "F1R", "F2L", "F2R"})
  {
    // Flags have form: (<name> (pol <distance> <angle1> <angle2>))
    auto const * flagSexp = seeSexp.getChildByPath(flagName + "/pol");
    if (flagSexp != nullptr) {
      rcss3d_agent_msgs::msg::Flag flag;
      flag.name = flagName;
      flag.base.r = std::stof(flagSexp->value.sexp.at(1).value.str);
      flag.base.theta = std::stof(flagSexp->value.sexp.at(2).value.str);
      flag.base.phi = std::stof(flagSexp->value.sexp.at(3).value.str);
      flags.push_back(flag);
    }
  }

  return flags;
}
std::vector<rcss3d_agent_msgs::msg::Goalpost> getGoalposts(sexpresso::Sexp & seeSexp)
{
  std::vector<rcss3d_agent_msgs::msg::Goalpost> goalposts;
  for (std::string & postName :
    std::vector<std::string>{"G1L", "G1R", "G2L", "G2R"})
  {
    // Goal posts have form: (<name> (pol <distance> <angle1> <angle2>))
    auto const * postSexp = seeSexp.getChildByPath(postName + "/pol");
    if (postSexp != nullptr) {
      rcss3d_agent_msgs::msg::Goalpost post;
      post.name = postName;
      post.top.r = std::stof(postSexp->value.sexp.at(1).value.str);
      post.top.theta = std::stof(postSexp->value.sexp.at(2).value.str);
      post.top.phi = std::stof(postSexp->value.sexp.at(3).value.str);
      goalposts.push_back(post);
    }
  }

  return goalposts;
}
std::vector<rcss3d_agent_msgs::msg::Player> getPlayers(sexpresso::Sexp & seeSexp)
{
  std::vector<rcss3d_agent_msgs::msg::Player> players;

  for (auto & arg : seeSexp.arguments()) {
    // Players have form:
    // (P (team <teamname>) (id <playerID>) +(<bodypart> (pol <distance> <angle1> <angle2>)))
    auto const & s = arg.value.sexp;
    if (s.at(0).value.str == "P") {
      rcss3d_agent_msgs::msg::Player player;
      player.team = s.at(1).value.sexp.at(1).value.str;
      player.id = std::stof(s.at(2).value.sexp.at(1).value.str);

      for (unsigned i = 3; i < s.size(); ++i) {
        auto bodyPartPolSexp = s.at(i).value.sexp.at(1).value.sexp;
        rcss3d_agent_msgs::msg::Spherical spherical;
        spherical.r = std::stof(bodyPartPolSexp.at(1).value.str);
        spherical.theta = std::stof(bodyPartPolSexp.at(2).value.str);
        spherical.phi = std::stof(bodyPartPolSexp.at(3).value.str);

        std::string bodyPart = s.at(i).value.sexp.at(0).value.str;
        if (bodyPart == "head") {
          player.head.push_back(spherical);
        } else if (bodyPart == "rlowerarm") {
          player.rlowerarm.push_back(spherical);
        } else if (bodyPart == "llowerarm") {
          player.llowerarm.push_back(spherical);
        } else if (bodyPart == "rfoot") {
          player.rfoot.push_back(spherical);
        } else if (bodyPart == "lfoot") {
          player.lfoot.push_back(spherical);
        }
      }

      players.push_back(player);
    }
  }
  return players;
}


}  // namespace sexp_vision
}  // namespace rcss3d_agent
