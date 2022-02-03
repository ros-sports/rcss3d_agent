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
#include <utility>
#include <vector>

#include "sexp_creator.hpp"

#define SEXPRESSO_OPT_OUT_PIKESTYLE
#include "sexpresso/sexpresso.hpp"

namespace rcss3d_agent
{

namespace sexp_creator
{

std::string createMessage(sexpresso::Sexp sexp, bool wrap = true)
{
  auto root = sexpresso::Sexp{};
  if (wrap) {
    root.addChild(std::move(sexp));
  } else {
    root = std::move(sexp);
  }

  auto msg = root.toString();
  return msg;
}

std::string createCreateMessage(const std::string & model)
{
  auto sceneSexp = sexpresso::Sexp{"scene"};
  sceneSexp.addChild(model);
  return createMessage(sceneSexp);
}

std::string createInitMessage(std::string const & team_name, int number)
{
  auto initSexp = sexpresso::Sexp{"init"};

  auto unumSexp = sexpresso::Sexp{"unum"};
  unumSexp.addChild(std::to_string(number));
  initSexp.addChild(std::move(unumSexp));

  auto teamSexp = sexpresso::Sexp{"teamname"};
  teamSexp.addChild(team_name);
  initSexp.addChild(std::move(teamSexp));

  return createMessage(initSexp);
}

std::string createJointMessage(std::vector<std::string> names, std::vector<float> speeds)
{
  // We assume the two vectors are of same length.
  auto sexp = sexpresso::Sexp{};
  for (auto i = 0u; i < names.size(); ++i) {
    auto jointSexp = sexpresso::Sexp{names[i]};
    jointSexp.addChild(std::to_string(speeds[i]));
    sexp.addChild(std::move(jointSexp));
  }
  return createMessage(sexp, false);
}

std::string createBeamMessage(const rcss3d_agent_msgs::msg::Beam & b)
{
  auto sexp = sexpresso::Sexp{"beam"};
  sexp.addChild(std::to_string(b.x));
  sexp.addChild(std::to_string(b.y));
  sexp.addChild(std::to_string(b.rot));
  return createMessage(sexp);
}

std::string createSayMessage(const rcss3d_agent_msgs::msg::Say & s)
{
  auto sexp = sexpresso::Sexp{"say"};
  sexp.addChild(s.message);
  return createMessage(sexp);
}

std::string createHingeJointVelMessage(const rcss3d_agent_msgs::msg::HingeJointVel & j)
{
  auto sexp = sexpresso::Sexp{j.name};
  sexp.addChild(std::to_string(j.ax));
  return createMessage(sexp);
}

std::string createUniversalJointVelMessage(const rcss3d_agent_msgs::msg::UniversalJointVel & j)
{
  auto sexp = sexpresso::Sexp{j.name};
  sexp.addChild(std::to_string(j.ax1));
  sexp.addChild(std::to_string(j.ax2));
  return createMessage(sexp);
}

std::string createSynchronizeMessage()
{
  return "(syn)";
}

}  // namespace sexp_creator

}  // namespace rcss3d_agent
