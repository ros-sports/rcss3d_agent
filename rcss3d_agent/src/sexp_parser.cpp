// Copyright 2019 Bold Hearts
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
#include "rcss3d_agent/angle_conversion.hpp"

namespace rcss3d_agent
{

SexpParser::SexpParser(std::string msg)
: sexp(sexpresso::parse(msg)),
  logger(rclcpp::get_logger("sexp_parser"))
{
  // Clock is used across many of the functions, so we parse it at the start and cache it.
  auto const * time_sexp = sexp.getChildByPath("time/now");
  if (time_sexp != nullptr) {
    auto rcss3d_now = std::stod(time_sexp->value.sexp[1].value.str);
    clock.clock.sec = std::floor(rcss3d_now);
    clock.clock.nanosec = (rcss3d_now - clock.clock.sec) * 1e9;
  } else {
    RCLCPP_WARN(
      logger, "Couldn't find time information in message. "
      "Time in the headers of all the getters will be incorrect.");
  }
}


rosgraph_msgs::msg::Clock SexpParser::getClock()
{
  return clock;
}

std::optional<sensor_msgs::msg::Imu> SexpParser::getImu()
{
  auto imuMsg = sensor_msgs::msg::Imu{};

  auto const * gyrSexp = sexp.getChildByPath("GYR/rt");
  auto const * accSexp = sexp.getChildByPath("ACC/a");

  if (gyrSexp != nullptr && accSexp != nullptr) {
    imuMsg.header.stamp = clock.clock;

    auto const & rateSexp = gyrSexp->value.sexp;
    imuMsg.angular_velocity.x = angle_conversion::deg2rad(std::stod(rateSexp[2].value.str));
    imuMsg.angular_velocity.y = -angle_conversion::deg2rad(std::stod(rateSexp[1].value.str));
    imuMsg.angular_velocity.z = angle_conversion::deg2rad(std::stod(rateSexp[3].value.str));

    auto const & aSexp = accSexp->value.sexp;
    imuMsg.linear_acceleration.x = std::stod(aSexp[2].value.str);
    imuMsg.linear_acceleration.y = -std::stod(aSexp[1].value.str);
    imuMsg.linear_acceleration.z = std::stod(aSexp[3].value.str);
    return imuMsg;
  }
  return {};
}

sensor_msgs::msg::JointState SexpParser::getJoints()
{
  auto joint_states = sensor_msgs::msg::JointState{};
  joint_states.header.stamp = clock.clock;
  for (auto const & arg : sexp.arguments()) {
    // Joint expressions have form: (HJ (n llj2) (ax -0.00))
    auto const & s = arg.value.sexp;
    if (s[0].value.str == "HJ") {
      joint_states.name.push_back(s[1].value.sexp[1].value.str);
      joint_states.position.push_back(
        angle_conversion::deg2rad(
          std::stod(
            s[2].value.sexp[1].value.
            str)));
    }
  }
  return joint_states;
}


// Eg. (See (B (pol 8.51 -0.21 -0.17)))
std::optional<rcss3d_agent_msgs::msg::Ball> SexpParser::getBall()
{
  rcss3d_agent_msgs::msg::Ball ball;

  auto const * ballSexp = sexp.getChildByPath("See/B/pol");
  if (ballSexp != nullptr) {
    RCLCPP_DEBUG(logger, "Found ball information");

    ball.header.stamp = clock.clock;
    ball.center.r = std::stof(ballSexp->value.sexp.at(1).value.str);
    ball.center.theta = angle_conversion::deg2rad(std::stof(ballSexp->value.sexp.at(2).value.str));
    ball.center.phi = angle_conversion::deg2rad(std::stof(ballSexp->value.sexp.at(3).value.str));
    return ball;
  }
  return {};
}


// Eg. (See (G2R (pol 17.55 -3.33 4.31))
//          (G1R (pol 17.52 3.27 4.07)))
std::optional<rcss3d_agent_msgs::msg::GoalpostArray> SexpParser::getGoalposts()
{
  rcss3d_agent_msgs::msg::GoalpostArray goalpostArray;

  for (std::string & postName :
    std::vector<std::string>{"G1L", "G1R", "G2L", "G2R"})
  {
    auto const * postSexp = sexp.getChildByPath("See/" + postName + "/pol");
    if (postSexp != nullptr) {
      RCLCPP_DEBUG(logger, "Found post information");

      rcss3d_agent_msgs::msg::Goalpost post;
      post.header.stamp = clock.clock;
      post.top.r = std::stof(postSexp->value.sexp.at(1).value.str);
      post.top.theta = angle_conversion::deg2rad(std::stof(postSexp->value.sexp.at(2).value.str));
      post.top.phi = angle_conversion::deg2rad(std::stof(postSexp->value.sexp.at(3).value.str));

      goalpostArray.posts.push_back(post);
    }
  }
  return goalpostArray.posts.size() > 0 ? std::make_optional(goalpostArray) : std::nullopt;
}

// Eg. (See (L (pol 12.11 -40.77 -2.40) (pol 12.95 -37.76 -2.41))
//          (L (pol 12.97 -37.56 -2.24) (pol 13.32 -32.98 -2.20)))
std::optional<rcss3d_agent_msgs::msg::FieldLineArray> SexpParser::getFieldLines()
{
  rcss3d_agent_msgs::msg::FieldLineArray fieldLineArray;

  auto const * seeSexp = sexp.getChildByPath("See");
  if (seeSexp != nullptr) {
    for (auto const & arg : sexp.getChildByPath("See")->arguments()) {
      auto const & s = arg.value.sexp;
      if (s.at(0).value.str == "L") {
        rcss3d_agent_msgs::msg::FieldLine line;
        line.header.stamp = clock.clock;

        line.start.r = std::stof(s.at(1).value.sexp.at(1).value.str);
        line.start.theta = angle_conversion::deg2rad(std::stof(s.at(1).value.sexp.at(2).value.str));
        line.start.phi = angle_conversion::deg2rad(std::stof(s.at(1).value.sexp.at(3).value.str));

        line.end.r = std::stof(s.at(2).value.sexp.at(1).value.str);
        line.end.theta = angle_conversion::deg2rad(std::stof(s.at(2).value.sexp.at(2).value.str));
        line.end.phi = angle_conversion::deg2rad(std::stof(s.at(2).value.sexp.at(3).value.str));

        fieldLineArray.lines.push_back(line);
      }
    }
  }
  return fieldLineArray.lines.size() > 0 ? std::make_optional(fieldLineArray) : std::nullopt;
}

// Eg. (See (P (team teamRed) (id 1)
//             (head (pol 16.98 -0.21 3.19))
//             (rlowerarm (pol 16.83 -0.06 2.80))
//             (llowerarm (pol 16.86 -0.36 3.10))
//             (rfoot (pol 17.00 0.29 1.68))
//             (lfoot (pol 16.95 -0.51 1.32))))
std::optional<rcss3d_agent_msgs::msg::RobotArray> SexpParser::getRobots()
{
  rcss3d_agent_msgs::msg::RobotArray robotArray;

  auto * seeSexp = sexp.getChildByPath("See");
  if (seeSexp != nullptr) {
    for (auto & arg : seeSexp->arguments()) {
      // Ignore if its not player info
      if (arg.value.sexp.at(0).value.str != "P") {
        continue;
      }

      // Ignore if we don't have head info
      auto * player_head = arg.getChildByPath("head");
      if (player_head == nullptr) {
        continue;
      }

      rcss3d_agent_msgs::msg::Robot robot;
      robot.header.stamp = clock.clock;
      robot.team = arg.getChildByPath("team")->value.sexp.at(1).value.str;
      robot.id = std::stoi(arg.getChildByPath("id")->value.sexp.at(1).value.str);

      auto & pol = arg.getChildByPath("head/pol")->value.sexp;
      robot.head.r = std::stof(pol.at(1).value.str);
      robot.head.theta = angle_conversion::deg2rad(std::stof(pol.at(2).value.str));
      robot.head.phi = angle_conversion::deg2rad(std::stof(pol.at(3).value.str));

      robotArray.robots.push_back(robot);
    }
  }
  return robotArray.robots.size() > 0 ? std::make_optional(robotArray) : std::nullopt;
}

// Eg. (See (F1R (pol 18.52 18.94 1.54))
//          (F2R (pol 18.52 -18.91 1.52))
std::optional<rcss3d_agent_msgs::msg::FlagArray> SexpParser::getFlags()
{
  rcss3d_agent_msgs::msg::FlagArray flagArray;

  for (std::string & flagName :
    std::vector<std::string>{"F1L", "F1R", "F2L", "F2R"})
  {
    auto const * flagSexp = sexp.getChildByPath("See/" + flagName + "/pol");
    if (flagSexp != nullptr) {
      RCLCPP_DEBUG(logger, "Found flag information");

      rcss3d_agent_msgs::msg::Flag flag;
      flag.header.stamp = clock.clock;
      flag.base.r = std::stof(flagSexp->value.sexp.at(1).value.str);
      flag.base.theta = angle_conversion::deg2rad(std::stof(flagSexp->value.sexp.at(2).value.str));
      flag.base.phi = angle_conversion::deg2rad(std::stof(flagSexp->value.sexp.at(3).value.str));

      flagArray.flags.push_back(flag);
    }
  }
  return flagArray.flags.size() > 0 ? std::make_optional(flagArray) : std::nullopt;
}


}  // namespace rcss3d_agent
