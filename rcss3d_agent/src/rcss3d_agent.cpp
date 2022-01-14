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

#include "rcss3d_agent/rcss3d_agent.hpp"

#include <netinet/tcp.h>
#include <cmath>
#include <string>
#include <utility>
#include <algorithm>
#include <memory>

#include "rcss3d_agent/sexp_creator.hpp"
#include "rcss3d_agent/sexp_parser.hpp"
#include "rcss3d_agent/angle_conversion.hpp"

using namespace std::chrono_literals;

namespace rcss3d_agent
{

Rcss3dAgent::Rcss3dAgent(const rclcpp::NodeOptions & options)
: rclcpp::Node{"rcss3d_agent", options}
{
  // Declare parameters
  RCLCPP_DEBUG(get_logger(), "Declare parameters");
  std::string rcss3d_host = this->declare_parameter<std::string>("rcss3d/host", "127.0.0.1");
  int rcss3d_port = this->declare_parameter<int>("rcss3d/port", 3100);
  std::string team = this->declare_parameter<std::string>("team", "Anonymous");
  int unum = this->declare_parameter<int>("unum", 0);
  std::string imu_frame = this->declare_parameter<std::string>("imu_frame", "base_link");
  std::string camera_frame = this->declare_parameter<std::string>("camera_frame", "camera");

  // Log parameters for debugging
  logParametersToRclcppDebug(
    rcss3d_host, rcss3d_port, team, unum, imu_frame, camera_frame);

  // Publishers
  clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  ball_pub_ = create_publisher<rcss3d_agent_msgs::msg::Ball>("vision/ball", 10);
  posts_pub_ = create_publisher<rcss3d_agent_msgs::msg::GoalpostArray>("vision/goalposts", 10);
  lines_pub_ =
    create_publisher<rcss3d_agent_msgs::msg::FieldLineArray>("vision/field_lines", 10);
  robots_pub_ = create_publisher<rcss3d_agent_msgs::msg::RobotArray>("vision/robots", 10);
  flags_pub_ = create_publisher<rcss3d_agent_msgs::msg::FlagArray>("vision/flags", 10);

  // Subscriptions
  joint_command_sub_ =
    create_subscription<rcss3d_agent_msgs::msg::JointCommand>(
    "/joint_commands",
    10,
    [this](rcss3d_agent_msgs::msg::JointCommand::SharedPtr cmd) {
      RCLCPP_DEBUG(get_logger(), "Got joint commands, for effectors:");
      for (auto const & n : cmd->name) {
        RCLCPP_DEBUG(get_logger(), n.c_str());
      }

      std::string msg = sexp_creator::createJointMessage(cmd->name, cmd->speed);
      RCLCPP_DEBUG(this->get_logger(), ("Sending: " + msg).c_str());
      connection.send(msg);
    });

  beam_sub_ =
    create_subscription<rcss3d_agent_msgs::msg::Beam>(
    "/beam", 10,
    [this](rcss3d_agent_msgs::msg::Beam::SharedPtr cmd) {
      RCLCPP_DEBUG(get_logger(), "Got beam msg to: %f, %f, %f", cmd->x, cmd->y, cmd->theta);
      connection.send(sexp_creator::createBeamMessage(cmd->x, cmd->y, cmd->theta));
    });

  // Initialise connection
  connection.initialise(rcss3d_host, rcss3d_port);

  // Create the robot
  connection.send(sexp_creator::createCreateMessage());

  // Receive, this is needed for the init message to be sent next
  connection.receive();

  // Send init
  connection.send(sexp_creator::createInitMessage(team, unum));

  // Start receive and send loop
  receive_thread_ = std::thread(
    [this](std::string imu_frame, std::string camera_frame) {
      while (rclcpp::ok()) {
        std::string recv = connection.receive();
        RCLCPP_DEBUG(this->get_logger(), ("Received: " + recv).c_str());
        handle(recv, imu_frame, camera_frame);
      }
    }, imu_frame, camera_frame);
}

Rcss3dAgent::~Rcss3dAgent()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
}

void Rcss3dAgent::handle(
  std::string const & msg, std::string const & imu_frame,
  std::string const & camera_frame)
{
  SexpParser parsed(msg);

  // Clock
  auto clock = parsed.getClock();
  clock_pub_->publish(clock);

  // IMU
  if (auto imu = parsed.getImu(); imu.has_value()) {
    RCLCPP_DEBUG(get_logger(), "Publishing IMU");
    auto imuVal = imu.value();
    imuVal.header.frame_id = imu_frame;
    imu_pub_->publish(std::make_unique<sensor_msgs::msg::Imu>(imuVal));
  }

  // Joints
  auto joint_states = parsed.getJoints();
  joint_state_pub_->publish(std::make_unique<sensor_msgs::msg::JointState>(joint_states));

  // Ball
  if (auto ball = parsed.getBall(); ball.has_value()) {
    auto ballVal = ball.value();
    ballVal.header.frame_id = camera_frame;
    ball_pub_->publish(ballVal);
  }

  // Posts
  if (auto posts = parsed.getGoalposts(); posts.has_value()) {
    auto postsVal = posts.value();
    for (auto & post : postsVal.posts) {
      post.header.frame_id = camera_frame;
    }
    posts_pub_->publish(postsVal);
  }

  // Lines
  if (auto lines = parsed.getFieldLines(); lines.has_value()) {
    auto linesVal = lines.value();
    for (auto & line : linesVal.lines) {
      line.header.frame_id = camera_frame;
    }
    lines_pub_->publish(linesVal);
  }

  // Robots
  if (auto robots = parsed.getRobots(); robots.has_value()) {
    auto robotsVal = robots.value();
    for (auto & robot : robotsVal.robots) {
      robot.header.frame_id = camera_frame;
    }
    robots_pub_->publish(robotsVal);
  }

  // Flags
  if (auto flags = parsed.getFlags(); flags.has_value()) {
    auto flagsVal = flags.value();
    for (auto & flag : flagsVal.flags) {
      flag.header.frame_id = camera_frame;
    }
    flags_pub_->publish(flagsVal);
  }
}

void Rcss3dAgent::logParametersToRclcppDebug(
  std::string rcss3d_host, int rcss3d_port, std::string team, int unum, std::string imu_frame,
  std::string camera_frame)
{
  RCLCPP_DEBUG(get_logger(), "Parameters: ");
  RCLCPP_DEBUG(get_logger(), "  rcss3d/host: %s", rcss3d_host.c_str());
  RCLCPP_DEBUG(get_logger(), "  rcss3d/port: %d", rcss3d_port);
  RCLCPP_DEBUG(get_logger(), "  team: %s", team.c_str());
  RCLCPP_DEBUG(get_logger(), "  unum: %d", unum);
  RCLCPP_DEBUG(get_logger(), "  imu_frame: %s", imu_frame.c_str());
  RCLCPP_DEBUG(get_logger(), "  camera_frame: %s", camera_frame.c_str());
}

}  // namespace rcss3d_agent
