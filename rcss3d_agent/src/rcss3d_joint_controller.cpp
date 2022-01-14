// Copyright 2019 Bold Hearts
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

#include "rcss3d_agent/rcss3d_joint_controller.hpp"

#include <memory>
#include <string>
#include <utility>

namespace rcss3d_agent
{

Rcss3DJointController::Rcss3DJointController(const rclcpp::NodeOptions & options)
: rclcpp::Node{"rcss3d_joint_controller", options}
{
  joint_command_pub_ = create_publisher<JointCommand>("/joint_commands", 1);

  joint_state_sub_ =
    create_subscription<JointState>(
    "/joint_states", 1,
    [this](JointState::UniquePtr joint_states) {
      // Do nothing if we haven't received joint targets
      if (joint_positions_ == nullptr) {
        RCLCPP_DEBUG(get_logger(), "Got no positions, skipping");
        return;
      }

      auto joint_commands = std::make_unique<JointCommand>();
      joint_commands->name = joint_positions_->name;
      for (auto i = 0u; i < joint_positions_->name.size(); ++i) {
        auto joint_effector_name = joint_positions_->name[i];
        // Effectors have names like he1 and lae1, whereas corresponding joint is hj1 and laj1
        auto joint_name = joint_effector_name;
        joint_name.replace(joint_name.end() - 2, joint_name.end() - 1, "j");

        RCLCPP_DEBUG(get_logger(), "effector name: %s", joint_effector_name.c_str());
        RCLCPP_DEBUG(get_logger(), "joint name: %s", joint_name.c_str());

        auto joint_position_target = joint_positions_->position[i];
        RCLCPP_DEBUG(get_logger(), "position target: %f", joint_position_target);

        auto it = std::find(joint_states->name.begin(), joint_states->name.end(), joint_name);

        if (it == joint_states->name.end()) {
          RCLCPP_ERROR(
            get_logger(), "Received an invalid effector name: %s",
            joint_effector_name.c_str());
          continue;
        }

        auto joint_state_idx = it - joint_states->name.begin();

        RCLCPP_DEBUG(get_logger(), "joint state index: %ld", joint_state_idx);

        auto joint_position_current = joint_states->position[joint_state_idx];

        RCLCPP_DEBUG(get_logger(), "current position: %f", joint_position_current);

        // Normalize position target to prevent very large speeds if targets with multiple
        // rotations are given
        joint_position_target = normalize_angle(joint_position_target);
        auto error = joint_position_target - joint_position_current;

        // TODO(sander): This is only P control add I and D gains as well
        auto speed_target = joint_positions_->p_gain[i] * error;
        RCLCPP_DEBUG(get_logger(), "speed target: %f", speed_target);
        joint_commands->speed.push_back(speed_target);
      }

      joint_command_pub_->publish(std::move(joint_commands));
    });

  joint_position_sub_ =
    create_subscription<JointPositionCommand>(
    "/joint_positions", 1,
    [this](JointPositionCommand::UniquePtr msg) {
      RCLCPP_DEBUG(get_logger(), "Got positions");
      joint_positions_ = std::move(msg);
    });
}

}  // namespace rcss3d_agent
