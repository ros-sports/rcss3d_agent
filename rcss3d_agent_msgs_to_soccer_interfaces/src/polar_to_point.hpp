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

#ifndef POLAR_TO_POINT_HPP_
#define POLAR_TO_POINT_HPP_

#include <cmath>
#include "geometry_msgs/msg/point.hpp"

namespace rcss3d_agent_msgs_to_soccer_interfaces
{

geometry_msgs::msg::Point polar_to_point(
  float distance_metres,
  float horizontal_angle_phi_radians,
  float vertical_angle_theta_radians)
{
  geometry_msgs::msg::Point point;

  point.x = distance_metres * cos(horizontal_angle_phi_radians) * cos(vertical_angle_theta_radians);
  point.y = distance_metres * sin(horizontal_angle_phi_radians) * cos(vertical_angle_theta_radians);
  point.z = distance_metres * sin(vertical_angle_theta_radians);

  return point;
}

}  // namespace rcss3d_agent_msgs_to_soccer_interfaces

#endif  // POLAR_TO_POINT_HPP_
