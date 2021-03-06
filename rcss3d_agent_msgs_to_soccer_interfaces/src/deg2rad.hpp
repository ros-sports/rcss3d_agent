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

#ifndef DEG2RAD_HPP_
#define DEG2RAD_HPP_

namespace rcss3d_agent_msgs_to_soccer_interfaces
{

static constexpr double deg2rad(double deg)
{
  return deg * 3.141592654 / 180.0;
}


}  // namespace rcss3d_agent_msgs_to_soccer_interfaces

#endif  // DEG2RAD_HPP_
