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

#ifndef RCSS3D_AGENT__ANGLE_CONVERSION_HPP_
#define RCSS3D_AGENT__ANGLE_CONVERSION_HPP_

namespace rcss3d_agent
{

namespace angle_conversion
{

static constexpr double deg2rad(double deg)
{
  return deg * 3.141592654 / 180.0;
}

static constexpr double rad2deg(double rad)
{
  return rad * 180.0 / 3.141592654;
}

}  // namespace angle_conversion

}  // namespace rcss3d_agent

#endif  // RCSS3D_AGENT__ANGLE_CONVERSION_HPP_
