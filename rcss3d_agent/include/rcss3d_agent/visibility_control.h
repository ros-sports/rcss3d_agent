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

#ifndef RCSS3D_AGENT__VISIBILITY_CONTROL_H_
#define RCSS3D_AGENT__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RCSS3D_AGENT_EXPORT __attribute__ ((dllexport))
    #define RCSS3D_AGENT_IMPORT __attribute__ ((dllimport))
  #else
    #define RCSS3D_AGENT_EXPORT __declspec(dllexport)
    #define RCSS3D_AGENT_IMPORT __declspec(dllimport)
  #endif
  #ifdef RCSS3D_AGENT_BUILDING_LIBRARY
    #define RCSS3D_AGENT_PUBLIC RCSS3D_AGENT_EXPORT
  #else
    #define RCSS3D_AGENT_PUBLIC RCSS3D_AGENT_IMPORT
  #endif
  #define RCSS3D_AGENT_PUBLIC_TYPE RCSS3D_AGENT_PUBLIC
  #define RCSS3D_AGENT_LOCAL
#else
  #define RCSS3D_AGENT_EXPORT __attribute__ ((visibility("default")))
  #define RCSS3D_AGENT_IMPORT
  #if __GNUC__ >= 4
    #define RCSS3D_AGENT_PUBLIC __attribute__ ((visibility("default")))
    #define RCSS3D_AGENT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RCSS3D_AGENT_PUBLIC
    #define RCSS3D_AGENT_LOCAL
  #endif
  #define RCSS3D_AGENT_PUBLIC_TYPE
#endif

#endif  // RCSS3D_AGENT__VISIBILITY_CONTROL_H_
