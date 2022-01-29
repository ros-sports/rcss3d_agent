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

#include "socket.hpp"

namespace rcss3d_agent
{

Socket::Socket(int _domain, int _type, int _protocol)
: refcnt(new unsigned)
{
  (*refcnt) = 1;

  // This is very, very scary, but it doesn't work without it!!??
  socket_handle = ::socket(_domain, _type, _protocol);
  socket_handle = ::socket(_domain, _type, _protocol);

  if (socket_handle == -1) {
    throw std::runtime_error("error creating the socket");
  }
}

}  // namespace rcss3d_agent
