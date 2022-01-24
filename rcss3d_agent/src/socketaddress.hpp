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

#ifndef SOCKETADDRESS_HPP_
#define SOCKETADDRESS_HPP_

#include <sys/socket.h>
#include <netinet/in.h>
#include <iosfwd>
#include <string>
#include <algorithm>

namespace rcss3d_agent
{

/**
 *  A C++ wrapper for socketaddress
 */
class SocketAddress
{
  friend std::ostream & operator<<(std::ostream &, const SocketAddress &);

  union sock {
    struct sockaddr s;
    struct sockaddr_in i;
  };

  sock sock_address;

  void NSLookup(std::string const & _host_name);

  void copy(SocketAddress const & _other);
  void destroy();

public:
  SocketAddress();

  explicit SocketAddress(SocketAddress const & _other);

  SocketAddress(int _family, int _port, in_addr_t addr = INADDR_ANY);

  explicit SocketAddress(int _port, in_addr_t = INADDR_ANY);

  SocketAddress(int _family, int _port, std::string const & _host_name);

  SocketAddress(int _port, std::string const & _host_name);

  SocketAddress & operator=(SocketAddress const & _other);

  ~SocketAddress();

  int getFamily() const
  {
    return sock_address.i.sin_family;
  }

  int getPort() const
  {
    return ntohs(sock_address.i.sin_port);
  }

  operator sockaddr *()
  {
    return &(sock_address.s);
  }

  operator sockaddr_in *()
  {
    return &(sock_address.i);
  }

  unsigned getLength() const
  {
    return sizeof(sock_address);
  }
};

}  // namespace rcss3d_agent

#endif  // SOCKETADDRESS_HPP_
