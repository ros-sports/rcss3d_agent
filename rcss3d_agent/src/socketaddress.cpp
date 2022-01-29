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

#include <netdb.h>
#include <arpa/inet.h>
#include <stdexcept>
#include <iostream>
#include <cstring>
#include <algorithm>
#include <string>

#include "socketaddress.hpp"

namespace rcss3d_agent
{

void SocketAddress::NSLookup(std::string const & _host_name)
{
  struct hostent * h = gethostbyname(_host_name.c_str());

  if (!h) {
    throw std::runtime_error(hstrerror(h_errno));
  }

  memcpy(&(sock_address.i.sin_addr), *(h->h_addr_list), sizeof(struct in_addr));
}

void SocketAddress::copy(SocketAddress const & _other)
{
  memcpy(&sock_address, &(_other.sock_address), sizeof(sock_address));
}

void SocketAddress::destroy()
{
}

SocketAddress::SocketAddress()
{
  sock_address.i.sin_family = AF_INET;
  sock_address.i.sin_port = 0;
}

SocketAddress::SocketAddress(SocketAddress const & _other)
{
  copy(_other);
}

SocketAddress::SocketAddress(int _family, int _port, in_addr_t addr)
{
  sock_address.i.sin_family = _family;
  sock_address.i.sin_port = htons(_port);
  sock_address.i.sin_addr.s_addr = addr;
}

SocketAddress::SocketAddress(int _port, in_addr_t addr)
{
  sock_address.i.sin_family = AF_INET;
  sock_address.i.sin_port = htons(_port);
  sock_address.i.sin_addr.s_addr = addr;
}

SocketAddress::SocketAddress(int _family, int _port, std::string const & _host_name)
{
  sock_address.i.sin_family = _family;
  sock_address.i.sin_port = htons(_port);

  NSLookup(_host_name);
}

SocketAddress::SocketAddress(int _port, std::string const & _host_name)
{
  sock_address.i.sin_family = AF_INET;
  sock_address.i.sin_port = htons(_port);

  NSLookup(_host_name);
}

SocketAddress & SocketAddress::operator=(SocketAddress const & _other)
{
  if (this != &_other) {
    destroy();
    copy(_other);
  }
  return *this;
}

SocketAddress::~SocketAddress()
{
  destroy();
}

std::ostream & operator<<(std::ostream & _os, const SocketAddress & _sa)
{
  _os <<
    inet_ntoa(_sa.sock_address.i.sin_addr);
  _os <<
    ":" << ntohs(_sa.sock_address.i.sin_port);

  return _os;
}

}  // namespace rcss3d_agent
