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

#ifndef SOCKET_HPP_
#define SOCKET_HPP_

#include <sys/types.h>
#include <sys/socket.h>
#include <fcntl.h>

extern "C"
{
#include <unistd.h>
}

#include <cstring>
#include <cerrno>
#include <stdexcept>
#include <cassert>

#include <iosfwd>

#include "socketaddress.hpp"

namespace rcss3d_agent
{

/**
 *  A C++ wrapper for a socket.
 */
class Socket
{
public:
  Socket()
  : socket_handle(-1), refcnt(new unsigned)
  {
    (*refcnt) = 1;
  }

  explicit Socket(int _socket_handle)
  : socket_handle(_socket_handle), refcnt(new unsigned)
  {
    (*refcnt) = 1;
  }

  Socket(int _domain, int _type, int _protocol);

  operator bool() const
  {
    return socket_handle != -1;
  }

  ~Socket()
  {
    destroy();
  }

  void setsockopt(int level, int optname, int value)
  {
    assert(socket_handle);
    if (::setsockopt(socket_handle, level, optname, &value, sizeof(int)) == -1) {
      throw std::runtime_error(strerror(errno));
    }
  }

  void setsockopt(int optname, int value)
  {
    setsockopt(SOL_SOCKET, optname, value);
  }

  void setsockopt(int optname, bool value)
  {
    setsockopt(optname, value ? 1 : 0);
  }

  operator int() const
  {
    return socket_handle;
  }

  void connect(SocketAddress & _server_address)
  {
    assert(socket_handle);
    if (::connect(socket_handle, _server_address, _server_address.getLength()) == -1) {
      throw std::runtime_error(strerror(errno));
    }
  }

  bool tryConnect(SocketAddress & _server_address)
  {
    assert(socket_handle);
    return ::connect(socket_handle, _server_address, _server_address.getLength()) != -1;
  }

  void bind(SocketAddress & _my_address)
  {
    assert(socket_handle);
    if (::bind(socket_handle, _my_address, _my_address.getLength()) == -1) {
      throw std::runtime_error(strerror(errno));
    }
  }

  void listen(int _backlog)
  {
    assert(socket_handle);
    if (::listen(socket_handle, _backlog) == -1) {
      throw std::runtime_error(strerror(errno));
    }
  }

  Socket accept(SocketAddress & _address)
  {
    assert(socket_handle);
    socklen_t len = _address.getLength();
    int sock = ::accept(socket_handle, _address, &len);
    if (sock == -1) {
      throw std::runtime_error(strerror(errno));
    }
    return Socket{sock};
  }

  void close()
  {
    ::close(socket_handle);
    socket_handle = -1;
  }

  void shutdown(int how)
  {
    assert(socket_handle);
    ::shutdown(socket_handle, how);
  }

  void shutdownRead()
  {
    shutdown(SHUT_RD);
  }

  void shutdownWrite()
  {
    shutdown(SHUT_WR);
  }

  unsigned read(char * _buf, unsigned _size)
  {
    assert(socket_handle);
    int res = ::read(socket_handle, _buf, _size);
    if (res == -1) {
      if (errno != EAGAIN) {
        throw std::runtime_error(strerror(errno));
      }
      res = 0;
    }
    return static_cast<unsigned>(res);
  }

  unsigned readExactly(char * _buf, unsigned _size)
  {
    unsigned total = 0;
    for (unsigned i = 0; i < 100; ++i) {  // Allow 100 retries
      if (total == _size) {
        break;
      } else {
        total += read(_buf + total, _size - total);
      }
    }
    return total;
  }

  unsigned write(char const * _buf, unsigned _size)
  {
    assert(socket_handle);
    int res = ::write(socket_handle, _buf, _size);
    if (res == -1 && errno != EAGAIN) {
      throw std::runtime_error(strerror(errno));
    }
    return static_cast<unsigned>(res);
  }

  unsigned writeExactly(char const * _buf, unsigned _size)
  {
    unsigned total;
    for (total = 0;
      total < _size;
      total += write(_buf + total, _size - total)) {}
    return total;
  }


  int fcntl(int cmd)
  {
    return ::fcntl(socket_handle, cmd);
  }

  int fcntl(int cmd, int64_t arg)
  {
    return ::fcntl(socket_handle, cmd, arg);
  }

  int fcntl(int cmd, struct flock * lock)
  {
    return ::fcntl(socket_handle, cmd, lock);
  }

  int setBlocking(bool blocking)
  {
    int flags = fcntl(F_GETFL);

    if (blocking && !(flags & O_NONBLOCK)) {
      return fcntl(F_SETFL, flags & ~O_NONBLOCK);
    } else {
      return fcntl(F_SETFL, flags | O_NONBLOCK);
    }
  }

  bool getBlocking()
  {
    int flags = fcntl(F_GETFL);
    return !(flags & O_NONBLOCK);
  }

private:
  Socket(Socket const & _other);  // NI

  Socket & operator=(Socket const & _other);  // NI

  int socket_handle;

  unsigned * refcnt;

  void destroy()
  {
    ::shutdown(socket_handle, SHUT_RDWR);
    ::close(socket_handle);
    delete refcnt;
  }
};

}  // namespace rcss3d_agent

#endif  // SOCKET_HPP_
