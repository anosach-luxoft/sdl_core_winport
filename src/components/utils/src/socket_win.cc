/*
 * Copyright (c) 2016, Ford Motor Company
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of the Ford Motor Company nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <string>
#include <cstdint>
#include <cstddef>

#include "utils/winhdr.h"
#include "utils/socket.h"
#include "utils/pimpl_impl.h"
#include "utils/logger.h"

CREATE_LOGGERPTR_GLOBAL(logger_ptr, "Utils.TcpSocket")

namespace {

void CloseSocket(SOCKET& socket) {
  LOG4CXX_AUTO_TRACE(logger_ptr);
  if (NULL == socket) {
    LOG4CXX_DEBUG(logger_ptr,
                  "Socket " << socket << " is not valid. Skip closing.");
    return;
  }
  if (SOCKET_ERROR != closesocket(socket)) {
    LOG4CXX_WARN(logger_ptr,
                 "Failed to close socket " << socket << ": "
                                           << WSAGetLastError());
    return;
  }
  socket = NULL;
}

}  // namespace

////////////////////////////////////////////////////////////////////////////////
/// utils::TcpSocketConnection::Impl
////////////////////////////////////////////////////////////////////////////////

class utils::TcpSocketConnection::Impl {
 public:
  friend utils::TcpSocketConnection;

  Impl();
  ~Impl();

  bool Send(const uint8_t* buffer,
            size_t bytes_to_send,
            size_t& bytes_sent);

  void Close();
  bool IsValid() const;

 private:
  SOCKET tcp_socket_;
};

utils::TcpSocketConnection::Impl::Impl() : tcp_socket_(NULL) {}

utils::TcpSocketConnection::Impl::~Impl() {
  Close();
}

bool utils::TcpSocketConnection::Impl::Send(const uint8_t* buffer,
                                            size_t bytes_to_send,
                                            size_t& bytes_sent) {
  LOG4CXX_AUTO_TRACE(logger_ptr);
  bytes_sent = 0;

  if (!IsValid()) {
    LOG4CXX_ERROR(logger_ptr, "Failed to send data socket is not valid");
    return false;
  }
  const int flags = 0;
  int result = send(tcp_socket_,
                    reinterpret_cast<const char*>(buffer),
                    static_cast<int>(bytes_to_send),
                    flags);
  if (SOCKET_ERROR == result) {
    LOG4CXX_ERROR(logger_ptr, "Failed to send data: " << WSAGetLastError());
    return false;
  }
  bytes_sent = static_cast<size_t>(result);
  return true;
}

void utils::TcpSocketConnection::Impl::Close() {
  CloseSocket(tcp_socket_);
}

bool utils::TcpSocketConnection::Impl::IsValid() const {
  return tcp_socket_ != NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// utils::TcpSocketConnection
////////////////////////////////////////////////////////////////////////////////

utils::TcpSocketConnection::TcpSocketConnection() {
}

utils::TcpSocketConnection::TcpSocketConnection(int tcp_socket) {
  impl_->tcp_socket_ = static_cast<SOCKET>(tcp_socket);
}

utils::TcpSocketConnection::~TcpSocketConnection() {
}

bool utils::TcpSocketConnection::Send(const uint8_t* buffer,
                                      size_t bytes_to_send,
                                      size_t& bytes_sent) {
  return impl_->Send(buffer, bytes_to_send, bytes_sent);
}

void utils::TcpSocketConnection::Close() {
  impl_->Close();
}

bool utils::TcpSocketConnection::IsValid() const {
  return impl_->IsValid();
}

////////////////////////////////////////////////////////////////////////////////
/// utils::ServerTcpSocket::Impl
////////////////////////////////////////////////////////////////////////////////

class utils::TcpServerSocket::Impl {
 public:
  Impl();
  ~Impl();

  bool Listen(const std::string& address, uint16_t port, uint16_t backlog);
  bool IsListening() const;

  TcpSocketConnection Accept();

  void Close();

 private:
  SOCKET server_socket_;
  bool   is_listening_;
};

utils::TcpServerSocket::Impl::Impl()
    : server_socket_(NULL), is_listening_(false) {}

utils::TcpServerSocket::Impl::~Impl() {
  Close();
}

bool utils::TcpServerSocket::Impl::IsListening() const {
  return server_socket_ && is_listening_;
}

void utils::TcpServerSocket::Impl::Close() {
  CloseSocket(server_socket_);
}

bool utils::TcpServerSocket::Impl::Listen(const std::string& address,
                                          uint16_t port,
                                          uint16_t backlog) {
  LOG4CXX_AUTO_TRACE(logger_ptr);
  if (IsListening()) {
    LOG4CXX_ERROR(logger_ptr, "Cannot listen. Already listeneing.");
    return false;
  }

  server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
  if (INVALID_SOCKET == server_socket_) {
    LOG4CXX_ERROR(logger_ptr,
                  "Failed to create server socket: " << WSAGetLastError());
    server_socket_ = NULL;
    return false;
  }

  char optval = 1;
  if (SOCKET_ERROR ==
      setsockopt(
          server_socket_, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval))) {
    LOG4CXX_ERROR(logger_ptr, "Unable to set sockopt: " << WSAGetLastError());
    server_socket_ = NULL;
    return false;
  }

  struct sockaddr_in server_address = {0};
  server_address.sin_addr.s_addr = inet_addr(address.c_str());
  server_address.sin_family = AF_INET;
  server_address.sin_port = htons(port);

  if (SOCKET_ERROR == bind(server_socket_,
                           reinterpret_cast<struct sockaddr*>(&server_address),
                           sizeof(server_address))) {
    LOG4CXX_ERROR(logger_ptr, "Unable to bind: " << WSAGetLastError());
    server_socket_ = NULL;
    return false;
  }

  if (SOCKET_ERROR == listen(server_socket_, static_cast<int>(backlog))) {
    LOG4CXX_ERROR(logger_ptr, "Unable to listen: " << WSAGetLastError());
    server_socket_ = NULL;
    return false;
  }

  is_listening_ = true;
  return true;
}

utils::TcpSocketConnection utils::TcpServerSocket::Impl::Accept() {
  LOG4CXX_AUTO_TRACE(logger_ptr);

  struct sockaddr client_addr = {0};
  int client_addr_length = sizeof(client_addr);
  SOCKET client_socket =
      accept(server_socket_, &client_addr, &client_addr_length);
  if (SOCKET_ERROR == client_socket) {
    LOG4CXX_ERROR(logger_ptr,
                  "Failed to accept client socket: " << WSAGetLastError());
    return utils::TcpSocketConnection();
  }
  return TcpSocketConnection(client_socket);
}

////////////////////////////////////////////////////////////////////////////////
/// utils::TcpServerSocket
////////////////////////////////////////////////////////////////////////////////

utils::TcpServerSocket::TcpServerSocket() {
}

utils::TcpServerSocket::~TcpServerSocket() {
}

bool utils::TcpServerSocket::IsListening() const {
  return impl_->IsListening();
}

void utils::TcpServerSocket::Close() {
  impl_->Close();
}

bool utils::TcpServerSocket::Listen(const std::string& address,
                                    uint16_t port,
                                    uint16_t backlog) {
  return impl_->Listen(address, port, backlog);
}

utils::TcpSocketConnection utils::TcpServerSocket::Accept() {
  return impl_->Accept();
}
