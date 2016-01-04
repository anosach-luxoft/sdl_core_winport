/*
 * Copyright (c) 2015, Ford Motor Company
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
#include <vector>
#include <cstdint>
#include <cstddef>

#include "utils/winhdr.h"
#include "utils/pipe.h"
#include "utils/logger.h"

CREATE_LOGGERPTR_GLOBAL(logger_ptr, "Utils.Pipe")

namespace utils {

class Pipe::Impl {
 public:
  friend Pipe;  

  Impl();
  ~Impl();

  bool Create();
  void Delete();
  bool IsCreated() const;

  bool Open();
  void Close();
  bool IsOpened() const;

  bool Read(
      std::vector<uint8_t>& buffer,
      size_t bytes_to_read,
      size_t& bytes_read);
  bool Write(
      const std::vector<uint8_t>& buffer,
      size_t bytes_to_write,
      size_t& bytes_written);

 private:
  std::string name_;
  HANDLE      handle_;
  bool        is_opened_;
};

}  // namespace utils

////////////////////////////////////////////////////////////////////////////////
/// utils::Pipe
////////////////////////////////////////////////////////////////////////////////

utils::Pipe::Pipe(const std::string& name) {
  impl_->name_ = name;
}

utils::Pipe::~Pipe() {}

bool utils::Pipe::Create() {
  return impl_->Create();
}

void utils::Pipe::Delete() {
  impl_->Delete();
}

bool utils::Pipe::IsCreated() const {
  return impl_->IsCreated();
}

bool utils::Pipe::Open() {
  return impl_->Open();
}

void utils::Pipe::Close() {
  impl_->Close();
}

bool utils::Pipe::IsOpened() const {
  return impl_->IsOpened();
}

bool utils::Pipe::Read(
    std::vector<uint8_t>& buffer,
    size_t bytes_to_read,
    size_t& bytes_read) {
  return impl_->Read(buffer, bytes_to_read, bytes_read);
}

bool utils::Pipe::Write(
    const std::vector<uint8_t>& buffer,
    size_t bytes_to_write,
    size_t& bytes_written) {
  return impl_->Write(buffer, bytes_to_write, bytes_written);
}

////////////////////////////////////////////////////////////////////////////////
/// utils::Pipe::Impl
////////////////////////////////////////////////////////////////////////////////

utils::Pipe::Impl::Impl()
  : name_(),
    handle_(NULL),
    is_opened_(false) {}

utils::Pipe::Impl::~Impl() {
  Close();
}

bool utils::Pipe::Impl::Create() {
  if (handle_) {
    LOG4CXX_ERROR(logger_ptr, "Named pipe: " << name_ << " already created");
    return false;
  }
  handle_ = CreateNamedPipe(TEXT(name_.c_str()),
                          PIPE_ACCESS_DUPLEX,
                          PIPE_TYPE_BYTE | PIPE_READMODE_BYTE,
                          1,
                          1024,
                          1024,
                          0,
                          NULL);
  if (INVALID_HANDLE_VALUE == handle_) {
    handle_ = NULL;
    LOG4CXX_ERROR(logger_ptr, "Cannot create named pipe: " << name_);
    return false;
  }
  return true;
}

void utils::Pipe::Impl::Delete() {
  if (0 == CloseHandle(handle_)) {
    LOG4CXX_WARN(logger_ptr, "Cannot delete named pipe: " << name_);
  }
}

bool utils::Pipe::Impl::IsCreated() const {
  return NULL != handle_;
}

bool utils::Pipe::Impl::Open() {
  if (NULL == handle_ || 0 == ConnectNamedPipe(handle_, NULL)) {
    LOG4CXX_ERROR(logger_ptr, "Cannot connect to named pipe: " << name_);
    return false;
  }
  is_opened_ = true;
  return true;
}

void utils::Pipe::Impl::Close() {
  if (0 == DisconnectNamedPipe(handle_)) {
    LOG4CXX_WARN(logger_ptr, "Named pipe " << name_ << " already deleted");
  }
  is_opened_ = false;
}

bool utils::Pipe::Impl::IsOpened() const {
  return is_opened_;
}

bool utils::Pipe::Impl::Read(
    std::vector<uint8_t>& buffer,
    size_t bytes_to_read,
    size_t& bytes_read) {
  return impl_->Read(buffer, bytes_to_read, bytes_read);
}

bool utils::Pipe::Impl::Write(
    const std::vector<uint8_t>& buffer,
    size_t bytes_to_write,
    size_t& bytes_written) {
   if (NULL == pipe_) {
    return -1;
  }
  DWORD bytes_written = 0;
  if (0 == WriteFile(pipe_, buf, length, &bytes_written, NULL)) {
    return -1;
  }
  return static_cast<size_t>(bytes_written);
}
