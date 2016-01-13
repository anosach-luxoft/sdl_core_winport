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
#include <cstddef>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "utils/pipe.h"
#include "utils/pimpl_impl.h"
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

  bool Read(uint8_t* buffer,
            size_t bytes_to_read,
            size_t& bytes_read);
  bool Write(const uint8_t* buffer,
             size_t bytes_to_write,
             size_t& bytes_written);

 private:
  std::string name_;
  int         handle_;
  bool        is_opened_;
};

}  // namespace utils

////////////////////////////////////////////////////////////////////////////////
/// utils::Pipe
////////////////////////////////////////////////////////////////////////////////

utils::Pipe::Pipe(const std::string& name) {
  impl_->name_ = name;
}

utils::Pipe::~Pipe() {
}

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

bool utils::Pipe::Read(uint8_t* buffer,
                       size_t bytes_to_read,
                       size_t& bytes_read) {
  return impl_->Read(buffer, bytes_to_read, bytes_read);
}

bool utils::Pipe::Write(const uint8_t* buffer,
                        size_t bytes_to_write,
                        size_t& bytes_written) {
  return impl_->Write(buffer, bytes_to_write, bytes_written);
}

////////////////////////////////////////////////////////////////////////////////
/// utils::Pipe::Impl
////////////////////////////////////////////////////////////////////////////////

utils::Pipe::Impl::Impl()
  : name_(),
    handle_(0),
    is_opened_(false) {
}

utils::Pipe::Impl::~Impl() {
  Delete();
}

bool utils::Pipe::Impl::Create() {
  if (IsCreated()) {
    LOG4CXX_WARN(logger_ptr, "Named pipe: " << name_ << " is already created");
    return true;
  }
  handle_ = mkfifo(name_.c_str(), S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
  if (-1 == handle_) {
    handle_ = 0;
    LOG4CXX_ERROR(logger_ptr, "Cannot create named pipe: " << name_);
    return false;
  }
  return true;
}

void utils::Pipe::Impl::Delete() {
  if (!IsCreated()) {
    LOG4CXX_WARN(logger_ptr, "Named pipe: " << name_ << " is not created");
    return;
  }
  Close();
  if (-1 == unlink(handle_)) {
    LOG4CXX_WARN(logger_ptr, "Cannot delete named pipe: " << name_);
  }
  handle_ = NULL;
}

bool utils::Pipe::Impl::IsCreated() const {
  return NULL != handle_;
}

bool utils::Pipe::Impl::Open() {
  if (!IsCreated()) {
    LOG4CXX_ERROR(logger_ptr, "Named pipe: " << name_ << " is not created");
    return false;
  }
  if (IsOpened()) {
    LOG4CXX_WARN(logger_ptr, "Named pipe: " << name_ << " is already opened");
    return true;
  }
  if (-1 == open(name_, O_RDWR, 0)) {
    LOG4CXX_ERROR(logger_ptr, "Cannot connect to named pipe: " << name_);
    return false;
  }
  is_opened_ = true;
  return true;
}

void utils::Pipe::Impl::Close() {
  if (!IsCreated()) {
    LOG4CXX_WARN(logger_ptr, "Named pipe: " << name_ << " is not created");
    return;
  }
  if (!IsOpened()) {
    LOG4CXX_WARN(logger_ptr, "Named pipe: " << name_ << " is not opened");
    return;
  }
  if (-1 == close(handle_)) {
    LOG4CXX_WARN(logger_ptr, "Cannot disconnect from named pipe: " << name_);
  }
  is_opened_ = false;
}

bool utils::Pipe::Impl::IsOpened() const {
  return is_opened_;
}

bool utils::Pipe::Impl::Read(uint8_t* buffer,
                             size_t bytes_to_read,
                             size_t& bytes_read) {
  bytes_read = 0;
  if (!IsOpened()) {
    LOG4CXX_ERROR(logger_ptr, "Named pipe: " << name_ << " is not opened");
    return false;
  }
  if (bytes_to_read == 0) {
    LOG4CXX_WARN(logger_ptr, "Trying to read 0 bytes");
    return true;
  }
  const ssize_t read = read(handle_,
                            static_cast<void*>(buffer),
                            bytes_to_read);
  if (-1 == read) {
    LOG4CXX_ERROR(logger_ptr, "Cannot read from named pipe: " << name_);
    return false;
  }
  bytes_read = static_cast<size_t>(read);
  return true;
}

bool utils::Pipe::Impl::Write(const uint8_t* buffer,
                              size_t bytes_to_write,
                              size_t& bytes_written) {
  bytes_written = 0;
  if (!IsOpened()) {
    LOG4CXX_ERROR(logger_ptr, "Named pipe: " << name_ << " is not opened");
    return false;
  }
  if (bytes_to_write == 0) {
    LOG4CXX_WARN(logger_ptr, "Trying to write 0 bytes");
    return true;
  }
  const ssize_t written = write(handle_,
                                static_cast<const void*>(buffer),
                                bytes_to_write);
  if (-1 == written) {
    LOG4CXX_ERROR(logger_ptr, "Cannot write to named pipe: " << name_);
    return false;
  }
  bytes_written = static_cast<size_t>(written);
  return true;
}
