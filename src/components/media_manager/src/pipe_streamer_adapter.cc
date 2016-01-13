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
#include "utils/logger.h"
#include "utils/file_system.h"
#include "config_profile/profile.h"
#include "media_manager/pipe_streamer_adapter.h"

namespace media_manager {

CREATE_LOGGERPTR_GLOBAL(logger, "PipeStreamerAdapter")

PipeStreamerAdapter::PipeStreamerAdapter(const std::string& named_pipe_path)
    : StreamerAdapter(new PipeStreamer(this, named_pipe_path)) {}

PipeStreamerAdapter::~PipeStreamerAdapter() {}

PipeStreamerAdapter::PipeStreamer::PipeStreamer(
    PipeStreamerAdapter* const adapter, const std::string& named_pipe_path)
    : Streamer(adapter), pipe_(named_pipe_path) {}

PipeStreamerAdapter::PipeStreamer::~PipeStreamer() {}

bool PipeStreamerAdapter::PipeStreamer::Connect() {
  LOG4CXX_AUTO_TRACE(logger);

  if (!file_system::CreateDirectoryRecursively(
          profile::Profile::instance()->app_storage_folder())) {
    LOG4CXX_ERROR(logger, "Cannot create app folder");
    return false;
  }

  if (!pipe_.Create() || !pipe_.Open()) {
    LOG4CXX_ERROR(logger, "Cannot connect to pipe");
    return false;
  }

  LOG4CXX_INFO(logger, "Streamer connected to pipe");
  return true;
}

void PipeStreamerAdapter::PipeStreamer::Disconnect() {
  LOG4CXX_AUTO_TRACE(logger);

  pipe_.Delete();
  LOG4CXX_INFO(logger, "Streamer disconnected from pipe");
}

bool PipeStreamerAdapter::PipeStreamer::Send(
    protocol_handler::RawMessagePtr msg) {
  LOG4CXX_AUTO_TRACE(logger);

  size_t sent = 0;
  if (!pipe_.Write(msg->data(), msg->data_size(), sent)) {
    LOG4CXX_ERROR(logger, "Cannot write to pipe");
    return false;
  }

  if (sent != msg->data_size()) {
    LOG4CXX_WARN(logger, "Couldn't write all the data to pipe");
  }

  LOG4CXX_INFO(logger, "Streamer sent to pipe " << sent << " bytes");
  return true;
}

}  // namespace media_manager
