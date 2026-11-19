#include "gsim_core/cpu_latency.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>

namespace gsim {

CpuLatencyHold::CpuLatencyHold(int max_wakeup_us, const std::string& device) {
  // Close-on-exec: a child process (the Gazebo window) must not carry the request on.
  fd_ = open(device.c_str(), O_WRONLY | O_CLOEXEC);
  if (fd_ < 0) {
    error_ = device + ": " + std::strerror(errno);
    return;
  }
  const int32_t limit = max_wakeup_us;   // the kernel reads a binary s32, in microseconds
  if (write(fd_, &limit, sizeof(limit)) != ssize_t(sizeof(limit))) {
    error_ = device + ": " + std::strerror(errno);
    close(fd_);
    fd_ = -1;
  }
}

CpuLatencyHold::~CpuLatencyHold() {
  if (fd_ >= 0) close(fd_);
}

}  // namespace gsim
