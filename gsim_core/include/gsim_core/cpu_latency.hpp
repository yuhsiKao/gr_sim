// Keeps the CPUs out of deep idle states while a simulation runs.
//
// A controller sleeps while it waits for the physics server's answer, and the server while it
// waits for the controllers. When a physics step takes a good part of a millisecond (Gazebo), the
// idle cores drop into a deep idle state in between, and leaving it costs about 0.1 ms on every
// wake-up: two robots on Gazebo then run at 0.85 times real time instead of 1.0. Linux lets a
// process cap that cost for the whole machine (PM QoS): the cap holds as long as the process
// keeps /dev/cpu_dma_latency open with the limit written to it, and is gone when it closes.
#pragma once
#include <string>

namespace gsim {

class CpuLatencyHold {
 public:
  // Idle states that take longer than this to leave are not used: a tenth of a 0.5 ms step.
  static constexpr int kMaxWakeupUs = 50;

  // Without the device (not Linux, not root, not passed into the container) nothing is held:
  // held() is false and error() says why.
  explicit CpuLatencyHold(int max_wakeup_us = kMaxWakeupUs, const std::string& device = "/dev/cpu_dma_latency");
  ~CpuLatencyHold();
  CpuLatencyHold(const CpuLatencyHold&) = delete;
  CpuLatencyHold& operator=(const CpuLatencyHold&) = delete;

  bool held() const { return fd_ >= 0; }
  const std::string& error() const { return error_; }

 private:
  int fd_ = -1;
  std::string error_;
};

}  // namespace gsim
