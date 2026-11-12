// CSV log of the simulated robot state, one row every `decimation` control ticks
// (500 Hz ticks -> 100 Hz rows by default). Columns:
//   t,px,py,pz,roll,pitch,yaw,vx,vy,vz,q0..qN-1,dq0..dqN-1,tau0..tauN-1
// and, with an arm: arm_locked,wmode,wq0..wq3,wdq0..wdq3,wcur0..wcur3 (arm_locked and wmode
// are the arm command sent on the previous step; wmode uses the SDK's numbers 0/2/4).
// Engine-agnostic: fed from TickInfo, so every backend logs the same columns.
#pragma once
#include <cstdio>
#include <string>
#include <vector>

#include "gsim_core/sim_loop.hpp"

namespace gsim {

class CsvLogger {
 public:
  explicit CsvLogger(int decimation = 5);
  ~CsvLogger();
  CsvLogger(const CsvLogger&) = delete;
  CsvLogger& operator=(const CsvLogger&) = delete;

  // Creates/truncates `path` and writes the header. On failure returns false and
  // sets *error (which names the path).
  bool open(const std::string& path, int num_joints, std::string* error, bool arm = false);
  void write(const TickInfo& tick);  // call every control tick; writes every decimation-th call
  void close();                      // flushes; safe to call twice

 private:
  int decimation_;
  int num_joints_ = 0;
  bool arm_ = false;
  long calls_ = 0;
  std::FILE* file_ = nullptr;
  std::vector<char> buffer_;
};

}  // namespace gsim
