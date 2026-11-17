// The physics server's side of the protocol: serves one ISimBackend to one controller
// (a RemoteBackend in another process). The backend steps only when the controller asks, so
// simulated time is shared; real-time pacing and pause are decided here, not by the controller.
#pragma once
#include <atomic>
#include <string>

#include "gsim_core/sim_backend.hpp"

namespace gsim::remote {

class BackendServer {
 public:
  // backend: already init()'ed. urdf_dir: the controller sends a URDF file name, resolved against
  // this directory. rtf: real-time factor (0 = as fast as the controller asks).
  BackendServer(ISimBackend& backend, std::string urdf_dir, LogFn log, double rtf = 0.0);

  // Serves the controller connected on fd (not closed here) until it leaves, the backend stops
  // running or keep_alive turns false. Returns the number of physics steps served.
  long serve(int fd, std::atomic_bool& keep_alive);

 private:
  void log(int level, const std::string& msg) const;

  ISimBackend& backend_;
  std::string urdf_dir_;
  LogFn log_;
  double rtf_;
};

}  // namespace gsim::remote
