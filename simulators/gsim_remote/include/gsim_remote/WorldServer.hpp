// The physics server's side of the protocol: serves one ISimWorld to its controllers (a
// RemoteBackend each, in other processes), one robot per controller. The world steps only when
// every connected controller has asked for the step, so simulated time is shared by all of them;
// real-time pacing and pause are decided here, not by the controllers.
#pragma once
#include <atomic>
#include <string>
#include <vector>

#include "gsim_core/pacer.hpp"
#include "gsim_core/sim_world.hpp"
#include "gsim_remote/protocol.hpp"

namespace gsim::remote {

// Robots must start at least this far apart (base to base, in the ground plane) [m].
inline constexpr double kMinSpawnDistance = 1.0;

class WorldServer {
 public:
  // world: already init()'ed, without robots. num_robots: 1..kMaxRobots, each needs a controller
  // before the simulation starts. urdf_dir: a controller sends a URDF file name, resolved against
  // this directory. rtf: real-time factor (0 = as fast as the controllers ask).
  WorldServer(ISimWorld& world, int num_robots, std::string urdf_dir, LogFn log, double rtf = 0.0);
  ~WorldServer();

  // A controller that is already connected (the server closes fd). For tests; before serve().
  void adopt(int fd);

  // Takes controllers from listen_fd (-1: only the adopted ones) until every robot has one that
  // sent its LoadRobot, builds the world and serves the steps. A controller that leaves while the
  // simulation runs leaves its robot in the world without torque; the others carry on.
  // Returns when every controller has left, the world could not be built, the world stops running
  // or keep_alive turns false. Returns the number of physics steps served.
  long serve(int listen_fd, std::atomic_bool& keep_alive);

 private:
  struct Client {
    int fd = -1;
    int robot = -1;          // -1 until its Hello was accepted
    bool has_spec = false;   // LoadRobot accepted
    RobotSpec spec;
    bool stepped = false;    // sent a Step before (ResetJoint is over)
    bool has_step = false;   // a Step is waiting for the other robots
    Step step{};
  };

  void log(int level, const std::string& msg) const;
  Client* clientOf(int robot);
  bool everyRobotLoaded();
  bool everyClientAsksForAStep() const;
  // Waits up to timeout_ms for a new connection or a message and handles what came.
  void pollOnce(int listen_fd, int timeout_ms);
  void handleMessage(size_t index);          // may drop the client
  void protocolError(size_t index, const std::string& text);
  void drop(size_t index, bool quiet);       // closes the connection; its robot goes limp
  bool buildWorld();                         // answers every LoadRobot
  void stepWorld();
  void sayStopped();

  ISimWorld& world_;
  int num_robots_;
  std::string urdf_dir_;
  LogFn log_;
  Pacer pacer_;
  std::vector<Client> clients_;
  bool built_ = false;
  long steps_ = 0;
  std::vector<uint8_t> payload_;
};

}  // namespace gsim::remote
