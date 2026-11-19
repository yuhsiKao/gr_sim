// gsim_server: the physics for `ghost_sim gsim_backend remote`.
//
//   gsim_server gsim_server <socket path> [gsim_robots 1] [gsim_headless 1] [gsim_rtf 1.0]
//               [gsim_backend mujoco|gazebo] [gsim_scene ...] [gsim_world ...] [gsim_urdf_dir ...]
//               [gsim_toe_friction "a b c"] [gsim_toe_solref "a b"]
//
// One world (MuJoCo, or Gazebo) with gsim_robots robots, stepped only when their controllers ask: it waits on
// the socket until every robot has a controller, serves them and stops when the last one has left
// (or on Ctrl-C, or when the window is closed). The real-time factor, pause and the window belong
// here; the controllers only follow.
#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

#include "gsim_core/cpu_latency.hpp"
#include "gsim_core/gsim_args.hpp"
#include "gsim_core/sim_loop.hpp"
#include "gsim_mujoco/MujocoWorld.hpp"
#ifdef GSIM_WITH_GAZEBO
#include "gsim_gazebo/GazeboWorld.hpp"
#endif
#include "gsim_remote/WorldServer.hpp"
#include "gsim_remote/protocol.hpp"

namespace {

std::atomic_bool g_keep_alive{true};
static_assert(std::atomic_bool::is_always_lock_free, "keep_alive must be signal-safe");

extern "C" void onStopSignal(int) { g_keep_alive.store(false); }

void say(int level, const std::string& msg) {
  const char* tag = level >= gsim::kError ? "error: " : level == gsim::kWarning ? "warning: " : "";
  std::fprintf(stderr, "gsim_server: %s%s\n", tag, msg.c_str());
}

int fail(const std::string& msg) {
  say(gsim::kError, msg);
  return 2;
}

// The one place that lists the worlds the server can run. Null: this build has no such world.
std::unique_ptr<gsim::ISimWorld> makeWorld(const std::string& name) {
#ifdef GSIM_WITH_GAZEBO
  if (name == "gazebo") return std::make_unique<gsim::GazeboWorld>();
#endif
  if (name == "mujoco") return std::make_unique<gsim::MujocoWorld>();
  return nullptr;
}

}  // namespace

int main(int argc, char* argv[]) {
  // gsim_backend names the physics here: mujoco (the default) or gazebo.
  const gsim::GsimArgs args = gsim::parseGsimArgs(
      gsim::argvLookup(argc, argv),
      {GSIM_DEFAULT_SCENE, gsim::defaultUrdfDir(std::getenv("CONTROLS_INSTALL_DIR"), GSIM_SDK_DIR), GSIM_DEFAULT_WORLD}, 0);
  const std::vector<std::string> errors = gsim::validateServerArgs(args);
  for (const auto& e : errors) say(gsim::kError, e);
  if (!errors.empty()) return 2;
  const std::unique_ptr<gsim::ISimWorld> owned = makeWorld(args.backend);
  if (!owned) return fail("gsim_backend: this gsim_server was built without " + args.backend);
  gsim::ISimWorld& world = *owned;

  gsim::SimConfig cfg;
  cfg.headless = args.headless;
  cfg.scene_path = args.scene;
  cfg.world_path = args.world;
  cfg.depth_camera = true;   // Gazebo: every robot gets its camera; a controller asks for frames while its cameras are on
  cfg.urdf_dir = args.urdf_dir;
  cfg.argc = argc;
  cfg.argv = argv;
  cfg.log = say;
  cfg.toe_friction = args.toe_friction;
  cfg.toe_solref = args.toe_solref;

  const bool gazebo = args.backend == "gazebo";
  if (!world.init(cfg)) return fail(std::string("failed to initialize ") + (gazebo ? "Gazebo" : "MuJoCo"));
  if (const std::string e = gsim::physicsTimestepError(world.timestep()); !e.empty())
    return fail(e + (gazebo ? " (gsim_world " + args.world : " (gsim_scene " + args.scene) + ")");

  std::string err;
  const int listen_fd = gsim::remote::listenUnix(args.server, &err);
  if (listen_fd < 0) return fail(err);

  std::signal(SIGINT, onStopSignal);
  std::signal(SIGTERM, onStopSignal);
  // A world whose step takes long enough for the waiting cores to fall into a deep idle state:
  // waking them every step costs more than the step has left (gsim_core/cpu_latency.hpp).
  std::unique_ptr<gsim::CpuLatencyHold> awake;
  if (world.wantsCpuLatencyHold()) {
    awake = std::make_unique<gsim::CpuLatencyHold>();
    if (awake->held())
      say(gsim::kInfo, "deep CPU idle states are held off while the server runs");
    else
      say(args.robots > 1 ? gsim::kWarning : gsim::kInfo,
          "deep CPU idle states are not held off (" + awake->error() + "): several robots may run below real time");
  }
  say(gsim::kInfo, "physics=" + args.backend + " headless=" + std::to_string(int(args.headless)) + " rtf=" + std::to_string(args.rtf) + " robots=" +
                       std::to_string(args.robots) + "; waiting for " +
                       (args.robots == 1 ? std::string("a controller") : std::to_string(args.robots) + " controllers") +
                       " on " + args.server);

  // Waits for the controllers, serves them, and ends the process when they are done.
  const auto serve = [&] {
    gsim::remote::WorldServer server(world, args.robots, args.urdf_dir, say, args.rtf);
    const long steps = server.serve(listen_fd, g_keep_alive);
    say(gsim::kInfo, "stopped at t=" + std::to_string(steps * world.timestep()) + " s after " + std::to_string(steps) + " steps");
    g_keep_alive.store(false);
  };

  if (world.wantsMainThreadViewer()) {
    // The window must own the main thread; the controller is served on a worker thread.
    std::thread worker(serve);
    world.runViewer(g_keep_alive);
    g_keep_alive.store(false);
    worker.join();
  } else {
    serve();
  }

  gsim::remote::closeFd(listen_fd);
  unlink(args.server.c_str());
  return 0;
}
