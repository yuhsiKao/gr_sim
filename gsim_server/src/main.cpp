// gsim_server: the physics for `ghost_sim gsim_backend remote`.
//
//   gsim_server gsim_server <socket path> [gsim_robots 1] [gsim_headless 1] [gsim_rtf 1.0]
//               [gsim_scene ...] [gsim_urdf_dir ...] [gsim_toe_friction "a b c"] [gsim_toe_solref "a b"]
//
// One MuJoCo world with gsim_robots robots, stepped only when their controllers ask: it waits on
// the socket until every robot has a controller, serves them and stops when the last one has left
// (or on Ctrl-C, or when the window is closed). The real-time factor, pause and the window belong
// here; the controllers only follow.
#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

#include "gsim_core/gsim_args.hpp"
#include "gsim_core/sim_loop.hpp"
#include "gsim_mujoco/MujocoWorld.hpp"
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

}  // namespace

int main(int argc, char* argv[]) {
  const gsim::ArgLookup given = gsim::argvLookup(argc, argv);
  // The physics here is always MuJoCo, whatever gsim_backend says.
  const gsim::ArgLookup lookup = [&given](const std::string& key, const std::string& def) {
    return key == "gsim_backend" ? std::string("mujoco") : given(key, def);
  };
  const gsim::GsimArgs args = gsim::parseGsimArgs(
      lookup, {GSIM_DEFAULT_SCENE, gsim::defaultUrdfDir(std::getenv("CONTROLS_INSTALL_DIR"), GSIM_SDK_DIR), ""}, 0);
  const std::vector<std::string> errors = gsim::validateServerArgs(args);
  for (const auto& e : errors) say(gsim::kError, e);
  if (!errors.empty()) return 2;

  gsim::SimConfig cfg;
  cfg.headless = args.headless;
  cfg.scene_path = args.scene;
  cfg.urdf_dir = args.urdf_dir;
  cfg.argc = argc;
  cfg.argv = argv;
  cfg.log = say;
  cfg.toe_friction = args.toe_friction;
  cfg.toe_solref = args.toe_solref;

  gsim::MujocoWorld world;
  if (!world.init(cfg)) return fail("failed to initialize MuJoCo");
  if (const std::string e = gsim::physicsTimestepError(world.timestep()); !e.empty())
    return fail(e + " (gsim_scene " + args.scene + ")");

  std::string err;
  const int listen_fd = gsim::remote::listenUnix(args.server, &err);
  if (listen_fd < 0) return fail(err);

  std::signal(SIGINT, onStopSignal);
  std::signal(SIGTERM, onStopSignal);
  say(gsim::kInfo, "headless=" + std::to_string(int(args.headless)) + " rtf=" + std::to_string(args.rtf) + " robots=" +
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
