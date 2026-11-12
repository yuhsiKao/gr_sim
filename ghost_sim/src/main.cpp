// Same structure as the SDK's FirstStep example; the simulator is chosen by the
// gsim_* arguments handled in sim_bsp.cpp.
#include <CmdLineArgs.hpp>
#include <ControlRate.hpp>
#include <SDK.h>
#include <loguru.hpp>

#include <atomic>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "gsim_core/gsim_args.hpp"
#include "gsim_core/sim_loop.hpp"
#include "sim_bsp.hpp"

extern int gr_ctrl_c_flag;  // what the SDK's own catchSIGINT() handler sets

namespace {

std::atomic_bool* g_keep_alive = nullptr;
static_assert(std::atomic_bool::is_always_lock_free, "keep_alive must be signal-safe");

// gr::catchSIGINT() only sets gr_ctrl_c_flag and never clears robot.keep_alive, so
// Ctrl-C could not stop the simulation loop. Do both.
extern "C" void onStopSignal(int) {
  gr_ctrl_c_flag = 1;
  if (g_keep_alive) g_keep_alive->store(false);
}

// The SDK's MBLink receiver binds this UDP port. When it is taken (another simulator is running)
// the SDK only logs "RX socket bind error", runs without MBLink and aborts at shutdown.
constexpr uint16_t kMblinkRxPort = 14999;

bool udpPortFree(uint16_t port) {
  const int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) return true;  // cannot tell: let the SDK try
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(port);
  const bool free = bind(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) == 0;
  close(fd);
  return free;
}

}  // namespace

int main(int argc, char* argv[]) {
  // The SDK reads CONTROLS_INSTALL_DIR without a null check and aborts when it is
  // unset; default it to the SDK this binary was built against (same as scripts/env.sh).
  const char* sdk = std::getenv("CONTROLS_INSTALL_DIR");
  if (!sdk || !*sdk) setenv("CONTROLS_INSTALL_DIR", GSIM_SDK_DIR, 1);

  // robot.init() writes user_params_config.yaml next to planner_config and .grlocaldata
  // into local_dir (both default to the working directory): never into the Ghost SDK.
  {
    const gr::ArgList_t args = gr::getArgList(argc, argv);
    const auto errors = gsim::validateSdkWritePaths(
        gr::updateFromArgs(args, "planner_config", ""), gr::updateFromArgs(args, "local_dir", ""),
        std::filesystem::current_path().string(), gsim::sdkProtectedDirs(std::getenv("CONTROLS_INSTALL_DIR"), GSIM_SDK_DIR));
    for (const auto& e : errors) std::fprintf(stderr, "ghost_sim: %s\n", e.c_str());
    if (!errors.empty()) return 2;

    // gsim_control_rate: the SDK's CONTROL_RATE, as on the robot's onboard computer. It has to be
    // written before the SDK is constructed (its modules read it from then on).
    const gsim::ArgLookup lookup = [&args](const std::string& key, const std::string& def) {
      return gr::updateFromArgs(args, key, def);
    };
    const int control_rate = gsim::parseGsimArgs(lookup, {}, 0).control_rate;
    if (const std::string e = gsim::controlRateError(control_rate); !e.empty()) {
      std::fprintf(stderr, "ghost_sim: %s\n", e.c_str());
      return 2;
    }
    gr::CONTROL_RATE = control_rate;
  }

  if (!udpPortFree(kMblinkRxPort)) {
    std::fprintf(stderr,
                 "ghost_sim: UDP port %d (the SDK's MBLink) is in use: another simulator is already running. "
                 "Stop it first.\n",
                 int(kMblinkRxPort));
    return 2;
  }

  gr::Robot robot;
  robot.init(argc, argv);
  robot.setVerbose(1);
  g_keep_alive = &robot.keep_alive;
  std::signal(SIGINT, onStopSignal);   // Ctrl-C
  std::signal(SIGTERM, onStopSignal);  // kill / timeout

  robot.begin();  // runs the simulation loop until the window closes or Ctrl-C

  robot.stop();
  LOG_F(INFO, "Exiting");
  return gsim::bspExitCode();
}
