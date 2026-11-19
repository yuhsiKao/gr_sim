// Parsing and validation of the gsim_* command-line parameters. Parameters are
// looked up by name (the SDK's "key value" convention), so unknown SDK keys never
// collide with ours.
#pragma once
#include <array>
#include <functional>
#include <string>
#include <vector>

namespace gsim {

struct GsimArgs {
  std::string backend = "mujoco";      // gsim_backend
  bool headless = false;               // gsim_headless (default: test_idx > 0)
  double rtf = 1.0;                    // gsim_rtf (NaN when unparsable; rejected by validate)
  std::string scene;                   // gsim_scene
  std::string world;                   // gsim_world (Gazebo)
  std::string urdf_dir;                // gsim_urdf_dir
  std::string log;                     // gsim_log
  bool selftest = false;               // gsim_selftest
  int control_rate = 500;              // gsim_control_rate [Hz]: written to the SDK's CONTROL_RATE (0 when unparsable)
  std::array<double, 3> toe_friction{3.0, 0.04, 0.0};   // gsim_toe_friction "a b c"
  std::array<double, 2> toe_solref{0.02, 1.0};          // gsim_toe_solref "a b"
  bool toe_parse_ok = true;            // false if either toe parameter failed to parse
  std::string server;                  // gsim_server: the physics server's socket path (gsim_backend remote, and gsim_server itself)
  // gsim_backend remote: parameters that were given although they belong to the physics server.
  std::vector<std::string> remote_conflicts;
  // Several robots in gsim_server's world.
  int robot = 0;                       // gsim_robot (ghost_sim): which robot this controller drives (-1 when unparsable)
  int robots = 1;                      // gsim_robots (gsim_server): robots in the world (-1 when unparsable)
  std::array<double, 3> spawn{0.0, 0.0, 0.0};   // gsim_spawn "x y yaw_deg" (ghost_sim): where the robot starts
  bool spawn_ok = true;                // false if gsim_spawn failed to parse
  // Parameters that were given at all: each belongs to one of the two programs.
  bool robots_given = false;
  std::vector<std::string> controller_only_given;   // of gsim_robot, gsim_spawn, gsim_control_rate, gsim_log
};

struct GsimDefaults {
  std::string scene;
  std::string urdf_dir;
  std::string world;
};

using ArgLookup = std::function<std::string(const std::string& key, const std::string& def)>;

// With gsim_backend remote the physics runs in gsim_server, which owns the scene, the toe
// contact, the real-time factor and the window: here headless is true, rtf is 0 (this side never
// waits for the clock) and selftest is false; giving one of those parameters is an error.
GsimArgs parseGsimArgs(const ArgLookup& lookup, const GsimDefaults& defaults, int test_idx);

// Empty = valid. Each message names the offending gsim_* parameter. For ghost_sim.
std::vector<std::string> validateGsimArgs(const GsimArgs& a);

// The same for gsim_server, where gsim_backend names the physics (mujoco or gazebo): its socket
// path, the number of robots and the physics parameters; parameters of the controller are refused.
std::vector<std::string> validateServerArgs(const GsimArgs& a);

// "key value" pairs from argv[1], argv[2], ... (the SDK's convention): the value of the first
// pair whose key matches, otherwise def.
ArgLookup argvLookup(int argc, char** argv);

// The SDK writes user_params_config.yaml next to planner_config and .grlocaldata into
// local_dir, both defaulting to the working directory. Empty = safe: both are given and
// neither they nor cwd lie inside one of protected_dirs (empty entries are ignored).
// Relative paths are taken relative to cwd.
std::vector<std::string> validateSdkWritePaths(const std::string& planner_config, const std::string& local_dir,
                                               const std::string& cwd, const std::vector<std::string>& protected_dirs);

// Directories ghost_sim must never write into: the SDK from the environment (if set) and
// the one it was built against, plus their parent when it is a GhostSDK_* folder (which
// also holds the macOS variant).
std::vector<std::string> sdkProtectedDirs(const char* env_controls_install_dir, const std::string& compiled_sdk_dir);

// env non-empty → env + "/share/gr/urdf", otherwise compiled_sdk_dir + "/share/gr/urdf".
std::string defaultUrdfDir(const char* env_controls_install_dir, const std::string& compiled_sdk_dir);

}  // namespace gsim
