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
};

struct GsimDefaults {
  std::string scene;
  std::string urdf_dir;
  std::string world;
};

using ArgLookup = std::function<std::string(const std::string& key, const std::string& def)>;

GsimArgs parseGsimArgs(const ArgLookup& lookup, const GsimDefaults& defaults, int test_idx);

// Empty = valid. Each message names the offending gsim_* parameter.
std::vector<std::string> validateGsimArgs(const GsimArgs& a);

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
