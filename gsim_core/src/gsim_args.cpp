#include "gsim_core/gsim_args.hpp"

#include "gsim_core/sim_loop.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <sstream>

namespace gsim {

namespace {

double parseDouble(const std::string& s) {
  char* end = nullptr;
  const double v = std::strtod(s.c_str(), &end);
  if (s.empty() || end == s.c_str() || *end != '\0') return std::numeric_limits<double>::quiet_NaN();
  return v;
}

// Parses exactly N whitespace-separated numbers into out; returns false otherwise.
template <size_t N>
bool parseNumbers(const std::string& s, std::array<double, N>& out) {
  std::istringstream in(s);
  std::array<double, N> tmp{};
  for (auto& v : tmp)
    if (!(in >> v)) return false;
  std::string extra;
  if (in >> extra) return false;
  out = tmp;
  return true;
}

bool parseFlag(const std::string& s) { return s == "1" || s == "true"; }

int parseInt(const std::string& s) {   // 0 unless the whole string is an integer
  char* end = nullptr;
  const long v = std::strtol(s.c_str(), &end, 10);
  if (s.empty() || end == s.c_str() || *end != '\0' || v < std::numeric_limits<int>::min() || v > std::numeric_limits<int>::max())
    return 0;
  return static_cast<int>(v);
}

// Parameters of the physics, which gsim_server owns when the backend is remote.
constexpr const char* kServerOnly[] = {"gsim_scene", "gsim_world", "gsim_toe_friction", "gsim_toe_solref", "gsim_rtf", "gsim_selftest"};

}  // namespace

GsimArgs parseGsimArgs(const ArgLookup& lookup, const GsimDefaults& defaults, int test_idx) {
  GsimArgs a;
  a.backend = lookup("gsim_backend", a.backend);
  a.headless = parseFlag(lookup("gsim_headless", test_idx > 0 ? "1" : "0"));
  a.rtf = parseDouble(lookup("gsim_rtf", "1.0"));
  a.scene = lookup("gsim_scene", defaults.scene);
  a.world = lookup("gsim_world", defaults.world);
  a.urdf_dir = lookup("gsim_urdf_dir", defaults.urdf_dir);
  a.log = lookup("gsim_log", "");
  a.selftest = parseFlag(lookup("gsim_selftest", "0"));
  a.control_rate = parseInt(lookup("gsim_control_rate", "500"));
  const bool friction_ok = parseNumbers(lookup("gsim_toe_friction", "3.0 0.04 0"), a.toe_friction);
  const bool solref_ok = parseNumbers(lookup("gsim_toe_solref", "0.02 1"), a.toe_solref);
  a.toe_parse_ok = friction_ok && solref_ok;
  a.server = lookup("gsim_server", "");
  if (a.backend == "remote") {
    const std::string unset = "\x01unset";
    for (const char* name : kServerOnly)
      if (lookup(name, unset) != unset) a.remote_conflicts.push_back(name);
    if (const std::string h = lookup("gsim_headless", unset); h != unset && !parseFlag(h))
      a.remote_conflicts.push_back("gsim_headless");
    a.headless = true;
    a.rtf = 0.0;
    a.selftest = false;
  }
  return a;
}

ArgLookup argvLookup(int argc, char** argv) {
  std::vector<std::pair<std::string, std::string>> pairs;
  for (int i = 1; i + 1 < argc; i += 2) pairs.emplace_back(argv[i], argv[i + 1]);
  return [pairs](const std::string& key, const std::string& def) {
    for (const auto& p : pairs)
      if (p.first == key) return p.second;
    return def;
  };
}

std::vector<std::string> validateGsimArgs(const GsimArgs& a) {
  namespace fs = std::filesystem;
  std::vector<std::string> errs;
  if (a.backend != "mujoco" && a.backend != "bullet" && a.backend != "gazebo" && a.backend != "remote")
    errs.push_back("gsim_backend: unknown backend '" + a.backend + "' (expected mujoco, bullet, gazebo or remote)");
  if (a.backend == "remote") {
    if (a.server.empty())
      errs.push_back("gsim_server: required with gsim_backend remote (the physics server's socket path)");
    for (const auto& name : a.remote_conflicts)
      errs.push_back(name == "gsim_headless"
                         ? "gsim_headless: always 1 with gsim_backend remote (the window belongs to gsim_server)"
                         : name + ": set it on gsim_server, not on ghost_sim (gsim_backend remote)");
  }
  if (!std::isfinite(a.rtf) || a.rtf < 0.0)
    errs.push_back("gsim_rtf: must be a finite number >= 0 (0 = as fast as possible)");
  if (const std::string e = controlRateError(a.control_rate); !e.empty()) errs.push_back(e);
  if (!a.toe_parse_ok)
    errs.push_back("gsim_toe_friction / gsim_toe_solref: expected 3 and 2 numbers, e.g. \"3.0 0.04 0\" and \"0.02 1\"");
  const fs::path urdf = fs::path(a.urdf_dir) / "vision60.urdf";
  if (!fs::exists(urdf))
    errs.push_back("gsim_urdf_dir: " + urdf.string() + " not found (check CONTROLS_INSTALL_DIR)");
  if (a.backend == "mujoco" && !fs::exists(a.scene))
    errs.push_back("gsim_scene: " + a.scene + " not found");
  if (a.backend == "gazebo" && !fs::exists(a.world))
    errs.push_back("gsim_world: " + a.world + " not found");
  return errs;
}

std::vector<std::string> validateSdkWritePaths(const std::string& planner_config, const std::string& local_dir,
                                               const std::string& cwd, const std::vector<std::string>& protected_dirs) {
  namespace fs = std::filesystem;
  std::vector<fs::path> roots;
  for (const auto& dir : protected_dirs)
    if (!dir.empty()) roots.push_back(fs::weakly_canonical(dir));
  // The protected directory containing p, or "" if none does.
  const auto insideSdk = [&roots](const fs::path& p) -> std::string {
    for (const auto& root : roots)
      if (std::mismatch(root.begin(), root.end(), p.begin(), p.end()).first == root.end()) return root.string();
    return "";
  };
  const auto resolve = [&cwd](const std::string& p) { return fs::weakly_canonical(fs::path(cwd) / p); };
  const std::string hint = "; start ghost_sim with scripts/run_sim.sh";

  std::vector<std::string> errs;
  // The SDK writes `file` into `dir`, which comes from the parameter `key`.
  const auto check = [&](const std::string& key, const std::string& value, const fs::path& dir, const char* file) {
    if (value.empty())
      errs.push_back(key + " is not set: the SDK would write " + file + " into the working directory" + hint);
    else if (const auto sdk = insideSdk(dir); !sdk.empty())
      errs.push_back(key + " " + value + " is inside the Ghost SDK (" + sdk + "): the SDK writes " + file + " there" + hint);
  };
  check("planner_config", planner_config, resolve(planner_config).parent_path(), "user_params_config.yaml");
  check("local_dir", local_dir, resolve(local_dir), ".grlocaldata");
  if (const auto sdk = insideSdk(fs::weakly_canonical(cwd)); !sdk.empty())
    errs.push_back("the working directory " + cwd + " is inside the Ghost SDK (" + sdk + "); run ghost_sim from elsewhere");
  return errs;
}

std::vector<std::string> sdkProtectedDirs(const char* env_controls_install_dir, const std::string& compiled_sdk_dir) {
  namespace fs = std::filesystem;
  std::vector<std::string> dirs;
  const auto add = [&dirs](const std::string& s) {
    if (std::find(dirs.begin(), dirs.end(), s) == dirs.end()) dirs.push_back(s);
  };
  for (const std::string dir : {std::string(env_controls_install_dir ? env_controls_install_dir : ""), compiled_sdk_dir}) {
    if (dir.empty()) continue;
    fs::path p = fs::path(dir).lexically_normal();
    if (!p.has_filename()) p = p.parent_path();  // "…/GhostSDK_bullet_linux/" → "…/GhostSDK_bullet_linux"
    add(p.string());
    if (p.parent_path().filename().string().rfind("GhostSDK", 0) == 0) add(p.parent_path().string());
  }
  return dirs;
}

std::string defaultUrdfDir(const char* env_controls_install_dir, const std::string& compiled_sdk_dir) {
  const std::string base = (env_controls_install_dir && *env_controls_install_dir) ? env_controls_install_dir
                                                                                     : compiled_sdk_dir;
  return base + "/share/gr/urdf";
}

}  // namespace gsim
