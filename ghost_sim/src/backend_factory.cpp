// The one place that lists the simulators. Each is compiled in by its GSIM_WITH_* option;
// adding a simulator means one #ifdef block here.
#include "backend_factory.hpp"

#ifdef GSIM_WITH_MUJOCO
#include "gsim_mujoco/MujocoBackend.hpp"
#endif
#ifdef GSIM_WITH_BULLET
#include "gsim_bullet/BulletBackend.hpp"
#endif
#ifdef GSIM_WITH_GAZEBO
#include "gsim_gazebo/GazeboBackend.hpp"
#endif

namespace gsim {

std::unique_ptr<ISimBackend> makeBackend(const std::string& name, std::string* error) {
  std::string available;
#ifdef GSIM_WITH_MUJOCO
  if (name == "mujoco") return std::make_unique<MujocoBackend>();
  available += " mujoco";
#endif
#ifdef GSIM_WITH_BULLET
  if (name == "bullet") return std::make_unique<BulletBackend>();
  available += " bullet";
#endif
#ifdef GSIM_WITH_GAZEBO
  if (name == "gazebo") return std::make_unique<GazeboBackend>();
  available += " gazebo";
#endif
  if (error) *error = "unknown simulator backend '" + name + "' (built in:" + available + ")";
  return nullptr;
}

}  // namespace gsim
