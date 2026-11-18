#include "gsim_mujoco/MujocoBackend.hpp"

namespace gsim {

bool MujocoBackend::loadRobot(const RobotSpec& spec) {
  world_.clear();
  return world_.addRobot(spec) == 0 && world_.build();
}

std::vector<std::string> MujocoBackend::selfTest(const RobotSpec& spec) {
  const SimConfig& cfg = world_.config();
  ModelOptions opt;
  static_cast<RobotOptions&>(opt) = robotOptions(spec);
  opt.scene_path = cfg.scene_path;
  opt.toe_friction = cfg.toe_friction;
  opt.toe_solref = cfg.toe_solref;
  BuiltModel built = buildMujocoModel(opt);
  for (const auto& w : built.warnings)
    if (cfg.log) cfg.log(kInfo, "MuJoCo: " + w);
  if (!built.model) return {"model build failed: " + built.error};
  std::vector<std::string> problems = checkMujocoModel(built.model, built.joints, opt);
  mj_deleteModel(built.model);
  return problems;
}

}  // namespace gsim
