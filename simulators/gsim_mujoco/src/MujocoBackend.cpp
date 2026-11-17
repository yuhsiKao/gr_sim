#include "gsim_mujoco/MujocoBackend.hpp"

#include "gsim_core/sim_math.hpp"

namespace gsim {

bool MujocoBackend::loadRobot(const RobotSpec& spec) {
  world_.clear();
  return world_.addRobot(spec) == 0 && world_.build();
}

std::vector<std::string> MujocoBackend::selfTest(const RobotSpec& spec) {
  const SimConfig& cfg = world_.config();
  ModelOptions opt;
  opt.urdf_path = spec.urdf_path;
  opt.scene_path = cfg.scene_path;
  opt.num_sdk_joints = spec.num_sdk_joints;
  opt.fixed_base = spec.fixed_base;
  opt.base_pos = spec.pos0_W.cast<double>();
  opt.base_quat = quatFromRpy(spec.rpy0).cast<double>();
  opt.toe_friction = cfg.toe_friction;
  opt.toe_solref = cfg.toe_solref;
  opt.arm_joints = spec.arm_joints;
  BuiltModel built = buildMujocoModel(opt);
  for (const auto& w : built.warnings)
    if (cfg.log) cfg.log(kInfo, "MuJoCo: " + w);
  if (!built.model) return {"model build failed: " + built.error};
  std::vector<std::string> problems = checkMujocoModel(built.model, built.joints, opt);
  mj_deleteModel(built.model);
  return problems;
}

}  // namespace gsim
