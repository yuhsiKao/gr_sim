// ISimBackend over MuJoCo: a MujocoWorld with one robot.
#pragma once
#include <atomic>
#include <string>
#include <vector>

#include "gsim_mujoco/MujocoWorld.hpp"
#include "gsim_core/sim_backend.hpp"

namespace gsim {

class MujocoBackend : public ISimBackend {
 public:
  bool init(const SimConfig& cfg) override { return world_.init(cfg); }
  bool loadRobot(const RobotSpec& spec) override;
  void resetJoint(int sdk_joint, float q) override { world_.resetJoint(0, sdk_joint, q); }
  void step() override { world_.step(); }
  void readState(BaseState& base, Eigen::ArrayXf& q, Eigen::ArrayXf& dq) override { world_.readState(0, base, q, dq); }
  void setJointTorques(const Eigen::ArrayXf& tau) override { world_.setJointTorques(0, tau); }
  bool running() const override { return world_.running(); }
  double timestep() const override { return world_.timestep(); }
  bool paused() const override { return world_.paused(); }
  bool wantsMainThreadViewer() const override { return world_.wantsMainThreadViewer(); }
  void runViewer(std::atomic_bool& keep_alive) override { world_.runViewer(keep_alive); }
  void applyExternalForce(const Eigen::Vector3f& f_B, const Eigen::Vector3f& p_B) override {
    world_.applyExternalForce(0, f_B, p_B);
  }
  std::vector<std::string> selfTest(const RobotSpec& spec) override;
  bool supportsArm() const override { return true; }
  void setArmLocked(bool locked) override { world_.setArmLocked(0, locked); }
  void setWristCommand(const WristCommand& cmd) override { world_.setWristCommand(0, cmd); }
  void readWrist(WristState& w) override { world_.readWrist(0, w); }
  bool supportsDepthCamera() const override { return true; }
  bool updateDepthCamera(PointCloud& cloud_B) override { return world_.updateDepthCamera(0, cloud_B); }

  // Diagnostics / tests.
  const mjModel* model() const { return world_.model(); }
  mjData* mutableData() { return world_.mutableData(); }
  int instabilityCount() const { return world_.instabilityCount(); }
  long nanTorqueCount() const { return world_.nanTorqueCount(); }

 private:
  MujocoWorld world_;
};

}  // namespace gsim
