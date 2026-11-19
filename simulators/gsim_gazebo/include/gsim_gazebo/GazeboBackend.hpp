// ISimBackend on Gazebo Harmonic: a gz::sim::Server inside ghost_sim, stepped one iteration at
// a time by SimLoop (Server::RunOnce), with Ghost's URDF converted to SDF in memory. Legs and
// the arm (Bullet-style lock and wrist servo as DART servo joints). This is GazeboWorld with one
// robot.
#pragma once
#include <string>
#include <vector>

#include "gsim_core/sim_backend.hpp"
#include "gsim_gazebo/GazeboWorld.hpp"

namespace gsim {

class GazeboBackend : public ISimBackend {
 public:
  bool init(const SimConfig& cfg) override { return world_.init(cfg); }
  bool loadRobot(const RobotSpec& spec) override;
  void resetJoint(int sdk_joint, float q) override { world_.resetJoint(0, sdk_joint, q); }
  void step() override { world_.step(); }
  void readState(BaseState& base, Eigen::ArrayXf& q, Eigen::ArrayXf& dq) override { world_.readState(0, base, q, dq); }
  void setJointTorques(const Eigen::ArrayXf& tau) override { world_.setJointTorques(0, tau); }
  bool running() const override { return world_.numRobots() > 0 && world_.running(); }   // not before loadRobot
  double timestep() const override { return world_.timestep(); }
  bool paused() const override { return world_.paused(); }
  void applyExternalForce(const Eigen::Vector3f& f_B, const Eigen::Vector3f& p_B) override {
    world_.applyExternalForce(0, f_B, p_B);
  }
  std::vector<std::string> selfTest(const RobotSpec& spec) override { return world_.selfTest(spec); }

  // Arm (RobotSpec::arm_joints not empty).
  bool supportsArm() const override { return true; }
  void setArmLocked(bool locked) override { world_.setArmLocked(0, locked); }
  void setWristCommand(const WristCommand& cmd) override { world_.setWristCommand(0, cmd); }
  void readWrist(WristState& w) override { world_.readWrist(0, w); }

  // Synthetic depth camera (SimConfig::depth_camera): Ghost's downward camera as a Gazebo
  // depth_camera sensor; frames come on the sim-time grid of FactoryDepthCamera::kStepsPerFrame steps.
  bool supportsDepthCamera() const override { return world_.supportsDepthCamera(); }
  bool updateDepthCamera(PointCloud& cloud_B) override { return world_.updateDepthCamera(0, cloud_B); }

  // Diagnostics / tests (GazeboWorld.hpp).
  long nanTorqueCount() const { return world_.nanTorqueCount(); }
  double simTime() const { return world_.simTime(); }
  int guiPid() const { return world_.guiPid(); }
  bool dartPluginPreloaded() const { return world_.dartPluginPreloaded(); }
  double wristJointPosition(int i) const { return world_.wristJointPosition(0, i); }
  double wristAxisTorque(int i) const { return world_.wristAxisTorque(0, i); }
  long nanWristCount() const { return world_.nanWristCount(0); }
  bool depthCameraDisabled() const { return world_.depthCameraDisabled(); }

 private:
  GazeboWorld world_;
};

}  // namespace gsim
