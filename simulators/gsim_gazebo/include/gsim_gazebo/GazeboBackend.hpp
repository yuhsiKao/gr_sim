// ISimBackend on Gazebo Harmonic: a gz::sim::Server inside ghost_sim, stepped one iteration at
// a time by SimLoop (Server::RunOnce), with Ghost's URDF converted to SDF in memory. Legs and
// the arm (Bullet-style lock and wrist servo as DART servo joints). Gazebo's headers stay behind
// a pimpl.
#pragma once
#include <memory>
#include <string>
#include <vector>

#include "gsim_core/sim_backend.hpp"

namespace gsim {

class GazeboBackend : public ISimBackend {
 public:
  GazeboBackend();
  ~GazeboBackend() override;

  bool init(const SimConfig& cfg) override;
  bool loadRobot(const RobotSpec& spec) override;
  void resetJoint(int sdk_joint, float q) override;
  void step() override;
  void readState(BaseState& base, Eigen::ArrayXf& q, Eigen::ArrayXf& dq) override;
  void setJointTorques(const Eigen::ArrayXf& tau) override;
  bool running() const override;
  double timestep() const override;
  bool paused() const override;
  void applyExternalForce(const Eigen::Vector3f& f_B, const Eigen::Vector3f& p_B) override;
  std::vector<std::string> selfTest(const RobotSpec& spec) override;

  // Arm (RobotSpec::arm_joints not empty).
  bool supportsArm() const override { return true; }
  void setArmLocked(bool locked) override;
  void setWristCommand(const WristCommand& cmd) override;
  void readWrist(WristState& w) override;

  // Synthetic depth camera (SimConfig::depth_camera): Ghost's downward camera as a Gazebo
  // depth_camera sensor; frames come on the sim-time grid of FactoryDepthCamera::kStepsPerFrame steps.
  bool supportsDepthCamera() const override;
  bool updateDepthCamera(PointCloud& cloud_B) override;

  // Diagnostics / tests.
  long nanTorqueCount() const;
  double simTime() const;   // [s] of the last update (0 before the first step)
  int guiPid() const;       // pid of the `gz sim -g` window process, 0 if none
  bool dartPluginPreloaded() const;   // the DART plugin was loaded with its own symbol scope (see the .cpp)
  double wristJointPosition(int i) const;   // i = 0..4 (jwrist_r1, p, r2, g, g2); NaN without an arm
  double wristAxisTorque(int i) const;      // i = 0..3: the transmitted wrench's torque about the joint axis (motor + damping)
  long nanWristCount() const;
  bool depthCameraDisabled() const;    // the camera gave up (no offscreen rendering); see the .cpp

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gsim
