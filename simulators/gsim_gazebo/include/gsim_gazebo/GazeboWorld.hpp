// ISimWorld on Gazebo Harmonic: one gz::sim::Server with every robot in it, each a model converted
// from Ghost's URDF with its own GazeboSystem (commands in, state out) and its own depth camera.
// GazeboBackend is this world with one robot. Gazebo's headers stay behind a pimpl.
#pragma once
#include <memory>
#include <string>
#include <vector>

#include "gsim_core/sim_world.hpp"

namespace gsim {

class GazeboWorld : public ISimWorld {
 public:
  GazeboWorld();
  ~GazeboWorld() override;

  bool init(const SimConfig& cfg) override;
  int addRobot(const RobotSpec& spec) override;
  bool build() override;
  void clear() override;
  int numRobots() const override;
  void step() override;
  void resetJoint(int robot, int sdk_joint, float q) override;
  void readState(int robot, BaseState& base, Eigen::ArrayXf& q, Eigen::ArrayXf& dq) override;
  void setJointTorques(int robot, const Eigen::ArrayXf& tau) override;
  void applyExternalForce(int robot, const Eigen::Vector3f& f_B, const Eigen::Vector3f& p_B) override;
  bool running() const override;   // false once the world was reset behind the simulation's back
  double timestep() const override;
  bool paused() const override;
  bool wantsCpuLatencyHold() const override { return true; }   // a step takes 0.1 to 0.3 ms

  bool supportsArm() const override { return true; }
  void setArmLocked(int robot, bool locked) override;
  void setWristCommand(int robot, const WristCommand& cmd) override;
  void readWrist(int robot, WristState& w) override;

  // Depth cameras (SimConfig::depth_camera): Gazebo renders a camera on its sim-time schedule,
  // every FactoryDepthCamera::kStepsPerFrame steps (depthFrameAtLastStep says which steps those
  // are), and only for a robot that wants frames: one that said so (wantDepthFrames) within the
  // last kStepsPerFrame steps. renderDepthCamera gives such a robot the newest frame rendered for
  // it (that step's on a step of the schedule, at most kStepsPerFrame - 1 steps old otherwise).
  // updateDepthCamera is ISimBackend's call for one robot: it says that the robot wants frames and
  // is true only on the steps of the schedule.
  bool supportsDepthCamera() const override;
  bool renderDepthCamera(int robot, PointCloud& cloud_B) override;
  bool depthOnOwnSchedule() const override { return true; }
  bool depthFrameAtLastStep() const override;
  void wantDepthFrames(int robot) override;
  bool updateDepthCamera(int robot, PointCloud& cloud_B);

  // Problems with spec in this world (empty = ok), without starting Gazebo.
  std::vector<std::string> selfTest(const RobotSpec& spec) const;

  // Diagnostics / tests.
  long nanTorqueCount() const;
  double simTime() const;   // [s] of the last update (0 before the first step)
  int guiPid() const;       // pid of the `gz sim -g` window process, 0 if none
  bool dartPluginPreloaded() const;
  double wristJointPosition(int robot, int i) const;   // i = 0..4 (jwrist_r1, p, r2, g, g2); NaN without an arm
  double wristAxisTorque(int robot, int i) const;      // i = 0..3
  long nanWristCount(int robot) const;
  bool depthCameraDisabled() const;    // the cameras gave up (no offscreen rendering)
  bool depthCameraOn(int robot) const; // Gazebo renders this robot's camera: the robot wants frames
  bool bodySlab(int robot) const;      // the robot's body is a slab in the sensor scene (see gazebo_system.cpp)

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gsim
