// A physics world that holds several robots. ISimBackend is the single-robot view the SDK
// platform layer uses; a world is what the physics server owns, with one more argument (the
// robot's index, 0-based) on everything that belongs to a robot. Conventions are those of
// sim_backend.hpp.
#pragma once
#include <Eigen/Core>
#include <atomic>

#include "gsim_core/sim_backend.hpp"

namespace gsim {

// Most robots one world holds.
inline constexpr int kMaxRobots = 8;

class ISimWorld {
 public:
  virtual ~ISimWorld() = default;
  virtual bool init(const SimConfig& cfg) = 0;
  // Robots are added first, then build() makes the model. Returns the robot's index, or -1 when
  // the world is full or already built.
  virtual int addRobot(const RobotSpec& spec) = 0;
  virtual bool build() = 0;                      // false: the world has no robots again
  virtual void clear() = 0;                      // drops the model and the robots; addRobot works again
  virtual int numRobots() const = 0;
  virtual void step() = 0;                       // the whole world, one physics step
  // A robot index out of range is ignored by the calls below.
  virtual void resetJoint(int robot, int sdk_joint, float q) = 0;
  virtual void readState(int robot, BaseState& base, Eigen::ArrayXf& q, Eigen::ArrayXf& dq) = 0;
  virtual void setJointTorques(int robot, const Eigen::ArrayXf& tau) = 0;   // effective at next step()
  virtual void applyExternalForce(int robot, const Eigen::Vector3f& f_B, const Eigen::Vector3f& p_B) = 0;
  // Arm and depth camera of one robot, as in ISimBackend. The defaults mean "not supported".
  virtual bool supportsArm() const { return false; }
  virtual void setArmLocked(int /*robot*/, bool /*locked*/) {}                    // effective at next step()
  virtual void setWristCommand(int /*robot*/, const WristCommand& /*cmd*/) {}     // effective at next step()
  virtual void readWrist(int /*robot*/, WristState& /*w*/) {}                     // state after the last step()
  virtual bool supportsDepthCamera() const { return false; }
  // Renders the robot's depth camera now, in the state after the last step(): the body-frame
  // cloud in Ghost's pixel order. False (cloud_B left alone) when it cannot render.
  virtual bool renderDepthCamera(int /*robot*/, PointCloud& /*cloud_B*/) { return false; }
  virtual bool running() const = 0;
  virtual double timestep() const = 0;
  virtual bool paused() const { return false; }
  virtual bool wantsMainThreadViewer() const { return false; }
  virtual void runViewer(std::atomic_bool& /*keep_alive*/) {}
};

}  // namespace gsim
