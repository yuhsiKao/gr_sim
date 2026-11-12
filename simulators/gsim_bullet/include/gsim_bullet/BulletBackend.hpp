// ISimBackend over the SDK's Bullet wrapper (share/gr/bullet3/SimWrapper.cpp,
// compiled from the SDK path unchanged). SimWrapper.h has no include guard and
// pulls in Bullet headers, so it stays behind a pimpl in the .cpp.
#pragma once
#include <memory>

#include "gsim_core/sim_backend.hpp"

namespace gsim {

class BulletBackend : public ISimBackend {
 public:
  BulletBackend();
  ~BulletBackend() override;

  bool init(const SimConfig& cfg) override;
  bool loadRobot(const RobotSpec& spec) override;
  void resetJoint(int sdk_joint, float q) override;
  void step() override;
  void readState(BaseState& base, Eigen::ArrayXf& q, Eigen::ArrayXf& dq) override;
  void setJointTorques(const Eigen::ArrayXf& tau) override;
  bool running() const override;
  double timestep() const override;
  void applyExternalForce(const Eigen::Vector3f& f_B, const Eigen::Vector3f& p_B) override;
  bool supportsArm() const override { return true; }
  void setArmLocked(bool locked) override;
  void setWristCommand(const WristCommand& cmd) override;
  void readWrist(WristState& w) override;
  bool supportsDepthCamera() const override { return true; }
  bool updateDepthCamera(PointCloud& cloud_B) override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gsim
