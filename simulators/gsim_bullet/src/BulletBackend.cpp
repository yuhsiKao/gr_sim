#include "gsim_bullet/BulletBackend.hpp"

#include "SimWrapper.h"  // from $SDK/share/gr/bullet3; no include guard, include once here only
#include "gsim_core/sim_math.hpp"

namespace gsim {

// WristMode uses the SDK's LimbCmdMode numbers (gsim_core cannot include the SDK); setWristCommand casts.
static_assert(static_cast<int>(WristMode::kOff) == LimbCmdMode_LIMB_OFF &&
                  static_cast<int>(WristMode::kCurrent) == LimbCmdMode_JOINT_CURRENT &&
                  static_cast<int>(WristMode::kPositionOverCurrent) == LimbCmdMode_JOINT_POSITION_OVER_CURRENT,
              "WristMode must use the SDK's LimbCmdMode numbers");

struct BulletBackend::Impl {
  gr::SimWrapper sim{2000};
  SimConfig cfg;
  bool connected = false;
  bool failed = false;     // getAllStates/inputUpdate reported an inaccessible server
  int robot_id = -1;
  int num_joints = 0;
  bool has_arm = false;
  long nan_wrist_count = 0;
  // SimWrapper's gripper camera output (unused here; perceptionUpdate needs somewhere to write it).
  std::shared_ptr<gr::PointCloud_t> gripper_cloud = std::make_shared<gr::PointCloud_t>();
  gr::KinematicsData_t kinematics{};
  LimbCmdMode wrist_mode = LimbCmdMode_LIMB_OFF;   // mode of the last wrist command (for the current report)

  void log(int level, const std::string& msg) const {
    if (cfg.log) cfg.log(level, msg);
  }
};

BulletBackend::BulletBackend() : impl_(std::make_unique<Impl>()) {}

BulletBackend::~BulletBackend() {
  if (impl_->connected) impl_->sim.disconnect();
}

bool BulletBackend::init(const SimConfig& cfg) {
  impl_->cfg = cfg;
  // Must precede init(): its automatic search would pick $PROJ/install/... because $PROJ has src/.
  impl_->sim.setURDFPath(cfg.urdf_dir);
  if (impl_->sim.init(cfg.headless, cfg.argc, cfg.argv) != 0) {
    impl_->log(kError, "Bullet: could not create the physics server");
    return false;
  }
  impl_->connected = true;
  return true;
}

bool BulletBackend::loadRobot(const RobotSpec& spec) {
  impl_->robot_id = impl_->sim.loadRobot(spec.urdf_path, spec.num_sdk_joints, spec.fixed_base, spec.pos0_W.z(),
                                         spec.rpy0.x(), spec.rpy0.y(), spec.rpy0.z(), 0);
  impl_->num_joints = spec.num_sdk_joints;
  impl_->has_arm = false;
  impl_->wrist_mode = LimbCmdMode_LIMB_OFF;
  if (impl_->robot_id < 0) {
    impl_->log(kError, "Bullet: failed to load " + spec.urdf_path);
    return false;
  }
  if (!spec.arm_joints.empty()) {
    // SimWrapper handles the arm itself: resetArm() locks it at 0 rad (as Ghost's bsp.cpp) and
    // loadRobot() set up the wrist when the URDF has jwrist_p.
    for (int id : spec.arm_joints)
      if (id < 0 || id >= spec.num_sdk_joints || impl_->sim.bJointId(id) <= 0) {
        impl_->log(kError, "Bullet: arm joint " + std::to_string(id) + " is not in " + spec.urdf_path);
        return false;
      }
    if (impl_->sim.wristPos().size() == 0) {
      impl_->log(kError, "Bullet: the arm has no wrist (jwrist_p) in " + spec.urdf_path);
      return false;
    }
    impl_->sim.resetArm(Eigen::Map<const Eigen::ArrayXi>(spec.arm_joints.data(), spec.arm_joints.size()));
    impl_->has_arm = true;
    return true;
  }
  // SimWrapper sends a torque for every SDK joint slot, including slots with no joint in
  // the URDF (Vision 60's 3 arm slots, Bullet index -1); Bullet applied those to the last
  // listed joint, so the arm slots' torques replaced joint 11's. Ghost's bsp.cpp avoids it
  // with resetArm(), which drops the trailing arm slots from the torque command.
  int n_present = spec.num_sdk_joints;
  while (n_present > 0 && impl_->sim.bJointId(n_present - 1) <= 0) --n_present;
  for (int j = 0; j < n_present; ++j)
    if (impl_->sim.bJointId(j) <= 0)
      impl_->log(kWarning, "Bullet: SDK joint " + std::to_string(j) + " has no joint in the URDF; its torque is misapplied");
  if (n_present < spec.num_sdk_joints)
    impl_->sim.resetArm(Eigen::ArrayXi::LinSpaced(spec.num_sdk_joints - n_present, n_present, spec.num_sdk_joints - 1));
  return true;
}

void BulletBackend::resetJoint(int sdk_joint, float q) {
  // SimWrapper stores Bullet index + 1 (SimWrapper.cpp:349) and bJointId() returns it
  // unadjusted; the SDK's own bsp.cpp:65 therefore resets the next Bullet joint.
  impl_->sim.resetJointState(impl_->robot_id, impl_->sim.bJointId(sdk_joint) - 1, q);
}

void BulletBackend::step() { impl_->sim.stepSimulation(); }

void BulletBackend::readState(BaseState& base, Eigen::ArrayXf& q, Eigen::ArrayXf& dq) {
  // The mode decides what SimWrapper reports as wrist current (motor torque in position mode).
  if (!impl_->sim.getAllStates(impl_->wrist_mode) || !impl_->sim.inputUpdate()) {
    impl_->failed = true;
    return;
  }
  const Eigen::Matrix<float, 6, 1> orn = impl_->sim.ornState();  // roll, pitch, yaw, angular velocity (world)
  const Eigen::Matrix<float, 6, 1> x = impl_->sim.x_W();         // position, linear velocity (world)
  base.quat_WB = quatFromRpy(orn.head<3>());
  base.angvel_W = orn.tail<3>();
  base.pos_W = x.head<3>();
  base.vel_W = x.tail<3>();
  q = impl_->sim.jointPos();
  dq = impl_->sim.jointVel();
}

void BulletBackend::setArmLocked(bool locked) {
  if (impl_->has_arm) impl_->sim.armUnlock(!locked);
}

void BulletBackend::setWristCommand(const WristCommand& cmd) {
  if (!impl_->has_arm) return;
  WristCommand c = cmd;
  if (const std::string w = wristCommandWarning(c, impl_->nan_wrist_count, "Bullet"); !w.empty()) impl_->log(kWarning, w);
  // WristMode uses the SDK's LimbCmdMode numbers.
  impl_->wrist_mode = static_cast<LimbCmdMode>(static_cast<int>(c.mode));
  impl_->sim.sendWristCommand(impl_->wrist_mode, c.pos_des, c.vel_des, c.max_current);
}

void BulletBackend::readWrist(WristState& w) {
  if (!impl_->has_arm) return;
  // SimWrapper has 5 wrist joints (the 5th mirrors the gripper); the SDK sees the first 4.
  w.q = impl_->sim.wristPos().head<4>();
  w.dq = impl_->sim.wristVel().head<4>();
  w.current = impl_->sim.wristCurrent().head<4>();
}

// SimWrapper::perceptionUpdate keeps Ghost's own frame counter and returns 1 for a new depth cloud,
// which it writes straight into cloud_B (a non-owning pointer, as Ghost's bsp passes the SDK's cloud);
// on the other calls it leaves cloud_B alone.
bool BulletBackend::updateDepthCamera(PointCloud& cloud_B) {
  if (!impl_->connected || impl_->robot_id < 0) return false;
  const std::shared_ptr<gr::PointCloud_t> out(std::shared_ptr<gr::PointCloud_t>{}, &cloud_B);
  return impl_->sim.perceptionUpdate(out, impl_->gripper_cloud, impl_->kinematics) == 1;
}

void BulletBackend::setJointTorques(const Eigen::ArrayXf& tau) { impl_->sim.sendTorqueCommands(tau); }

bool BulletBackend::running() const { return impl_->connected && !impl_->failed && impl_->sim.serverAccessible(); }

double BulletBackend::timestep() const { return impl_->sim.FIXED_TIMESTEP; }

void BulletBackend::applyExternalForce(const Eigen::Vector3f& f_B, const Eigen::Vector3f& p_B) {
  impl_->sim.applyExtForceBody(f_B, p_B);
}

}  // namespace gsim
