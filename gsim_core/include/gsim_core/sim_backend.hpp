// Simulator-agnostic interface between the generic Ghost SDK platform layer
// (sim_bsp.cpp) and a physics engine. Conventions (world frame, quat_WB, SDK
// joint indices 0..N-1, NaN for missing joints) are fixed here; every backend
// converts its engine's conventions to these.
#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <atomic>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "gsim_core/depth_camera.hpp"

namespace gsim {

enum LogLevel : int { kInfo = 0, kWarning = 1, kError = 2 };
using LogFn = std::function<void(int level, const std::string& msg)>;

// Rate limit for a warning that can repeat every step: true on the 1st, 1000th, 2000th, ... call.
inline bool shouldReport(long& count) { return ++count == 1 || count % 1000 == 0; }

struct SimConfig {
  bool headless = false;
  std::string scene_path;                       // MuJoCo only
  std::string world_path;                       // Gazebo only (gsim_world)
  bool depth_camera = false;                    // attach the synthetic depth camera (Gazebo; MuJoCo and Bullet always have theirs)
  std::string urdf_dir;                         // absolute, e.g. $SDK/share/gr/urdf
  int argc = 0;
  char** argv = nullptr;                        // Bullet GUI needs these
  LogFn log;                                    // backends must not print directly
  std::array<double, 3> toe_friction{3.0, 0.04, 0.0};
  std::array<double, 2> toe_solref{0.02, 1.0};
  std::string server_path;                      // remote only (gsim_server): the physics server's unix socket
  std::function<bool()> stop_requested;         // remote only: true once the process is stopping (may be empty)
  int robot_index = 0;                          // remote only (gsim_robot): which robot of the server's world this is
};

struct RobotSpec {
  std::string urdf_path;                        // absolute
  int num_sdk_joints = 12;
  bool fixed_base = false;
  Eigen::Vector3f pos0_W{0.f, 0.f, 0.4f};
  Eigen::Vector3f rpy0{0.f, 0.f, 0.f};
  std::vector<int> arm_joints;                  // SDK ids of the arm joints, root to tip; empty = no arm
};

// Wrist command modes the SDK supports (WristCmdLL::cmd_mode); same numbers as the SDK's
// LimbCmdMode so that logs read the same as the SDK.
enum class WristMode : int { kOff = 0, kCurrent = 2, kPositionOverCurrent = 4 };

// 0, 2, 4 -> the matching mode; any other SDK value -> nullopt (not supported for the wrist).
std::optional<WristMode> wristModeFromSdkValue(int v);

// Wrist joints in the SDK's order: r1, p, r2, gripper. A simulator mirrors extra joints
// (e.g. Vision 60's second jaw) itself.
struct WristCommand {
  WristMode mode = WristMode::kOff;
  Eigen::Array4f pos_des = Eigen::Array4f::Zero();
  Eigen::Array4f vel_des = Eigen::Array4f::Zero();
  Eigen::Array4f max_current = Eigen::Array4f::Zero();   // force limit (mode 4) or torque (mode 2)
};

// A joint whose command has a non-finite value (pos_des, vel_des or max_current) gets no force:
// its pos_des, vel_des and max_current become 0, in every mode. Returns the number of such joints.
int sanitizeWristCommand(WristCommand& cmd);
// sanitizeWristCommand plus the rate-limited warning every backend logs for it: the message to
// log at kWarning, or "" when there is nothing to report this call.
std::string wristCommandWarning(WristCommand& cmd, long& nan_count, const std::string& backend);

struct WristState {
  Eigen::Array4f q = Eigen::Array4f::Constant(std::numeric_limits<float>::quiet_NaN());
  Eigen::Array4f dq = Eigen::Array4f::Constant(std::numeric_limits<float>::quiet_NaN());
  // Mode 4: the force the servo applied; mode 2: the commanded max_current; off: 0 (as Ghost's Bullet sim).
  Eigen::Array4f current = Eigen::Array4f::Constant(std::numeric_limits<float>::quiet_NaN());
};

struct BaseState {                              // all world frame
  Eigen::Vector3f pos_W = Eigen::Vector3f::Zero();
  Eigen::Vector3f vel_W = Eigen::Vector3f::Zero();
  Eigen::Quaternionf quat_WB = Eigen::Quaternionf::Identity();
  Eigen::Vector3f angvel_W = Eigen::Vector3f::Zero();
};

class ISimBackend {
 public:
  virtual ~ISimBackend() = default;
  virtual bool init(const SimConfig& cfg) = 0;
  virtual bool loadRobot(const RobotSpec& spec) = 0;
  virtual void resetJoint(int sdk_joint, float q) = 0;
  virtual void step() = 0;
  virtual void readState(BaseState& base, Eigen::ArrayXf& q, Eigen::ArrayXf& dq) = 0;
  virtual void setJointTorques(const Eigen::ArrayXf& tau) = 0;   // effective at next step()
  virtual bool running() const = 0;
  virtual double timestep() const = 0;
  virtual bool paused() const { return false; }
  virtual bool wantsMainThreadViewer() const { return false; }
  virtual void runViewer(std::atomic_bool& /*keep_alive*/) {}
  virtual void applyExternalForce(const Eigen::Vector3f& /*f_B*/, const Eigen::Vector3f& /*p_B*/) {}
  virtual std::vector<std::string> selfTest(const RobotSpec&) { return {}; }

  // Arm (RobotSpec::arm_joints not empty). The defaults mean "no arm support". The arm is
  // locked after loadRobot().
  virtual bool supportsArm() const { return false; }
  virtual void setArmLocked(bool /*locked*/) {}                  // effective at next step()
  virtual void setWristCommand(const WristCommand& /*cmd*/) {}   // effective at next step(); send every step
  virtual void readWrist(WristState& /*w*/) {}                   // state after the last step()

  // Synthetic depth camera for the SDK's terrain perception (FactoryDepthCamera). Call once per
  // physics step, after readState() of that step (Bullet places the camera at the pose read there),
  // while the SDK has its cameras on; every kStepsPerFrame calls (first on the 66th, as Ghost's
  // SimWrapper) it renders, writes the body-frame cloud in Ghost's pixel order and returns true.
  // Other calls return false and leave cloud_B alone.
  virtual bool supportsDepthCamera() const { return false; }
  virtual bool updateDepthCamera(PointCloud& /*cloud_B*/) { return false; }
};

// Empty if the backend can run this robot, otherwise the message ghost_sim reports.
std::string armSupportError(const ISimBackend& backend, bool has_arm, const std::string& backend_name);

}  // namespace gsim
