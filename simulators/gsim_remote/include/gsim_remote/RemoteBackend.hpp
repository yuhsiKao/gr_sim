// A backend whose physics runs in another process: gsim_server, reached over a unix socket
// (SimConfig::server_path). Every step() is one round trip: the torques and the external force
// go out, the state comes back. The server decides how fast simulated time runs, pauses and
// owns the window; this side only waits for it.
//
// The server's world may hold several robots; this backend is one of them (SimConfig::robot_index).
#pragma once
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "gsim_core/sim_backend.hpp"
#include "gsim_remote/protocol.hpp"

namespace gsim::remote {

class RemoteBackend final : public ISimBackend {
 public:
  ~RemoteBackend() override;

  bool init(const SimConfig& cfg) override;
  // init() on a socket that is already connected (takes ownership of fd). For tests.
  bool initFromFd(int fd, const SimConfig& cfg);

  bool loadRobot(const RobotSpec& spec) override;
  void resetJoint(int sdk_joint, float q) override;
  // Returns once the server has stepped, or when the server is gone, has stopped its simulation
  // or SimConfig::stop_requested says this process is stopping; running() is false after those.
  void step() override;
  void readState(BaseState& base, Eigen::ArrayXf& q, Eigen::ArrayXf& dq) override;
  void setJointTorques(const Eigen::ArrayXf& tau) override;
  bool running() const override { return alive_; }
  double timestep() const override { return timestep_; }
  void applyExternalForce(const Eigen::Vector3f& f_B, const Eigen::Vector3f& p_B) override;
  // Arm and depth camera: whatever the server's world supports. The arm command travels with the
  // next step() and the wrist state comes back with it; a depth frame is one more round trip,
  // every kStepsPerFrame calls, or on the steps of the world's own camera schedule if it has one.
  // The step() after a call of the depth camera tells the world that this robot wants frames.
  bool supportsArm() const override { return supports_arm_; }
  void setArmLocked(bool locked) override;
  void setWristCommand(const WristCommand& cmd) override;
  void readWrist(WristState& w) override;
  bool supportsDepthCamera() const override { return supports_depth_; }
  bool updateDepthCamera(PointCloud& cloud_B) override;

 private:
  void log(int level, const std::string& msg) const;
  void fail(const std::string& msg);   // logs at kError and ends the connection
  // Waits for a message of this type (total_ms < 0: for as long as it takes). False, with the
  // connection ended, when it does not come.
  bool waitFor(MsgType want, int total_ms, std::vector<uint8_t>* payload);

  int fd_ = -1;
  LogFn log_;
  std::function<bool()> stop_requested_;
  bool alive_ = false;
  double timestep_ = 0.0;
  int num_robots_ = 1;   // of the server's world
  int num_joints_ = 0;
  bool supports_arm_ = false, supports_depth_ = false;
  bool depth_on_schedule_ = false;   // the world renders its cameras on a schedule of its own
  bool has_arm_ = false;   // the loaded robot has an arm
  long depth_calls_ = 0;   // frame counter as Ghost's SimWrapper
  Step out_{};
  State last_{};
  std::vector<uint8_t> buf_;
};

}  // namespace gsim::remote

namespace gsim {
using remote::RemoteBackend;
}
