#include "gsim_remote/RemoteBackend.hpp"

#include <algorithm>
#include <cstring>
#include <limits>

namespace gsim::remote {

namespace {

constexpr int kSliceMs = 100;   // how often a wait looks at stop_requested
constexpr int kWelcomeTimeoutMs = 5000;
const std::string kSameBuild = "(are ghost_sim and gsim_server the same build?)";

}  // namespace

RemoteBackend::~RemoteBackend() {
  if (alive_) sendMsg(fd_, MsgType::kBye, nullptr, 0);
  closeFd(fd_);
}

void RemoteBackend::log(int level, const std::string& msg) const {
  if (log_) log_(level, "remote: " + msg);
}

void RemoteBackend::fail(const std::string& msg) {
  log(kError, msg);
  alive_ = false;
}

bool RemoteBackend::init(const SimConfig& cfg) {
  log_ = cfg.log;
  if (cfg.server_path.empty()) {
    log(kError, "no socket path for gsim_server (parameter gsim_server)");
    return false;
  }
  std::string err;
  const int fd = connectUnix(cfg.server_path, &err);
  if (fd < 0) {
    log(kError, "cannot reach gsim_server at " + cfg.server_path + ": " + err);
    return false;
  }
  if (!initFromFd(fd, cfg)) return false;
  log(kInfo, "connected to gsim_server at " + cfg.server_path);
  return true;
}

bool RemoteBackend::initFromFd(int fd, const SimConfig& cfg) {
  fd_ = fd;
  log_ = cfg.log;
  stop_requested_ = cfg.stop_requested;
  alive_ = true;
  Welcome w{};
  if (!sendMsg(fd_, MsgType::kHello, Hello{static_cast<uint32_t>(cfg.robot_index)}) || !waitFor(MsgType::kWelcome, kWelcomeTimeoutMs, &buf_)) {
    if (alive_) fail("gsim_server did not answer");
    return false;
  }
  if (!payloadAs(buf_, &w)) {
    fail("bad answer from gsim_server " + kSameBuild);
    return false;
  }
  timestep_ = w.timestep;
  num_robots_ = static_cast<int>(w.num_robots);
  supports_arm_ = w.supports_arm != 0;
  supports_depth_ = w.supports_depth != 0;
  depth_on_schedule_ = w.depth_on_schedule != 0;
  return true;
}

bool RemoteBackend::waitFor(MsgType want, int total_ms, std::vector<uint8_t>* payload) {
  for (int waited = 0; alive_ && (total_ms < 0 || waited < total_ms); waited += kSliceMs) {
    MsgType type{};
    switch (recvMsg(fd_, kSliceMs, &type, payload)) {
      case Recv::kTimeout:
        if (stop_requested_ && stop_requested_()) {
          sendMsg(fd_, MsgType::kBye, nullptr, 0);
          alive_ = false;   // this process is stopping: not an error
          return false;
        }
        continue;
      case Recv::kClosed:
        fail("gsim_server closed the connection");
        return false;
      case Recv::kBad:
        fail("bad message from gsim_server " + kSameBuild);
        return false;
      case Recv::kOk:
        break;
    }
    if (type == want) return true;
    Error e{};
    if (type == MsgType::kError && payloadAs(*payload, &e))
      fail("gsim_server reported: " + getText(e.text));
    else if (type == MsgType::kState)   // not asked for: the server's simulation has stopped
      alive_ = false, log(kInfo, "gsim_server stopped the simulation");
    else
      fail("unexpected message " + std::to_string(static_cast<int>(type)) + " from gsim_server");
    return false;
  }
  if (alive_) fail("gsim_server did not answer within " + std::to_string(total_ms / 1000) + " s");
  return false;
}

bool RemoteBackend::loadRobot(const RobotSpec& spec) {
  if (!alive_) return false;
  if (!spec.arm_joints.empty() && !supports_arm_) {
    log(kError, "the physics of gsim_server has no arm support");
    return false;
  }
  if (spec.arm_joints.size() > static_cast<size_t>(kMaxArmJoints)) {
    log(kError, std::to_string(spec.arm_joints.size()) + " arm joints: the protocol carries at most " +
                    std::to_string(kMaxArmJoints));
    return false;
  }
  const std::string file = spec.urdf_path.substr(spec.urdf_path.find_last_of('/') + 1);
  LoadRobot m{};
  if (file.empty() || file.size() >= sizeof(m.urdf_file)) {
    log(kError, "URDF file name '" + file + "' is empty or too long");
    return false;
  }
  if (spec.num_sdk_joints < 1 || spec.num_sdk_joints > kMaxJoints) {
    log(kError, std::to_string(spec.num_sdk_joints) + " joints: the protocol carries 1.." + std::to_string(kMaxJoints));
    return false;
  }
  setText(m.urdf_file, file);
  m.num_sdk_joints = spec.num_sdk_joints;
  m.fixed_base = spec.fixed_base;
  Eigen::Map<Eigen::Vector3f>(m.pos0_W) = spec.pos0_W;
  Eigen::Map<Eigen::Vector3f>(m.rpy0) = spec.rpy0;
  m.num_arm_joints = static_cast<int32_t>(spec.arm_joints.size());
  std::copy(spec.arm_joints.begin(), spec.arm_joints.end(), m.arm_joints);
  Loaded l{};
  // The server answers once every robot of its world has a controller, however long that takes.
  if (num_robots_ > 1) log(kInfo, "waiting until all " + std::to_string(num_robots_) + " robots have joined gsim_server");
  if (!sendMsg(fd_, MsgType::kLoadRobot, m) || !waitFor(MsgType::kLoaded, -1, &buf_)) {
    if (alive_) fail("gsim_server closed the connection");
    return false;
  }
  if (!payloadAs(buf_, &l)) {
    fail("bad answer from gsim_server " + kSameBuild);
    return false;
  }
  if (!l.ok) {
    log(kError, "gsim_server: " + getText(l.error));
    return false;
  }
  num_joints_ = spec.num_sdk_joints;
  has_arm_ = !spec.arm_joints.empty();
  out_.arm_locked = 1;   // as every backend: locked after loadRobot()
  depth_calls_ = 0;
  for (float* a : {last_.wrist_q, last_.wrist_dq, last_.wrist_current}) std::fill(a, a + 4, std::numeric_limits<float>::quiet_NaN());
  std::fill(std::begin(last_.q), std::end(last_.q), std::numeric_limits<float>::quiet_NaN());
  std::fill(std::begin(last_.dq), std::end(last_.dq), std::numeric_limits<float>::quiet_NaN());
  last_.quat_WB[0] = 1.f;
  Eigen::Map<Eigen::Vector3f>(last_.pos_W) = spec.pos0_W;
  return true;
}

void RemoteBackend::resetJoint(int sdk_joint, float q) {
  if (alive_ && !sendMsg(fd_, MsgType::kResetJoint, ResetJoint{sdk_joint, q})) fail("gsim_server closed the connection");
}

void RemoteBackend::setJointTorques(const Eigen::ArrayXf& tau) {
  out_.n_tau = static_cast<int32_t>(std::min<Eigen::Index>(tau.size(), kMaxJoints));
  std::copy(tau.data(), tau.data() + out_.n_tau, out_.tau);
}

void RemoteBackend::applyExternalForce(const Eigen::Vector3f& f_B, const Eigen::Vector3f& p_B) {
  Eigen::Map<Eigen::Vector3f>(out_.f_B) = f_B;
  Eigen::Map<Eigen::Vector3f>(out_.p_B) = p_B;
}

void RemoteBackend::setArmLocked(bool locked) { out_.arm_locked = locked; }

void RemoteBackend::setWristCommand(const WristCommand& cmd) {
  out_.wrist_mode = static_cast<int32_t>(cmd.mode);
  Eigen::Map<Eigen::Array4f>(out_.wrist_pos_des) = cmd.pos_des;
  Eigen::Map<Eigen::Array4f>(out_.wrist_vel_des) = cmd.vel_des;
  Eigen::Map<Eigen::Array4f>(out_.wrist_max_current) = cmd.max_current;
}

void RemoteBackend::readWrist(WristState& w) {
  if (!has_arm_) return;
  w.q = Eigen::Map<const Eigen::Array4f>(last_.wrist_q);
  w.dq = Eigen::Map<const Eigen::Array4f>(last_.wrist_dq);
  w.current = Eigen::Map<const Eigen::Array4f>(last_.wrist_current);
}

// Ghost's SimWrapper::perceptionUpdate: a new frame every kStepsPerFrame calls, the first on the
// kStepsPerFrame-th. Only then is the server asked; it renders the state after the last step().
// A world with a camera schedule of its own is asked on the steps of that schedule instead, which
// is when its newest frame is the one of the last step().
bool RemoteBackend::updateDepthCamera(PointCloud& cloud_B) {
  if (!alive_ || !supports_depth_) return false;
  out_.wants_depth = 1;   // goes with the next step
  if (depth_on_schedule_ ? !last_.depth_frame : !depthFrameDue(depth_calls_)) return false;
  sendMsg(fd_, MsgType::kRenderDepth, nullptr, 0);
  if (!waitFor(MsgType::kCloud, -1, &buf_)) return false;
  if (buf_.empty()) return false;   // the server could not render
  if (buf_.size() % (3 * sizeof(float)) != 0) {
    fail("bad point cloud from gsim_server " + kSameBuild);
    return false;
  }
  cloud_B.resize(static_cast<Eigen::Index>(buf_.size() / (3 * sizeof(float))), 3);
  std::memcpy(cloud_B.data(), buf_.data(), buf_.size());
  return true;
}

void RemoteBackend::step() {
  if (!alive_) return;
  ++out_.seq;
  // A failed send is not reported here: the server's last message (why it left) may still be
  // waiting to be read, and waitFor reports the closed connection otherwise.
  sendMsg(fd_, MsgType::kStep, out_);
  out_.wants_depth = 0;   // until the depth camera is called again
  if (!waitFor(MsgType::kState, -1, &buf_)) return;
  State s{};
  if (!payloadAs(buf_, &s)) return fail("bad state from gsim_server " + kSameBuild);
  if (!s.running) {
    alive_ = false;
    log(kInfo, "gsim_server stopped the simulation");
    return;
  }
  if (s.seq != out_.seq)
    return fail("gsim_server answered step " + std::to_string(s.seq) + ", expected " + std::to_string(out_.seq));
  last_ = s;
}

void RemoteBackend::readState(BaseState& base, Eigen::ArrayXf& q, Eigen::ArrayXf& dq) {
  base.pos_W = Eigen::Map<const Eigen::Vector3f>(last_.pos_W);
  base.vel_W = Eigen::Map<const Eigen::Vector3f>(last_.vel_W);
  base.quat_WB = Eigen::Quaternionf(last_.quat_WB[0], last_.quat_WB[1], last_.quat_WB[2], last_.quat_WB[3]);
  base.angvel_W = Eigen::Map<const Eigen::Vector3f>(last_.angvel_W);
  q = Eigen::Map<const Eigen::ArrayXf>(last_.q, num_joints_);
  dq = Eigen::Map<const Eigen::ArrayXf>(last_.dq, num_joints_);
}

}  // namespace gsim::remote
