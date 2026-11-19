#include "gsim_remote/WorldServer.hpp"

#include <poll.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace gsim::remote {

namespace {

constexpr int kIdleSliceMs = 100;   // how often an idle server looks at keep_alive and the world
constexpr int kPauseSliceMs = 10;
constexpr int kPaceSteps = 4;       // pace every 4 steps (2 ms of simulated time at 2000 Hz)
constexpr double kWaitingReportS = 5.0;

double wallSeconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string robotName(int robot) { return "robot " + std::to_string(robot); }

}  // namespace

WorldServer::WorldServer(ISimWorld& world, int num_robots, std::string urdf_dir, LogFn log, double rtf)
    : world_(world), num_robots_(num_robots), urdf_dir_(std::move(urdf_dir)), log_(std::move(log)), pacer_(rtf) {}

WorldServer::~WorldServer() {
  for (const Client& c : clients_) closeFd(c.fd);
}

void WorldServer::log(int level, const std::string& msg) const {
  if (log_) log_(level, "remote: " + msg);
}

void WorldServer::adopt(int fd) {
  Client c;
  c.fd = fd;
  clients_.push_back(c);
}

WorldServer::Client* WorldServer::clientOf(int robot) {
  for (Client& c : clients_)
    if (c.robot == robot) return &c;
  return nullptr;
}

bool WorldServer::everyRobotLoaded() {
  for (int i = 0; i < num_robots_; ++i) {
    const Client* c = clientOf(i);
    if (!c || !c->has_spec) return false;
  }
  return true;
}

bool WorldServer::everyClientAsksForAStep() const {
  bool any = false;
  for (const Client& c : clients_) {
    if (c.robot < 0) continue;
    if (!c.has_step) return false;
    any = true;
  }
  return any;
}

void WorldServer::drop(size_t index, bool quiet) {
  const Client c = clients_[index];
  closeFd(c.fd);
  clients_.erase(clients_.begin() + static_cast<long>(index));
  if (c.robot < 0) return;   // never became a robot's controller (e.g. another server's probe)
  if (!built_) {
    if (!quiet) log(kInfo, "the controller of " + robotName(c.robot) + " left before the simulation started");
    return;
  }
  // The robot stays in the world: no torque, no external force, the arm locked and the wrist off.
  world_.setJointTorques(c.robot, Eigen::ArrayXf::Zero(c.spec.num_sdk_joints));
  if (!c.spec.arm_joints.empty()) {
    world_.setArmLocked(c.robot, true);
    world_.setWristCommand(c.robot, WristCommand{});
  }
  world_.applyExternalForce(c.robot, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero());
  std::string msg = "the controller of " + robotName(c.robot) + " left after " + std::to_string(steps_) + " steps";
  if (num_robots_ > 1) msg += "; the robot stays in the world without torque";
  log(kInfo, msg);
}

void WorldServer::protocolError(size_t index, const std::string& text) {
  Error e{};
  setText(e.text, text);
  sendMsg(clients_[index].fd, MsgType::kError, e);
  const int robot = clients_[index].robot;
  log(kError, (robot >= 0 ? robotName(robot) + ": " : std::string()) + text + "; closing the connection");
  drop(index, true);
}

void WorldServer::pollOnce(int listen_fd, int timeout_ms) {
  std::vector<pollfd>& fds = poll_fds_;
  fds.clear();
  if (listen_fd >= 0) fds.push_back({listen_fd, POLLIN, 0});
  const size_t first_client = fds.size();
  for (const Client& c : clients_) fds.push_back({c.fd, POLLIN, 0});
  if (fds.empty() || poll(fds.data(), fds.size(), timeout_ms) <= 0) {
    if (fds.empty() && timeout_ms > 0) poll(nullptr, 0, timeout_ms);
    return;
  }
  // Clients first (from the back: handling one may remove it), then a new connection.
  for (size_t i = fds.size(); i-- > first_client;)
    if (fds[i].revents) handleMessage(i - first_client);
  if (listen_fd >= 0 && fds[0].revents) {
    const int fd = acceptClient(listen_fd, 0);
    if (fd >= 0) adopt(fd);
  }
}

void WorldServer::handleMessage(size_t index) {
  Client& c = clients_[index];
  MsgType type{};
  const Recv r = recvMsg(c.fd, 0, &type, &payload_);
  if (r == Recv::kTimeout) return;
  if (r == Recv::kClosed) return drop(index, false);
  if (r == Recv::kBad)
    return protocolError(index, "bad message from the controller (protocol version " + std::to_string(kVersion) +
                                    " here; are ghost_sim and gsim_server the same build?)");
  if (type == MsgType::kBye) return drop(index, false);

  if (type == MsgType::kHello) {
    Hello h{};
    if (!payloadAs(payload_, &h)) return protocolError(index, "Hello has the wrong size");
    if (c.robot >= 0) return protocolError(index, "Hello sent twice");
    if (built_) return protocolError(index, "the simulation is already running; a controller cannot join now");
    if (h.robot >= static_cast<uint32_t>(num_robots_))
      return protocolError(index, robotName(static_cast<int>(h.robot)) + ": this server has robots 0.." +
                                      std::to_string(num_robots_ - 1) + " (gsim_robots " + std::to_string(num_robots_) + ")");
    if (clientOf(static_cast<int>(h.robot)))
      return protocolError(index, robotName(static_cast<int>(h.robot)) + " already has a controller");
    c.robot = static_cast<int>(h.robot);
    Welcome w{};
    w.timestep = world_.timestep();
    w.num_robots = static_cast<uint32_t>(num_robots_);
    w.supports_arm = world_.supportsArm();
    w.supports_depth = world_.supportsDepthCamera();
    if (!sendMsg(c.fd, MsgType::kWelcome, w))
      return drop(index, false);
    int joined = 0;
    for (const Client& k : clients_) joined += k.robot >= 0;
    log(kInfo, robotName(c.robot) + " joined (" + std::to_string(joined) + " of " + std::to_string(num_robots_) + ")");
    return;
  }
  if (c.robot < 0) return protocolError(index, "the first message must be Hello");

  if (type == MsgType::kLoadRobot) {
    LoadRobot m{};
    if (!payloadAs(payload_, &m)) return protocolError(index, "LoadRobot has the wrong size");
    if (c.has_spec || built_) return protocolError(index, "the robot is already loaded");
    const std::string file = getText(m.urdf_file);
    Loaded refused{};
    if (file.empty() || file.find('/') != std::string::npos || file.find("..") != std::string::npos)
      setText(refused.error, "bad URDF file name");
    else if (m.num_sdk_joints < 1 || m.num_sdk_joints > kMaxJoints)
      setText(refused.error, "bad joint count " + std::to_string(m.num_sdk_joints));
    else if (m.num_arm_joints < 0 || m.num_arm_joints > kMaxArmJoints)
      setText(refused.error, "bad arm joint count " + std::to_string(m.num_arm_joints));
    else if (m.num_arm_joints > 0 && !world_.supportsArm())
      setText(refused.error, "the physics of this server has no arm support");
    else
      for (int i = 0; i < m.num_arm_joints; ++i)
        if (m.arm_joints[i] < 0 || m.arm_joints[i] >= m.num_sdk_joints)
          setText(refused.error, "bad arm joint " + std::to_string(m.arm_joints[i]));
    if (refused.error[0]) {
      log(kError, robotName(c.robot) + ": LoadRobot refused: " + getText(refused.error));
      if (!sendMsg(c.fd, MsgType::kLoaded, refused)) drop(index, false);
      return;
    }
    c.spec = RobotSpec{};
    c.spec.urdf_path = urdf_dir_ + "/" + file;
    c.spec.num_sdk_joints = m.num_sdk_joints;
    c.spec.fixed_base = m.fixed_base != 0;
    c.spec.pos0_W = Eigen::Map<const Eigen::Vector3f>(m.pos0_W);
    c.spec.rpy0 = Eigen::Map<const Eigen::Vector3f>(m.rpy0);
    c.spec.arm_joints.assign(m.arm_joints, m.arm_joints + m.num_arm_joints);
    c.has_spec = true;   // answered by buildWorld() once every robot is here
    return;
  }
  if (type == MsgType::kResetJoint) {
    ResetJoint m{};
    if (!payloadAs(payload_, &m)) return protocolError(index, "ResetJoint has the wrong size");
    if (!built_) return protocolError(index, "ResetJoint before LoadRobot");
    if (c.stepped) return protocolError(index, "ResetJoint after the first Step");
    world_.resetJoint(c.robot, m.joint, m.q);
    return;
  }
  if (type == MsgType::kStep) {
    Step m{};
    if (!built_) return protocolError(index, "Step before LoadRobot");
    if (!payloadAs(payload_, &m) || m.n_tau < 0 || m.n_tau > kMaxJoints)
      return protocolError(index, "Step has the wrong size or joint count");
    if (c.has_step) return protocolError(index, "Step sent before the answer to the previous one");
    c.step = m;
    c.has_step = true;
    c.stepped = true;
    return;
  }
  if (type == MsgType::kRenderDepth) {
    // Between two steps only: the frame is the world's newest (ISimWorld::renderDepthCamera).
    if (!built_ || c.has_step) return protocolError(index, "RenderDepth before LoadRobot or while a Step is waiting");
    const bool ok = world_.renderDepthCamera(c.robot, cloud_);
    const uint32_t bytes = ok ? static_cast<uint32_t>(cloud_.size() * sizeof(float)) : 0;
    if (!sendMsg(c.fd, MsgType::kCloud, cloud_.data(), bytes)) drop(index, false);
    return;
  }
  protocolError(index, "unexpected message " + std::to_string(static_cast<int>(type)) + " from the controller");
}

bool WorldServer::buildWorld() {
  Loaded reply{};
  // Robots that start inside each other would be thrown apart by the contact solver.
  for (int i = 0; i < num_robots_ && !reply.error[0]; ++i)
    for (int j = i + 1; j < num_robots_ && !reply.error[0]; ++j) {
      const double d = (clientOf(i)->spec.pos0_W.head<2>() - clientOf(j)->spec.pos0_W.head<2>()).norm();
      if (d >= kMinSpawnDistance) continue;
      char text[200];
      std::snprintf(text, sizeof(text),
                    "robots %d and %d start %.2f m apart; give each its own gsim_spawn (at least %.0f m apart)", i, j, d,
                    kMinSpawnDistance);
      setText(reply.error, text);
    }
  if (!reply.error[0]) {
    bool ok = true;
    for (int i = 0; i < num_robots_ && ok; ++i) ok = world_.addRobot(clientOf(i)->spec) == i;
    ok = ok && world_.build();
    if (!ok) {
      world_.clear();
      setText(reply.error, "could not build the world (see the output of gsim_server)");
    }
  }
  reply.ok = reply.error[0] == 0;
  if (reply.ok) {
    for (int i = 0; i < num_robots_; ++i) log(kInfo, robotName(i) + ": loaded " + clientOf(i)->spec.urdf_path);
  } else {
    log(kError, "LoadRobot refused: " + getText(reply.error));
  }
  built_ = reply.ok;
  for (size_t k = clients_.size(); k-- > 0;)
    if (clients_[k].robot >= 0 && !sendMsg(clients_[k].fd, MsgType::kLoaded, reply)) drop(k, false);
  return reply.ok;
}

void WorldServer::stepWorld() {
  const double dt = world_.timestep();
  // In the order of the robots, whatever order the controllers connected in.
  std::vector<size_t>& order = order_;
  order.clear();
  for (size_t k = 0; k < clients_.size(); ++k)
    if (clients_[k].robot >= 0) order.push_back(k);
  std::sort(order.begin(), order.end(), [this](size_t a, size_t b) { return clients_[a].robot < clients_[b].robot; });

  Eigen::ArrayXf& tau = tau_;
  for (size_t k : order) {
    const Client& c = clients_[k];
    if (c.step.n_tau > 0) {
      tau = Eigen::Map<const Eigen::ArrayXf>(c.step.tau, c.step.n_tau);
      world_.setJointTorques(c.robot, tau);
    }
    world_.applyExternalForce(c.robot, Eigen::Map<const Eigen::Vector3f>(c.step.f_B),
                              Eigen::Map<const Eigen::Vector3f>(c.step.p_B));
    if (!c.spec.arm_joints.empty()) {
      WristCommand cmd;
      cmd.mode = wristModeFromSdkValue(c.step.wrist_mode).value_or(WristMode::kOff);
      cmd.pos_des = Eigen::Map<const Eigen::Array4f>(c.step.wrist_pos_des);
      cmd.vel_des = Eigen::Map<const Eigen::Array4f>(c.step.wrist_vel_des);
      cmd.max_current = Eigen::Map<const Eigen::Array4f>(c.step.wrist_max_current);
      world_.setArmLocked(c.robot, c.step.arm_locked != 0);
      world_.setWristCommand(c.robot, cmd);
    }
  }
  world_.step();
  ++steps_;

  std::vector<State>& states = states_;
  states.assign(order.size(), State{});
  BaseState base;
  Eigen::ArrayXf &q = q_, &dq = dq_;
  for (size_t n = 0; n < order.size(); ++n) {
    Client& c = clients_[order[n]];
    world_.readState(c.robot, base, q, dq);
    State& s = states[n];
    s.seq = c.step.seq;
    s.running = 1;
    Eigen::Map<Eigen::Vector3f>(s.pos_W) = base.pos_W;
    Eigen::Map<Eigen::Vector3f>(s.vel_W) = base.vel_W;
    s.quat_WB[0] = base.quat_WB.w();
    s.quat_WB[1] = base.quat_WB.x();
    s.quat_WB[2] = base.quat_WB.y();
    s.quat_WB[3] = base.quat_WB.z();
    Eigen::Map<Eigen::Vector3f>(s.angvel_W) = base.angvel_W;
    std::copy(q.data(), q.data() + std::min<Eigen::Index>(q.size(), kMaxJoints), s.q);
    std::copy(dq.data(), dq.data() + std::min<Eigen::Index>(dq.size(), kMaxJoints), s.dq);
    WristState wrist;   // NaN unless the robot has an arm
    if (!c.spec.arm_joints.empty()) world_.readWrist(c.robot, wrist);
    Eigen::Map<Eigen::Array4f>(s.wrist_q) = wrist.q;
    Eigen::Map<Eigen::Array4f>(s.wrist_dq) = wrist.dq;
    Eigen::Map<Eigen::Array4f>(s.wrist_current) = wrist.current;
    c.has_step = false;
  }
  if (steps_ % kPaceSteps == 0) pacer_.wait(steps_ * dt);
  // From the back: a controller that is gone is removed, which shifts the ones after it.
  std::vector<size_t> gone;
  for (size_t n = 0; n < order.size(); ++n)
    if (!sendMsg(clients_[order[n]].fd, MsgType::kState, states[n])) gone.push_back(order[n]);
  std::sort(gone.rbegin(), gone.rend());
  for (size_t k : gone) drop(k, false);
}

void WorldServer::sayStopped() {
  // Tells the controllers the simulation is over; they read this instead of the next state.
  State s{};
  s.running = 0;
  for (const Client& c : clients_)
    if (c.robot >= 0) sendMsg(c.fd, MsgType::kState, s);
}

long WorldServer::serve(int listen_fd, std::atomic_bool& keep_alive) {
  auto stopping = [&] { return !keep_alive.load() || !world_.running(); };

  // Until every robot has a controller that sent its LoadRobot.
  while (!everyRobotLoaded()) {
    if (stopping()) {
      sayStopped();
      return 0;
    }
    if (listen_fd < 0 && clients_.empty()) return 0;
    pollOnce(listen_fd, kIdleSliceMs);
  }
  if (!buildWorld()) return 0;

  const double dt = world_.timestep();
  bool was_paused = false;
  double waiting_since = wallSeconds();
  for (;;) {
    if (stopping()) {
      sayStopped();
      break;
    }
    bool any = false;
    for (const Client& c : clients_) any = any || c.robot >= 0;
    if (!any) break;

    const bool paused = world_.paused();
    was_paused = was_paused || paused;
    if (!paused && everyClientAsksForAStep()) {
      if (was_paused) pacer_.reanchor(steps_ * dt);
      was_paused = false;
      stepWorld();
      waiting_since = wallSeconds();
      continue;   // every Step is answered: the next round waits for the controllers (or a late one)
    }
    // Paused, or a controller has not asked yet. A message from a controller whose Step is
    // waiting means it is leaving.
    pollOnce(listen_fd, paused ? kPauseSliceMs : kIdleSliceMs);
    if (paused) {
      waiting_since = wallSeconds();
    } else if (wallSeconds() - waiting_since > kWaitingReportS) {
      for (const Client& c : clients_)
        if (c.robot >= 0 && !c.has_step) log(kInfo, "waiting for " + robotName(c.robot));
      waiting_since = wallSeconds();
    }
  }
  return steps_;
}

}  // namespace gsim::remote
