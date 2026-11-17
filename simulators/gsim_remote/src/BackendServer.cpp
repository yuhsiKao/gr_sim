#include "gsim_remote/BackendServer.hpp"

#include <algorithm>
#include <vector>

#include "gsim_core/pacer.hpp"
#include "gsim_remote/protocol.hpp"

namespace gsim::remote {

namespace {

constexpr int kIdleSliceMs = 100;   // how often an idle server looks at keep_alive and the backend
constexpr int kPauseSliceMs = 10;
constexpr int kPaceSteps = 4;       // pace every 4 steps (2 ms of simulated time at 2000 Hz)

}  // namespace

BackendServer::BackendServer(ISimBackend& backend, std::string urdf_dir, LogFn log, double rtf)
    : backend_(backend), urdf_dir_(std::move(urdf_dir)), log_(std::move(log)), rtf_(rtf) {}

void BackendServer::log(int level, const std::string& msg) const {
  if (log_) log_(level, "remote: " + msg);
}

long BackendServer::serve(int fd, std::atomic_bool& keep_alive) {
  const double dt = backend_.timestep();
  Pacer pacer(rtf_);
  long steps = 0;
  bool loaded = false;
  std::vector<uint8_t> payload, extra;
  Eigen::ArrayXf tau, q, dq;
  BaseState base;

  auto stopping = [&] { return !keep_alive.load() || !backend_.running(); };
  // Tells the controller the simulation is over; it reads this instead of the next state.
  auto say_stopped = [&] {
    State s{};
    s.running = 0;
    sendMsg(fd, MsgType::kState, s);
  };
  auto protocol_error = [&](const std::string& text) {
    Error e{};
    setText(e.text, text);
    sendMsg(fd, MsgType::kError, e);
    log(kError, text + "; closing the connection");
  };

  for (;;) {
    if (stopping()) {
      say_stopped();
      break;
    }
    MsgType type{};
    const Recv r = recvMsg(fd, kIdleSliceMs, &type, &payload);
    if (r == Recv::kTimeout) continue;
    if (r == Recv::kClosed) {
      log(kInfo, "the controller left after " + std::to_string(steps) + " steps");
      break;
    }
    if (r == Recv::kBad) {
      protocol_error("bad message from the controller (protocol version " + std::to_string(kVersion) +
                     " here; are ghost_sim and gsim_server the same build?)");
      break;
    }

    if (type == MsgType::kBye) {
      log(kInfo, "the controller left after " + std::to_string(steps) + " steps");
      break;
    }
    if (type == MsgType::kHello) {
      Hello h{};
      if (!payloadAs(payload, &h)) {
        protocol_error("Hello has the wrong size");
        break;
      }
      if (!sendMsg(fd, MsgType::kWelcome, Welcome{dt, 1})) break;
    } else if (type == MsgType::kLoadRobot) {
      LoadRobot m{};
      if (!payloadAs(payload, &m) || loaded) {
        protocol_error(loaded ? "the robot is already loaded" : "LoadRobot has the wrong size");
        break;
      }
      Loaded reply{};
      const std::string file = getText(m.urdf_file);
      if (file.empty() || file.find('/') != std::string::npos || file.find("..") != std::string::npos) {
        setText(reply.error, "bad URDF file name");
      } else if (m.num_sdk_joints < 1 || m.num_sdk_joints > kMaxJoints) {
        setText(reply.error, "bad joint count " + std::to_string(m.num_sdk_joints));
      } else {
        RobotSpec spec;
        spec.urdf_path = urdf_dir_ + "/" + file;
        spec.num_sdk_joints = m.num_sdk_joints;
        spec.fixed_base = m.fixed_base != 0;
        spec.pos0_W = Eigen::Map<const Eigen::Vector3f>(m.pos0_W);
        spec.rpy0 = Eigen::Map<const Eigen::Vector3f>(m.rpy0);
        loaded = backend_.loadRobot(spec);
        if (loaded)
          log(kInfo, "loaded " + spec.urdf_path);
        else
          setText(reply.error, "could not load " + file + " (see the output of gsim_server)");
      }
      reply.ok = loaded;
      if (!loaded) log(kError, "LoadRobot refused: " + getText(reply.error));
      if (!sendMsg(fd, MsgType::kLoaded, reply)) break;
    } else if (type == MsgType::kResetJoint) {
      ResetJoint m{};
      if (!payloadAs(payload, &m) || !loaded) {
        protocol_error(loaded ? "ResetJoint has the wrong size" : "ResetJoint before LoadRobot");
        break;
      }
      backend_.resetJoint(m.joint, m.q);
    } else if (type == MsgType::kStep) {
      Step m{};
      if (!payloadAs(payload, &m) || !loaded || m.n_tau < 0 || m.n_tau > kMaxJoints) {
        protocol_error(!loaded ? "Step before LoadRobot" : "Step has the wrong size or joint count");
        break;
      }
      // Paused: the step waits here. Anything the controller sends meanwhile means it is leaving.
      bool was_paused = false, left = false;
      while (backend_.paused() && !stopping() && !left) {
        was_paused = true;
        MsgType t{};
        left = recvMsg(fd, kPauseSliceMs, &t, &extra) != Recv::kTimeout;
      }
      if (left) {
        log(kInfo, "the controller left after " + std::to_string(steps) + " steps");
        break;
      }
      if (stopping()) {
        say_stopped();
        break;
      }
      if (was_paused) pacer.reanchor(steps * dt);

      if (m.n_tau > 0) {
        tau = Eigen::Map<const Eigen::ArrayXf>(m.tau, m.n_tau);
        backend_.setJointTorques(tau);
      }
      backend_.applyExternalForce(Eigen::Map<const Eigen::Vector3f>(m.f_B), Eigen::Map<const Eigen::Vector3f>(m.p_B));
      backend_.step();
      ++steps;
      backend_.readState(base, q, dq);

      State s{};
      s.seq = m.seq;
      s.running = 1;
      Eigen::Map<Eigen::Vector3f>(s.pos_W) = base.pos_W;
      Eigen::Map<Eigen::Vector3f>(s.vel_W) = base.vel_W;
      s.quat_WB[0] = base.quat_WB.w();
      s.quat_WB[1] = base.quat_WB.x();
      s.quat_WB[2] = base.quat_WB.y();
      s.quat_WB[3] = base.quat_WB.z();
      Eigen::Map<Eigen::Vector3f>(s.angvel_W) = base.angvel_W;
      s.n = static_cast<int32_t>(std::min<Eigen::Index>(q.size(), kMaxJoints));
      std::copy(q.data(), q.data() + s.n, s.q);
      std::copy(dq.data(), dq.data() + std::min<Eigen::Index>(dq.size(), kMaxJoints), s.dq);
      if (steps % kPaceSteps == 0) pacer.wait(steps * dt);
      if (!sendMsg(fd, MsgType::kState, s)) {
        log(kInfo, "the controller left after " + std::to_string(steps) + " steps");
        break;
      }
    } else {
      protocol_error("unexpected message " + std::to_string(static_cast<int>(type)) + " from the controller");
      break;
    }
  }
  return steps;
}

}  // namespace gsim::remote
