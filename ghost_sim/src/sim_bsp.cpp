// Generic simulation platform layer: implements the SDK's per-arch hooks
// gr::Robot::bspInit / bspBegin on top of gsim::ISimBackend. No physics-engine
// types appear in this file.
#include "sim_bsp.hpp"

#include <SDK.h>
#include <SimLLC.hpp>
#include <CmdLineArgs.hpp>
#include <loguru.hpp>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "backend_factory.hpp"
#include "gsim_core/csv_logger.hpp"
#include "gsim_core/gsim_args.hpp"
#include "gsim_core/pacer.hpp"
#include "gsim_core/robot_select.hpp"
#include "gsim_core/sim_loop.hpp"
#include "gsim_core/sim_math.hpp"

namespace {

std::unique_ptr<gsim::ISimBackend> g_backend;
gsim::GsimArgs g_args;
bool g_selftest_done = false;
int g_exit_code = 0;

void logToLoguru(int level, const std::string& msg) {
  if (level >= gsim::kError)
    LOG_F(ERROR, "%s", msg.c_str());
  else if (level == gsim::kWarning)
    LOG_F(WARNING, "%s", msg.c_str());
  else
    LOG_F(INFO, "%s", msg.c_str());
}

void fail(const std::string& msg) {
  LOG_F(ERROR, "%s", msg.c_str());
  g_exit_code = 2;
}

// The robot to load. With the arm, arm_joints holds the SDK ids of the arm joints, root to tip (the
// order Ghost's bsp passes to SimWrapper::resetArm). b_arm ("arm 1") decides whether the arm is
// mounted: Vision 60 v5 reports an arm limb (kNumArms_ = 1, joints 12..14) even without it.
template <class Limbs>
gsim::RobotSpec robotSpec(const Limbs* L, bool b_arm, const std::string& urdf_path, bool fixed, bool inverted) {
  gsim::RobotSpec spec = gsim::makeRobotSpec(urdf_path, L->get_total_num_joints(), fixed, inverted);
  if (b_arm && L->kNumArms_ > 0) {
    const auto* arm = L->limbs_[L->kNumLegs_];
    for (int i = 0; i < arm->kNumJoints_; ++i) spec.arm_joints.push_back(arm->joints_[i]->get_ID());
  }
  return spec;
}

}  // namespace

namespace gsim {
int bspExitCode() { return g_exit_code; }
}  // namespace gsim

namespace gr {

void Robot::bspInit(int argc, char* argv[]) {
  const ArgList_t args = getArgList(argc, argv);
  const gsim::ArgLookup lookup = [&args](const std::string& key, const std::string& def) {
    return updateFromArgs(args, key, def);
  };
  g_args = gsim::parseGsimArgs(
      lookup, {GSIM_DEFAULT_SCENE, gsim::defaultUrdfDir(std::getenv("CONTROLS_INSTALL_DIR"), GSIM_SDK_DIR), GSIM_DEFAULT_WORLD},
      test_idx_);
  const auto errors = gsim::validateGsimArgs(g_args);
  if (!errors.empty()) {
    for (const auto& e : errors) fail(e);
    return;
  }

  gsim::SimConfig cfg;
  cfg.headless = g_args.headless;
  cfg.scene_path = g_args.scene;
  cfg.world_path = g_args.world;
  cfg.depth_camera = true;   // the SDK's perception always exists; vision_mode may turn the cameras on at any time
  cfg.urdf_dir = g_args.urdf_dir;
  cfg.argc = argc;
  cfg.argv = argv;
  cfg.log = logToLoguru;
  cfg.toe_friction = g_args.toe_friction;
  cfg.toe_solref = g_args.toe_solref;

  std::string err;
  g_backend = gsim::makeBackend(g_args.backend, &err);
  if (!g_backend) return fail(err);
  if (!g_backend->init(cfg)) {
    g_backend.reset();
    return fail("gsim: failed to initialize the " + g_args.backend + " backend");
  }
  if (const std::string e = gsim::physicsTimestepError(g_backend->timestep()); !e.empty()) {
    g_backend.reset();
    const std::string where = g_args.backend == "gazebo"   ? " (gsim_world " + g_args.world + ")"
                              : g_args.backend == "mujoco" ? " (gsim_scene " + g_args.scene + ")"
                                                           : "";
    return fail("gsim: " + e + where);
  }
  LOG_F(INFO, "gsim: backend=%s headless=%d rtf=%g", g_args.backend.c_str(), int(g_args.headless), g_args.rtf);
  if (CONTROL_RATE != 500)
    LOG_F(WARNING, "gsim: CONTROL_RATE=%d Hz (gsim_control_rate); the SDK's default and Ghost's sims run 500 Hz", CONTROL_RATE);

  if (g_args.selftest) {
    std::string why;
    const auto urdf = gsim::chooseV1Urdf(L->robot_type_ == RobotParams_Type_NGR, L->robot_version_, b_arm_, &why);
    if (!urdf) return fail("gsim: " + why);
    const auto spec = robotSpec(L, b_arm_, g_args.urdf_dir + "/" + *urdf, false, false);
    const std::string arm_err = gsim::armSupportError(*g_backend, !spec.arm_joints.empty(), g_args.backend);
    if (!arm_err.empty()) return fail("gsim: " + arm_err);
    const auto problems = g_backend->selfTest(spec);
    for (const auto& p : problems) LOG_F(ERROR, "gsim selftest: %s", p.c_str());
    if (problems.empty()) {
      LOG_F(INFO, "gsim selftest: PASS");
    } else {
      LOG_F(ERROR, "gsim selftest: FAIL (%zu)", problems.size());
      g_exit_code = 1;
    }
    g_selftest_done = true;
  }
}

void Robot::bspBegin() {
  if (!g_backend || g_selftest_done) {
    keep_alive.exchange(false);
    return;
  }
  // Stop the process on any setup error below.
  auto abort_begin = [this](const std::string& msg) {
    fail(msg);
    keep_alive.exchange(false);
  };

  std::string why;
  const auto urdf = gsim::chooseV1Urdf(L->robot_type_ == RobotParams_Type_NGR, L->robot_version_, b_arm_, &why);
  if (!urdf) return abort_begin("gsim: " + why);
  // Synthetic perception as Ghost's Bullet bsp: initProcessing(4) (without it the perception thread
  // crashes on shutdown), and with a depth camera all 4 cameras reported present and healthy, so the
  // app shows the vision button and the SDK accepts vision modes.
  const bool depth_camera = perception && g_backend->supportsDepthCamera();
  if (perception) {
    perception->initProcessing(4);
    if (depth_camera) {
      for (int i = 0; i < 4; ++i) {
        diag_.present |= Diag_t::CAM0 << i;
        diag_.enabled |= Diag_t::CAM0 << i;
        diag_.health |= Diag_t::CAM0 << i;
      }
      LOG_F(INFO, "gsim: synthetic depth camera on (30 Hz while vision_mode turns the cameras on)");
    } else {
      LOG_F(WARNING, "gsim: the %s simulator has no depth camera; perception stays off", g_args.backend.c_str());
    }
  }

  const int substeps = gsim::computeSubsteps(CONTROL_RATE, g_backend->timestep());
  if (substeps < 1)
    return abort_begin("gsim: physics timestep " + std::to_string(g_backend->timestep()) +
                       " s does not divide the control period 1/" + std::to_string(CONTROL_RATE) + " s");

  // SimLLC plays the mainboard: state -> S, C -> joint torques.
  SimLLC sim_llc(&S, &C, &L->Sw, &L->Cw, g_backend->timestep());
  sim_llc.initLLCCallbacks(L);
  const int num_joints = L->get_total_num_joints();
  const auto spec = robotSpec(L, b_arm_, g_args.urdf_dir + "/" + *urdf, b_fixed_base_, b_inverted_start_);
  const bool has_arm = !spec.arm_joints.empty();
  const std::string arm_err = gsim::armSupportError(*g_backend, has_arm, g_args.backend);
  if (!arm_err.empty()) return abort_begin("gsim: " + arm_err);
  if (!g_backend->loadRobot(spec)) return abort_begin("gsim: failed to load " + spec.urdf_path);
  sim_llc.resetJoints(
      b_inverted_start_ ? SimLLC::ResetPose::Inverted : SimLLC::ResetPose::Normal,
      [](int j, float q) { g_backend->resetJoint(j, q); },
      [](int, int, const float*, const float*) { LOG_F(ERROR, "gsim: closed-chain constraints are not supported"); });

  gsim::Pacer pacer(g_args.rtf);
  Eigen::Vector3f last_pos = spec.pos0_W;
  float last_yaw = spec.rpy0.z();
  pacer.setReport([&last_pos, &last_yaw](double rtf, int lag_resets) {
    LOG_F(INFO, "gsim: pos=(%.2f, %.2f, %.3f)m yaw=%.1fdeg rtf=%.2f lag_resets=%d", last_pos.x(), last_pos.y(),
          last_pos.z(), last_yaw * 180.f / float(M_PI), rtf, lag_resets);
  });

  gsim::CsvLogger logger;
  if (!g_args.log.empty()) {
    std::string log_err;
    if (!logger.open(g_args.log, num_joints, &log_err, has_arm)) return abort_begin("gsim: " + log_err);
    LOG_F(INFO, "gsim: logging to %s", g_args.log.c_str());
  }

  Eigen::ArrayXf wrist_q, wrist_dq, wrist_cur;  // without the arm: zero-length, as Ghost's Bullet bsp passes
  gsim::LoopHooks hooks;
  hooks.external_force = [this] { return std::make_pair(getBsimExtForce(), getBsimExtPos()); };
  hooks.on_state = [&](const gsim::BaseState& b, const Eigen::ArrayXf& q, const Eigen::ArrayXf& dq,
                       const gsim::WristState* w) {
    if (w) {
      wrist_q = w->q;
      wrist_dq = w->dq;
      wrist_cur = w->current;
    }
    sim_llc.stateUpdate(gsim::makeOrnState(b), q, dq, wrist_q, wrist_dq, wrist_cur, true, gsim::makeXW(b));
  };
  // As Ghost's Bullet bsp, every step: the arm is unlocked while unstowed (or forced), and the
  // wrist command L->Cw is passed on. If Cw is busy, the previous command is kept.
  gsim::WristCommand wrist_cmd;
  std::set<int> unsupported_modes;
  hooks.arm_command = [&] {
    gsim::ArmCommand c;
    c.locked = !(b_force_arm_unlocked_ || checkBehavior(Behav_ARM) > 0);
    if (L->Cw.mutex.try_lock_for(std::chrono::microseconds(10))) {
      const auto& cw = L->Cw.cmd;
      const int sdk_mode = static_cast<int>(cw.cmd_mode);
      const auto mode = gsim::wristModeFromSdkValue(sdk_mode);
      if (!mode && unsupported_modes.insert(sdk_mode).second)
        LOG_F(WARNING, "gsim: wrist cmd_mode=%d is not supported; the wrist gets no force", sdk_mode);
      wrist_cmd.mode = mode.value_or(gsim::WristMode::kOff);
      wrist_cmd.pos_des = cw.pos_des;
      wrist_cmd.vel_des = cw.vel_des;
      wrist_cmd.max_current = cw.max_current;
      L->Cw.mutex.unlock();
    }
    c.wrist = wrist_cmd;
    return c;
  };
  hooks.on_control_tick = [&](const gsim::TickInfo& tick) {
    last_pos = tick.base.pos_W;
    last_yaw = gsim::rpyFromQuat(tick.base.quat_WB).z();
    sdkUpdateA();
    logger.write(tick);
  };
  hooks.torques = [&] { return Eigen::ArrayXf(sim_llc.generateTorques().array()); };
  // As Ghost's Bullet bsp, after the torques: while the SDK has its cameras on, the depth camera is
  // called every step and every new frame goes into the SDK's cloud for its terrain perception.
  if (depth_camera) {
    hooks.after_torques = [this] {
      if (!decodeVisionMode(checkBehavior(Behav_VISION_MODE), VMode_CAMERAS_ON)) return;
      if (g_backend->updateDepthCamera(*perception->lockedRefToCloudB())) perception->notifyNewCloud();
    };
  }

  gsim::SimLoop loop(*g_backend, std::move(hooks), pacer, substeps, num_joints, has_arm);
  LOG_F(INFO, "gsim: starting simulation (substeps=%d, CONTROL_RATE=%d, joints=%d, arm=%d)", substeps, CONTROL_RATE,
        num_joints, int(has_arm));
  if (g_backend->wantsMainThreadViewer() && !g_args.headless) {
    // The viewer must own the main thread; physics + SDK run on a worker thread.
    std::thread sim_thread([&] {
      loop.run(keep_alive);
      keep_alive.exchange(false);
    });
    g_backend->runViewer(keep_alive);
    keep_alive.exchange(false);
    sim_thread.join();
  } else {
    loop.run(keep_alive);
  }
  logger.close();
  LOG_F(INFO, "gsim: simulation stopped at t=%.2fs after %ld steps", loop.simTime(), loop.steps());
  keep_alive.exchange(false);
  // The simulator goes down here, before main returns: destroying it during static destruction is
  // too late for engines with statics of their own (Gazebo's transport aborted with a double free).
  g_backend.reset();
}

}  // namespace gr
