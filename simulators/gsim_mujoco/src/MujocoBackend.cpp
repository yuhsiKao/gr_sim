#include "gsim_mujoco/MujocoBackend.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <memory>

#include "gsim_core/bullet_motor.hpp"
#include "gsim_core/sim_math.hpp"

namespace gsim {

namespace {

ModelOptions modelOptions(const RobotSpec& spec, const SimConfig& cfg) {
  ModelOptions opt;
  opt.urdf_path = spec.urdf_path;
  opt.scene_path = cfg.scene_path;
  opt.num_sdk_joints = spec.num_sdk_joints;
  opt.fixed_base = spec.fixed_base;
  opt.base_pos = spec.pos0_W.cast<double>();
  opt.base_quat = quatFromRpy(spec.rpy0).cast<double>();
  opt.toe_friction = cfg.toe_friction;
  opt.toe_solref = cfg.toe_solref;
  opt.arm_joints = spec.arm_joints;
  return opt;
}

// MuJoCo's default log handler appends to MUJOCO_LOG.TXT in the working directory;
// while a backend exists, its messages go to the backend's log callback instead.
LogFn g_runtime_log;

// MuJoCo expects error handlers not to return (mju_error() would return into its caller,
// e.g. GlfwAdapter carrying on with a null window), so errors end the process like
// MuJoCo's default handler does.
void runtimeLogHandler(const mjLogMessage* msg) {
  if (msg->level < mjLOG_WARNING) return;
  const std::string text = std::string("MuJoCo: ") + msg->subject;
  if (msg->level < mjLOG_ERROR) {
    if (g_runtime_log) g_runtime_log(kWarning, text);
    return;
  }
  if (g_runtime_log)
    g_runtime_log(kError, text + " (exiting)");
  else
    std::fprintf(stderr, "%s (exiting)\n", text.c_str());
  std::exit(2);
}

}  // namespace

MujocoBackend::MujocoBackend() = default;

MujocoBackend::~MujocoBackend() {
  viewer_.reset();
  freeModel();
  if (log_handler_installed_) {
    mju_setLogHandler(previous_log_handler_);
    g_runtime_log = nullptr;
  }
}

void MujocoBackend::log(int level, const std::string& msg) const {
  if (cfg_.log) cfg_.log(level, msg);
}

void MujocoBackend::freeModel() {
  freeDepthRenderer();   // its MuJoCo context belongs to the old model
  if (d_) mj_deleteData(d_);
  if (m_) mj_deleteModel(m_);
  d_ = nullptr;
  m_ = nullptr;
}

bool MujocoBackend::init(const SimConfig& cfg) {
  cfg_ = cfg;
  EglContext::disableGlvndPatching();   // before the viewer's window exists, so the depth camera can render
  if (!log_handler_installed_) {
    g_runtime_log = cfg.log;
    previous_log_handler_ = mju_setLogHandler(runtimeLogHandler);
    log_handler_installed_ = true;
  }
  if (!std::filesystem::exists(cfg.scene_path)) {
    log(kError, "MuJoCo: scene file not found: " + cfg.scene_path);
    return false;
  }
  char err[1024] = "";
  std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> scene(mj_parseXML(cfg.scene_path.c_str(), nullptr, err, sizeof(err)),
                                                          &mj_deleteSpec);
  if (!scene) {
    log(kError, "MuJoCo: failed to parse scene " + cfg.scene_path + ": " + err);
    return false;
  }
  scene_timestep_ = scene->option.timestep;
  return true;
}

bool MujocoBackend::loadRobot(const RobotSpec& spec) {
  freeModel();
  BuiltModel built = buildMujocoModel(modelOptions(spec, cfg_));
  for (const auto& w : built.warnings) log(kInfo, "MuJoCo: " + w);
  if (!built.model) {
    log(kError, "MuJoCo: " + built.error);
    return false;
  }
  // Arm joints and wrist are checked once here, so the per-step code can rely on them.
  if (!spec.arm_joints.empty()) {
    bool complete = built.arm.has_wrist;
    for (size_t i = 0; i < built.arm.joints.size(); ++i) {
      const int id = built.arm.joints[i];
      complete = complete && id >= 0 && id < spec.num_sdk_joints && built.joints.qposadr[id] >= 0 &&
                 built.arm.lock_actuator[i] >= 0;
    }
    if (!complete) {
      log(kError, "MuJoCo: the arm joints or the wrist of the arm are missing from " + spec.urdf_path);
      mj_deleteModel(built.model);
      return false;
    }
  }
  m_ = built.model;
  d_ = mj_makeData(m_);
  joints_ = built.joints;
  arm_ = built.arm;
  arm_locked_ = true;
  wrist_cmd_ = WristCommand{};
  applied_wrist_ = WristCommand{};
  base_body_ = built.base_body;
  depth_cam_ = built.depth_camera;
  depth_calls_ = 0;
  num_joints_ = spec.num_sdk_joints;
  floating_ = !spec.fixed_base;
  model_timestep_ = m_->opt.timestep;
  mj_resetData(m_, d_);
  mj_forward(m_, d_);
  needs_forward_ = false;
  return true;
}

void MujocoBackend::resetJoint(int sdk_joint, float q) {
  if (!d_ || sdk_joint < 0 || sdk_joint >= num_joints_ || joints_.qposadr[sdk_joint] < 0) return;
  d_->qpos[joints_.qposadr[sdk_joint]] = q;
  d_->qvel[joints_.dofadr[sdk_joint]] = 0.0;
  needs_forward_ = true;
}

void MujocoBackend::ensureForward() {
  if (needs_forward_) {
    mj_forward(m_, d_);
    needs_forward_ = false;
  }
}

// mj_check* reset mjData (which zeroes d->warning) and then count 1, so the count never
// grows past 1: any count is a new reset; clear it to catch the next one.
bool MujocoBackend::takeInstabilityWarnings() {
  bool reset = false;
  for (int w : {mjWARN_BADQACC, mjWARN_BADQPOS, mjWARN_BADQVEL}) {
    reset |= d_->warning[w].number > 0;
    d_->warning[w].number = 0;
  }
  return reset;
}

void MujocoBackend::step() {
  if (!d_) return;
  std::lock_guard<std::mutex> viewer_guard(viewer_mtx_);
  if (viewer_ && !viewer_attach_tried_) {
    viewer_attach_tried_ = true;
    ensureForward();
    viewer_attached_ = viewer_->attach(m_, d_, base_body_);
  }
  if (viewer_ && viewer_attached_ && !viewer_->closed()) {
    const auto ui_lock = viewer_->lock();
    stepPhysics(viewer_.get());
    viewer_->syncIfDue(d_->time);
  } else {
    stepPhysics(nullptr);
  }
}

void MujocoBackend::stepPhysics(MujocoViewer* viewer) {
  ensureForward();
  if (m_->opt.timestep != model_timestep_) {
    // The SDK runs a fixed number of physics steps per control tick (see bspBegin).
    log(kWarning, "MuJoCo: the physics timestep is fixed at " + std::to_string(model_timestep_) +
                      " s while the controller runs; change it in the scene file instead");
    m_->opt.timestep = model_timestep_;
  }
  mju_zero(d_->xfrc_applied, 6 * m_->nbody);
  if (viewer) viewer->applyPerturb(m_, d_);
  if (!ext_f_B_.isZero()) {
    // Body-frame force at a body-frame point -> world force + torque on the base body.
    Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>> R(d_->xmat + 9 * base_body_);
    const Eigen::Vector3d f_W = R * ext_f_B_.cast<double>();
    const Eigen::Vector3d tau_W = (R * ext_p_B_.cast<double>()).cross(f_W);
    for (int i = 0; i < 3; ++i) {
      d_->xfrc_applied[6 * base_body_ + i] += f_W[i];
      d_->xfrc_applied[6 * base_body_ + 3 + i] += tau_W[i];
    }
  }
  applyArm();
  mj_step(m_, d_);

  if (takeInstabilityWarnings()) {
    ++instability_count_;
    log(kWarning, "MuJoCo reset the simulation after an instability (count=" + std::to_string(instability_count_) + ")");
  }
}

void MujocoBackend::readState(BaseState& base, Eigen::ArrayXf& q, Eigen::ArrayXf& dq) {
  if (!d_) return;
  ensureForward();
  if (floating_) {
    // Free joint: position and linear velocity are world frame; angular velocity is body frame.
    base.pos_W = Eigen::Vector3d(d_->qpos[0], d_->qpos[1], d_->qpos[2]).cast<float>();
    base.quat_WB = Eigen::Quaterniond(d_->qpos[3], d_->qpos[4], d_->qpos[5], d_->qpos[6]).cast<float>();
    base.vel_W = Eigen::Vector3d(d_->qvel[0], d_->qvel[1], d_->qvel[2]).cast<float>();
    base.angvel_W = angvelBodyToWorld(base.quat_WB, Eigen::Vector3d(d_->qvel[3], d_->qvel[4], d_->qvel[5]).cast<float>());
  } else {
    const mjtNum* p = d_->xpos + 3 * base_body_;
    const mjtNum* qt = d_->xquat + 4 * base_body_;
    base.pos_W = Eigen::Vector3d(p[0], p[1], p[2]).cast<float>();
    base.quat_WB = Eigen::Quaterniond(qt[0], qt[1], qt[2], qt[3]).cast<float>();
    base.vel_W.setZero();
    base.angvel_W.setZero();
  }
  const float nan = std::numeric_limits<float>::quiet_NaN();
  q.setConstant(num_joints_, nan);
  dq.setConstant(num_joints_, nan);
  for (int j = 0; j < num_joints_; ++j) {
    if (joints_.qposadr[j] < 0) continue;
    q[j] = static_cast<float>(d_->qpos[joints_.qposadr[j]]);
    dq[j] = static_cast<float>(d_->qvel[joints_.dofadr[j]]);
  }
}

void MujocoBackend::setJointTorques(const Eigen::ArrayXf& tau) {
  if (!d_) return;
  const int n = std::min<int>(num_joints_, static_cast<int>(tau.size()));
  for (int j = 0; j < n; ++j) {
    const int a = joints_.actuator[j];
    if (a < 0) continue;  // SDK joint slot without a joint in the model (e.g. arm)
    float t = tau[j];
    if (!std::isfinite(t)) {
      t = 0.f;
      if (shouldReport(nan_torque_count_))
        log(kWarning, "MuJoCo: NaN/inf torque for joint " + std::to_string(j) + " replaced by 0 (count=" +
                          std::to_string(nan_torque_count_) + ")");
    }
    d_->ctrl[a] = t;
  }
}

void MujocoBackend::setArmLocked(bool locked) { arm_locked_ = locked; }

void MujocoBackend::setWristCommand(const WristCommand& cmd) {
  wrist_cmd_ = cmd;
  if (const std::string w = wristCommandWarning(wrist_cmd_, nan_wrist_count_, "MuJoCo"); !w.empty()) log(kWarning, w);
}

// Ghost's Bullet sim (SimWrapper::armUnlock / sendWristCommand): the stowed arm is held at 0 rad
// and the wrist servo tracks pos_des with Bullet's constraint motor; MuJoCo velocity actuators
// reproduce that motor by driving the joint to the velocity it demands (gsim_core/bullet_motor.hpp).
// A tracker's gain follows the joint's inertia (see kTrackerGain); a switched-off tracker gets
// gain 0 (not only a zero force range), so no implicit damping remains.
void MujocoBackend::applyArm() {
  if (arm_.joints.empty()) return;
  const double dt = m_->opt.timestep;
  auto setTracker = [this, dt](int a, int dof, bool on, double target_vel, double force_limit) {
    on = on && force_limit > 0.0;   // Bullet's motor with max force 0 applies nothing
    const double kv = on ? kTrackerGain * std::max(d_->M[m_->mapM2M[m_->dof_Madr[dof]]], 1e-9) / dt : 0.0;
    m_->actuator_gainprm[a * mjNGAIN] = kv;
    m_->actuator_biasprm[a * mjNBIAS + 2] = -kv;
    m_->actuator_forcerange[2 * a] = -force_limit;
    m_->actuator_forcerange[2 * a + 1] = force_limit;
    d_->ctrl[a] = on ? target_vel : 0.0;
  };
  for (size_t i = 0; i < arm_.joints.size(); ++i) {
    const int id = arm_.joints[i];
    const int a = arm_.lock_actuator[i];
    const float q = static_cast<float>(d_->qpos[joints_.qposadr[id]]);
    const float v = static_cast<float>(d_->qvel[joints_.dofadr[id]]);
    setTracker(a, joints_.dofadr[id], arm_locked_, bulletMotorTargetVelocity(q, v, 0.f, 0.f, kArmLockGains, dt),
               kArmLockMaxForce);
    if (arm_locked_) d_->ctrl[joints_.actuator[id]] = 0.0;   // SDK torque not applied
  }
  const WristCommand& c = wrist_cmd_;
  const bool servo = c.mode == WristMode::kPositionOverCurrent;
  const bool current = c.mode == WristMode::kCurrent;
  for (int i = 0; i < 5; ++i) {
    const int k = i < 4 ? i : 3;   // the second jaw (jwrist_g2) follows jwrist_g
    const float q = static_cast<float>(d_->qpos[arm_.wrist_qposadr[i]]);
    const float v = static_cast<float>(d_->qvel[arm_.wrist_dofadr[i]]);
    setTracker(arm_.wrist_servo[i], arm_.wrist_dofadr[i], servo,
               bulletMotorTargetVelocity(q, v, c.pos_des[k], c.vel_des[k], kWristServoGains, dt),
               std::max(c.max_current[k], 0.f));
    d_->ctrl[arm_.wrist_torque[i]] = current ? c.max_current[k] : 0.0;
  }
  applied_wrist_ = c;
}

void MujocoBackend::readWrist(WristState& w) {
  if (!d_ || arm_.joints.empty()) return;
  ensureForward();
  for (int i = 0; i < 4; ++i) {
    w.q[i] = static_cast<float>(d_->qpos[arm_.wrist_qposadr[i]]);
    w.dq[i] = static_cast<float>(d_->qvel[arm_.wrist_dofadr[i]]);
  }
  switch (applied_wrist_.mode) {
    case WristMode::kPositionOverCurrent:
      for (int i = 0; i < 4; ++i) w.current[i] = static_cast<float>(d_->actuator_force[arm_.wrist_servo[i]]);
      break;
    case WristMode::kCurrent:
      w.current = applied_wrist_.max_current;
      break;
    default:
      w.current.setZero();
  }
}

// Ghost's SimWrapper::perceptionUpdate: a new frame every kStepsPerFrame calls, the first on the
// kStepsPerFrame-th. Calls only happen while the SDK has its cameras on, so the counter pauses with them.
bool MujocoBackend::updateDepthCamera(PointCloud& cloud_B) {
  if (!d_ || depth_failed_ || depth_cam_ < 0) return false;
  depth_calls_ = (depth_calls_ + 1) % FactoryDepthCamera::kStepsPerFrame;
  if (depth_calls_ != 0) return false;
  if (!egl_ && !startDepthRenderer()) return false;
  if (!egl_->makeCurrent()) {
    depth_failed_ = true;
    log(kWarning, "MuJoCo: depth camera disabled: its EGL context cannot be made current");
    return false;
  }
  const auto t0 = std::chrono::steady_clock::now();
  renderDepth(cloud_B);
  egl_->release();   // so that any thread can free it later
  depth_ms_ += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  constexpr int kStatsFrames = 150;   // about every 5 s at 30 Hz
  if (++depth_frames_ % kStatsFrames == 0) {
    char line[128];
    std::snprintf(line, sizeof(line), "MuJoCo: depth camera: %ld frames, %.1f ms per frame (last %d)", depth_frames_,
                  depth_ms_ / kStatsFrames, kStatsFrames);
    log(kInfo, line);
    depth_ms_ = 0.0;
  }
  return true;
}

bool MujocoBackend::startDepthRenderer() {
  using C = FactoryDepthCamera;
  egl_ = std::make_unique<EglContext>();
  std::string err;
  if (!egl_->create(&err)) {
    egl_.reset();
    depth_failed_ = true;
    log(kWarning, "MuJoCo: depth camera disabled: " + err);
    return false;
  }
  mjv_defaultScene(&depth_scn_);
  mjv_makeScene(m_, &depth_scn_, 2000);
  for (int f : {mjRND_SHADOW, mjRND_REFLECTION, mjRND_SKYBOX, mjRND_FOG}) depth_scn_.flags[f] = 0;
  mjr_defaultContext(&depth_con_);
  mjr_makeContext(m_, &depth_con_, mjFONTSCALE_50);
  mjr_resizeOffscreen(C::kWidth, C::kHeight, &depth_con_);   // only this context; the window keeps its own size
  depth_con_.readDepthMap = mjDEPTH_ZERONEAR;   // standard OpenGL depth (0 = near), as Bullet's buffer
  log(kInfo, "MuJoCo: depth camera renders with EGL on " + egl_->renderer());
  return true;
}

void MujocoBackend::renderDepth(PointCloud& cloud_B) {
  using C = FactoryDepthCamera;
  {
    // The scene is built from m_/d_, which the window also reads: same locks as step().
    std::lock_guard<std::mutex> viewer_guard(viewer_mtx_);
    MujocoViewer::Lock ui_lock;
    if (viewer_ && viewer_attached_ && !viewer_->closed()) ui_lock = viewer_->lock();
    ensureForward();
    mjvOption opt;
    mjv_defaultOption(&opt);
    mjvCamera cam;
    mjv_defaultCamera(&cam);
    cam.type = mjCAMERA_FIXED;
    cam.fixedcamid = depth_cam_;
    mjv_updateScene(m_, d_, &opt, nullptr, &cam, mjCAT_ALL, &depth_scn_);
  }
  // Ghost's projection: near 0.05 m, far 10 m (set on the scene, not on m_->vis, which the window uses).
  const float top = C::kNear * C::tanHalfFovy();
  for (int i = 0; i < 2; ++i) {
    mjvGLCamera& c = depth_scn_.camera[i];
    c.frustum_near = C::kNear;
    c.frustum_far = C::kFar;
    c.frustum_top = top;
    c.frustum_bottom = -top;
    c.frustum_center = 0.f;
    c.frustum_width = 0.f;   // from the viewport: 75 / 1500 = Ghost's aspect 0.05
  }
  const mjrRect viewport = {0, 0, C::kWidth, C::kHeight};
  mjr_setBuffer(mjFB_OFFSCREEN, &depth_con_);
  mjr_render(viewport, &depth_scn_, &depth_con_);
  depth_img_.resize(C::kWidth * C::kHeight);
  mjr_readPixels(nullptr, depth_img_.data(), viewport, &depth_con_);
  // OpenGL rows run bottom-up; Ghost's (Bullet's) image rows run top-down.
  for (int i = 0; i < C::kHeight / 2; ++i) {
    float* top_row = depth_img_.data() + i * C::kWidth;
    std::swap_ranges(top_row, top_row + C::kWidth, depth_img_.data() + (C::kHeight - 1 - i) * C::kWidth);
  }
  depthToBodyCloud(depth_img_.data(), cloud_B);
}

void MujocoBackend::freeDepthRenderer() {
  if (egl_ && egl_->makeCurrent()) {
    mjr_freeContext(&depth_con_);
    mjv_freeScene(&depth_scn_);
    egl_->release();
  }
  egl_.reset();
}

bool MujocoBackend::running() const { return !window_closed_.load(); }

double MujocoBackend::timestep() const { return m_ ? m_->opt.timestep : scene_timestep_; }

bool MujocoBackend::paused() const {
  std::lock_guard<std::mutex> g(viewer_mtx_);
  return viewer_ && viewer_->paused();
}

bool MujocoBackend::wantsMainThreadViewer() const { return !cfg_.headless; }

void MujocoBackend::runViewer(std::atomic_bool& keep_alive) {
  MujocoViewer* viewer;
  {
    std::lock_guard<std::mutex> g(viewer_mtx_);
    viewer_ = std::make_unique<MujocoViewer>(cfg_.log);
    viewer = viewer_.get();
  }
  viewer->run(keep_alive);
  window_closed_.store(true);
  std::lock_guard<std::mutex> g(viewer_mtx_);
  viewer_.reset();  // destroys the window on this (main) thread
}

void MujocoBackend::applyExternalForce(const Eigen::Vector3f& f_B, const Eigen::Vector3f& p_B) {
  ext_f_B_ = f_B;
  ext_p_B_ = p_B;
}

std::vector<std::string> MujocoBackend::selfTest(const RobotSpec& spec) {
  const ModelOptions opt = modelOptions(spec, cfg_);
  BuiltModel built = buildMujocoModel(opt);
  for (const auto& w : built.warnings) log(kInfo, "MuJoCo: " + w);
  if (!built.model) return {"model build failed: " + built.error};
  std::vector<std::string> problems = checkMujocoModel(built.model, built.joints, opt);
  mj_deleteModel(built.model);
  return problems;
}

}  // namespace gsim
