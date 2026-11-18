#include "gsim_mujoco/MujocoWorld.hpp"

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

// MuJoCo's default log handler appends to MUJOCO_LOG.TXT in the working directory;
// while a world exists, its messages go to the world's log callback instead.
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

MujocoWorld::MujocoWorld() = default;

MujocoWorld::~MujocoWorld() {
  viewer_.reset();
  freeModel();
  if (log_handler_installed_) {
    mju_setLogHandler(previous_log_handler_);
    g_runtime_log = nullptr;
  }
}

void MujocoWorld::log(int level, const std::string& msg) const {
  if (cfg_.log) cfg_.log(level, msg);
}

MujocoWorld::Robot* MujocoWorld::robot(int i) {
  return d_ && i >= 0 && i < static_cast<int>(robots_.size()) ? &robots_[i] : nullptr;
}

void MujocoWorld::freeModel() {
  freeDepthRenderer();   // its MuJoCo context belongs to the old model
  if (d_) mj_deleteData(d_);
  if (m_) mj_deleteModel(m_);
  d_ = nullptr;
  m_ = nullptr;
}

void MujocoWorld::clear() {
  freeModel();
  robots_.clear();
}

bool MujocoWorld::init(const SimConfig& cfg) {
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

int MujocoWorld::addRobot(const RobotSpec& spec) {
  if (m_) {
    log(kError, "MuJoCo: a robot cannot be added once the model is built");
    return -1;
  }
  if (numRobots() >= kMaxRobots) {
    log(kError, "MuJoCo: a world holds at most " + std::to_string(kMaxRobots) + " robots");
    return -1;
  }
  Robot r;
  r.spec = spec;
  robots_.push_back(r);
  return numRobots() - 1;
}

bool MujocoWorld::build() {
  freeModel();
  WorldOptions opt;
  opt.scene_path = cfg_.scene_path;
  opt.toe_friction = cfg_.toe_friction;
  opt.toe_solref = cfg_.toe_solref;
  for (const Robot& r : robots_) opt.robots.push_back(robotOptions(r.spec));
  BuiltWorld built = buildMujocoWorld(opt);
  for (const auto& w : built.warnings) log(kInfo, "MuJoCo: " + w);
  if (!built.model) {
    log(kError, "MuJoCo: " + built.error);
    robots_.clear();
    return false;
  }
  // Arm joints and wrist are checked once here, so the per-step code can rely on them.
  for (size_t k = 0; k < robots_.size(); ++k) {
    const RobotSpec& spec = robots_[k].spec;
    if (spec.arm_joints.empty()) continue;
    const BuiltRobot& b = built.robots[k];
    bool complete = b.arm.has_wrist;
    for (size_t i = 0; i < b.arm.joints.size(); ++i) {
      const int id = b.arm.joints[i];
      complete = complete && id >= 0 && id < spec.num_sdk_joints && b.joints.qposadr[id] >= 0 && b.arm.lock_actuator[i] >= 0;
    }
    if (!complete) {
      log(kError, "MuJoCo: the arm joints or the wrist of the arm are missing from " + spec.urdf_path);
      mj_deleteModel(built.model);
      robots_.clear();
      return false;
    }
  }
  m_ = built.model;
  d_ = mj_makeData(m_);
  for (size_t k = 0; k < robots_.size(); ++k) {
    const RobotSpec spec = robots_[k].spec;
    robots_[k] = Robot{};
    robots_[k].spec = spec;
    robots_[k].map = built.robots[k];
  }
  model_timestep_ = m_->opt.timestep;
  mj_resetData(m_, d_);
  mj_forward(m_, d_);
  needs_forward_ = false;
  return true;
}

void MujocoWorld::resetJoint(int robot_index, int sdk_joint, float q) {
  const Robot* r = robot(robot_index);
  if (!r || sdk_joint < 0 || sdk_joint >= r->spec.num_sdk_joints || r->map.joints.qposadr[sdk_joint] < 0) return;
  d_->qpos[r->map.joints.qposadr[sdk_joint]] = q;
  d_->qvel[r->map.joints.dofadr[sdk_joint]] = 0.0;
  needs_forward_ = true;
}

void MujocoWorld::ensureForward() {
  if (needs_forward_) {
    mj_forward(m_, d_);
    needs_forward_ = false;
  }
}

// mj_check* reset mjData (which zeroes d->warning) and then count 1, so the count never
// grows past 1: any count is a new reset; clear it to catch the next one.
bool MujocoWorld::takeInstabilityWarnings() {
  bool reset = false;
  for (int w : {mjWARN_BADQACC, mjWARN_BADQPOS, mjWARN_BADQVEL}) {
    reset |= d_->warning[w].number > 0;
    d_->warning[w].number = 0;
  }
  return reset;
}

void MujocoWorld::step() {
  if (!d_) return;
  std::lock_guard<std::mutex> viewer_guard(viewer_mtx_);
  if (viewer_ && !viewer_attach_tried_) {
    viewer_attach_tried_ = true;
    ensureForward();
    viewer_attached_ = viewer_->attach(m_, d_, robots_.empty() ? -1 : robots_[0].map.base_body);
  }
  if (viewer_ && viewer_attached_ && !viewer_->closed()) {
    const auto ui_lock = viewer_->lock();
    stepPhysics(viewer_.get());
    viewer_->syncIfDue(d_->time);
  } else {
    stepPhysics(nullptr);
  }
}

void MujocoWorld::stepPhysics(MujocoViewer* viewer) {
  ensureForward();
  if (m_->opt.timestep != model_timestep_) {
    // The SDK runs a fixed number of physics steps per control tick (see bspBegin).
    log(kWarning, "MuJoCo: the physics timestep is fixed at " + std::to_string(model_timestep_) +
                      " s while the controller runs; change it in the scene file instead");
    m_->opt.timestep = model_timestep_;
  }
  mju_zero(d_->xfrc_applied, 6 * m_->nbody);
  if (viewer) viewer->applyPerturb(m_, d_);
  for (Robot& r : robots_) {
    if (!r.ext_f_B.isZero()) {
      // Body-frame force at a body-frame point -> world force + torque on the base body.
      const int base = r.map.base_body;
      Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>> R(d_->xmat + 9 * base);
      const Eigen::Vector3d f_W = R * r.ext_f_B.cast<double>();
      const Eigen::Vector3d tau_W = (R * r.ext_p_B.cast<double>()).cross(f_W);
      for (int i = 0; i < 3; ++i) {
        d_->xfrc_applied[6 * base + i] += f_W[i];
        d_->xfrc_applied[6 * base + 3 + i] += tau_W[i];
      }
    }
    applyArm(r);
  }
  mj_step(m_, d_);

  if (takeInstabilityWarnings()) {
    ++instability_count_;
    log(kWarning, "MuJoCo reset the simulation after an instability (count=" + std::to_string(instability_count_) + ")");
  }
}

void MujocoWorld::readState(int robot_index, BaseState& base, Eigen::ArrayXf& q, Eigen::ArrayXf& dq) {
  const Robot* r = robot(robot_index);
  if (!r) return;
  ensureForward();
  if (r->map.free_qposadr >= 0) {
    // Free joint: position and linear velocity are world frame; angular velocity is body frame.
    const mjtNum* p = d_->qpos + r->map.free_qposadr;
    const mjtNum* v = d_->qvel + r->map.free_dofadr;
    base.pos_W = Eigen::Vector3d(p[0], p[1], p[2]).cast<float>();
    base.quat_WB = Eigen::Quaterniond(p[3], p[4], p[5], p[6]).cast<float>();
    base.vel_W = Eigen::Vector3d(v[0], v[1], v[2]).cast<float>();
    base.angvel_W = angvelBodyToWorld(base.quat_WB, Eigen::Vector3d(v[3], v[4], v[5]).cast<float>());
  } else {
    const mjtNum* p = d_->xpos + 3 * r->map.base_body;
    const mjtNum* qt = d_->xquat + 4 * r->map.base_body;
    base.pos_W = Eigen::Vector3d(p[0], p[1], p[2]).cast<float>();
    base.quat_WB = Eigen::Quaterniond(qt[0], qt[1], qt[2], qt[3]).cast<float>();
    base.vel_W.setZero();
    base.angvel_W.setZero();
  }
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const int n = r->spec.num_sdk_joints;
  q.setConstant(n, nan);
  dq.setConstant(n, nan);
  for (int j = 0; j < n; ++j) {
    if (r->map.joints.qposadr[j] < 0) continue;
    q[j] = static_cast<float>(d_->qpos[r->map.joints.qposadr[j]]);
    dq[j] = static_cast<float>(d_->qvel[r->map.joints.dofadr[j]]);
  }
}

void MujocoWorld::setJointTorques(int robot_index, const Eigen::ArrayXf& tau) {
  const Robot* r = robot(robot_index);
  if (!r) return;
  const int n = std::min<int>(r->spec.num_sdk_joints, static_cast<int>(tau.size()));
  for (int j = 0; j < n; ++j) {
    const int a = r->map.joints.actuator[j];
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

void MujocoWorld::setArmLocked(int robot_index, bool locked) {
  if (Robot* r = robot(robot_index)) r->arm_locked = locked;
}

void MujocoWorld::setWristCommand(int robot_index, const WristCommand& cmd) {
  Robot* r = robot(robot_index);
  if (!r) return;
  r->wrist_cmd = cmd;
  if (const std::string w = wristCommandWarning(r->wrist_cmd, r->nan_wrist_count, "MuJoCo"); !w.empty()) log(kWarning, w);
}

// Ghost's Bullet sim (SimWrapper::armUnlock / sendWristCommand): the stowed arm is held at 0 rad
// and the wrist servo tracks pos_des with Bullet's constraint motor; MuJoCo velocity actuators
// reproduce that motor by driving the joint to the velocity it demands (gsim_core/bullet_motor.hpp).
// A tracker's gain follows the joint's inertia (see kTrackerGain); a switched-off tracker gets
// gain 0 (not only a zero force range), so no implicit damping remains.
void MujocoWorld::applyArm(Robot& r) {
  const ArmMap& arm = r.map.arm;
  const JointMap& joints = r.map.joints;
  if (arm.joints.empty()) return;
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
  for (size_t i = 0; i < arm.joints.size(); ++i) {
    const int id = arm.joints[i];
    const int a = arm.lock_actuator[i];
    const float q = static_cast<float>(d_->qpos[joints.qposadr[id]]);
    const float v = static_cast<float>(d_->qvel[joints.dofadr[id]]);
    setTracker(a, joints.dofadr[id], r.arm_locked, bulletMotorTargetVelocity(q, v, 0.f, 0.f, kArmLockGains, dt),
               kArmLockMaxForce);
    if (r.arm_locked) d_->ctrl[joints.actuator[id]] = 0.0;   // SDK torque not applied
  }
  const WristCommand& c = r.wrist_cmd;
  const bool servo = c.mode == WristMode::kPositionOverCurrent;
  const bool current = c.mode == WristMode::kCurrent;
  for (int i = 0; i < 5; ++i) {
    const int k = i < 4 ? i : 3;   // the second jaw (jwrist_g2) follows jwrist_g
    const float q = static_cast<float>(d_->qpos[arm.wrist_qposadr[i]]);
    const float v = static_cast<float>(d_->qvel[arm.wrist_dofadr[i]]);
    setTracker(arm.wrist_servo[i], arm.wrist_dofadr[i], servo,
               bulletMotorTargetVelocity(q, v, c.pos_des[k], c.vel_des[k], kWristServoGains, dt),
               std::max(c.max_current[k], 0.f));
    d_->ctrl[arm.wrist_torque[i]] = current ? c.max_current[k] : 0.0;
  }
  r.applied_wrist = c;
}

void MujocoWorld::readWrist(int robot_index, WristState& w) {
  const Robot* r = robot(robot_index);
  if (!r || r->map.arm.joints.empty()) return;
  const ArmMap& arm = r->map.arm;
  ensureForward();
  for (int i = 0; i < 4; ++i) {
    w.q[i] = static_cast<float>(d_->qpos[arm.wrist_qposadr[i]]);
    w.dq[i] = static_cast<float>(d_->qvel[arm.wrist_dofadr[i]]);
  }
  switch (r->applied_wrist.mode) {
    case WristMode::kPositionOverCurrent:
      for (int i = 0; i < 4; ++i) w.current[i] = static_cast<float>(d_->actuator_force[arm.wrist_servo[i]]);
      break;
    case WristMode::kCurrent:
      w.current = r->applied_wrist.max_current;
      break;
    default:
      w.current.setZero();
  }
}

// Ghost's SimWrapper::perceptionUpdate: a new frame every kStepsPerFrame calls, the first on the
// kStepsPerFrame-th. Calls only happen while the SDK has its cameras on, so the counter pauses with them.
bool MujocoWorld::updateDepthCamera(int robot_index, PointCloud& cloud_B) {
  Robot* r = robot(robot_index);
  return r && depthFrameDue(r->depth_calls) && renderDepthCamera(robot_index, cloud_B);
}

bool MujocoWorld::renderDepthCamera(int robot_index, PointCloud& cloud_B) {
  Robot* r = robot(robot_index);
  if (!r || depth_failed_ || r->map.depth_camera < 0) return false;
  if (!egl_ && !startDepthRenderer()) return false;
  if (!egl_->makeCurrent()) {
    depth_failed_ = true;
    log(kWarning, "MuJoCo: depth camera disabled: its EGL context cannot be made current");
    return false;
  }
  const auto t0 = std::chrono::steady_clock::now();
  renderDepth(r->map.depth_camera, cloud_B);
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

bool MujocoWorld::startDepthRenderer() {
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

void MujocoWorld::renderDepth(int camera, PointCloud& cloud_B) {
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
    cam.fixedcamid = camera;
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

void MujocoWorld::freeDepthRenderer() {
  if (egl_ && egl_->makeCurrent()) {
    mjr_freeContext(&depth_con_);
    mjv_freeScene(&depth_scn_);
    egl_->release();
  }
  egl_.reset();
}

bool MujocoWorld::running() const { return !window_closed_.load(); }

double MujocoWorld::timestep() const { return m_ ? m_->opt.timestep : scene_timestep_; }

bool MujocoWorld::paused() const {
  std::lock_guard<std::mutex> g(viewer_mtx_);
  return viewer_ && viewer_->paused();
}

bool MujocoWorld::wantsMainThreadViewer() const { return !cfg_.headless; }

void MujocoWorld::runViewer(std::atomic_bool& keep_alive) {
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

void MujocoWorld::applyExternalForce(int robot_index, const Eigen::Vector3f& f_B, const Eigen::Vector3f& p_B) {
  Robot* r = robot(robot_index);
  if (!r) return;
  r->ext_f_B = f_B;
  r->ext_p_B = p_B;
}

}  // namespace gsim
