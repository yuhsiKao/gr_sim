#include "gsim_gazebo/GazeboWorld.hpp"

#include <gz/common/Console.hh>
#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/Util.hh>
#include <sdf/Element.hh>
#include <sdf/Physics.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <dlfcn.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <vector>
#include <limits>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <thread>

extern char** environ;

#include "egl_probe.hpp"
#include "gazebo_system.hpp"
#include "gsim_core/depth_camera.hpp"
#include "gsim_gazebo/gazebo_model.hpp"

namespace gsim {

namespace {

// A program that links the SDK (ghost_sim) has the SDK's octomap (liboctomap.so.1.10), and DART
// brings the system's (liboctomap.so.1.9). Loaded into one process with the default symbol lookup, the two copies
// interpose each other's globals and the process aborts at exit with a double free (the SDK's
// copy freeing what the other already freed). Loading the DART physics plugin first with
// RTLD_DEEPBIND keeps its dependencies bound to their own symbols; Gazebo's later dlopen of the
// same file reuses this handle. The plugin lives next to libgz-physics7: <libdir>/gz-physics-7/engine-plugins.
bool preloadDartPlugin(const SimConfig* cfg) {
  const auto warn = [cfg](const std::string& what) {
    if (cfg->log) cfg->log(kWarning, "Gazebo: could not pre-load the DART plugin (" + what + "); the simulation runs, but ghost_sim may abort at exit");
  };
  void* physics = dlopen("libgz-physics7.so.7", RTLD_NOW);
  if (!physics) return warn(dlerror()), false;
  char origin[4096] = "";
  if (dlinfo(physics, RTLD_DI_ORIGIN, origin) != 0) return warn("dlinfo: " + std::string(dlerror())), false;
  const std::string plugin = std::string(origin) + "/gz-physics-7/engine-plugins/libgz-physics7-dartsim-plugin.so";
  if (!dlopen(plugin.c_str(), RTLD_NOW | RTLD_LOCAL | RTLD_DEEPBIND)) return warn(dlerror()), false;
  return true;
}

// The robot as an SDF model; "" with *err set.
std::string sdfModel(const GazeboModelOptions& opt, std::string* err) {
  const std::string urdf = prepareUrdf(opt, err);
  return urdf.empty() ? "" : urdfToSdfModel(urdf, opt, err);
}

}  // namespace

struct GazeboWorld::Impl {
  struct Robot {
    RobotSpec spec;
    GazeboBuffers buffers;
    std::shared_ptr<GazeboSystem> system;
    long short_torque_count = 0;
    long nan_wrist_count = 0;
    // Depth camera: while we listen to its topic (depth_listening), Gazebo renders it on its
    // sim-time schedule every kStepsPerFrame steps and publishes the linear depth; the image
    // callback turns each frame into OpenGL depth.
    std::string depth_topic;
    bool depth_listening = false;
    long depth_wanted_step = 0;       // the step after which the robot last said it wants frames
    long depth_first_step = 0;        // the first step whose frame Gazebo renders for this robot
    std::vector<float> depth_frame;   // latest frame as OpenGL depth, row 0 at the top (guarded by depth_mu)
    long depth_frame_step = -1;       // physics step it was rendered at (guarded by depth_mu)
    std::vector<float> depth_gl;      // the frame last taken (sim thread only)
    long depth_gl_step = -1;          // its step
    long depth_misses_in_row = 0;
  };

  SimConfig cfg;
  double timestep = 0.0;
  std::string world_sdf;     // the world file's text; the robot models are inserted at build()
  std::vector<std::unique_ptr<Robot>> robots;   // the systems keep pointers into them
  std::unique_ptr<gz::sim::Server> server;
  bool reset_reported = false;
  long nan_torque_count = 0;
  bool depth_on = false;        // the cameras are attached and working (off: not asked for, no EGL, or gave up)
  std::unique_ptr<gz::transport::Node> depth_node;
  std::mutex depth_mu;
  std::condition_variable depth_cv;
  long depth_frames = 0, depth_miss_reports = 0;
  double depth_wait_ms = 0.0;
  pid_t gui_pid = 0;         // `gz sim -g`, a separate process that attaches over gz-transport
  bool dart_preloaded = false;

  Robot* robot(int i) const { return server && i >= 0 && i < int(robots.size()) ? robots[i].get() : nullptr; }
  double simTime() const { return robots.empty() ? 0.0 : robots[0]->buffers.sim_time; }
  long stepCount() const { return timestep > 0.0 ? std::lround(simTime() / timestep) : 0; }
  bool resetDetected() const { return !robots.empty() && robots[0]->buffers.reset_detected; }

  // Gazebo renders a camera only while somebody listens to its topic: listening is the switch.
  // The camera's frames arrive on a gz-transport thread: accept only Ghost's 75 x 1500 float image,
  // turn it into OpenGL depth there and swap it in under the lock.
  void cameraOn(Robot* r) {
    const std::function<void(const gz::msgs::Image&)> cb = [this, r](const gz::msgs::Image& msg) {
      constexpr size_t kN = size_t(FactoryDepthCamera::kWidth) * FactoryDepthCamera::kHeight;
      if (msg.width() != unsigned(FactoryDepthCamera::kWidth) || msg.height() != unsigned(FactoryDepthCamera::kHeight) ||
          msg.pixel_format_type() != gz::msgs::PixelFormatType::R_FLOAT32 || msg.data().size() < kN * sizeof(float))
        return;
      std::vector<float> gl(kN);
      const auto* linear = reinterpret_cast<const float*>(msg.data().data());
      for (size_t k = 0; k < kN; ++k) gl[k] = glDepthFromLinear(linear[k]);
      const double stamp = msg.header().stamp().sec() + 1e-9 * msg.header().stamp().nsec();
      std::lock_guard<std::mutex> lock(depth_mu);
      r->depth_frame.swap(gl);
      r->depth_frame_step = std::lround(stamp / timestep);
      depth_cv.notify_all();
    };
    if (!depth_node->Subscribe("/" + r->depth_topic, cb)) {
      depth_on = false;
      log(kWarning, "Gazebo: depth camera disabled: cannot subscribe to /" + r->depth_topic);
      return;
    }
    r->depth_listening = true;
    r->depth_first_step = stepCount() + 1;
  }
  void cameraOff(Robot* r) {
    depth_node->Unsubscribe("/" + r->depth_topic);
    r->depth_listening = false;
  }

  // Takes the frame Gazebo rendered at `step` into r.depth_gl. Waits for it: the renderer runs on
  // its own thread.
  bool fetchFrame(Robot& r, long step) {
    const auto t0 = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::milliseconds(r.depth_gl_step < 0 ? 2000 : 200);   // the first frame starts the renderer
    bool got = false;
    {
      std::unique_lock<std::mutex> lock(depth_mu);
      got = depth_cv.wait_for(lock, timeout, [&] { return r.depth_frame_step >= step; }) && r.depth_frame_step == step;
      if (got) r.depth_gl.swap(r.depth_frame);
    }
    if (!got) {
      ++r.depth_misses_in_row;
      if (r.depth_gl_step < 0 || r.depth_misses_in_row >= 10) {
        depth_on = false;
        log(kWarning, "Gazebo: depth camera disabled: no frame from Gazebo's renderer for step " + std::to_string(step) +
                          " (offscreen rendering unavailable?); the simulation goes on without it");
      } else if (shouldReport(depth_miss_reports)) {
        log(kWarning, "Gazebo: depth camera: no frame for step " + std::to_string(step));
      }
      return false;
    }
    r.depth_gl_step = step;
    r.depth_misses_in_row = 0;
    depth_wait_ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    constexpr int kStatsFrames = 150;   // about every 5 s at 30 Hz with one robot
    if (++depth_frames % kStatsFrames == 0) {
      char line[128];
      std::snprintf(line, sizeof(line), "Gazebo: depth camera: %ld frames, %.2f ms wait per frame (last %d)", depth_frames,
                    depth_wait_ms / kStatsFrames, kStatsFrames);
      log(kInfo, line);
      depth_wait_ms = 0.0;
    }
    return true;
  }

  // The frame Gazebo rendered at `step` (a step of the camera's schedule) as r's body-frame cloud.
  bool takeFrame(Robot& r, long step, PointCloud& cloud_B) {
    if (r.depth_gl_step != step && !fetchFrame(r, step)) return false;
    depthToBodyCloud(r.depth_gl.data(), cloud_B);
    return true;
  }

  // The window: the Gazebo GUI is its own process. Without a `gz` binary we just run headless.
  void startGui() {
    const char* argv[] = {"gz", "sim", "-g", nullptr};
    const int rc = posix_spawnp(&gui_pid, "gz", nullptr, nullptr, const_cast<char* const*>(argv), environ);
    if (rc != 0) {
      gui_pid = 0;
      log(kWarning, std::string("Gazebo: could not start `gz sim -g` (") + std::strerror(rc) + "): running without a window");
    }
  }
  void stopGui() {
    if (gui_pid <= 0) return;
    kill(gui_pid, SIGTERM);
    for (int i = 0; i < 200; ++i) {   // up to 2 s
      if (waitpid(gui_pid, nullptr, WNOHANG) != 0) { gui_pid = 0; return; }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    kill(gui_pid, SIGKILL);
    waitpid(gui_pid, nullptr, 0);
    gui_pid = 0;
  }

  void log(int level, const std::string& msg) const {
    if (cfg.log) cfg.log(level, msg);
  }
  // Robot `index` of `count`. A world with one robot keeps the names it always had.
  GazeboModelOptions modelOptions(const RobotSpec& spec, int index, int count) const {
    GazeboModelOptions opt;
    opt.urdf_path = spec.urdf_path;
    opt.pos0_W = spec.pos0_W;
    opt.rpy0 = spec.rpy0;
    opt.fixed_base = spec.fixed_base;
    opt.toe_mu = cfg.toe_friction[0];
    opt.arm_joints = spec.arm_joints;
    opt.depth_camera = depth_on;
    opt.depth_topic = "gsim_depth_" + std::to_string(getpid());   // per process: two simulations never share frames
    if (count > 1) {
      opt.model_name = "r" + std::to_string(index);
      opt.depth_topic += "_" + std::to_string(index);
    }
    if (timestep > 0.0) opt.timestep = timestep;
    return opt;
  }
  void stopServer() {
    depth_node.reset();   // no more frame callbacks into the robots
    server.reset();
  }
  void drop() {
    stopServer();
    robots.clear();
  }
};

GazeboWorld::GazeboWorld() : impl_(std::make_unique<Impl>()) {}

GazeboWorld::~GazeboWorld() { clear(); }

bool GazeboWorld::init(const SimConfig& cfg) {
  impl_->cfg = cfg;
  gz::common::Console::SetVerbosity(1);   // Gazebo's own console: errors only (it cannot be routed into cfg.log)
  impl_->dart_preloaded = preloadDartPlugin(&impl_->cfg);
  impl_->depth_on = cfg.depth_camera;
  if (cfg.depth_camera) {
    std::string why;
    if (!offscreenRenderingAvailable(&why)) {   // checked here, before anyone asks supportsDepthCamera()
      impl_->depth_on = false;
      impl_->log(kWarning, "Gazebo: depth camera disabled: offscreen rendering is unavailable (" + why + "); perception stays off");
    }
  }
  sdf::Root root;
  const sdf::Errors errors = root.Load(cfg.world_path);
  const sdf::World* world = root.WorldByIndex(0);
  if (!errors.empty() || !world) {
    std::string msg = "Gazebo: cannot load the world " + cfg.world_path;
    for (const auto& e : errors) msg += "; " + e.Message();
    impl_->log(kError, msg);
    return false;
  }
  const sdf::Physics* physics = world->PhysicsByIndex(0);
  if (!physics) {
    impl_->log(kError, "Gazebo: " + cfg.world_path + " has no <physics> element");
    return false;
  }
  impl_->timestep = physics->MaxStepSize();
  // Gazebo's default real_time_update_rate (1000 Hz) throttles RunOnce to that many steps per
  // second; pacing belongs to SimLoop (gsim_rtf), so the world must switch it off.
  const double rate = physics->Element() && physics->Element()->HasElement("real_time_update_rate")
                          ? physics->Element()->Get<double>("real_time_update_rate")
                          : 1000.0;
  if (rate != 0.0) {
    std::ostringstream w;
    w << "Gazebo: " << cfg.world_path << " has real_time_update_rate " << rate
      << ", so Gazebo throttles itself to " << rate << " steps per second (" << rate * impl_->timestep
      << "x real time); set <real_time_update_rate>0</real_time_update_rate> and leave pacing to gsim_rtf";
    impl_->log(kWarning, w.str());
  }
  // Gazebo gets the world as a string, which loses the file's folder: relative mesh and
  // texture URIs (e.g. a world's own meshes/ or textures/) would not resolve and Gazebo would refuse
  // the whole world. Add the folder to Gazebo's resource paths (GZ_SIM_RESOURCE_PATH, which the
  // `gz sim -g` window inherits as well).
  gz::sim::addResourcePaths({std::filesystem::absolute(cfg.world_path).parent_path().string()});
  std::ifstream f(cfg.world_path);
  std::stringstream ss;
  ss << f.rdbuf();
  impl_->world_sdf = ss.str();
  return true;
}

int GazeboWorld::addRobot(const RobotSpec& spec) {
  Impl& im = *impl_;
  if (im.server) {
    im.log(kError, "Gazebo: a robot cannot be added once the world is built");
    return -1;
  }
  if (numRobots() >= kMaxRobots) {
    im.log(kError, "Gazebo: a world holds at most " + std::to_string(kMaxRobots) + " robots");
    return -1;
  }
  auto r = std::make_unique<Impl::Robot>();
  r->spec = spec;
  im.robots.push_back(std::move(r));
  return numRobots() - 1;
}

bool GazeboWorld::build() {
  Impl& im = *impl_;
  im.stopServer();   // a second build() replaces the first
  const int count = numRobots();
  if (count == 0) {
    im.log(kError, "Gazebo: the world has no robots");
    return false;
  }
  std::string world = im.depth_on ? worldWithSensorsSystem(im.world_sdf) : im.world_sdf;
  for (int i = 0; i < count; ++i) {
    Impl::Robot& r = *im.robots[i];
    const GazeboModelOptions opt = im.modelOptions(r.spec, i, count);
    std::string err;
    const std::string model = sdfModel(opt, &err);
    if (model.empty()) {
      im.log(kError, "Gazebo: " + err);
      im.drop();
      return false;
    }
    world = worldWithModel(world, model);
    r.buffers = GazeboBuffers{};
    r.buffers.model_name = opt.model_name;
    r.buffers.num_sdk_joints = r.spec.num_sdk_joints;
    r.buffers.tau.assign(r.spec.num_sdk_joints, 0.0);
    r.buffers.arm_joints = r.spec.arm_joints;
    r.buffers.hide_body_from_sensors = opt.depth_camera;
    r.buffers.toe_torsional = im.cfg.toe_friction[1];
    r.system = std::make_shared<GazeboSystem>(&r.buffers);
    r.depth_topic = opt.depth_topic;
    r.depth_frame_step = r.depth_gl_step = -1;
    r.depth_listening = false;
    r.depth_wanted_step = r.depth_first_step = 0;
    r.depth_misses_in_row = 0;
  }
  im.depth_frames = 0;
  im.depth_wait_ms = 0.0;

  gz::sim::ServerConfig config;
  if (im.depth_on) config.SetHeadlessRendering(true);   // the Sensors system renders offscreen (EGL), window or not
  config.SetSdfString(world);
  // The Server installs its own SIGINT/SIGTERM handlers. The program's (the SDK's Ctrl-C flag that
  // stops SimLoop, gsim_server's stop flag) must stay in charge of shutdown, so they are put back.
  struct sigaction on_int, on_term;
  sigaction(SIGINT, nullptr, &on_int);
  sigaction(SIGTERM, nullptr, &on_term);
  im.server = std::make_unique<gz::sim::Server>(config);
  sigaction(SIGINT, &on_int, nullptr);
  sigaction(SIGTERM, &on_term, nullptr);
  for (auto& r : im.robots) im.server->AddSystem(r->system);
  // One paused iteration: the systems' Configure runs and the buffers hold the start state; sim
  // time does not advance. The server starts paused; stepping clears that.
  im.server->RunOnce(true);
  for (auto& r : im.robots) {
    std::string problem;
    if (!r->buffers.configured)
      problem = "the world has no model '" + r->buffers.model_name + "' with a link 'body' after loading " + r->spec.urdf_path;
    else if (!r->spec.arm_joints.empty() && !r->buffers.arm_configured)
      problem = "the arm joints or the wrist of the arm are missing from " + r->spec.urdf_path;
    if (problem.empty()) continue;
    im.log(kError, "Gazebo: " + problem);
    im.drop();
    return false;
  }
  im.server->SetPaused(false);
  if (im.depth_on) im.depth_node = std::make_unique<gz::transport::Node>();   // no camera is on yet
  im.reset_reported = false;
  if (!im.cfg.headless && im.gui_pid <= 0) im.startGui();
  return true;
}

void GazeboWorld::clear() {
  impl_->stopGui();
  impl_->drop();
}

int GazeboWorld::numRobots() const { return int(impl_->robots.size()); }

void GazeboWorld::resetJoint(int robot, int sdk_joint, float q) {
  Impl::Robot* r = impl_->robot(robot);
  if (!r || sdk_joint < 0 || sdk_joint >= r->buffers.num_sdk_joints) return;
  r->buffers.pending_resets.emplace_back(sdk_joint, double(q));
}

void GazeboWorld::step() {
  Impl& im = *impl_;
  if (!im.server) return;
  im.server->RunOnce(false);
  const long now = im.stepCount();
  for (auto& r : im.robots)
    if (r->depth_listening && now - r->depth_wanted_step >= FactoryDepthCamera::kStepsPerFrame) im.cameraOff(r.get());
  if (im.resetDetected() && !im.reset_reported) {
    im.reset_reported = true;
    im.log(kError, "Gazebo: the world was reset behind the SDK's back (sim time went backwards); stopping");
  }
}

void GazeboWorld::readState(int robot, BaseState& base, Eigen::ArrayXf& q, Eigen::ArrayXf& dq) {
  Impl& im = *impl_;
  Impl::Robot* r = im.robot(robot);
  if (!r) return;
  // Resets requested before the next step (the start pose) must be readable now: a paused
  // iteration applies them (every robot's) and refreshes the buffers without advancing sim time.
  bool pending = false;
  for (const auto& other : im.robots) pending = pending || !other->buffers.pending_resets.empty();
  if (pending) {
    const bool was_paused = im.server->Paused().value_or(false);
    im.server->RunOnce(true);   // leaves the server paused, which must not outlive this call
    if (!was_paused) im.server->SetPaused(false);
  }
  base = r->buffers.base;
  q = r->buffers.q;
  dq = r->buffers.dq;
}

void GazeboWorld::setJointTorques(int robot, const Eigen::ArrayXf& tau) {
  Impl& im = *impl_;
  Impl::Robot* r = im.robot(robot);
  if (!r) return;
  const int n = int(r->buffers.tau.size());
  if (tau.size() != n && shouldReport(r->short_torque_count))
    im.log(kWarning, "Gazebo: torque vector length " + std::to_string(tau.size()) + " != " + std::to_string(n) +
                         " joints; missing entries get no torque (count=" + std::to_string(r->short_torque_count) + ")");
  int nans = 0;
  for (int j = 0; j < n; ++j) {
    const float v = j < tau.size() ? tau[j] : 0.f;
    const bool finite = std::isfinite(v);
    nans += !finite;
    r->buffers.tau[j] = finite ? double(v) : 0.0;
  }
  if (nans > 0) {
    if (shouldReport(im.nan_torque_count))   // shouldReport counts the call
      im.log(kWarning, "Gazebo: NaN/inf torque on " + std::to_string(nans) + " joint(s), applied as 0 (count=" +
                           std::to_string(im.nan_torque_count) + ")");
  }
}

bool GazeboWorld::running() const { return !impl_->resetDetected(); }

double GazeboWorld::timestep() const { return impl_->timestep; }

bool GazeboWorld::paused() const {
  Impl& im = *impl_;
  if (!im.server) return false;
  const bool p = im.server->Paused().value_or(false);
  // While paused, the GUI's play button (a request on the world's control service) is only seen
  // by the next iteration: a paused iteration serves it without advancing physics.
  if (p) im.server->RunOnce(true);
  return p;
}

void GazeboWorld::applyExternalForce(int robot, const Eigen::Vector3f& f_B, const Eigen::Vector3f& p_B) {
  Impl::Robot* r = impl_->robot(robot);
  if (!r) return;
  r->buffers.ext_f_B = f_B.cast<double>();
  r->buffers.ext_p_B = p_B.cast<double>();
}

std::vector<std::string> GazeboWorld::selfTest(const RobotSpec& spec) const {
  const GazeboModelOptions opt = impl_->modelOptions(spec, 0, 1);
  std::string err;
  const std::string model = sdfModel(opt, &err);
  if (model.empty()) return {err};
  std::vector<std::string> problems = checkGazeboModel(model, opt.toe_mu, opt.arm_joints, opt.depth_camera);
  const double per_tick = 1.0 / (500.0 * impl_->timestep);
  if (impl_->timestep <= 0.0 || std::fabs(per_tick - std::round(per_tick)) > 1e-6)
    problems.push_back("max_step_size " + std::to_string(impl_->timestep) + " does not divide the SDK's 2 ms control period");
  return problems;
}

long GazeboWorld::nanTorqueCount() const { return impl_->nan_torque_count; }

double GazeboWorld::simTime() const { return impl_->simTime(); }

int GazeboWorld::guiPid() const { return impl_->gui_pid; }

bool GazeboWorld::dartPluginPreloaded() const { return impl_->dart_preloaded; }

void GazeboWorld::setArmLocked(int robot, bool locked) {
  if (Impl::Robot* r = impl_->robot(robot)) r->buffers.arm_locked = locked;
}

void GazeboWorld::setWristCommand(int robot, const WristCommand& cmd) {
  Impl& im = *impl_;
  Impl::Robot* r = im.robot(robot);
  if (!r) return;
  r->buffers.wrist_cmd = cmd;
  if (const std::string w = wristCommandWarning(r->buffers.wrist_cmd, r->nan_wrist_count, "Gazebo"); !w.empty()) im.log(kWarning, w);
}

void GazeboWorld::readWrist(int robot, WristState& w) {
  if (Impl::Robot* r = impl_->robot(robot)) w = r->buffers.wrist;
}

double GazeboWorld::wristAxisTorque(int robot, int i) const {
  const Impl::Robot* r = impl_->robot(robot);
  return r && r->buffers.arm_configured && i >= 0 && i < 4 ? r->buffers.wrist_axis_torque[i] : std::numeric_limits<double>::quiet_NaN();
}

double GazeboWorld::wristJointPosition(int robot, int i) const {
  const Impl::Robot* r = impl_->robot(robot);
  return r && r->buffers.arm_configured && i >= 0 && i < 5 ? r->buffers.wrist_q5[i] : std::numeric_limits<double>::quiet_NaN();
}

long GazeboWorld::nanWristCount(int robot) const {
  const Impl::Robot* r = impl_->robot(robot);
  return r ? r->nan_wrist_count : 0;
}

bool GazeboWorld::supportsDepthCamera() const { return impl_->depth_on; }

bool GazeboWorld::depthFrameAtLastStep() const {
  const Impl& im = *impl_;
  if (!im.depth_on) return false;
  const long step = im.stepCount();
  return step > 0 && step % FactoryDepthCamera::kStepsPerFrame == 0;
}

void GazeboWorld::wantDepthFrames(int robot) {
  Impl& im = *impl_;
  Impl::Robot* r = im.robot(robot);
  if (!r || !im.depth_on) return;
  r->depth_wanted_step = im.stepCount();
  if (!r->depth_listening) im.cameraOn(r);
}

bool GazeboWorld::updateDepthCamera(int robot, PointCloud& cloud_B) {
  wantDepthFrames(robot);
  return depthFrameAtLastStep() && renderDepthCamera(robot, cloud_B);   // the newest frame is this step's
}

bool GazeboWorld::renderDepthCamera(int robot, PointCloud& cloud_B) {
  Impl& im = *impl_;
  Impl::Robot* r = im.robot(robot);
  if (!r || !im.depth_on || !r->depth_listening) return false;
  const long step = im.stepCount();
  const long frame = step - step % FactoryDepthCamera::kStepsPerFrame;   // the newest step of the camera's schedule
  // Gazebo renders no frame for a step before the camera came on: there is none to wait for.
  return frame > 0 && frame >= r->depth_first_step && im.takeFrame(*r, frame, cloud_B);
}

bool GazeboWorld::bodySlab(int robot) const {
  const Impl::Robot* r = impl_->robot(robot);
  return r && r->buffers.body_slab;
}

bool GazeboWorld::depthCameraDisabled() const { return impl_->cfg.depth_camera && !impl_->depth_on; }

bool GazeboWorld::depthCameraOn(int robot) const {
  const Impl::Robot* r = impl_->robot(robot);
  return r && r->depth_listening;
}

}  // namespace gsim
