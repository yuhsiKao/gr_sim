#include "gsim_gazebo/GazeboBackend.hpp"

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

// ghost_sim links the SDK's octomap (liboctomap.so.1.10) and DART brings the system's
// (liboctomap.so.1.9). Loaded into one process with the default symbol lookup, the two copies
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

}  // namespace

struct GazeboBackend::Impl {
  SimConfig cfg;
  double timestep = 0.0;
  std::string world_sdf;     // the world file's text; the robot model is inserted at loadRobot
  std::string world_name;
  GazeboBuffers buffers;
  std::shared_ptr<GazeboSystem> system;
  std::unique_ptr<gz::sim::Server> server;
  bool reset_reported = false;
  long nan_torque_count = 0;
  long short_torque_count = 0;
  long nan_wrist_count = 0;
  // Depth camera (cfg.depth_camera): Gazebo renders it on its sim-time schedule every
  // kStepsPerFrame steps and publishes the linear depth; the image callback turns each frame into
  // OpenGL depth and updateDepthCamera waits for the frame of its step.
  bool depth_on = false;        // the camera is attached and working (off: not asked for, no EGL, or gave up)
  std::string depth_topic;
  std::unique_ptr<gz::transport::Node> depth_node;
  std::mutex depth_mu;
  std::condition_variable depth_cv;
  std::vector<float> depth_frame;   // latest frame as OpenGL depth, row 0 at the top (guarded by depth_mu)
  long depth_frame_step = -1;       // physics step it was rendered at (guarded by depth_mu)
  std::vector<float> depth_gl;      // the frame handed to depthToBodyCloud (sim thread only)
  long depth_frames = 0, depth_misses_in_row = 0, depth_miss_reports = 0;
  double depth_wait_ms = 0.0;
  pid_t gui_pid = 0;         // `gz sim -g`, a separate process that attaches over gz-transport
  bool dart_preloaded = false;

  // The camera's frames arrive on a gz-transport thread: accept only Ghost's 75 x 1500 float image,
  // turn it into OpenGL depth there and swap it in under the lock.
  void subscribeDepth() {
    depth_node = std::make_unique<gz::transport::Node>();
    const std::function<void(const gz::msgs::Image&)> cb = [this](const gz::msgs::Image& msg) {
      constexpr size_t kN = size_t(FactoryDepthCamera::kWidth) * FactoryDepthCamera::kHeight;
      if (msg.width() != unsigned(FactoryDepthCamera::kWidth) || msg.height() != unsigned(FactoryDepthCamera::kHeight) ||
          msg.pixel_format_type() != gz::msgs::PixelFormatType::R_FLOAT32 || msg.data().size() < kN * sizeof(float))
        return;
      std::vector<float> gl(kN);
      const auto* linear = reinterpret_cast<const float*>(msg.data().data());
      for (size_t k = 0; k < kN; ++k) gl[k] = glDepthFromLinear(linear[k]);
      const double stamp = msg.header().stamp().sec() + 1e-9 * msg.header().stamp().nsec();
      std::lock_guard<std::mutex> lock(depth_mu);
      depth_frame.swap(gl);
      depth_frame_step = std::lround(stamp / timestep);
      depth_cv.notify_all();
    };
    if (!depth_node->Subscribe("/" + depth_topic, cb)) {
      depth_on = false;
      log(kWarning, "Gazebo: depth camera disabled: cannot subscribe to /" + depth_topic);
    }
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
  GazeboModelOptions modelOptions(const RobotSpec& spec) const {
    GazeboModelOptions opt;
    opt.urdf_path = spec.urdf_path;
    opt.pos0_W = spec.pos0_W;
    opt.rpy0 = spec.rpy0;
    opt.fixed_base = spec.fixed_base;
    opt.toe_mu = cfg.toe_friction[0];
    opt.arm_joints = spec.arm_joints;
    opt.depth_camera = depth_on;
    opt.depth_topic = "gsim_depth_" + std::to_string(getpid());   // per process: two simulations never share frames
    if (timestep > 0.0) opt.timestep = timestep;
    return opt;
  }
};

GazeboBackend::GazeboBackend() : impl_(std::make_unique<Impl>()) {}

GazeboBackend::~GazeboBackend() {
  impl_->depth_node.reset();   // no more frame callbacks into this object
  impl_->stopGui();
  impl_->server.reset();
}

bool GazeboBackend::init(const SimConfig& cfg) {
  impl_->cfg = cfg;
  gz::common::Console::SetVerbosity(1);   // Gazebo's own console: errors only (it cannot be routed into cfg.log)
  impl_->dart_preloaded = preloadDartPlugin(&impl_->cfg);
  impl_->depth_on = cfg.depth_camera;
  if (cfg.depth_camera) {
    std::string why;
    if (!offscreenRenderingAvailable(&why)) {   // checked here, before ghost_sim asks supportsDepthCamera()
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
  impl_->world_name = world->Name();
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
  // ghost_sim hands Gazebo the world as a string, which loses the file's folder: relative mesh and
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

bool GazeboBackend::loadRobot(const RobotSpec& spec) {
  Impl& im = *impl_;
  const GazeboModelOptions opt = im.modelOptions(spec);
  std::string err;
  const std::string urdf = prepareUrdf(opt, &err);
  if (urdf.empty()) {
    im.log(kError, "Gazebo: " + err);
    return false;
  }
  const std::string model = urdfToSdfModel(urdf, opt, &err);
  if (model.empty()) {
    im.log(kError, "Gazebo: " + err);
    return false;
  }
  im.buffers = GazeboBuffers{};
  im.buffers.model_name = opt.model_name;
  im.buffers.num_sdk_joints = spec.num_sdk_joints;
  im.buffers.tau.assign(spec.num_sdk_joints, 0.0);
  im.buffers.arm_joints = spec.arm_joints;
  im.buffers.hide_body_from_sensors = opt.depth_camera;
  im.system = std::make_shared<GazeboSystem>(&im.buffers);

  gz::sim::ServerConfig config;
  im.depth_node.reset();
  im.depth_topic = opt.depth_topic;
  im.depth_frame_step = -1;
  im.depth_frames = im.depth_misses_in_row = 0;
  im.depth_wait_ms = 0.0;
  if (opt.depth_camera) config.SetHeadlessRendering(true);   // the Sensors system renders offscreen (EGL), window or not
  config.SetSdfString(worldWithModel(opt.depth_camera ? worldWithSensorsSystem(im.world_sdf) : im.world_sdf, model));
  // The Server installs its own SIGINT/SIGTERM handlers. ghost_sim's (the SDK's Ctrl-C flag that
  // stops SimLoop) must stay in charge of shutdown, so they are put back afterwards.
  struct sigaction on_int, on_term;
  sigaction(SIGINT, nullptr, &on_int);
  sigaction(SIGTERM, nullptr, &on_term);
  im.server = std::make_unique<gz::sim::Server>(config);
  sigaction(SIGINT, &on_int, nullptr);
  sigaction(SIGTERM, &on_term, nullptr);
  im.server->AddSystem(im.system);
  // One paused iteration: the system's Configure runs and the buffers hold the start state; sim
  // time does not advance. The server starts paused; stepping clears that.
  im.server->RunOnce(true);
  if (!im.buffers.configured) {
    im.log(kError, "Gazebo: the world has no model '" + opt.model_name + "' with a link 'body' after loading " + spec.urdf_path);
    im.server.reset();
    return false;
  }
  if (!spec.arm_joints.empty() && !im.buffers.arm_configured) {
    im.log(kError, "Gazebo: the arm joints or the wrist of the arm are missing from " + spec.urdf_path);
    im.server.reset();
    return false;
  }
  im.server->SetPaused(false);
  if (im.depth_on) im.subscribeDepth();
  im.reset_reported = false;
  if (!im.cfg.headless) im.startGui();
  return true;
}

void GazeboBackend::resetJoint(int sdk_joint, float q) {
  if (sdk_joint < 0 || sdk_joint >= impl_->buffers.num_sdk_joints) return;
  impl_->buffers.pending_resets.emplace_back(sdk_joint, double(q));
}

void GazeboBackend::step() {
  Impl& im = *impl_;
  if (!im.server) return;
  im.server->RunOnce(false);
  if (im.buffers.reset_detected && !im.reset_reported) {
    im.reset_reported = true;
    im.log(kError, "Gazebo: the world was reset behind the SDK's back (sim time went backwards); stopping");
  }
}

void GazeboBackend::readState(BaseState& base, Eigen::ArrayXf& q, Eigen::ArrayXf& dq) {
  // Resets requested before the next step (the start pose) must be readable now: a paused
  // iteration applies them and refreshes the buffers without advancing sim time.
  if (impl_->server && !impl_->buffers.pending_resets.empty()) {
    const bool was_paused = impl_->server->Paused().value_or(false);
    impl_->server->RunOnce(true);   // leaves the server paused, which must not outlive this call
    if (!was_paused) impl_->server->SetPaused(false);
  }
  const GazeboBuffers& b = impl_->buffers;
  base = b.base;
  q = b.q;
  dq = b.dq;
}

void GazeboBackend::setJointTorques(const Eigen::ArrayXf& tau) {
  Impl& im = *impl_;
  const int n = int(im.buffers.tau.size());   // 0 before loadRobot: nothing to write to
  if (n == 0) return;
  if (tau.size() != n && shouldReport(im.short_torque_count))
    im.log(kWarning, "Gazebo: torque vector length " + std::to_string(tau.size()) + " != " + std::to_string(n) +
                         " joints; missing entries get no torque (count=" + std::to_string(im.short_torque_count) + ")");
  int nans = 0;
  for (int j = 0; j < n; ++j) {
    const float v = j < tau.size() ? tau[j] : 0.f;
    const bool finite = std::isfinite(v);
    nans += !finite;
    im.buffers.tau[j] = finite ? double(v) : 0.0;
  }
  if (nans > 0) {
    if (shouldReport(im.nan_torque_count))   // shouldReport counts the call
      im.log(kWarning, "Gazebo: NaN/inf torque on " + std::to_string(nans) + " joint(s), applied as 0 (count=" +
                           std::to_string(im.nan_torque_count) + ")");
  }
}

bool GazeboBackend::running() const { return impl_->server && !impl_->buffers.reset_detected; }

double GazeboBackend::timestep() const { return impl_->timestep; }

bool GazeboBackend::paused() const {
  Impl& im = *impl_;
  if (!im.server) return false;
  const bool p = im.server->Paused().value_or(false);
  // While paused, the GUI's play button (a request on the world's control service) is only seen
  // by the next iteration: a paused iteration serves it without advancing physics.
  if (p) im.server->RunOnce(true);
  return p;
}

void GazeboBackend::applyExternalForce(const Eigen::Vector3f& f_B, const Eigen::Vector3f& p_B) {
  impl_->buffers.ext_f_B = f_B.cast<double>();
  impl_->buffers.ext_p_B = p_B.cast<double>();
}

std::vector<std::string> GazeboBackend::selfTest(const RobotSpec& spec) {
  const GazeboModelOptions opt = impl_->modelOptions(spec);
  std::string err;
  const std::string urdf = prepareUrdf(opt, &err);
  if (urdf.empty()) return {err};
  const std::string model = urdfToSdfModel(urdf, opt, &err);
  if (model.empty()) return {err};
  std::vector<std::string> problems = checkGazeboModel(model, opt.toe_mu, opt.arm_joints, opt.depth_camera);
  const double per_tick = 1.0 / (500.0 * impl_->timestep);
  if (impl_->timestep <= 0.0 || std::fabs(per_tick - std::round(per_tick)) > 1e-6)
    problems.push_back("max_step_size " + std::to_string(impl_->timestep) + " does not divide the SDK's 2 ms control period");
  return problems;
}

long GazeboBackend::nanTorqueCount() const { return impl_->nan_torque_count; }

double GazeboBackend::simTime() const { return impl_->buffers.sim_time; }

int GazeboBackend::guiPid() const { return impl_->gui_pid; }

bool GazeboBackend::dartPluginPreloaded() const { return impl_->dart_preloaded; }

void GazeboBackend::setArmLocked(bool locked) { impl_->buffers.arm_locked = locked; }

void GazeboBackend::setWristCommand(const WristCommand& cmd) {
  Impl& im = *impl_;
  im.buffers.wrist_cmd = cmd;
  if (const std::string w = wristCommandWarning(im.buffers.wrist_cmd, im.nan_wrist_count, "Gazebo"); !w.empty()) im.log(kWarning, w);
}

void GazeboBackend::readWrist(WristState& w) { w = impl_->buffers.wrist; }

double GazeboBackend::wristAxisTorque(int i) const {
  const GazeboBuffers& b = impl_->buffers;
  return b.arm_configured && i >= 0 && i < 4 ? b.wrist_axis_torque[i] : std::numeric_limits<double>::quiet_NaN();
}

long GazeboBackend::nanWristCount() const { return impl_->nan_wrist_count; }

bool GazeboBackend::supportsDepthCamera() const { return impl_->depth_on; }

bool GazeboBackend::updateDepthCamera(PointCloud& cloud_B) {
  Impl& im = *impl_;
  if (!im.depth_on || !im.server || im.timestep <= 0.0) return false;
  const long step = std::lround(im.buffers.sim_time / im.timestep);
  if (step <= 0 || step % FactoryDepthCamera::kStepsPerFrame != 0) return false;

  const auto t0 = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::milliseconds(im.depth_frames == 0 ? 2000 : 200);   // the first frame starts the renderer
  bool got = false;
  {
    std::unique_lock<std::mutex> lock(im.depth_mu);
    got = im.depth_cv.wait_for(lock, timeout, [&] { return im.depth_frame_step >= step; }) && im.depth_frame_step == step;
    if (got) im.depth_gl.swap(im.depth_frame);
  }
  if (!got) {
    ++im.depth_misses_in_row;
    if (im.depth_frames == 0 || im.depth_misses_in_row >= 10) {
      im.depth_on = false;
      im.log(kWarning, "Gazebo: depth camera disabled: no frame from Gazebo's renderer for step " + std::to_string(step) +
                           " (offscreen rendering unavailable?); the simulation goes on without it");
    } else if (shouldReport(im.depth_miss_reports)) {
      im.log(kWarning, "Gazebo: depth camera: no frame for step " + std::to_string(step));
    }
    return false;
  }
  im.depth_misses_in_row = 0;
  depthToBodyCloud(im.depth_gl.data(), cloud_B);
  im.depth_wait_ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  constexpr int kStatsFrames = 150;   // about every 5 s at 30 Hz
  if (++im.depth_frames % kStatsFrames == 0) {
    char line[128];
    std::snprintf(line, sizeof(line), "Gazebo: depth camera: %ld frames, %.2f ms wait per frame (last %d)", im.depth_frames,
                  im.depth_wait_ms / kStatsFrames, kStatsFrames);
    im.log(kInfo, line);
    im.depth_wait_ms = 0.0;
  }
  return true;
}

bool GazeboBackend::depthCameraDisabled() const { return impl_->cfg.depth_camera && !impl_->depth_on; }

double GazeboBackend::wristJointPosition(int i) const {
  const GazeboBuffers& b = impl_->buffers;
  return b.arm_configured && i >= 0 && i < 5 ? b.wrist_q5[i] : std::numeric_limits<double>::quiet_NaN();
}

}  // namespace gsim
