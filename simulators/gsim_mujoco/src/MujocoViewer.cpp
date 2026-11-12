#include "gsim_mujoco/MujocoViewer.hpp"

#include <GLFW/glfw3.h>

#include <chrono>
#include <cstdio>
#include <functional>
#include <thread>
#include <utility>

#include "glfw_adapter.h"
#include "simulate.h"

namespace gsim {

namespace {

// simulate ignores Space (Pause/Run) in passive mode because it does not own the
// physics loop; here Space pauses the simulation loop instead.
class GlfwAdapterWithPause : public mujoco::GlfwAdapter {
 public:
  explicit GlfwAdapterWithPause(std::function<void()> on_space) : on_space_(std::move(on_space)) {}

 protected:
  void OnKey(int key, int scancode, int act) override {
    if (key == GLFW_KEY_SPACE && act == GLFW_PRESS) on_space_();
    GlfwAdapter::OnKey(key, scancode, act);
  }

 private:
  std::function<void()> on_space_;
};

}  // namespace

MujocoViewer::MujocoViewer(LogFn log) : log_(std::move(log)) {
  mjv_defaultCamera(&cam_);
  mjv_defaultOption(&opt_);
  mjv_defaultPerturb(&pert_);
}

MujocoViewer::~MujocoViewer() = default;

void MujocoViewer::log(int level, const std::string& msg) const {
  if (log_) log_(level, msg);
}

void MujocoViewer::run(std::atomic_bool& keep_alive) {
  auto set_ready = [this] {
    {
      std::lock_guard<std::mutex> g(ready_mtx_);
      ready_ = true;
    }
    ready_cv_.notify_all();
  };
  // Checked here because simulate's own GLFW failure path calls mju_error (exits).
  if (glfwInit() != GLFW_TRUE) {
    log(kError, "MuJoCo viewer: could not initialize GLFW (no display?); run with gsim_headless 1");
    finished_.store(true);
    set_ready();
    return;
  }
  sim_ = std::make_unique<mujoco::Simulate>(std::make_unique<GlfwAdapterWithPause>([this] { togglePause(); }),
                                            &cam_, &opt_, &pert_, /*is_passive=*/true);
  set_ready();

  std::atomic_bool render_done{false};
  std::thread watcher([&] {
    while (!render_done.load()) {
      if (!keep_alive.load()) sim_->exitrequest.store(1);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  });
  sim_->RenderLoop();
  render_done.store(true);
  watcher.join();

  {
    Lock lock(sim_->mtx);
    finished_.store(true);
  }
  sim_->cond_loadrequest.notify_all();  // wakes an attach() whose model was never loaded
}

bool MujocoViewer::attach(mjModel* m, mjData* d, int track_body) {
  {
    std::unique_lock<std::mutex> l(ready_mtx_);
    ready_cv_.wait(l, [this] { return ready_; });
  }
  if (closed()) return false;

  Lock lock(sim_->mtx);
  // Simulate::Load, but also giving up when the window closes before the model is loaded.
  sim_->mnew_ = m;
  sim_->dnew_ = d;
  std::snprintf(sim_->filename, sizeof(sim_->filename), "%s", "ghost_sim");
  sim_->loadrequest = 2;
  sim_->cond_loadrequest.wait(lock, [this] { return sim_->loadrequest == 0 || finished_.load(); });
  if (closed()) return false;

  cam_.type = mjCAMERA_TRACKING;
  cam_.trackbodyid = track_body;
  cam_.fixedcamid = -1;
  cam_.distance = 2.5;
  cam_.azimuth = 135.0;
  cam_.elevation = -20.0;
  sim_->camera = 1;  // "Tracking" in the Rendering panel
  sim_->pending_.ui_update_rendering = true;
  return true;
}

MujocoViewer::Lock MujocoViewer::lock() { return Lock(sim_->mtx); }

void MujocoViewer::syncIfDue(double sim_time) {
  if (!sync_.due(sim_time)) return;
  // Resetting the physics behind the SDK's back would desynchronize its state estimate
  // and gait; restarting ghost_sim is the supported way to start over.
  if (sim_->pending_.reset) {
    sim_->pending_.reset = false;
    log(kWarning, "MuJoCo viewer: Reset is not supported (restart ghost_sim to start over)");
  }
  if (sim_->pending_.load_key) {
    sim_->pending_.load_key = false;
    log(kWarning, "MuJoCo viewer: Load key is not supported (restart ghost_sim to start over)");
  }
  // Sync() copies a Joint slider (enabled while paused) into mjData when it differs from
  // the value it last showed; put it back instead.
  if (sim_->qpos_ != sim_->qpos_prev_) {
    sim_->qpos_ = sim_->qpos_prev_;
    log(kWarning, "MuJoCo viewer: moving joints with the Joint sliders is not supported (the SDK controls them)");
  }
  sim_->Sync();
}

void MujocoViewer::applyPerturb(const mjModel* m, mjData* d) { mjv_applyPerturbForce(m, d, &pert_); }

bool MujocoViewer::closed() const { return finished_.load() || (sim_ && sim_->exitrequest.load() != 0); }

void MujocoViewer::togglePause() {
  if (sim_->ui0.editsect || sim_->ui1.editsect) return;  // typing a space into a text field
  const bool paused = !paused_.load();
  paused_.store(paused);
  sim_->run = paused ? 0 : 1;  // PAUSE label and the (passive-mode greyed) Pause/Run radio
  sim_->pending_.ui_update_simulation = true;
  log(kInfo, paused ? "MuJoCo viewer: paused (Space to resume)" : "MuJoCo viewer: resumed");
}

}  // namespace gsim
