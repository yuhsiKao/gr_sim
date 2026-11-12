// MuJoCo's simulate UI (built from source) in passive mode: physics stays in
// MujocoBackend::step() on the simulation thread; the window runs on the main thread and
// shows a copy of the state that syncIfDue() hands over at 60 Hz of simulated time.
#pragma once
#include <mujoco/mujoco.h>

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>

#include "gsim_mujoco/sync_schedule.hpp"
#include "gsim_core/sim_backend.hpp"

namespace mujoco {
class Simulate;
}

namespace gsim {

class MujocoViewer {
 public:
  using Lock = std::unique_lock<std::recursive_mutex>;  // = mujoco::MutexLock

  explicit MujocoViewer(LogFn log);
  ~MujocoViewer();  // main thread: destroys the window

  // Main thread. Opens the window and runs the render loop until the window is closed,
  // Quit is chosen, or keep_alive becomes false.
  void run(std::atomic_bool& keep_alive);

  // Simulation thread. Waits until run() has created the UI, shows m/d and tracks
  // track_body. Returns false if the window is already gone.
  bool attach(mjModel* m, mjData* d, int track_body);

  // The callers below hold lock() (the UI's mutex).
  Lock lock();
  void syncIfDue(double sim_time);
  void applyPerturb(const mjModel* m, mjData* d);  // mouse drag (Ctrl + right button) as a force

  bool closed() const;
  bool paused() const { return paused_.load(); }

 private:
  void log(int level, const std::string& msg) const;
  void togglePause();  // render thread, under the UI's mutex (from the key callback)

  LogFn log_;
  mjvCamera cam_;
  mjvOption opt_;
  mjvPerturb pert_;
  std::unique_ptr<mujoco::Simulate> sim_;

  std::mutex ready_mtx_;
  std::condition_variable ready_cv_;
  bool ready_ = false;     // sim_ constructed (or run() gave up)
  std::atomic_bool finished_{false};
  std::atomic_bool paused_{false};
  SyncSchedule sync_{1.0 / 60.0};  // simulated seconds between UI syncs
};

}  // namespace gsim
