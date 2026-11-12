#include "gsim_core/sim_loop.hpp"

#include <chrono>
#include <cmath>
#include <sstream>
#include <thread>

namespace gsim {

int computeSubsteps(int control_rate_hz, double timestep_s) {
  if (control_rate_hz <= 0 || timestep_s <= 0.0) return -1;
  const double ratio = 1.0 / (control_rate_hz * timestep_s);
  const double rounded = std::round(ratio);
  if (rounded < 1.0 || std::fabs(ratio - rounded) > 1e-6) return -1;
  return static_cast<int>(rounded);
}

std::string physicsTimestepError(double timestep_s) {
  if (std::fabs(timestep_s - kPhysicsTimestep) < 1e-9) return "";
  std::ostringstream msg;
  msg << "physics timestep " << timestep_s << " s: Ghost's controller needs " << kPhysicsTimestep
      << " s (2000 Hz physics, as Ghost's Bullet sim; a coarser step changes the gait); set the world's "
         "max_step_size or the scene's timestep to " << kPhysicsTimestep;
  return msg.str();
}

std::string controlRateError(int control_rate_hz) {
  if (computeSubsteps(control_rate_hz, kPhysicsTimestep) >= 1) return "";
  return "gsim_control_rate: " + std::to_string(control_rate_hz) +
         " Hz is not a positive whole divisor of the 2000 Hz physics rate";
}

SimLoop::SimLoop(ISimBackend& backend, LoopHooks hooks, Pacer& pacer, int substeps, int num_sdk_joints, bool has_arm)
    : backend_(backend),
      hooks_(std::move(hooks)),
      pacer_(pacer),
      substeps_(substeps),
      has_arm_(has_arm),
      last_tau_(Eigen::ArrayXf::Zero(num_sdk_joints)) {}

void SimLoop::run(std::atomic_bool& keep_alive) {
  const double dt = backend_.timestep();
  bool was_paused = false;
  pacer_.reanchor(sim_time_);
  while (backend_.running() && keep_alive.load()) {
    if (backend_.paused()) {
      was_paused = true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
    if (was_paused) {
      pacer_.reanchor(sim_time_);  // resume without trying to catch up the paused time
      was_paused = false;
    }

    if (hooks_.external_force) {
      const auto [f_B, p_B] = hooks_.external_force();
      backend_.applyExternalForce(f_B, p_B);
    }
    backend_.step();
    const long ctr = steps_++;
    sim_time_ = steps_ * dt;
    backend_.readState(base_, q_, dq_);
    if (has_arm_) backend_.readWrist(wrist_);
    const WristState* wrist = has_arm_ ? &wrist_ : nullptr;
    if (hooks_.on_state) hooks_.on_state(base_, q_, dq_, wrist);

    if (ctr % substeps_ == 0) {
      if (hooks_.on_control_tick)
        hooks_.on_control_tick(TickInfo{sim_time_, base_, q_, dq_, last_tau_, wrist, has_arm_ ? &last_arm_ : nullptr});
      pacer_.wait(sim_time_);
    }

    if (hooks_.torques) {
      last_tau_ = hooks_.torques();
      backend_.setJointTorques(last_tau_);
    }
    if (hooks_.after_torques) hooks_.after_torques();
    if (has_arm_) {
      if (hooks_.arm_command) last_arm_ = hooks_.arm_command();
      backend_.setArmLocked(last_arm_.locked);
      backend_.setWristCommand(last_arm_.wrist);
    }
  }
}

}  // namespace gsim
