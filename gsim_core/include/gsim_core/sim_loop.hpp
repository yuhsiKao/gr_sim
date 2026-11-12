// Engine-agnostic simulation loop: steps the backend at its physics rate, feeds the
// state to the low-level controller every step and runs the SDK control tick every
// `substeps` steps (the SDK's CONTROL_RATE, 500 Hz by default), paced against wall-clock time.
#pragma once
#include <Eigen/Core>
#include <atomic>
#include <functional>
#include <string>
#include <utility>

#include "gsim_core/pacer.hpp"
#include "gsim_core/sim_backend.hpp"

namespace gsim {

// 1 / (rate * dt) when it is a positive integer (within 1e-6), otherwise -1.
int computeSubsteps(int control_rate_hz, double timestep_s);

// Ghost's controller is built for 2000 Hz physics (its Bullet sim's FIXED_TIMESTEP): with a 2 ms
// step the gait frequency more than doubled. Every backend must step at this size.
inline constexpr double kPhysicsTimestep = 0.0005;   // [s]

// "" when timestep_s is kPhysicsTimestep, otherwise why the simulation cannot start.
std::string physicsTimestepError(double timestep_s);

// "" when control_rate_hz divides the 2000 Hz physics rate (an integer number of physics steps
// per SDK control tick), otherwise an error naming gsim_control_rate.
std::string controlRateError(int control_rate_hz);

// Arm commands sent to the backend every step after the joint torques.
struct ArmCommand {
  bool locked = true;
  WristCommand wrist;
};

struct TickInfo {
  double sim_time;
  const BaseState& base;
  const Eigen::ArrayXf& q;
  const Eigen::ArrayXf& dq;
  const Eigen::ArrayXf& last_tau;      // torques sent on the previous step (zeros before the first)
  const WristState* wrist = nullptr;   // arm only: wrist state after this step
  const ArmCommand* last_arm = nullptr;  // arm only: arm command sent on the previous step (default before the first)
};

struct LoopHooks {
  std::function<std::pair<Eigen::Vector3f, Eigen::Vector3f>()> external_force;  // (f_B, p_B); may be empty
  // wrist is nullptr when the robot has no arm.
  std::function<void(const BaseState&, const Eigen::ArrayXf&, const Eigen::ArrayXf&, const WristState* wrist)> on_state;
  std::function<void(const TickInfo&)> on_control_tick;
  std::function<Eigen::ArrayXf()> torques;
  std::function<void()> after_torques;       // every step, after the torques are sent (Ghost's bsp: perception)
  std::function<ArmCommand()> arm_command;   // arm only: called every step after the torques are sent
};

class SimLoop {
 public:
  // has_arm: also read the wrist state and send the arm command every step.
  SimLoop(ISimBackend& backend, LoopHooks hooks, Pacer& pacer, int substeps, int num_sdk_joints, bool has_arm = false);

  // Runs until the backend stops running or keep_alive becomes false; never writes keep_alive.
  void run(std::atomic_bool& keep_alive);

  long steps() const { return steps_; }
  double simTime() const { return sim_time_; }

 private:
  ISimBackend& backend_;
  LoopHooks hooks_;
  Pacer& pacer_;
  int substeps_;
  bool has_arm_;
  long steps_ = 0;
  double sim_time_ = 0.0;
  BaseState base_;
  Eigen::ArrayXf q_, dq_, last_tau_;
  WristState wrist_;
  ArmCommand last_arm_;
};

}  // namespace gsim
