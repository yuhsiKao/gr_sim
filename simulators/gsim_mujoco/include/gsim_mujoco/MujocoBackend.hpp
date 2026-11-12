// ISimBackend over MuJoCo. The model is built at run time from the SDK's
// URDF plus the scene file (see model_builder.hpp).
#pragma once
#include <mujoco/mujoco.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "gsim_mujoco/MujocoViewer.hpp"
#include "gsim_mujoco/egl_context.hpp"
#include "gsim_mujoco/model_builder.hpp"
#include "gsim_core/sim_backend.hpp"

namespace gsim {

class MujocoBackend : public ISimBackend {
 public:
  MujocoBackend();
  ~MujocoBackend() override;

  bool init(const SimConfig& cfg) override;
  bool loadRobot(const RobotSpec& spec) override;
  void resetJoint(int sdk_joint, float q) override;
  void step() override;
  void readState(BaseState& base, Eigen::ArrayXf& q, Eigen::ArrayXf& dq) override;
  void setJointTorques(const Eigen::ArrayXf& tau) override;
  bool running() const override;
  double timestep() const override;
  bool paused() const override;
  bool wantsMainThreadViewer() const override;
  void runViewer(std::atomic_bool& keep_alive) override;
  void applyExternalForce(const Eigen::Vector3f& f_B, const Eigen::Vector3f& p_B) override;
  std::vector<std::string> selfTest(const RobotSpec& spec) override;
  bool supportsArm() const override { return true; }
  void setArmLocked(bool locked) override;
  void setWristCommand(const WristCommand& cmd) override;
  void readWrist(WristState& w) override;
  bool supportsDepthCamera() const override { return true; }
  bool updateDepthCamera(PointCloud& cloud_B) override;

  // Diagnostics / tests.
  const mjModel* model() const { return m_; }
  mjData* mutableData() { return d_; }
  int instabilityCount() const { return instability_count_; }
  long nanTorqueCount() const { return nan_torque_count_; }

 private:
  void log(int level, const std::string& msg) const;
  void freeModel();
  void ensureForward();
  bool takeInstabilityWarnings();  // true if MuJoCo reset mjData since the last call
  void stepPhysics(MujocoViewer* viewer);
  void applyArm();                 // sets the arm and wrist actuators for the coming mj_step
  bool startDepthRenderer();       // EGL context + MuJoCo scene/context, on the calling thread
  void renderDepth(PointCloud& cloud_B);
  void freeDepthRenderer();

  SimConfig cfg_;
  double scene_timestep_ = 0.0;
  mjfLogHandler previous_log_handler_ = nullptr;
  bool log_handler_installed_ = false;

  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;
  JointMap joints_;
  ArmMap arm_;
  bool arm_locked_ = true;
  WristCommand wrist_cmd_;         // latest command (sanitized)
  WristCommand applied_wrist_;     // command used by the last mj_step (for the current report)
  long nan_wrist_count_ = 0;

  // Synthetic depth camera: rendered with its own EGL context on the thread that calls
  // updateDepthCamera (the simulation thread), never with the window's context.
  int depth_cam_ = -1;
  long depth_calls_ = 0;           // frame counter as Ghost's SimWrapper (perc_dec_ctr_)
  bool depth_failed_ = false;
  std::unique_ptr<EglContext> egl_;   // set once the scene and context below exist
  mjvScene depth_scn_;
  mjrContext depth_con_;
  std::vector<float> depth_img_;
  long depth_frames_ = 0;
  double depth_ms_ = 0.0;          // render time since the last stats line
  int base_body_ = -1;
  int num_joints_ = 0;
  bool floating_ = true;
  bool needs_forward_ = false;
  double model_timestep_ = 0.0;
  Eigen::Vector3f ext_f_B_ = Eigen::Vector3f::Zero(), ext_p_B_ = Eigen::Vector3f::Zero();
  int instability_count_ = 0;
  long nan_torque_count_ = 0;

  // Window (gsim_headless 0): created and destroyed by runViewer() on the main thread,
  // used by step() on the simulation thread; viewer_mtx_ guards the pointer.
  mutable std::mutex viewer_mtx_;
  std::unique_ptr<MujocoViewer> viewer_;
  bool viewer_attach_tried_ = false;
  bool viewer_attached_ = false;
  std::atomic_bool window_closed_{false};
};

}  // namespace gsim
