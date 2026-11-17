// ISimWorld over MuJoCo: one model with every robot in it, built at run time from the SDK's
// URDF plus the scene file (see model_builder.hpp). MujocoBackend is this world with one robot.
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
#include "gsim_core/sim_world.hpp"

namespace gsim {

class MujocoWorld : public ISimWorld {
 public:
  MujocoWorld();
  ~MujocoWorld() override;

  bool init(const SimConfig& cfg) override;
  int addRobot(const RobotSpec& spec) override;
  bool build() override;
  void clear() override;
  int numRobots() const override { return static_cast<int>(robots_.size()); }
  void step() override;
  void resetJoint(int robot, int sdk_joint, float q) override;
  void readState(int robot, BaseState& base, Eigen::ArrayXf& q, Eigen::ArrayXf& dq) override;
  void setJointTorques(int robot, const Eigen::ArrayXf& tau) override;
  void applyExternalForce(int robot, const Eigen::Vector3f& f_B, const Eigen::Vector3f& p_B) override;
  bool running() const override;
  double timestep() const override;
  bool paused() const override;
  bool wantsMainThreadViewer() const override;
  void runViewer(std::atomic_bool& keep_alive) override;

  // Arm and depth camera of one robot, as in ISimBackend.
  void setArmLocked(int robot, bool locked);
  void setWristCommand(int robot, const WristCommand& cmd);
  void readWrist(int robot, WristState& w);
  bool updateDepthCamera(int robot, PointCloud& cloud_B);

  // Diagnostics / tests.
  const SimConfig& config() const { return cfg_; }
  const mjModel* model() const { return m_; }
  mjData* mutableData() { return d_; }
  int instabilityCount() const { return instability_count_; }
  long nanTorqueCount() const { return nan_torque_count_; }

 private:
  struct Robot {
    RobotSpec spec;
    BuiltRobot map;
    bool arm_locked = true;
    WristCommand wrist_cmd;          // latest command (sanitized)
    WristCommand applied_wrist;      // command used by the last mj_step (for the current report)
    long nan_wrist_count = 0;
    long depth_calls = 0;            // frame counter as Ghost's SimWrapper (perc_dec_ctr_)
    Eigen::Vector3f ext_f_B = Eigen::Vector3f::Zero(), ext_p_B = Eigen::Vector3f::Zero();
  };

  void log(int level, const std::string& msg) const;
  Robot* robot(int i);             // null before build() and for an index out of range
  void freeModel();
  void ensureForward();
  bool takeInstabilityWarnings();  // true if MuJoCo reset mjData since the last call
  void stepPhysics(MujocoViewer* viewer);
  void applyArm(Robot& r);         // sets the arm and wrist actuators for the coming mj_step
  bool startDepthRenderer();       // EGL context + MuJoCo scene/context, on the calling thread
  void renderDepth(int camera, PointCloud& cloud_B);
  void freeDepthRenderer();

  SimConfig cfg_;
  double scene_timestep_ = 0.0;
  mjfLogHandler previous_log_handler_ = nullptr;
  bool log_handler_installed_ = false;

  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;
  std::vector<Robot> robots_;

  // Synthetic depth cameras: rendered with their own EGL context on the thread that calls
  // updateDepthCamera (the simulation thread), never with the window's context.
  bool depth_failed_ = false;
  std::unique_ptr<EglContext> egl_;   // set once the scene and context below exist
  mjvScene depth_scn_;
  mjrContext depth_con_;
  std::vector<float> depth_img_;
  long depth_frames_ = 0;
  double depth_ms_ = 0.0;          // render time since the last stats line
  bool needs_forward_ = false;
  double model_timestep_ = 0.0;
  int instability_count_ = 0;
  long nan_torque_count_ = 0;

  // Window (gsim_headless 0): created and destroyed by runViewer() on the main thread,
  // used by step() on the simulation thread; viewer_mtx_ guards the pointer. It follows robot 0.
  mutable std::mutex viewer_mtx_;
  std::unique_ptr<MujocoViewer> viewer_;
  bool viewer_attach_tried_ = false;
  bool viewer_attached_ = false;
  std::atomic_bool window_closed_{false};
};

}  // namespace gsim
