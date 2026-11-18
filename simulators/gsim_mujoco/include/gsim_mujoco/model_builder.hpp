// Builds the MuJoCo model at run time from the SDK's URDF (never a hand-edited
// MJCF) attached to a user-editable scene file, and checks it against the facts
// the Ghost controller relies on.
#pragma once
#include <mujoco/mujoco.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <string>
#include <vector>

#include "gsim_core/robot_select.hpp"   // kWristJointNames
#include "gsim_core/sim_world.hpp"      // kMaxRobots

namespace gsim {

// Velocity actuators reproduce Bullet's constraint motor (arm lock, wrist servo). Bullet solves
// for the impulse that reaches the target velocity and saturates only when that needs more than
// the force limit. The backend therefore sets each tracker's gain every step from the joint's
// inertia: kv = kTrackerGain * M_jj / dt, i.e. kTrackerGain times the force that removes the
// velocity error in one step. A fixed stiff gain instead saturates light joints (the 0.1 kg jaws)
// at once, and the clamped force then chatters step to step. MuJoCo integrates the velocity term
// implicitly (implicitfast), so the gain stays stable. Trackers are built switched off (kv = 0).
inline constexpr double kTrackerGain = 10.0;

// Ghost's synthetic depth camera (gsim_core/depth_camera.hpp), a camera on the robot's body.
inline constexpr const char* kDepthCameraName = "depth_underside";

// Contact dimension of the toes: 4 = sliding + torsional friction (ModelOptions::toe_friction[1]).
inline constexpr int kToeCondim = 4;

// One robot of a model.
struct RobotOptions {
  std::string urdf_path;                         // absolute
  int num_sdk_joints = 12;
  bool fixed_base = false;
  Eigen::Vector3d base_pos{0.0, 0.0, 0.4};
  Eigen::Quaterniond base_quat = Eigen::Quaterniond::Identity();
  std::vector<int> arm_joints;                   // SDK ids of the arm joints (root to tip); empty = no arm
};

// The robot a backend was asked to load, as model options.
RobotOptions robotOptions(const RobotSpec& spec);

// A model with one robot: the robot plus the world it stands in.
struct ModelOptions : RobotOptions {
  std::string scene_path;                        // absolute
  std::array<double, 3> toe_friction{3.0, 0.04, 0.0};   // MJCF order: slide, torsional, rolling
  std::array<double, 2> toe_solref{0.02, 1.0};
};

struct JointMap {                                // index = SDK joint id; -1 = missing
  std::vector<int> qposadr, dofadr, actuator;
};

// Arm actuators (empty vectors and -1 entries when the robot has no arm).
//   lock<id>:    velocity actuator per arm joint, holds the stowed arm (Bullet-style lock)
//   ws_<wrist>:  velocity actuator per wrist joint, position mode 4 (Bullet-style servo)
//   wt_<wrist>:  torque actuator per wrist joint, current mode 2
struct ArmMap {
  std::vector<int> joints, lock_actuator;        // same order as RobotOptions::arm_joints
  std::array<int, 5> wrist_qposadr{-1, -1, -1, -1, -1}, wrist_dofadr{-1, -1, -1, -1, -1};
  std::array<int, 5> wrist_servo{-1, -1, -1, -1, -1}, wrist_torque{-1, -1, -1, -1, -1};
  bool has_wrist = false;
};

// Where one robot's joints, actuators and camera are in the compiled model.
struct BuiltRobot {
  JointMap joints;
  ArmMap arm;
  int base_body = -1;
  int depth_camera = -1;                         // camera id of this robot's kDepthCameraName
  int free_qposadr = -1, free_dofadr = -1;       // the floating base's free joint; -1 with a fixed base
};

struct BuiltModel : BuiltRobot {
  mjModel* model = nullptr;                      // caller frees with mj_deleteModel
  std::string error;
  std::vector<std::string> warnings;             // MuJoCo warnings during the build (never written to disk)
};

// One robot; the same as buildMujocoWorld with a single entry.
BuiltModel buildMujocoModel(const ModelOptions& opt);

// --- several robots in one model ---------------------------------------------------------

struct WorldOptions {
  std::string scene_path;                        // absolute
  std::array<double, 3> toe_friction{3.0, 0.04, 0.0};   // MJCF order: slide, torsional, rolling
  std::array<double, 2> toe_solref{0.02, 1.0};
  std::vector<RobotOptions> robots;              // 1..kMaxRobots
};

struct BuiltWorld {
  mjModel* model = nullptr;                      // caller frees with mj_deleteModel
  std::vector<BuiltRobot> robots;                // same order as WorldOptions::robots
  std::string error;
  std::vector<std::string> warnings;             // MuJoCo warnings during the build (never written to disk)
};

// Name prefix of robot `robot` in a model of num_robots: none for a single robot (the model is
// then exactly the single-robot model), "r<i>_" otherwise. It applies to every body, joint, geom,
// mesh, actuator and camera of that robot.
std::string robotPrefix(int robot, int num_robots);

// The scene with every robot attached. Robots collide with the world and with each other, never
// with themselves.
BuiltWorld buildMujocoWorld(const WorldOptions& opt);

// Empty = model matches expectations; otherwise one message per failed check.
std::vector<std::string> checkMujocoModel(const mjModel* m, const JointMap& j, const ModelOptions& opt);

}  // namespace gsim
