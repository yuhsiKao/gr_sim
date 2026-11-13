// Ghost's URDF -> the SDF world string a gz::sim::Server loads. Pure functions (no Server), so
// the conversion is testable on its own and selfTest can run it without starting Gazebo.
#pragma once
#include <Eigen/Core>
#include <string>
#include <vector>

#include "gsim_core/robot_select.hpp"   // kWristJointNames

namespace gsim {

// Effort and velocity limit written into every revolute joint (the SDK limits torques itself);
// also what a joint gets back after a servo (force-limited) drive.
inline constexpr double kLiftedJointLimit = 1e6;

struct GazeboModelOptions {
  std::string urdf_path;            // absolute
  Eigen::Vector3f pos0_W{0.f, 0.f, 0.4f};
  Eigen::Vector3f rpy0{0.f, 0.f, 0.f};
  bool fixed_base = false;
  double toe_mu = 3.0;              // SimConfig::toe_friction[0] (sliding friction of the toes)
  std::string model_name = "vision60";
  std::vector<int> arm_joints;      // SDK ids of the arm joints, root to tip; empty = no arm
  bool depth_camera = false;        // Ghost's downward depth camera on the body (depthCameraSensorSdf)
  std::string depth_topic = "gsim_depth";
  double timestep = 0.0005;         // physics step [s]: the camera renders every FactoryDepthCamera::kStepsPerFrame steps
};

// The URDF file as the converter needs it: mesh paths absolute (file://), every revolute joint's
// effort and velocity limits lifted (the SDK limits torques itself, like the other backends; the
// arm lock and wrist servo set their own force limits per step), collision meshes given as .obj
// replaced by the .stl of the same name when it exists (the gripper jaws: the .obj has no
// normals, DART drops it and ODE aborts), and Gazebo tags for the toe friction and no
// self-collision. "" on failure (error set).
std::string prepareUrdf(const GazeboModelOptions& opt, std::string* error);

// The prepared URDF converted by sdformat into a <model> element string, named opt.model_name,
// placed at pos0_W / rpy0, and fixed to the world when opt.fixed_base. "" on failure (error set).
std::string urdfToSdfModel(const std::string& urdf, const GazeboModelOptions& opt, std::string* error);

// Ghost's downward depth camera (gsim_core FactoryDepthCamera) as a Gazebo depth_camera sensor for
// the body link: 75 x 1500 linear depth (R_FLOAT32) published on topic, rendered on Gazebo's
// sim-time schedule every kStepsPerFrame physics steps of timestep_s.
std::string depthCameraSensorSdf(const std::string& topic, double timestep_s);

// world_sdf with Gazebo's Sensors system (ogre2) added, unless the world already loads it.
std::string worldWithSensorsSystem(const std::string& world_sdf);

// world_sdf with model_sdf inserted before its closing </world>.
std::string worldWithModel(const std::string& world_sdf, const std::string& model_sdf);

// sdformat lumps each toe into its lower leg; the toe's collision there is named
// "<link>_fixed_joint_lump__toe<i>_collision_1".
inline constexpr const char* kToeCollisionTag = "fixed_joint_lump__toe";

// Problems with a converted model (empty = ok): the 12 revolute joints "0".."11", the 4 lumped toe
// collisions and their friction toe_mu; with arm_joints also those joints, the 5 wrist joints and
// the gripper jaw meshes (.stl, no .obj left); with depth_camera the camera on the body link.
std::vector<std::string> checkGazeboModel(const std::string& model_sdf, double toe_mu,
                                          const std::vector<int>& arm_joints = {}, bool depth_camera = false);

}  // namespace gsim
