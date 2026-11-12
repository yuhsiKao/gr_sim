// Version-1 robot support policy and initial pose, shared by all backends.
#pragma once
#include <cstdint>
#include <optional>
#include <string>

#include "gsim_core/sim_backend.hpp"

namespace gsim {

// Vision 60 wrist joints in vision60_arm.urdf: the first 4 in the SDK's order (r1, p, r2, gripper);
// the 5th is the second jaw, which mirrors jwrist_g (as in Ghost's SimWrapper).
inline constexpr const char* kWristJointNames[5] = {"jwrist_r1", "jwrist_p", "jwrist_r2", "jwrist_g", "jwrist_g2"};

// Only the Vision 60 (NGR, robot_version < 0x60) is supported, with or without the arm
// (as Ghost's Bullet bsp: vision60_arm.urdf / vision60.urdf). Returns the URDF file name, or
// nullopt with a reason in *why.
std::optional<std::string> chooseV1Urdf(bool is_ngr, uint32_t robot_version, bool arm, std::string* why);

// Same initial pose as the SDK's Bullet resetSim(): dropped from z = 0.4 m
// (1.6 m when the base is fixed), rolled by pi when starting inverted.
RobotSpec makeRobotSpec(const std::string& urdf_path, int num_sdk_joints, bool fixed_base, bool inverted);

}  // namespace gsim
