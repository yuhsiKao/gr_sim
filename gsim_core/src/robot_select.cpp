#include "gsim_core/robot_select.hpp"

#include <cmath>

namespace gsim {

std::optional<std::string> chooseV1Urdf(bool is_ngr, uint32_t robot_version, bool arm, std::string* why) {
  std::string reason;
  if (!is_ngr)
    reason = "robot type is not NGR (Vision 60)";
  else if (robot_version >= 0x60)
    reason = "robot_version >= 0x60 (kpibot) is not supported";
  if (reason.empty()) return std::string(arm ? "vision60_arm.urdf" : "vision60.urdf");
  if (why) *why = reason + "; only the Vision 60 (with or without arm) is supported";
  return std::nullopt;
}

RobotSpec makeRobotSpec(const std::string& urdf_path, int num_sdk_joints, bool fixed_base, bool inverted) {
  RobotSpec s;
  s.urdf_path = urdf_path;
  s.num_sdk_joints = num_sdk_joints;
  s.fixed_base = fixed_base;
  s.pos0_W = {0.f, 0.f, fixed_base ? 1.6f : 0.4f};
  s.rpy0 = {inverted ? float(M_PI) : 0.f, 0.f, 0.f};
  return s;
}

}  // namespace gsim
