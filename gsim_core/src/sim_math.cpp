#include "gsim_core/sim_math.hpp"

#include <algorithm>
#include <cmath>

namespace gsim {

Eigen::Quaternionf quatFromRpy(const Eigen::Vector3f& rpy) {
  return Eigen::AngleAxisf(rpy.z(), Eigen::Vector3f::UnitZ()) *
         Eigen::AngleAxisf(rpy.y(), Eigen::Vector3f::UnitY()) *
         Eigen::AngleAxisf(rpy.x(), Eigen::Vector3f::UnitX());
}

Eigen::Vector3f rpyFromQuat(const Eigen::Quaternionf& q_WB) {
  const Eigen::Quaternionf q = q_WB.normalized();
  const float w = q.w(), x = q.x(), y = q.y(), z = q.z();
  const float roll = std::atan2(2.f * (w * x + y * z), 1.f - 2.f * (x * x + y * y));
  const float pitch = std::asin(std::clamp(2.f * (w * y - z * x), -1.f, 1.f));
  const float yaw = std::atan2(2.f * (w * z + x * y), 1.f - 2.f * (y * y + z * z));
  return {roll, pitch, yaw};
}

Eigen::Vector3f angvelBodyToWorld(const Eigen::Quaternionf& q_WB, const Eigen::Vector3f& w_B) {
  return q_WB.normalized() * w_B;
}

Eigen::Matrix<float, 6, 1> makeOrnState(const BaseState& b) {
  Eigen::Matrix<float, 6, 1> orn;
  orn << rpyFromQuat(b.quat_WB), b.angvel_W;
  return orn;
}

Eigen::Matrix<float, 6, 1> makeXW(const BaseState& b) {
  Eigen::Matrix<float, 6, 1> x;
  x << b.pos_W, b.vel_W;
  return x;
}

}  // namespace gsim
