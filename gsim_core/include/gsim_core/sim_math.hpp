// Pure orientation helpers shared by the generic platform layer and backends.
// Euler angles follow Bullet's getEulerFromQuaternion convention used by the
// Ghost SDK: R_WB = Rz(yaw) * Ry(pitch) * Rx(roll), returned as (roll, pitch, yaw).
#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "gsim_core/sim_backend.hpp"

namespace gsim {

Eigen::Quaternionf quatFromRpy(const Eigen::Vector3f& rpy);
Eigen::Vector3f rpyFromQuat(const Eigen::Quaternionf& q_WB);
Eigen::Vector3f angvelBodyToWorld(const Eigen::Quaternionf& q_WB, const Eigen::Vector3f& w_B);

// SimLLC::stateUpdate inputs: orn_state = [roll, pitch, yaw, angvel_W], x_W = [pos_W, vel_W].
Eigen::Matrix<float, 6, 1> makeOrnState(const BaseState& b);
Eigen::Matrix<float, 6, 1> makeXW(const BaseState& b);

}  // namespace gsim
