#include "gsim_core/bullet_motor.hpp"

namespace gsim {

float bulletMotorTargetVelocity(float q, float v, float q_des, float v_des, BulletMotorGains g, double dt) {
  return static_cast<float>(g.kp * (q_des - q) / dt) + v + g.kd * (v_des - v);
}

}  // namespace gsim
