// Bullet's CONTROL_MODE_POSITION_VELOCITY_PD (btMultiBodyJointMotor), which Ghost's SimWrapper
// uses to lock the stowed arm and to servo the wrist. It is a
// constraint, not a torque law: each step the joint is asked to reach the velocity
//   v* = kp * (q_des - q) / dt + v + kd * (v_des - v)
// within +-max_force. Simulators without such a motor reproduce it by driving the joint
// velocity to v* every step.
#pragma once

namespace gsim {

struct BulletMotorGains {
  float kp, kd;   // dimensionless: fraction of the position / velocity error removed per step
};

inline constexpr BulletMotorGains kWristServoGains{1e-3f, 0.5f};   // SimWrapper::setupFullWrist
inline constexpr BulletMotorGains kArmLockGains{0.01f, 0.1f};      // SimWrapper::resetJointToRigid
inline constexpr float kArmLockMaxForce = 1000.f;                  // SimWrapper::resetJointToRigid

float bulletMotorTargetVelocity(float q, float v, float q_des, float v_des, BulletMotorGains g, double dt);

}  // namespace gsim
