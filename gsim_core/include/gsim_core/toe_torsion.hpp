// Torsional friction of a toe on the ground, for engines whose contacts have none (DART).
// Ghost's Bullet toes have spinning_friction (a torque about the contact normal of up to
// mu * normal force) and a friction anchor (no creep below that limit). The legs have no
// vertical-axis joint, so this is what holds the body's yaw over planted feet.
//
// Model: an anchored spring-damper on the toe's twist about the contact normal, saturated at the
// friction limit. While it saturates the anchor follows the toe (slip); lifting the toe drops it.
#pragma once

namespace gsim {

struct ToeTorsion {
  static constexpr double kStiffness = 2000.0;   // [N m/rad] of the anchor
  static constexpr double kDamping = 20.0;       // [N m s/rad]
  double twist = 0.0;                            // [rad] toe rotation about the normal since it anchored

  // spin: the toe's angular velocity about the contact normal [rad/s]; normal_force [N] (<= 0: no
  // contact); mu [m]: the torsional friction coefficient. Returns the torque about the normal [N m].
  double update(double spin, double normal_force, double mu, double dt);
};

}  // namespace gsim
