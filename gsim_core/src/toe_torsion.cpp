#include "gsim_core/toe_torsion.hpp"

#include <algorithm>

namespace gsim {

double ToeTorsion::update(double spin, double normal_force, double mu, double dt) {
  const double limit = mu * normal_force;
  if (!(limit > 0.0)) {
    twist = 0.0;
    return 0.0;
  }
  twist += spin * dt;
  const double torque = -(kStiffness * twist + kDamping * spin);
  if (torque > limit || torque < -limit) {
    // Slipping: the anchor follows, staying one friction limit behind the toe.
    twist = std::clamp(twist, -limit / kStiffness, limit / kStiffness);
    return std::clamp(torque, -limit, limit);
  }
  return torque;
}

}  // namespace gsim
