#include "gsim_gazebo/GazeboBackend.hpp"

namespace gsim {

bool GazeboBackend::loadRobot(const RobotSpec& spec) {
  world_.clear();
  return world_.addRobot(spec) == 0 && world_.build();
}

}  // namespace gsim
