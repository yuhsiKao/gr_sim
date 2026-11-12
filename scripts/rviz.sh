#!/usr/bin/env bash
# Show the SDK's perception output (heightmap, obstacle map) live in RViz2, as in Ghost's docs.
# The controller publishes it over DDS while vision_mode has the cameras on; Ghost's simulators
# (ghost_sim and the factory sim) use DDS domain 124, or $ROS_DOMAIN_ID if set. Works next to mavgui:
#   scripts/rviz.sh                  # ROS 2 from /opt/ros/jazzy
#   ROS_SETUP=/opt/ros/humble/setup.bash scripts/rviz.sh
set -eo pipefail
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
if [ ! -f "$ROS_SETUP" ]; then
  echo "rviz.sh: $ROS_SETUP not found; install ROS 2 or set ROS_SETUP" >&2
  exit 1
fi
# shellcheck disable=SC1090
source "$ROS_SETUP"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-124}"   # the same default as scripts/env.sh passes to the controller
exec rviz2 -d "$(dirname "${BASH_SOURCE[0]}")/rviz/perception.rviz" "$@"
