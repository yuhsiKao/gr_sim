#!/usr/bin/env bash
# Start gsim_server: the physics for `ghost_sim gsim_backend remote`. It waits on the socket
# until every robot has a controller and stops when the last one has left. Arguments are passed
# through:
#   scripts/run_server.sh gsim_server /tmp/gsim.sock                    # MuJoCo with viewer
#   scripts/run_server.sh gsim_server /tmp/gsim.sock gsim_headless 1
#   scripts/run_server.sh gsim_server /tmp/gsim.sock gsim_robots 2      # two robots (scripts/multi.sh)
# then, in another terminal:
#   scripts/run_sim.sh gsim_backend remote gsim_server /tmp/gsim.sock
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"

gsim_require_binary gsim_server || exit 1

cd "$GSIM_PROJ"
exec build/gsim_server "$@"
