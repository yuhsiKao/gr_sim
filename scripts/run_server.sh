#!/usr/bin/env bash
# Start gsim_server: the physics for `ghost_sim gsim_backend remote`. It waits for one
# controller on the socket and stops when that controller leaves. Arguments are passed through:
#   scripts/run_server.sh gsim_server /tmp/gsim.sock                    # MuJoCo with viewer
#   scripts/run_server.sh gsim_server /tmp/gsim.sock gsim_headless 1
# then, in another terminal:
#   scripts/run_sim.sh gsim_backend remote gsim_server /tmp/gsim.sock
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"

if [ ! -x "$GSIM_PROJ/build/gsim_server" ]; then
  echo "run_server.sh: $GSIM_PROJ/build/gsim_server not found; build first:" >&2
  echo "  source scripts/env.sh && cmake -S . -B build && cmake --build build -j" >&2
  exit 1
fi

cd "$GSIM_PROJ"
exec build/gsim_server "$@"
