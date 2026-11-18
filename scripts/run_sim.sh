#!/usr/bin/env bash
# Start ghost_sim (Ghost controller + simulator backend) with the same SDK arguments
# as `launch.py -s`, but with planner_config/local_dir redirected into run/ so the SDK
# directory is never written. Extra arguments are passed through, e.g.:
#   scripts/run_sim.sh                       # MuJoCo with viewer (default)
#   scripts/run_sim.sh gsim_backend bullet   # Bullet reference backend
#   scripts/run_sim.sh gsim_headless 1 gsim_rtf 0
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"

if [ ! -f "$CONTROLS_INSTALL_DIR/lib/libgrcontrols.a" ]; then
  echo "run_sim.sh: CONTROLS_INSTALL_DIR=$CONTROLS_INSTALL_DIR is not a Ghost SDK directory" >&2
  exit 1
fi
gsim_require_binary ghost_sim || exit 1

gsim_prepare_run_dir
cd "$GSIM_PROJ"
# shellcheck disable=SC2046
exec build/ghost_sim $(gsim_sdk_args) "$@"
