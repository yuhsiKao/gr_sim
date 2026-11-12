#!/usr/bin/env bash
# Launch Ghost's official Bullet simulation (M0 baseline) without writing into the SDK.
#
# launch.py -s is NOT used: it hard-codes planner_config to the SDK's
# share/gr/examples/default_settings.yaml, and grcontrols_proc rewrites
# user_params_config.yaml next to that file. In sim mode launch.py only starts
# grcontrols_proc, so we start it directly with the same arguments, except that
# planner_config and local_dir point into $GSIM_RUN_DIR.
#
# grcontrols_proc locates URDFs by walking up from the working directory looking
# for a folder named `bin`, so we run it from $GSIM_PROJ/m0_run/bin with
# $GSIM_PROJ/m0_run/share/gr/urdf -> SDK urdf folder. m0_run/ only exists while the
# simulator runs: it is removed again when grcontrols_proc exits (window closed).
#
# Extra arguments are passed through, e.g. `scripts/m0_official_sim.sh arm 1`.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
gsim_prepare_run_dir

m0_dir="$GSIM_PROJ/m0_run"
# Removes only the link (never what it points to in the SDK) and empty folders; anything
# grcontrols_proc wrote into m0_run is left in place.
cleanup() {
  cd "$GSIM_PROJ"
  rm -f "$m0_dir/share/gr/urdf"
  rmdir "$m0_dir/share/gr" "$m0_dir/share" "$m0_dir/bin" "$m0_dir" 2>/dev/null ||
    echo "m0_official_sim.sh: $m0_dir is not empty, left in place" >&2
}
trap cleanup EXIT
mkdir -p "$m0_dir/bin" "$m0_dir/share/gr"
[ -e "$m0_dir/share/gr/urdf" ] || ln -s "$CONTROLS_INSTALL_DIR/share/gr/urdf" "$m0_dir/share/gr/urdf"

cd "$m0_dir/bin"
# shellcheck disable=SC2046
"$CONTROLS_INSTALL_DIR/bin/grcontrols_proc" $(gsim_sdk_args) "$@"
