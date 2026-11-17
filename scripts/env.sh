# Source this file (do not execute it): sets up the environment used by the
# gr_sim scripts and build.
#   source scripts/env.sh

GSIM_PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export GSIM_PROJ

# The Ghost SDK is treated as read-only; nothing in this project writes into it. Its path comes
# from the environment (the container sets it), on a host from the per-machine docker/.env, and
# otherwise it is the third_party/GhostSDK_1.0.1 submodule.
if [ -z "${CONTROLS_INSTALL_DIR:-}" ] && [ -f "$GSIM_PROJ/docker/.env" ]; then
  CONTROLS_INSTALL_DIR="$(sed -n 's/^CONTROLS_INSTALL_DIR=//p' "$GSIM_PROJ/docker/.env" | tail -n 1 | tr -d "\"'")"
fi
CONTROLS_INSTALL_DIR="${CONTROLS_INSTALL_DIR:-$GSIM_PROJ/third_party/GhostSDK_1.0.1/GhostSDK_bullet_linux}"
if [ ! -f "$CONTROLS_INSTALL_DIR/lib/libgrcontrols.a" ]; then
  echo "scripts/env.sh: no Ghost SDK at $CONTROLS_INSTALL_DIR." >&2
  echo "  Fetch the submodule (git submodule update --init) or set CONTROLS_INSTALL_DIR in docker/.env." >&2
  return 1
fi
export CONTROLS_INSTALL_DIR

case ":${LD_LIBRARY_PATH:-}:" in
  *":$CONTROLS_INSTALL_DIR/lib:"*) ;;
  *) export LD_LIBRARY_PATH="$CONTROLS_INSTALL_DIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ;;
esac

# The Python 3.10 interpreter for mavgui and examples/ (the SDK's grmblinkpy is built for 3.10):
# the ghost_mavgui conda env by default; the docker image sets its own.
export GSIM_PY="${GSIM_PY:-$HOME/miniconda3/envs/ghost_mavgui/bin/python}"

# Runtime files the Ghost controller writes (user_params_config.yaml next to the
# planner_config file, .grlocaldata in local_dir) must land here, never in the SDK.
export GSIM_RUN_DIR="$GSIM_PROJ/run"

# Copy the SDK's planner config into $GSIM_RUN_DIR/config (once) so that
# `planner_config $GSIM_RUN_DIR/config/default_settings.yaml` keeps writes out of the SDK.
gsim_prepare_run_dir() {
  mkdir -p "$GSIM_RUN_DIR/config"
  local f
  for f in default_settings.yaml user_params_config.yaml; do
    [ -f "$GSIM_RUN_DIR/config/$f" ] || cp "$CONTROLS_INSTALL_DIR/share/gr/examples/$f" "$GSIM_RUN_DIR/config/$f"
  done
}

# SDK arguments equivalent to `launch.py -s`, but with planner_config/local_dir redirected.
gsim_sdk_args() {
  echo planner_config "$GSIM_RUN_DIR/config/default_settings.yaml" local_dir "$GSIM_RUN_DIR" \
    extra_urdf_load_flags 0 dds_domain_id "${ROS_DOMAIN_ID:-124}" test -1 millis 0
}
