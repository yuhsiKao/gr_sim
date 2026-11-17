#!/usr/bin/env bash
# Run Ghost's MBLink GUI against a local simulation (127.0.0.1).
# Uses a Python 3.10 env (`ghost_mavgui` conda env, or $GSIM_PY) because the SDK ships a
# prebuilt grmblinkpy for CPython 3.10 only. -B keeps __pycache__ out of the SDK.
#   scripts/mavgui.sh                    keyboard (gamepads are hidden from mavgui)
#   scripts/mavgui.sh --joystick         gamepad, through scripts/mavgui_launch.py
#   scripts/mavgui.sh --joystick-check   print what the gamepad would send, no GUI
#   scripts/mavgui.sh --record build/grip.csv [--joystick]
#                                        also record gripper telemetry to CSV (see mavgui_launch.py)
#   GSIM_JOY_ROBOT=<i> GSIM_JOY_ROBOTS=<n> scripts/mavgui.sh --joystick
#                                        one gamepad for n robots, each with its own mavgui: this
#                                        one is robot i; D-pad left / right hands the gamepad to another robot
#                                        (scripts/multi.sh mavgui <i> --joystick sets both)
# Other arguments are passed to mavgui.py. Both GUI modes start mavgui through
# scripts/mavgui_launch.py, which also fixes mavgui's drop-down lookups for pygame_gui >= 0.6.10.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"

py="$GSIM_PY"
[ -x "$py" ] || { echo "mavgui.sh: missing $py (create the ghost_mavgui conda env, see install.md, or set GSIM_PY)" >&2; exit 1; }

mode=keyboard
args=()
launch=()
while [ $# -gt 0 ]; do
  case "$1" in
    --joystick) mode=joystick ;;
    --joystick-check) mode=check ;;
    --record)
      [ $# -ge 2 ] || { echo "mavgui.sh: --record needs a CSV path" >&2; exit 2; }
      mkdir -p "$(dirname "$2")"
      launch=(--record "$(realpath -m "$2")"); shift ;;
    *) args+=("$1") ;;
  esac
  shift
done

cd "$GSIM_PROJ"
export PYTHONDONTWRITEBYTECODE=1
export SDL_AUDIODRIVER="${SDL_AUDIODRIVER:-dummy}"   # mavgui plays no sound; avoids ALSA errors without a sound device
mavgui="$CONTROLS_INSTALL_DIR/share/gr/mblink/examples/mavgui.py"
case "$mode" in
  keyboard)
    # mavgui ignores the keyboard while any gamepad is connected (and misreads Xbox axes
    # on Linux), so hide all gamepads from SDL.
    export SDL_GAMECONTROLLER_IGNORE_DEVICES_EXCEPT=0x0000/0x0000
    exec "$py" -B "$GSIM_PROJ/scripts/mavgui_launch.py" "${launch[@]}" "$mavgui" -s "${args[@]}" ;;
  joystick)
    # Several robots share the gamepad: the file every mavgui reads is in the common run/ folder.
    if [ -n "${GSIM_JOY_ROBOT:-}" ]; then
      launch+=(--select "$GSIM_JOY_ROBOT" "${GSIM_JOY_ROBOTS:-2}" "$GSIM_PROJ/run/joystick_robot")
    fi
    export SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS="${SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS:-1}"
    exec "$py" -B "$GSIM_PROJ/scripts/mavgui_launch.py" "${launch[@]}" "$mavgui" -s "${args[@]}" ;;
  check)
    exec "$py" -B "$GSIM_PROJ/scripts/mavgui_launch.py" --check ;;
esac
