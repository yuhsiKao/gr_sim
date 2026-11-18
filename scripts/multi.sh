#!/usr/bin/env bash
# Several robots in one world, each controller in its own container (docker/compose.multi.yaml).
# Run on the host, from anywhere:
#   scripts/multi.sh up                  start the containers (gr-physics, gr-robot-0, gr-robot-1)
#   scripts/multi.sh server [args]       the physics and its window: gsim_server with 2 robots
#   scripts/multi.sh robot <i> [args]    the controller of robot i (ghost_sim, gsim_backend remote)
#   scripts/multi.sh mavgui <i> [args]   mavgui for robot i (arguments as scripts/mavgui.sh); with
#                                        --joystick on every robot, D-pad left / right switches which one it drives
#   scripts/multi.sh down                stop the containers
# Start the server first, then every robot; the simulation begins when all of them have joined.
# Robot i starts 1.5 m beside robot i - 1 unless gsim_spawn "x y yaw_deg" is given.
# GSIM_ROBOTS=<n> scripts/multi.sh server changes the number of robots the server waits for
# (compose.multi.yaml has two containers for robots). The server notes the number in run/robots,
# which is where the mavgui learn how many robots share the gamepad.
set -euo pipefail
proj="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
compose=(docker compose -f "$proj/docker/compose.multi.yaml")
sock=/root/gr_sim/run/gsim.sock   # in the repository folder, which every container mounts
run="$proj/run"

usage() {
  sed -n '2,14p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//' >&2
  exit 2
}

# inside <container> <command...>: runs the command in the container, interactively on a terminal.
inside() {
  local tty=-i
  [ -t 0 ] && tty=-it
  local container=$1
  shift
  exec docker exec "$tty" "$container" "$@"
}

robot_index() {
  case "${1:-}" in
    '' | *[!0-9]*) echo "multi.sh: expected a robot index (0, 1, ...), got '${1:-}'" >&2; exit 2 ;;
  esac
}

case "${1:-}" in
  up) exec "${compose[@]}" up -d ;;
  down) exec "${compose[@]}" down ;;
  server)
    shift
    robots="${GSIM_ROBOTS:-2}"
    count=(gsim_robots "$robots")
    for ((k = 1; k < $#; k++)); do   # gsim_robots among the arguments wins
      if [ "${!k}" = gsim_robots ]; then
        next=$((k + 1))
        robots="${!next}"
        count=()
      fi
    done
    # A new world: the gamepad starts on robot 0 again, and the mavgui see this world's robots.
    mkdir -p "$run"
    rm -f "$run/joystick_robot" "$run/robots"
    echo "$robots" > "$run/robots"
    inside gr-physics scripts/run_server.sh gsim_server "$sock" "${count[@]}" "$@"
    ;;
  robot)
    robot_index "${2:-}"
    i=$2
    shift 2
    spawn=(gsim_spawn "0 $(awk "BEGIN { print 1.5 * $i }") 0")
    for a in "$@"; do [ "$a" = gsim_spawn ] && spawn=(); done
    inside "gr-robot-$i" scripts/run_sim.sh gsim_backend remote gsim_server "$sock" gsim_robot "$i" "${spawn[@]}" "$@"
    ;;
  mavgui)
    robot_index "${2:-}"
    i=$2
    shift 2
    # with --joystick all mavgui share the gamepad: D-pad left / right hands it to another robot
    # (among the robots of the running server, see scripts/mavgui.sh)
    inside "gr-robot-$i" env GSIM_JOY_ROBOT="$i" ${GSIM_ROBOTS:+GSIM_JOY_ROBOTS="$GSIM_ROBOTS"} scripts/mavgui.sh "$@"
    ;;
  *) usage ;;
esac
