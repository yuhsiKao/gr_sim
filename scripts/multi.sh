#!/usr/bin/env bash
# Several robots in one world, each controller in its own container. Run on the host, from anywhere.
# The robots are listed in a configuration file, config/multi_robots.yaml unless another is given:
#   scripts/multi.sh start [config]      everything from this terminal: the containers, the physics
#                                        and its window here, every robot's controller and mavgui in
#                                        the background. Ctrl-C stops it all.
#   scripts/multi.sh logs <i>            follow the output of robot i's controller (run/logs/robot<i>.log)
#   scripts/multi.sh down                stop the containers
# One program per terminal, for a closer look at one of them:
#   scripts/multi.sh up [config]         start the containers: gr-physics and one gr-robot-<i> per robot
#   scripts/multi.sh server [args]       the physics and its window: gsim_server with 2 robots
#                                        (MuJoCo; `server gsim_backend gazebo` for Gazebo)
#   scripts/multi.sh robot <i> [args]    the controller of robot i (ghost_sim, gsim_backend remote)
#   scripts/multi.sh mavgui <i> [args]   mavgui for robot i (arguments as scripts/mavgui.sh); with
#                                        --joystick on every robot, D-pad left / right switches which one it drives
# By hand, start the server first, then every robot; the simulation begins when all of them have joined.
# Robot i starts 1.5 m beside robot i - 1 unless gsim_spawn "x y yaw_deg" is given.
# GSIM_ROBOTS=<n> scripts/multi.sh server changes the number of robots the server waits for. The
# server notes the number in run/robots, which is where the mavgui learn how many robots share the gamepad.
set -euo pipefail
proj="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
sock=/root/gr_sim/run/gsim.sock   # in the repository folder, which every container mounts
run="$proj/run"
# multi_launch.py needs PyYAML and runs with the system's python3, which `apt install python3-yaml`
# serves, whatever conda or virtual environment is active in the terminal.
python=/usr/bin/python3
[ -x "$python" ] || python=python3
launch=("$python" -B "$proj/scripts/multi_launch.py")
project=gr-multi   # the containers' compose project
# The containers' compose file: docker/compose.multi.yaml with one robot container per robot of the
# configuration file. `up` writes it.
compose_file="$run/compose.multi.yaml"

usage() {
  awk 'NR > 1 { if (!/^#/) exit; sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}" >&2   # the comment block above
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
  up | start)
    [ $# -le 2 ] || usage
    config="${2:-$proj/config/multi_robots.yaml}"
    mkdir -p "$run"
    # a configuration with a mistake stops here, before anything is written or started
    compose="$("${launch[@]}" compose "$config" "$proj/docker/compose.multi.yaml")"
    printf '%s\n' "$compose" > "$compose_file"
    # --remove-orphans: the containers of robots the configuration no longer has. -t 0: a container
    # is stopped at once; what waits in it is an idle shell, which only ends when it is killed.
    docker compose -p "$project" --project-directory "$proj/docker" -f "$compose_file" \
      up -d --remove-orphans -t 0
    [ "$1" = up ] || exec "${launch[@]}" start "$config" "$proj"
    ;;
  down) exec docker compose -p "$project" down --remove-orphans -t 0 ;;
  logs)
    robot_index "${2:-}"
    exec tail -n +1 -F "$run/logs/robot$2.log"
    ;;
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
