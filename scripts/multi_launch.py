#!/usr/bin/env python3
"""Several robots from one configuration file (config/multi_robots.yaml). Used by scripts/multi.sh:
  multi_launch.py compose <config> <docker/compose.multi.yaml>   the containers' compose file, on stdout
  multi_launch.py start <config> <repository>                    the world and every robot, from one terminal
"""
import os
import shlex
import signal
import subprocess
import sys
import threading
import time

try:
    import yaml
except ImportError:
    sys.exit("multi.sh: %s has no yaml module (sudo apt install python3-yaml)" % sys.executable)

MAX_ROBOTS = 8       # gsim_core/sim_world.hpp: kMaxRobots
FIRST_DOMAIN = 124   # robot i is in DDS domain 124 + i
PHYSICS = "gr-physics"   # the server's container
# The programs scripts/multi.sh starts in the containers, as pgrep and pkill find them.
SERVER = ("-x", "gsim_server")
CONTROLLER = ("-x", "ghost_sim")
MAVGUI = ("-f", r"mavgui_launch\.py")
READY_TIMEOUT = 60.0      # [s] for gsim_server to open its socket
STOP_TIMEOUT = 5.0        # [s] for a program to end once it should
EARLY_EXIT_GRACE = 1.0    # [s] a controller that ends this close to the world's end ended with it


class ConfigError(Exception):
    pass


def _text(v):
    if not isinstance(v, str):
        raise ValueError("expected a text")
    return v


def _flag(v):
    if not isinstance(v, bool):
        raise ValueError("expected true or false")
    return "1" if v else "0"


def _is_number(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _scalar(v):
    if isinstance(v, bool):
        return _flag(v)
    return format(v, "g") if isinstance(v, float) else str(v)


def _number(v):
    if not _is_number(v):
        raise ValueError("expected a number")
    return _scalar(v)


def _whole(v):
    if not isinstance(v, int) or isinstance(v, bool):
        raise ValueError("expected a whole number")
    return str(v)


def _spawn(v):
    if not isinstance(v, list) or len(v) != 3 or not all(_is_number(x) for x in v):
        raise ValueError("expected [x, y, yaw_deg]")
    return " ".join(_scalar(x) for x in v)


def _pairs(v):
    if not isinstance(v, list) or len(v) % 2 or not all(isinstance(x, (str, int, float)) for x in v):
        raise ValueError("expected [key, value, ...]")
    return [_scalar(x) for x in v]


def _mavgui(v):
    # joystick: the gamepad, which mavgui leaves for the keyboard when none is connected;
    # keyboard: the keyboard also while a gamepad is connected
    modes = {False: None, "joystick": ["--joystick"], "keyboard": []}
    if not isinstance(v, (bool, str)) or v not in modes:
        raise ValueError("expected false, joystick or keyboard")
    return modes[v]


# Key of the file -> the program's parameter and how its value is written.
WORLD_KEYS = {"backend": ("gsim_backend", _text), "headless": ("gsim_headless", _flag), "rtf": ("gsim_rtf", _number),
              "scene": ("gsim_scene", _text), "world": ("gsim_world", _text)}
ROBOT_KEYS = {"spawn": ("gsim_spawn", _spawn), "arm": ("arm", _flag), "control_rate": ("gsim_control_rate", _whole),
              "log": ("gsim_log", _text)}


def _mapping(v, where):
    if v is None:
        return {}
    if not isinstance(v, dict):
        raise ValueError("%sexpected a mapping" % (where + ": " if where else ""))
    return v


def _at(where, key, write, value):
    """write(value); its error names the place in the file."""
    try:
        return write(value)
    except ValueError as e:
        raise ValueError("%s.%s: %s" % (where, key, e)) from None


def _section(section, where, keys, other=()):
    """The parameters of one section, in the file's order with `args` last. `other`: the keys the caller reads."""
    args = []
    for key, value in section.items():
        if key in keys:
            name, write = keys[key]
            args += [name, _at(where, key, write, value)]
        elif key != "args" and key not in other:
            raise ValueError("%s: unknown key %r (known: %s)" % (where, key, ", ".join([*keys, *other, "args"])))
    return args + _at(where, "args", _pairs, section.get("args", []))


def load(path):
    """The validated configuration: {"server": [gsim_server's parameters], "robots": [{"args": [ghost_sim's
    parameters], "mavgui": None (no mavgui) or the arguments of scripts/mavgui.sh}]}."""
    try:
        with open(path) as f:
            top = _mapping(yaml.safe_load(f), "")
        for key in top:
            if key not in ("world", "robots"):
                raise ValueError("unknown key %r (known: world, robots)" % key)
        robots = top.get("robots")
        if not isinstance(robots, list) or not robots:
            raise ValueError("robots: at least one robot is needed, as a list")
        if len(robots) > MAX_ROBOTS:
            raise ValueError("robots: at most %d robots share a world, the file has %d" % (MAX_ROBOTS, len(robots)))
        world = _section(_mapping(top.get("world"), "world"), "world", WORLD_KEYS)
        cfg = {"server": ["gsim_robots", str(len(robots))] + world, "robots": []}
        for i, robot in enumerate(robots):
            where = "robots[%d]" % i
            robot = _mapping(robot, where)
            cfg["robots"].append({"args": _section(robot, where, ROBOT_KEYS, other=("mavgui",)),
                                  "mavgui": _at(where, "mavgui", _mavgui, robot.get("mavgui", False))})
        return cfg
    except (OSError, yaml.YAMLError, ValueError) as e:
        text = str(e)
        raise ConfigError(text if str(path) in text else "%s: %s" % (path, text)) from None


def robot_container(i):
    return "gr-robot-%d" % i


def compose_text(cfg, base_file):
    """The compose file for the configuration's robots: every container is the one of base_file
    (docker/compose.multi.yaml), a robot's with its name, its DDS domain and its folder in run/."""
    services = {PHYSICS: {"extends": {"file": base_file, "service": PHYSICS}}}
    for i in range(len(cfg["robots"])):
        services[robot_container(i)] = {
            "extends": {"file": base_file, "service": "gr-robot"},
            "container_name": robot_container(i),
            "environment": {"ROS_DOMAIN_ID": str(FIRST_DOMAIN + i), "GSIM_RUN_DIR": "/root/gr_sim/run/robot%d" % i},
        }
    return ("# Written by scripts/multi.sh from the robots' configuration file; edit that file or\n"
            "# docker/compose.multi.yaml instead.\n" +
            yaml.safe_dump({"services": services}, sort_keys=False))


def _exec_all(containers, *cmd):
    """Runs a command in all of these containers at once, quietly: {container: its exit code}."""
    quiet = {"stdin": subprocess.DEVNULL, "stdout": subprocess.DEVNULL, "stderr": subprocess.DEVNULL}
    running = {c: subprocess.Popen(["docker", "exec", c, *cmd], **quiet) for c in containers}
    return {c: p.wait() for c, p in running.items()}


def _wait_all(procs):
    """True when all of them have ended within STOP_TIMEOUT."""
    deadline = time.monotonic() + STOP_TIMEOUT
    for p in procs:
        try:
            p.wait(max(0.0, deadline - time.monotonic()))
        except subprocess.TimeoutExpired:
            return False
    return True


def _stop(clients, sig, program):
    """Ends a program in the containers of clients ({container: the docker exec that runs it}): sig, and
    SIGKILL where it still runs STOP_TIMEOUT later. By its name, in every one of these containers: docker
    exec passes no signal on, and the program goes on when its docker client is ended (a closed terminal
    ends them all). Returns the containers where it was killed."""
    _exec_all(clients, "pkill", sig, *program)
    if _wait_all(clients.values()):
        return []
    stuck = [c for c, p in clients.items() if p.poll() is None]
    _exec_all(stuck, "pkill", "-KILL", *program)
    _wait_all(clients.values())
    return stuck


def start(cfg, proj, out=sys.stderr):
    """gsim_server in this terminal, every robot's ghost_sim and mavgui in the background with
    their output in run/logs. Returns when the server has ended and everything else is stopped."""
    eol = "\r\n" if out.isatty() else "\n"   # `docker exec -t` keeps the terminal in raw mode while the server runs
    said = threading.Lock()

    def say(*lines):
        with said:
            try:
                out.write("".join("multi.sh: " + line + eol for line in lines))
                out.flush()
            except OSError:   # the terminal was closed
                pass

    n = len(cfg["robots"])
    containers = [PHYSICS] + [robot_container(i) for i in range(n)]
    multi_sh = os.path.join(proj, "scripts", "multi.sh")
    logs = os.path.join(proj, "run", "logs")
    sock = os.path.join(proj, "run", "gsim.sock")

    # What already runs in these containers is not ours to stop, and would fight over the ports.
    found = [_exec_all(containers, "pgrep", how, name)
             for how, name in (("-x", "|".join((SERVER[1], CONTROLLER[1]))), MAVGUI)]
    busy = [c for c in containers if any(codes[c] == 0 for codes in found)]
    if busy:
        say("a simulation or a mavgui is already running in " + ", ".join(busy) +
            ": stop it first (scripts/multi.sh down stops everything)")
        return 1

    os.makedirs(logs, exist_ok=True)
    for name in os.listdir(logs):   # of the run before, which may have had more robots
        if name.endswith(".log"):
            os.remove(os.path.join(logs, name))

    def background(what, i, args):
        cmd = [multi_sh, what, str(i), *args]
        with open(os.path.join(logs, "%s%d.log" % (what, i)), "w") as log:
            log.write("scripts/multi.sh " + shlex.join(cmd[1:]) + "\n")   # first line: the command, to run it by hand
            log.flush()
            return subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT)

    world_ended = threading.Event()

    def watch(i, proc):
        code = proc.wait()
        if world_ended.wait(EARLY_EXIT_GRACE):
            return
        log = "robot%d.log" % i
        try:
            with open(os.path.join(logs, log), errors="replace") as f:
                tail = f.read().splitlines()[1:][-5:]
        except OSError:
            tail = []
        say("robot %d's controller stopped (exit %d) while the world runs; the end of run/logs/%s:" % (i, code, log),
            *("  " + line for line in tail))

    # SIGTERM and SIGHUP end the launch as Ctrl-C does
    handlers = {s: signal.signal(s, signal.default_int_handler) for s in (signal.SIGTERM, signal.SIGHUP)}
    handlers[signal.SIGINT] = signal.getsignal(signal.SIGINT)
    server = None
    controllers, guis = {}, {}
    try:
        if os.path.lexists(sock):   # left by a server that was killed: the new one is ready when it makes its own
            os.remove(sock)
        say("%d robot%s; the controllers write to run/logs/robot<i>.log (scripts/multi.sh logs <i>)"
            % (n, "" if n == 1 else "s"))
        server = subprocess.Popen([multi_sh, "server", *cfg["server"]])
        deadline = time.monotonic() + READY_TIMEOUT
        while not os.path.exists(sock):
            if server.poll() is not None:
                return server.returncode or 1
            if time.monotonic() > deadline:
                say("gsim_server did not open its socket within %g s" % READY_TIMEOUT)
                return 1
            time.sleep(0.05)
        for i, robot in enumerate(cfg["robots"]):
            controllers[robot_container(i)] = proc = background("robot", i, robot["args"])
            threading.Thread(target=watch, args=(i, proc), daemon=True).start()
        for i, robot in enumerate(cfg["robots"]):
            if robot["mavgui"] is not None:
                guis[robot_container(i)] = background("mavgui", i, robot["mavgui"])
        return server.wait()
    except KeyboardInterrupt:
        return 130
    finally:
        world_ended.set()
        for s in handlers:   # the rest is short and must finish
            signal.signal(s, signal.SIG_IGN)
        if server is not None and _stop({PHYSICS: server}, "-INT", SERVER):
            say("gsim_server did not stop: killed")
        # The controllers leave when the server does; one that stays is interrupted, then killed.
        if not _wait_all(controllers.values()):
            killed = _stop({c: p for c, p in controllers.items() if p.poll() is None}, "-INT", CONTROLLER)
            say(*("robot %d's controller did not stop: killed" % i for i in range(n) if robot_container(i) in killed))
        _stop(guis, "-TERM", MAVGUI)
        for p in [server, *controllers.values(), *guis.values()]:
            if p is not None and p.poll() is None:
                p.kill()
        for s, handler in handlers.items():
            signal.signal(s, handler)


def main(argv):
    try:
        if len(argv) == 3 and argv[0] == "compose":
            sys.stdout.write(compose_text(load(argv[1]), argv[2]))
            return 0
        if len(argv) == 3 and argv[0] == "start":
            return start(load(argv[1]), argv[2])
    except ConfigError as e:
        print("multi.sh: %s" % e, file=sys.stderr)
        return 2
    except OSError as e:
        print("multi.sh: %s" % e, file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130
    print(__doc__.strip(), file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
