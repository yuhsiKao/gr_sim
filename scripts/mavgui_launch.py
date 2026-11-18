#!/usr/bin/env python3
"""Run Ghost's mavgui.py (read-only, straight from the SDK) with a working gamepad mapping
and drop-down menus that work with pygame_gui >= 0.6.10 (see DropDownChoice).

  mavgui_launch.py [--record CSV] [--select ROBOT ROBOTS FILE] MAVGUI_PY [mavgui args...]
                                                 used by `scripts/mavgui.sh`
      --record CSV   also write the gripper telemetry (current, position, setpoint, arm/wrist
                     FSM) to CSV with wall time, to line up with the grcontrols_proc log
      --select ROBOT ROBOTS FILE
                     one gamepad for several robots, each with its own mavgui: this mavgui is
                     robot ROBOT of ROBOTS, and FILE (shared by all of them) says which robot
                     the gamepad drives. D-pad right hands it to the next robot, D-pad left to
                     the previous one; the others see a gamepad at rest. See RobotSelector.
  mavgui_launch.py --check                       print what mavgui would send from the gamepad

mavgui reads the first joystick assuming the XInput axis order 0 LX, 1 LY, 2 RX, 3 RY,
4 LT, 5 RT: forward = right stick up/down, turn = right stick left/right, sideways = left
stick left/right, body height = left stick up/down. On Linux SDL reports Xbox controllers
as 0 LX, 1 LY, 2 LT, 3 RX, 4 RY, 5 RT, so mavgui's turn read the left trigger (-1 at rest:
a constant full-speed turn) and forward read the right stick's left/right. This launcher
hands mavgui every Xbox controller through RemappedJoystick and removes stick drift with a
dead zone. Nothing in the SDK is modified; run it with `python -B`.
"""
import csv
import datetime
import os
import runpy
import sys
import time

STICK_DEADZONE = 0.1
XBOX_LINUX_AXES = (0, 1, 3, 4, 2, 5)  # device axis behind each mavgui axis 0..5
STICK_AXES = range(4)  # mavgui axes 0..3 are sticks, 4 and 5 are triggers
SWITCH_DEBOUNCE_S = 0.5  # every mavgui sees the same press: only the first one within this time switches


def apply_deadzone(value, deadzone):
    if abs(value) <= deadzone:
        return 0.0
    magnitude = (abs(value) - deadzone) / (1.0 - deadzone)
    return magnitude if value > 0 else -magnitude


def is_xbox(name):
    name = name.lower()
    return "xbox" in name or "x-box" in name


class RemappedJoystick:
    """A pygame joystick as mavgui sees it: axes in XInput order, sticks with a dead zone."""

    def __init__(self, joystick, axes=None, deadzone=STICK_DEADZONE, active=None):
        self._joystick = joystick
        self._axes = axes
        self._deadzone = deadzone
        self._active = active  # --select: false while the gamepad drives another robot

    @property
    def remapped(self):
        return self._axes is not None

    def get_axis(self, i):
        if self._active is not None and not self._active():
            return 0.0 if i in STICK_AXES else -1.0  # at rest: sticks centred, triggers released
        device_axis = self._axes[i] if self.remapped and i < len(self._axes) else i
        value = self._joystick.get_axis(device_axis)
        return apply_deadzone(value, self._deadzone) if i in STICK_AXES else value

    def __getattr__(self, name):
        return getattr(self._joystick, name)


def wrap(joystick, active=None):
    return RemappedJoystick(joystick, XBOX_LINUX_AXES if is_xbox(joystick.get_name()) else None, active=active)


class RobotSelector:
    """Which robot one gamepad drives, when every robot has its own mavgui (--select).

    All mavgui read the same gamepad, so they share a small file: "<robot> <time of the switch>".
    The robot named there gets the gamepad; the others see it at rest. Every mavgui gets the
    press of the D-pad, and each one writes "the next robot" (right) or "the previous robot"
    (left) unless the file was switched within SWITCH_DEBOUNCE_S, so one press counts once however many mavgui are running (and
    also works when the mavgui of the robot that has the gamepad is not running).
    Without a file, or with an unreadable one, robot 0 has the gamepad.
    """

    def __init__(self, path, robot, robots, clock=time.time):
        self.path = path
        self.robot = robot
        self.robots = robots
        self._clock = clock
        self._seen = None
        self._cached = None  # the selected robot as of the last refresh(); None = read the file every time

    def _read(self):
        try:
            with open(self.path) as f:
                fields = f.read().split()
            robot, switched = int(fields[0]), float(fields[1])
        except (OSError, ValueError, IndexError):
            return 0, 0.0
        return (robot, switched) if 0 <= robot < self.robots else (0, 0.0)

    def selected(self):
        return self._read()[0] if self._cached is None else self._cached

    def refresh(self):
        """Reads the file now; selected() and active() answer from this until the next refresh()
        (once per GUI frame instead of on every axis read)."""
        self._cached = self._read()[0]

    def active(self):
        return self.selected() == self.robot

    def switch(self, step=1):
        """Hands the gamepad to the next (step 1) or the previous (step -1) robot."""
        robot, switched = self._read()
        now = self._clock()
        if 0.0 <= now - switched < SWITCH_DEBOUNCE_S:
            return
        tmp = f"{self.path}.{os.getpid()}.tmp"
        with open(tmp, "w") as f:
            f.write(f"{(robot + step) % self.robots} {now!r}\n")
        os.replace(tmp, self.path)  # the other mavgui never read a half-written file
        if self._cached is not None:
            self.refresh()

    def changed(self):
        """True the first time, and whenever the selected robot is another one than at the last call."""
        selected = self.selected()
        if selected == self._seen:
            return False
        self._seen = selected
        return True


def filter_events(events, selector, button_down, hat_motion):
    """The events mavgui may see: the D-pad (a hat: left / right) switches the robot and is
    dropped, and the gamepad's buttons (soft e-stop, arm) only reach the mavgui of the robot that
    has the gamepad."""
    kept = []
    for event in events:
        if event.type == hat_motion:
            x = event.value[0]
            if x != 0:
                selector.switch(1 if x > 0 else -1)
        elif event.type != button_down or selector.active():
            kept.append(event)
    return kept


def patch_pygame(selector=None):
    import pygame

    original = pygame.joystick.Joystick

    def joystick(device_index):
        joy = wrap(original(device_index), selector.active if selector else None)
        layout = "Xbox axes remapped" if joy.remapped else "axis order unchanged (not an Xbox controller)"
        print(f"mavgui_launch: {joy.get_name()}: {layout}, stick dead zone {STICK_DEADZONE}", flush=True)
        return joy

    pygame.joystick.Joystick = joystick
    if selector:
        patch_selection(pygame, selector)
    return pygame


def patch_selection(pygame, selector):
    """mavgui's event loop also switches robots; the window title says whose mavgui this is and
    whether it has the gamepad."""
    original_get = pygame.event.get
    original_caption = pygame.display.set_caption

    def announce():
        has_it = selector.active()
        original_caption(f"mavgui robot {selector.robot}" + (" [GAMEPAD]" if has_it else ""))
        print(f"mavgui_launch: the gamepad drives robot {selector.selected()}"
              + (" (this one)" if has_it else "; D-pad left / right switches"), flush=True)

    def get(*args, **kwargs):  # mavgui calls this once per frame
        selector.refresh()
        events = filter_events(original_get(*args, **kwargs), selector, pygame.JOYBUTTONDOWN, pygame.JOYHATMOTION)
        if selector.changed():
            announce()
        return events

    def set_caption(*args, **kwargs):  # mavgui's own title is replaced by ours
        selector.changed()
        announce()

    pygame.event.get = get
    pygame.display.set_caption = set_caption


class DropDownChoice(tuple):
    """A pygame_gui drop-down selection, (text, object_id), that also equals and hashes as its text.

    mavgui indexes dicts with UIDropDownMenu.selected_option, e.g. param_types[...] for the
    Param row, which worked when pygame_gui returned the text. pygame_gui >= 0.6.10 returns
    (text, object_id) and the lookup raises KeyError: ('MB', 'MB').
    """

    def __eq__(self, other):
        return self[0] == other if isinstance(other, str) else tuple.__eq__(self, other)

    def __ne__(self, other):
        return not self == other

    def __hash__(self):
        return hash(self[0])


def patch_drop_down(menu_class):
    """Make menu_class.selected_option hand out DropDownChoice instead of a plain tuple."""

    def get(self):
        return self.__dict__["_mavgui_selected_option"]

    def set(self, value):
        if isinstance(value, tuple) and not isinstance(value, DropDownChoice):
            value = DropDownChoice(value)
        self.__dict__["_mavgui_selected_option"] = value

    menu_class.selected_option = property(get, set)


def patch_pygame_gui():
    import pygame_gui

    patch_drop_down(pygame_gui.elements.UIDropDownMenu)


ARM_FSM = ("Stowed", "Unstowing", "Stowing", "PosCtrl", "PosCtrlContact", "ReleaseToStow", "DoorForceCtrl",
           "DoorHighImpedance", "DoorHoldOpen", "BlockDoor", "RegraspToPush")
WRIST_FSM = ("PositionControl", "OpeningGripper", "ClosingGripper", "Grasped", "GraspedDoor")
GRIPPER = 3  # wrist arrays: 3 wrist joints, then the gripper
GRIPPER_FIELDS = ("wrist_position", "wrist_pos_des", "wrist_velocity", "wrist_current", "wrist_max_current")


def fsm_name(names, value):
    value = int(value)
    return names[value] if 0 <= value < len(names) else str(value)


class TelemetryRecorder:
    """Writes the gripper telemetry mavgui receives to CSV, once per robot time stamp.

    These are the values the arm's grasp-lost check compares (gripper current, position and
    position setpoint), with wall time to line up with the grcontrols_proc log.
    """

    HEADER = ("wall_time", "robot_t", "arm_fsm", "wrist_fsm", "grip_pos", "grip_pos_des", "grip_vel", "grip_cur",
              "grip_max_cur")

    def __init__(self, out, wall_clock=None):
        self._out = out
        self._writer = csv.writer(out)
        self._wall_clock = wall_clock or (lambda: datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3])
        self._last_t = None

    def record(self, rxdata):
        fsm = rxdata.get("arm_wrist_fsm", [])
        if len(fsm) < 2 or any(len(rxdata.get(k, [])) <= GRIPPER for k in GRIPPER_FIELDS):
            return
        robot_t = float(rxdata["y"][-1])
        if robot_t == self._last_t:
            return
        if self._last_t is None:
            self._writer.writerow(self.HEADER)
        self._last_t = robot_t
        self._writer.writerow([self._wall_clock(), robot_t, fsm_name(ARM_FSM, fsm[0]), fsm_name(WRIST_FSM, fsm[1])]
                              + [float(rxdata[k][GRIPPER]) for k in GRIPPER_FIELDS])
        self._out.flush()


def record_get_latest(mb_class, recorder):
    """Make mb_class.getLatest (called by mavgui once per frame) also hand its result to recorder."""
    original = mb_class.getLatest

    def getLatest(self, *args, **kwargs):
        rxdata = original(self, *args, **kwargs)
        recorder.record(rxdata)
        return rxdata

    mb_class.getLatest = getLatest


def check(seconds=15.0):
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    pygame = patch_pygame()
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("mavgui_launch: no gamepad found")
        return 1
    joy = pygame.joystick.Joystick(0)
    print(f"moving the sticks for {seconds:g} s shows what mavgui would send (-1..1); Ctrl-C stops:")
    t_end = time.time() + seconds
    try:
        while time.time() < t_end:
            pygame.event.pump()
            # + 0.0 turns -0.0 into 0.0 for display
            fwd, lateral, turn, height = (-joy.get_axis(i) + 0.0 for i in (3, 0, 2, 1))
            pressed = [b for b in range(joy.get_numbuttons()) if joy.get_button(b)]
            dpad = [joy.get_hat(h) for h in range(joy.get_numhats())]
            print(f"\rforward={fwd:+.2f} sideways={lateral:+.2f} turn={turn:+.2f} height={height:+.2f} "
                  f"buttons={pressed} dpad={dpad}      ", end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    print()
    return 0


def parse_options(argv):
    """Leading --record CSV and --select ROBOT ROBOTS FILE -> ({"record": ..., "select": ...}, the rest)."""
    opts = {}
    while argv[:1] in (["--record"], ["--select"]):
        if argv[0] == "--record":
            if len(argv) < 2:
                raise ValueError("--record needs a CSV path")
            opts["record"], argv = argv[1], argv[2:]
        else:
            if len(argv) < 4:
                raise ValueError("--select needs ROBOT ROBOTS FILE")
            robot, robots = int(argv[1]), int(argv[2])
            if not 0 <= robot < robots:
                raise ValueError(f"--select: robot {robot} is not one of {robots} robots")
            opts["select"], argv = (robot, robots, argv[3]), argv[4:]
    return opts, argv


def main(argv):
    if argv[:1] == ["--check"]:
        return check()
    try:
        opts, argv = parse_options(argv)
    except ValueError as e:
        print(f"mavgui_launch: {e}", file=sys.stderr)
        return 2
    if not argv:
        print(__doc__, file=sys.stderr)
        return 2
    mavgui = os.path.abspath(argv[0])
    selector = None
    if "select" in opts:
        robot, robots, path = opts["select"]
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        selector = RobotSelector(path, robot, robots)
    patch_pygame(selector)
    patch_pygame_gui()
    sys.argv = [mavgui] + argv[1:]
    sys.path.insert(0, os.path.dirname(mavgui))  # mavgui imports grpy and mavudptest from its folder
    record_path = opts.get("record")
    if record_path is None:
        runpy.run_path(mavgui, run_name="__main__")
        return 0
    from grpy import mb80v2

    with open(record_path, "w", newline="") as out:
        record_get_latest(mb80v2.MB80v2, TelemetryRecorder(out))
        print(f"mavgui_launch: recording gripper telemetry to {record_path}", flush=True)
        runpy.run_path(mavgui, run_name="__main__")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
