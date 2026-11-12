#!/usr/bin/env python3
"""Run Ghost's mavgui.py (read-only, straight from the SDK) with a working gamepad mapping
and drop-down menus that work with pygame_gui >= 0.6.10 (see DropDownChoice).

  mavgui_launch.py [--record CSV] MAVGUI_PY [mavgui args...]   used by `scripts/mavgui.sh`
      --record CSV   also write the gripper telemetry (current, position, setpoint, arm/wrist
                     FSM) to CSV with wall time, to line up with the grcontrols_proc log
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

    def __init__(self, joystick, axes=None, deadzone=STICK_DEADZONE):
        self._joystick = joystick
        self._axes = axes
        self._deadzone = deadzone

    @property
    def remapped(self):
        return self._axes is not None

    def get_axis(self, i):
        device_axis = self._axes[i] if self.remapped and i < len(self._axes) else i
        value = self._joystick.get_axis(device_axis)
        return apply_deadzone(value, self._deadzone) if i in STICK_AXES else value

    def __getattr__(self, name):
        return getattr(self._joystick, name)


def wrap(joystick):
    return RemappedJoystick(joystick, XBOX_LINUX_AXES if is_xbox(joystick.get_name()) else None)


def patch_pygame():
    import pygame

    original = pygame.joystick.Joystick

    def joystick(device_index):
        joy = wrap(original(device_index))
        layout = "Xbox axes remapped" if joy.remapped else "axis order unchanged (not an Xbox controller)"
        print(f"mavgui_launch: {joy.get_name()}: {layout}, stick dead zone {STICK_DEADZONE}", flush=True)
        return joy

    pygame.joystick.Joystick = joystick
    return pygame


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
            print(f"\rforward={fwd:+.2f} sideways={lateral:+.2f} turn={turn:+.2f} height={height:+.2f}  ", end="",
                  flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    print()
    return 0


def main(argv):
    if argv[:1] == ["--check"]:
        return check()
    record_path = None
    if argv[:1] == ["--record"] and len(argv) >= 2:
        record_path, argv = argv[1], argv[2:]
    if not argv:
        print(__doc__, file=sys.stderr)
        return 2
    mavgui = os.path.abspath(argv[0])
    patch_pygame()
    patch_pygame_gui()
    sys.argv = [mavgui] + argv[1:]
    sys.path.insert(0, os.path.dirname(mavgui))  # mavgui imports grpy and mavudptest from its folder
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
