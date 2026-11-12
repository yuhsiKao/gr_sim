# examples

Scripts that command the robot over MBLink, like mavgui does. They talk to whichever controller
is running on 127.0.0.1 (Ghost's factory sim or `ghost_sim`) and belong to no simulator.

## Run

From the project root, with a simulator already running:

```bash
source scripts/env.sh
$GSIM_PY -B examples/<script>.py
```

`$GSIM_PY` is the Python 3.10 env from `scripts/env.sh` (the SDK's `grmblinkpy` is built for 3.10);
`-B` keeps `__pycache__` out of the SDK.

## Scripts

| Script | Purpose |
| --- | --- |
| `sim_grasp_threshold.py` | Lowers `WRIST_GRASPTAU` to 10 (and clears `WG`) so the gripper's grasp detection works; `--tau` sets another value. |

It works with any simulator that has the arm (`arm 1`): run it once after every simulator start,
before grasping with mavgui.

## Grasping in the Simulators

The SDK decides that the gripper holds something when the measured gripper current exceeds
`WRIST_CLOSEPCT * WRIST_GRASPTAU` (0.7 × 14 = 9.8 by default). Ghost's SimWrapper reports the
commanded closing current (8) instead of a measured one (`SimWrapper.cpp:474`), so a grasp is never
detected, however well the gripper is placed. `ghost_sim` reports the current the same way, so the
same applies there.

Lowering `WRIST_GRASPTAU` to 10 brings the threshold to 7:

```bash
$GSIM_PY -B examples/sim_grasp_threshold.py
```

- Side effect: `WRIST_GRASPTAU` is also the torque the gripper applies once a grasp is detected,
  so the grip is weaker (10 instead of 14 N m).
- The value is not saved: set it after every simulator start, with the script or by writing
  `WRIST_GRASPTAU` = 10 in mavgui's parameter field.
