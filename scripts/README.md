# scripts

Launchers and tools. Every script sources `env.sh` itself and keeps the SDK directory untouched:
the SDK's writable files are redirected to `run/`, and its Python runs with `-B`.

## Scripts

| Script | Purpose |
| --- | --- |
| `env.sh` | Environment for the scripts and the build: `GSIM_PROJ`, `GSIM_PY`, SDK paths (`source scripts/env.sh`). |
| `run_sim.sh` | Starts `ghost_sim` with the same SDK arguments as `launch.py -s`, redirected into `run/`. Extra arguments pass through; see [`ghost_sim/README.md`](../ghost_sim/README.md). |
| `m0_official_sim.sh` | Starts Ghost's factory Bullet sim (`[arm 1]`), for comparison, without writing into the SDK. |
| `mavgui.sh` | Ghost's MBLink GUI against the local simulation (127.0.0.1). |
| `mavgui_launch.py` | Used by `mavgui.sh`: gamepad mapping, and a fix for mavgui's drop-down rows with pygame_gui >= 0.6.10. |
| `rviz.sh` | RViz2 with the SDK's perception output (`rviz/perception.rviz`); `ROS_SETUP=...` picks another ROS 2. |
| `check_sdk_untouched.sh` | Verifies that nothing under the SDK was added, removed or modified (`--init` records the baseline). |

## mavgui

```bash
scripts/mavgui.sh                        # keyboard (arrow keys, t/y)
scripts/mavgui.sh --joystick             # gamepad
scripts/mavgui.sh --joystick-check       # print what the gamepad would send
scripts/mavgui.sh --record build/grip.csv   # record gripper telemetry
```

- mavgui expects the Windows axis order and ignores the keyboard once a gamepad is connected, so
  `mavgui.sh` hides gamepads unless `--joystick` is given.
- `--joystick` remaps Xbox controllers: right stick forward / turn, left stick sideways / height,
  triggers gripper; dead zone 0.1.
- **Do not press "Start" under Log**: it writes logs into the SDK. Use `--record` instead.
