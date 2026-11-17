# ghost_sim

`ghost_sim` is the executable: the Ghost SDK's controller plus the simulators that were built in.
It implements the SDK's bsp hooks, runs the simulation loop, and hands the chosen backend's state,
wrist telemetry and depth point cloud to the SDK.

## Layout

| File | Purpose |
| --- | --- |
| `src/main.cpp` | Ctrl-C handling, `CONTROLS_INSTALL_DIR` default, `gsim_control_rate`, and the guard that refuses to start where the SDK would write into itself or the working directory. |
| `src/sim_bsp.cpp` | The SDK's link-time hooks `gr::Robot::bspInit` / `bspBegin`: parameters, `SimLLC`, `SimLoop`, the window thread, the depth camera's point cloud to the SDK. |
| `src/backend_factory.cpp` | The only place that lists the simulators (`gsim_backend <name>`). |

## Run

```bash
scripts/run_sim.sh [parameter value ...]
```

`run_sim.sh` passes the SDK arguments that keep the SDK's files out of the SDK directory.
Exit codes: 0 normal, 1 selftest failed, 2 configuration or start-up error.

## Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `gsim_backend` | `mujoco` | `mujoco`, `bullet`, `gazebo` or `remote`. |
| `gsim_headless` | `0` (`1` when `test > 0`) | `1` = no window. |
| `gsim_rtf` | `1.0` | Real-time factor; `0` = as fast as possible. |
| `gsim_control_rate` | `500` | The SDK's control rate in Hz (`CONTROL_RATE`), to match the robot's onboard computer. Must divide 2000 (e.g. `500`, `1000`). |
| `gsim_scene` | `simulators/gsim_mujoco/models/scene_flat.xml` | MuJoCo world. |
| `gsim_world` | `simulators/gsim_gazebo/models/world_flat.sdf` | Gazebo world. |
| `gsim_urdf_dir` | `$CONTROLS_INSTALL_DIR/share/gr/urdf` | URDF folder. |
| `gsim_log` | (off) | CSV log path. |
| `gsim_selftest` | `0` | `1` = build the model, check it, exit (0 = pass). |
| `gsim_toe_friction` | `"3.0 0.04 0"` | Toe friction (slide, torsional, rolling). Gazebo uses the first two. |
| `gsim_toe_solref` | `"0.02 1"` | MuJoCo toe contact solref. |
| `gsim_server` | (none) | With `gsim_backend remote`: the socket path of the physics server. |
| `gsim_robot` | `0` | With `gsim_backend remote`: which robot of the server's world this controller drives. |
| `gsim_spawn` | `"0 0 0"` | Where the robot starts: `"x y yaw_deg"` on the ground. Bullet uses the yaw only. |
| `arm` | (off) | The SDK's own argument (as `launch.py -a`): `1` loads `vision60_arm.urdf`. |

- **Physics always steps at 0.5 ms** (2000 Hz, as Ghost's Bullet sim). A scene or world with
  another step size is refused at start-up: with a 2 ms step the gait frequency more than doubles.
- With `gsim_backend remote` the physics runs in `gsim_server`
  ([`simulators/gsim_remote`](../simulators/gsim_remote/README.md)). The scene, the toe contact,
  `gsim_rtf` and the window are then parameters of `gsim_server`; giving them to `ghost_sim` is an
  error. The server's world can hold several robots, one `ghost_sim` each.
- `gsim_control_rate` other than 500 Hz is logged as a warning: the SDK's default and Ghost's sims
  run 500 Hz. Measured on MuJoCo: 1000 Hz walks like 500 Hz; 250 Hz walks slower and rolls more.

## Arm

With `arm 1` the simulator acts like Ghost's Bullet sim: the stowed arm is held at 0 rad, the
unstowed arm gets the SDK's torques, the wrist follows the SDK's position / current / off modes,
and the wrist current is reported the factory way.

Against the factory sim, the same arm sequence (stand, unstow, GraspHighFront, stow) passes through
the same arm / wrist states, with angles within 0.012 rad (MuJoCo), 0.001 rad (Bullet) and 0.012 rad
(Gazebo).

- Grasp detection does not trigger (as in Ghost's sim): run `examples/sim_grasp_threshold.py`
  after start-up; see [`examples/README.md`](../examples/README.md).
- Ghost's door exists in the Bullet world only.

## Terrain Perception

Every backend renders Ghost's synthetic downward depth camera (as Ghost's Bullet sim does, 30 Hz)
and hands its point cloud to the SDK, which builds the terrain heightmap from it.

Pick the mode with mavgui's Vision Mode buttons:

| `vision_mode` | Behaviour |
| --- | --- |
| `2` | Switches to stair mode by itself in front of stairs. |
| `3` | Forces stair mode. |
| `15` | Perception off. |

Live view (second terminal; needs ROS 2, by default `/opt/ros/jazzy`):

```bash
scripts/rviz.sh
```

RViz2 shows, in the body frame, what the controller publishes over DDS (domain 124) while the
cameras are on:

```text
/mcu/state/heightmap     # the SDK's heightmap, coloured by height
/mcu/state/pointcloud    # its 2D heightmap (vision_mode 2, irregular rate)
/mcu/state/obstmap       # obstacle map, white balls
```

In front of the stairs, the SDK's heightmap shows the same step heights and depths on all three
backends as on the factory sim.

- The heightmap reaches about 2 m ahead, so the stairs only show up when the robot is close.
- In front of the stairs, `vision_mode 2` keeps switching between 2 and 3 (the factory sim does too).

## CSV Log

`gsim_log <path>` writes the state at 100 Hz (world frame; arm slots are NaN without the arm):

```text
t,px,py,pz,roll,pitch,yaw,vx,vy,vz,q0..q14,dq0..dq14,tau0..tau14
arm_locked,wmode,wq0..wq3,wdq0..wdq3,wcur0..wcur3        # with arm 1 only
```

`wmode`: 0 off, 2 current, 4 position.

## Notes

- Switch into CRAWL (gait 3) only while standing still: switched while walking, the controller
  keeps crawling and ignores later mode changes until `ghost_sim` is restarted.
- Falls save a rolling log (about 6 MB) to `~/logs/rl_*.pbl`.
- Only one simulator runs at a time: the SDK's MBLink listens on UDP 14999, so a second `ghost_sim`
  exits with code 2 and `UDP port 14999 ... is in use` instead of starting.
