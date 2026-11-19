# gsim_gazebo

Gazebo Harmonic backend (`gsim_backend gazebo`). A `gz::sim::Server` runs inside `ghost_sim`
(DART physics, 0.5 ms steps) and is stepped by the same loop as the other backends; the window
(`gz sim -g`) is a separate process. Built only where gz-sim 8 is installed, i.e. in the `gr-sim`
container ([`docker/README.md`](../../docker/README.md)).

`GazeboWorld` is the same world with several robots, one model each; `gsim_server` runs it with
`gsim_backend gazebo` ([`gsim_remote`](../gsim_remote/README.md)). The robots collide with each
other, and one robot's state does not depend on the others while they do not touch.

## Run

```bash
scripts/run_sim.sh gsim_backend gazebo                       # flat ground, with the window
scripts/run_sim.sh gsim_backend gazebo gsim_headless 1       # no window
scripts/run_sim.sh gsim_backend gazebo gsim_world simulators/gsim_gazebo/models/world_stairs.sdf
```

`gsim_world` takes any SDF world. It must use `max_step_size 0.0005` and
`real_time_update_rate 0` (pacing belongs to `ghost_sim`). Relative `<uri>`s resolve against the
world file's folder.

## Worlds

| File | Contents |
| --- | --- |
| `models/world_flat.sdf` | Flat ground (default). |
| `models/world_stairs.sdf` | Ghost's staircase, the same boxes as MuJoCo's `scene_stairs.xml` (first edge at x = 2.851 m). |

## Window

Without `gsim_headless 1`, `ghost_sim` starts `gz sim -g`, which attaches to the server
(`xhost +local:docker` first).

- Pause / play work.
- Closing the window does **not** stop the simulation: Ctrl-C does, and `gz sim -g` can be
  started again.
- **Do not press reset**: the world would be reset behind the SDK's back, so `ghost_sim` logs an
  error and stops instead.

## Behaviour

- **Robot**: Ghost's URDF is converted to SDF in memory (DART physics, ODE collisions).
- **Toes**: DART's contacts have no torsional friction, so `ghost_sim` adds it: each toe gets a
  torque about its contact normal that holds it up to `torsional` × normal force
  (`gsim_toe_friction`, 0.04 by default), like the spinning friction and friction anchor of
  Ghost's Bullet toes. Without it the body yaws over planted feet once the arm is unstowed.
- **Arm**: lock and wrist servo as DART servo joints, driven like Ghost's Bullet motor (same gains
  and force limits); the SDK's torques go straight to the joints once unstowed. The gripper jaws
  collide with the part's `.stl` (the URDF's `.obj` has no normals, which DART cannot load). A jaw
  pressed into its own limit reports about 0 current, as the factory sim does.
- **Depth camera**: Ghost's downward camera as a Gazebo `depth_camera` sensor, rendered offscreen
  on the GPU (EGL), headless or with the window, and only while the SDK has its cameras on.
  Without an EGL device it is turned off with a warning and the rest runs. libEGL prints two
  harmless `failed to create dri2 screen` warnings at start-up. The camera sits inside the body,
  so for the cameras the body is a slab (the part of its collision box above the camera): a robot
  never sees its own, and sees the other robots'.

## Against MuJoCo and the Factory Sim

| Quantity | Gazebo | MuJoCo | Factory sim |
| --- | --- | --- | --- |
| Standing height | 0.536 m | 0.529 m | — |
| Walking, half speed command | 0.80 m/s | 0.79 m/s | — |
| Turning, half rate command | 41.1 °/s | 41.2 °/s | 40.6 °/s (`gsim_backend bullet`) |
| Wrist step time constant | 0.254 s | 0.254 s | 0.2475 s |
| Wrist holding current | -0.597 N m | -0.597 N m | -0.592 N m |

On `world_stairs.sdf`, the SDK's heightmap matches the MuJoCo and factory ones
(rises 0.206 / 0.199 m, run 0.29 m). `rtf` stays 1.00 headless, with the arm and with the cameras
on (about 1.7 ms per frame; RTX 5070 host).

## Layout

```text
gsim_gazebo/
├── include/gsim_gazebo/   # GazeboBackend (ISimBackend), gazebo_model (URDF -> SDF)
├── src/
│   ├── GazeboBackend.cpp  # server, stepping, window, pause/reset, depth frames
│   ├── gazebo_system.cpp  # gz::sim::System: torques, arm/wrist drives, state readout
│   ├── gazebo_model.cpp   # URDF -> SDF model, depth camera sensor, world assembly, self-check
│   └── egl_probe.cpp      # offscreen rendering check
└── models/
    └── *.sdf              # worlds (flat / stairs)
```
