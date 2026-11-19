# gr_sim

`gr_sim` runs Ghost Robotics' factory controller (GhostSDK 1.0.1, Vision 60) on swappable
simulators. The pipeline is **Ghost SDK controller → `ghost_sim` (bsp hooks + simulation loop) →
`ISimBackend` → MuJoCo / Bullet / Gazebo Harmonic**.

> **The Ghost SDK is read-only.** It is the `third_party/GhostSDK_1.0.1` submodule (a private
> repository). Never build, install or run anything inside it, and run its
> Python with `python -B`. `scripts/check_sdk_untouched.sh` checks it (`--init` records the
> baseline).

## Architecture

The repository follows one guiding principle:

- **`gsim_core` is the shared root** — the `ISimBackend` interface, the simulation loop, the
  `gsim_*` parameters and the CSV log. It knows neither a physics engine nor the Ghost SDK.
- **Each simulator gets its own self-contained package** named `simulators/gsim_<name>`,
  depending only on `gsim_core` and its own engine.
- **`ghost_sim` is the executable** — it links the SDK and the simulators that were built in.
- **`gsim_server` is the physics on its own** — a MuJoCo or Gazebo world without the SDK, which
  `ghost_sim` reaches over a unix socket (`gsim_backend remote`). One world can hold several
  robots, each driven by its own `ghost_sim`.

Dependencies only point downwards (`gsim_core` ← `simulators/*` ← `ghost_sim`, `gsim_server`).

| `gsim_backend` | Package | Engine | Runs on | Role |
| --- | --- | --- | --- | --- |
| `mujoco` (default) | [`gsim_mujoco`](simulators/gsim_mujoco/README.md) | MuJoCo 4.0.0 | container or host | main simulator |
| `bullet` | [`gsim_bullet`](simulators/gsim_bullet/README.md) | Bullet, through the SDK's SimWrapper | container or host | reference (Ghost's own physics and world) |
| `gazebo` | [`gsim_gazebo`](simulators/gsim_gazebo/README.md) | Gazebo Harmonic (DART) | container | custom SDF worlds |
| `remote` | [`gsim_remote`](simulators/gsim_remote/README.md) | MuJoCo or Gazebo, in `gsim_server` | container or host | physics in its own process; several robots in one world |

Every backend drives the legs, the arm (`arm 1`) and Ghost's synthetic depth camera
(terrain perception). Physics always steps at 0.5 ms (2000 Hz); the SDK's control tick runs every
`2000 / gsim_control_rate` steps (500 Hz by default).

## Repository Layout

```text
gr_sim/
├── gsim_core/      # shared root: ISimBackend, simulation loop, gsim_* parameters, CSV log
├── simulators/     # one package per simulator: gsim_mujoco, gsim_bullet, gsim_gazebo, gsim_remote
├── ghost_sim/      # the executable: the SDK's bsp hooks + the simulator factory
├── gsim_server/    # the physics server for gsim_backend remote
├── scripts/        # run_sim.sh, run_server.sh, mavgui.sh, rviz.sh, m0_official_sim.sh, check_sdk_untouched.sh
├── examples/       # scripts that command the robot over MBLink
├── ros2/           # ghost_interfaces: the controller's ROS 2 interface packages
├── third_party/    # GhostSDK_1.0.1: the Ghost SDK (submodule, read-only)
├── cmake/          # find modules: Ghost SDK, MuJoCo, simulate, Gazebo
├── docker/         # Dockerfile + compose.yaml (container gr-sim: the default install)
└── install.md      # installation from scratch
```

## Getting Started

1. **Install** — set up the Docker workspace, clone this repository with the Ghost SDK submodule,
   build the `gr-sim` image and build `gr_sim` inside it: see [`install.md`](install.md). The same
   guide covers the optional ROS 2 interface packages and building without Docker.

## Quick Start

From the project root (inside `gr-sim`), after building:

```bash
# 1. Start the simulator (one of)
scripts/run_sim.sh                                   # MuJoCo, flat ground
scripts/run_sim.sh gsim_scene simulators/gsim_mujoco/models/scene_stairs.xml   # MuJoCo, Ghost's stairs
scripts/run_sim.sh gsim_backend bullet               # Bullet, Ghost's world
scripts/run_sim.sh gsim_backend gazebo               # Gazebo, flat ground
scripts/run_sim.sh gsim_backend gazebo gsim_world simulators/gsim_gazebo/models/world_stairs.sdf
# ... or with the physics in its own process (start the server first, in its own terminal)
scripts/run_server.sh gsim_server /tmp/gsim.sock
scripts/run_sim.sh gsim_backend remote gsim_server /tmp/gsim.sock
# ... or two robots in one world, each controller in its own container (on the host; see
# simulators/gsim_remote/README.md)
scripts/multi.sh up && scripts/multi.sh server       # then: scripts/multi.sh robot 0, robot 1, mavgui 0, mavgui 1

# 2. Options, appended to any line above
#    arm 1              with the arm
#    gsim_headless 1    no window; status line every 5 s

# 3. Command the robot (second terminal)
scripts/mavgui.sh                                    # keyboard (arrow keys, t/y)
scripts/mavgui.sh --joystick                         # gamepad (works while another window has the focus)

# Ghost's factory Bullet sim, for comparison
scripts/m0_official_sim.sh [arm 1]
```

Ctrl-C stops `ghost_sim`; close the window to stop the factory sim. All parameters are listed in
[`ghost_sim/README.md`](ghost_sim/README.md).

## Safety Notes

- **Never use the SDK's `launch.py`**: it points the controller's config at the SDK, which then
  rewrites `share/gr/examples/user_params_config.yaml`. The scripts here redirect those files to
  `run/`.
- **Do not press "Start" under Log in mavgui**: it writes logs into the SDK. To record gripper
  telemetry, use `scripts/mavgui.sh --record build/grip.csv`.
- Run one simulation at a time, host or container: they share the host network, and two
  controllers fight over the SDK's UDP port.

## Documentation Map

- [`install.md`](install.md) — installation from scratch, ROS 2 interfaces, troubleshooting.
- [`docker/README.md`](docker/README.md) — the `gr-sim` container: what is in it.
- [`ghost_sim/README.md`](ghost_sim/README.md) — parameters, arm, terrain perception, CSV log.
- [`gsim_core/README.md`](gsim_core/README.md) — the shared interface layer.
- [`simulators/README.md`](simulators/README.md) — the simulator packages.
- [`simulators/gsim_remote/README.md`](simulators/gsim_remote/README.md) — the physics server, the remote backend and several robots in one world.
- [`scripts/README.md`](scripts/README.md) — the scripts, mavgui / gamepad notes.
- [`examples/README.md`](examples/README.md) — MBLink example scripts and grasping in simulation.

## Environment

Ubuntu 24.04 · GhostSDK 1.0.1 · MuJoCo 4.0.0 · Bullet (SDK) · Gazebo Harmonic (gz-sim 8, DART) ·
ROS 2 Jazzy · C++17 · CMake ≥ 3.16 · Python 3.10 · Docker (NVIDIA GPU). Commands run inside the
`gr-sim` container by default; see [`install.md`](install.md) for details.
