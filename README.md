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
├── scripts/        # run_sim.sh, run_server.sh, multi.sh, mavgui.sh, rviz.sh, m0_official_sim.sh, check_sdk_untouched.sh
├── examples/       # scripts that command the robot over MBLink
├── ros2/           # ghost_interfaces: the controller's ROS 2 interface packages
├── third_party/    # GhostSDK_1.0.1: the Ghost SDK (submodule, read-only)
├── cmake/          # find modules: Ghost SDK, MuJoCo, simulate, Gazebo
├── docker/         # Dockerfile, compose.yaml (container gr-sim), compose.multi.yaml (several robots)
└── install.md      # installation from scratch
```

## Getting Started

1. **Install** — set up the Docker workspace, clone this repository with the Ghost SDK submodule,
   build the `gr-sim` image and build `gr_sim` inside it: see [`install.md`](install.md). The same
   guide covers the optional ROS 2 interface packages and building without Docker.

## Quick Start

There are two setups. They use the same image and the same build, but they are started in
different places:

- **One robot** — one `ghost_sim` holds the controller and the physics. The commands run inside
  the `gr-sim` container.
- **Several robots in one world** — one `gsim_server` holds the world and every robot has its own
  `ghost_sim`, each program in its own container. The commands run on the host, through
  `scripts/multi.sh`.

### One Robot

From the project root, inside `gr-sim`, after building:

```bash
# 1. Start the simulator (one of)
scripts/run_sim.sh                                   # MuJoCo, flat ground
scripts/run_sim.sh gsim_scene simulators/gsim_mujoco/models/scene_stairs.xml   # MuJoCo, Ghost's stairs
scripts/run_sim.sh gsim_backend bullet               # Bullet, Ghost's world
scripts/run_sim.sh gsim_backend gazebo               # Gazebo, flat ground
scripts/run_sim.sh gsim_backend gazebo gsim_world simulators/gsim_gazebo/models/world_stairs.sdf

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

One robot can also run with the physics in its own process. This is the split the several-robots
setup uses, without its containers (start the server first, in its own terminal):

```bash
scripts/run_server.sh gsim_server /tmp/gsim.sock     # MuJoCo; add gsim_backend gazebo for Gazebo
scripts/run_sim.sh gsim_backend remote gsim_server /tmp/gsim.sock
```

### Several Robots

From the project root, on the host, with the build made in `gr-sim`:

```bash
scripts/multi.sh up            # once: the containers gr-physics, gr-robot-0, gr-robot-1
scripts/multi.sh server        # terminal 1: the world and its window (MuJoCo), waits for 2 robots
scripts/multi.sh robot 0       # terminal 2
scripts/multi.sh robot 1       # terminal 3: the simulation starts when every robot has joined
scripts/multi.sh mavgui 0      # terminal 4: commands robot 0
scripts/multi.sh mavgui 1      # terminal 5: commands robot 1
scripts/multi.sh down          # when done

# Options
#    scripts/multi.sh server gsim_backend gazebo     Gazebo instead of MuJoCo
#    scripts/multi.sh server gsim_headless 1         no window
#    scripts/multi.sh robot 0 arm 1                  robot 0 with the arm
#    GSIM_ROBOTS=1 scripts/multi.sh server           one robot (more than two: one more
#                                                    gr-robot-<i> block in docker/compose.multi.yaml)
```

Ctrl-C in the server's terminal stops every robot. The full guide is
[`simulators/gsim_remote/README.md`](simulators/gsim_remote/README.md).

### What Differs

| | One robot | Several robots |
| --- | --- | --- |
| Where the commands run | inside `gr-sim` | on the host; `scripts/multi.sh` runs each program in its container |
| Containers | `gr-sim` (`docker/compose.yaml`), on the host's network | `gr-physics` and one `gr-robot-<i>` per robot (`docker/compose.multi.yaml`), each with its own network |
| Processes | one `ghost_sim`: the controller and the physics | one `gsim_server` (the world) and one `ghost_sim` per robot (its controller) |
| Simulators | MuJoCo, Bullet, Gazebo | MuJoCo, Gazebo |
| Parameters | all to `scripts/run_sim.sh` | the world's to `multi.sh server`, a robot's to `multi.sh robot <i>`; one given to the wrong side is an error |
| Window, real-time factor, pause | `ghost_sim`'s | the server's, for all robots at once |
| mavgui | `scripts/mavgui.sh` | `scripts/multi.sh mavgui <i>`, one per robot |
| ROS 2 topics | DDS domain `ROS_DOMAIN_ID`, 124 if unset | robot `i` is DDS domain 124 + `i`; the topic names are the same for every robot |
| Stopping | Ctrl-C, or close the window | Ctrl-C in the server's terminal stops every robot; a robot stopped on its own stays in the world without torque |

With several robots, a ROS 2 tool picks its robot by the domain, for example
`ROS_DOMAIN_ID=125 ros2 topic list` for robot 1. It sees that robot from any of these containers,
from `gr-sim` and from the host. Give the domain on the command itself: a `ROS_DOMAIN_ID` the
shell exports (in `.bashrc`, say) takes the place of the container's.

## Safety Notes

- **Never use the SDK's `launch.py`**: it points the controller's config at the SDK, which then
  rewrites `share/gr/examples/user_params_config.yaml`. The scripts here redirect those files to
  `run/`.
- **Do not press "Start" under Log in mavgui**: it writes logs into the SDK. To record gripper
  telemetry, use `scripts/mavgui.sh --record build/grip.csv`.
- Run one setup at a time. Single-robot simulations, host or `gr-sim`, share the host network, and
  two controllers fight over the SDK's UDP port. The several-robots setup gives every controller
  its own container, but its robot 0 is in the DDS domain a single robot uses by default (124), so
  the two would share their topics.

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
