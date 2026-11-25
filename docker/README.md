# Docker Workspace

This folder is the default install of `gr_sim`: the `gr-sim` container builds and runs all
three backends, and is the only place the Gazebo backend builds. The install steps are in
[`install.md`](../install.md); this page explains what the container is made of.

The Dockerfile builds a reusable development image. Source code is not copied into the image: the
folder that holds this repository is mounted as the container's home, so the project, the ROS 2
interface workspace, the shell history and the logs stay on the host and survive the container.
The container runs as root, so files it writes there are owned by root on the host.

```text
~/docker_ws/gr_sim/        # host; /root in the container
├── gr_sim/                # this repository
├── gr_ros2_ws/            # colcon workspace: the build of ros2/ghost_interfaces (install.md, step 7)
└── .bashrc, logs/, ...    # the container's home files
```

## Image

| Part | Contents |
| --- | --- |
| Base | `osrf/ros:jazzy-desktop` — ROS 2 Jazzy with RViz2. |
| Simulator | Gazebo Harmonic (`gz-harmonic`); MuJoCo is downloaded by the build, Bullet comes with the SDK. |
| Build | The packages `ghost_sim` builds with. |
| Python 3.10 | `/opt/gsim/venv` (managed with uv, `docker/requirements-mavgui.txt`), used through `$GSIM_PY` for mavgui and `examples/`. Not on `PATH`: `python3` stays the system 3.12 that ROS 2 needs. |

## Mounts

| In the container | From the host |
| --- | --- |
| `/root` | The folder that holds this repository (`../..` from `docker/`). |
| `~/gr_sim/build` | `build-docker/` (kept apart from a host `build/`, which links against the host's libraries). |
| `/ws/GhostSDK_bullet_linux` | The SDK (`third_party/GhostSDK_1.0.1/GhostSDK_bullet_linux`), read-only. |

The compose setup also gives the container host networking (MBLink and DDS reach tools on the
host), the NVIDIA GPU (windows and the offscreen depth camera), X11 and `/dev/input` (gamepads).

## Configure

`docker/.env` (optional; copied from `docker/.env.example`, not committed) holds the per-machine
settings:

| Variable | Meaning |
| --- | --- |
| `CONTROLS_INSTALL_DIR` | Another `GhostSDK_bullet_linux` directory on the host, instead of the submodule. |

## Build and Run

From the repository root:

```bash
docker compose -f docker/compose.yaml build       # once; the image, gr-sim:dev, is about 7 GB
xhost +local:docker                               # once per login, for the windows
docker compose -f docker/compose.yaml up -d
docker exec -it gr-sim bash
docker compose -f docker/compose.yaml down        # stop
```

Instead of building, the published image can be pulled and given the name the compose files use:

```bash
docker pull ghcr.io/yuhsikao/gr-sim:latest
docker tag ghcr.io/yuhsikao/gr-sim:latest gr-sim:dev
```

Rebuild the image only after changing `docker/Dockerfile` or `docker/requirements-mavgui.txt`, then
`up -d` again to replace the container. Nothing is lost: everything you work on is on the host.

## Several Robots

`docker/compose.multi.yaml` is a second, separate setup for several robots in one world: the same
image and mounts, but one container per program instead of host networking.

| Container | Runs | Network |
| --- | --- | --- |
| `gr-physics` | `gsim_server`: the world and its window | bridge |
| `gr-robot-<i>`, one per robot | one `ghost_sim` each, and that robot's mavgui | bridge, each its own namespace; DDS domain 124 + index |

Every controller binds the SDK's fixed MBLink port, so each needs a network namespace of its own;
the socket to the physics is a file in the mounted repository folder (`gr_sim/run/gsim.sock`).
`scripts/multi.sh` starts the containers and runs the programs in them; see
[`simulators/gsim_remote/README.md`](../simulators/gsim_remote/README.md).

The compose file is not started as it is. It describes the physics container and one robot
container, and `scripts/multi.sh` writes `run/compose.multi.yaml` from it: one `gr-robot-<i>` per
robot of [`config/multi_robots.yaml`](../config/multi_robots.yaml), each with its name, its DDS
domain and its folder `run/robot<i>` for the files the controller writes. One more robot is one
more entry in that file; what all containers share (image, mounts, GPU) is changed here.

`gr-physics` is also given the host's `/dev/cpu_dma_latency`: `gsim_server` holds it open to keep
a Gazebo world with several robots at real time (the same README, Real Time).

## Notes

- The container shares the host network: run one simulation at a time, in the container or on
  the host.
- ROS 2: the image has ROS 2 Jazzy; the controller's interface packages and the shell setup are
  step 7 of [`install.md`](../install.md).
