# Installation

This guide sets up `gr_sim` from scratch with Docker. One image carries everything — the build
packages, the Python 3.10 env, Gazebo Harmonic and ROS 2 Jazzy — and one build produces all three
simulator backends (MuJoCo, Bullet, Gazebo). Nothing but Docker is installed on the host; the
setup with several robots also uses the host's `python3`.

## Requirements

- Ubuntu host with an X11 session (developed on Ubuntu 24.04), for the simulator windows.
- NVIDIA GPU with its driver, for the windows and the offscreen depth camera.
- Docker Engine with the Docker Compose plugin (Compose ≥ 2.30).
- For several robots in one world (`scripts/multi.sh`): PyYAML for the host's system `python3`
  (`sudo apt install python3-yaml`). `multi.sh` runs `/usr/bin/python3`, so a conda or virtual
  environment that is active in the terminal does not matter.
- NVIDIA Container Toolkit.
- Access to the private `GhostSDK_1.0.1` repository on GitHub: GhostSDK 1.0.1 is a submodule of
  this repository.

## 1. Prepare the Host

Install [Docker Engine](https://docs.docker.com/engine/install/ubuntu/) (it includes the Compose
plugin) and the
[NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html),
then let your user run Docker without `sudo`:

```bash
sudo usermod -aG docker $USER                     # log out and in again afterwards
```

Check the host:

```bash
nvidia-smi                                        # lists the GPU
docker compose version                            # v2.30 or newer
docker run --rm --gpus all ubuntu:24.04 nvidia-smi   # the GPU is visible inside a container
echo $XDG_SESSION_TYPE                            # x11
```

> The GPU is used for rendering only: the simulator windows, RViz2 and Gazebo's depth camera.
> Physics and the controller run on the CPU.

## 2. Create the Workspace

Create a folder of its own for `gr_sim` on the host:

```bash
mkdir -p ~/docker_ws/gr_sim
cd ~/docker_ws/gr_sim
```

The Docker Compose file mounts this folder as the container's home:

```text
~/docker_ws/gr_sim -> /root
```

so it must not be your home folder or a folder shared with other projects. The container runs as
root: files it writes there are owned by root on the host.

## 3. Download `gr_sim`

Clone this repository into the workspace, with its submodule (the Ghost SDK):

```bash
git clone --recurse-submodules https://github.com/yuhsiKao/gr_sim.git
cd gr_sim                                         # every host command below starts from here
```

Check that the SDK is there (after a clone without `--recurse-submodules`, run
`git submodule update --init` first):

```bash
ls third_party/GhostSDK_1.0.1/GhostSDK_bullet_linux/lib/libgrcontrols.a
```

> The SDK is treated as read-only: the container mounts it read-only and nothing in `gr_sim`
> writes into it. Never build, install or run anything inside it.

Your workspace should now look like:

```text
~/docker_ws/gr_sim/                    # /root in the container
└── gr_sim/                            # this repository; ~/gr_sim in the container
    ├── third_party/GhostSDK_1.0.1/    # the Ghost SDK (submodule)
    └── ros2/ghost_interfaces/         # the controller's ROS 2 interface packages
```

To use another copy of the SDK, copy `docker/.env.example` to `docker/.env` and set
`CONTROLS_INSTALL_DIR` to its `GhostSDK_bullet_linux` directory.

## 4. Build and Start the Docker Environment

Build the development image (once; it downloads several GB):

```bash
docker compose -f docker/compose.yaml build
```

Allow Docker containers to use the host X11 display (once per login):

```bash
xhost +local:docker
```

Start the container and enter it:

```bash
docker compose -f docker/compose.yaml up -d
docker exec -it gr-sim bash
```

## 5. Build `gr_sim`

Inside the container, from `~/gr_sim`:

```bash
source scripts/env.sh
cmake -S . -B build
cmake --build build -j
```

> The first configure downloads the MuJoCo 4.0.0 release, so it needs network access inside the container.

After changing C++ code, only `cmake --build build -j` is needed.

## 6. First-Run Checks

Inside the container, from `~/gr_sim`:

```bash
scripts/check_sdk_untouched.sh --init             # records the SDK's state
scripts/run_sim.sh gsim_selftest 1                                       # expect "gsim selftest: PASS"
scripts/run_sim.sh gsim_selftest 1 gsim_backend bullet gsim_headless 1   # expect "gsim selftest: PASS"
scripts/run_sim.sh gsim_selftest 1 gsim_backend gazebo gsim_headless 1   # expect "gsim selftest: PASS"
```

Then start a simulator with its window, and command the robot from a second terminal
(`docker exec -it gr-sim bash`):

```bash
scripts/run_sim.sh                                # terminal 1: MuJoCo window with the robot; Ctrl-C stops it
scripts/mavgui.sh                                 # terminal 2: keyboard control (arrow keys, t/y)
```

Later, `scripts/check_sdk_untouched.sh` prints `SDK untouched` as long as nothing has written into
the SDK.

The install is complete: continue with the [Quick Start](README.md#quick-start). The two steps
below are optional.

## 7. ROS 2 Interfaces (Optional)

Needed only to use the controller's ROS 2 topics. The simulator, mavgui and the MBLink examples
work without it.

> **The simulator has topics only, no ROS 2 services.** `/ensure_mode`, `/set_param` and the other
> services come from the `ghost_manager` node on the robot, so `ros2 service call` waits forever
> here. Set modes over topics instead: see
> [Setting Modes](#setting-modes-in-the-simulator) below.

The SDK's controller talks DDS. Its API uses three interface packages that are installed on the
robot's onboard computer (`~/current_ros2/share`); their definitions are in `ros2/ghost_interfaces`:

| Package | Depends on |
| --- | --- |
| `ghost_manager_interfaces` | `std_msgs` |
| `robot_control_msgs` | `builtin_interfaces` `std_msgs` `geometry_msgs` `sensor_msgs` `action_msgs` |
| `draper_msgs` | `builtin_interfaces` `std_msgs` `geometry_msgs` `sensor_msgs` |

Build the three packages inside the container, in a workspace next to the repository (expect
`Summary: 3 packages finished`):

```bash
mkdir -p /root/gr_ros2_ws && cd /root/gr_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --base-paths /root/gr_sim/ros2
```

Add the ROS 2 environment setup to `/root/.bashrc` (once: the home folder stays on the host):

```bash
echo "###### ROS2 ######" >> /root/.bashrc && \
echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
echo "###### Ghost interfaces ######" >> /root/.bashrc && \
echo "source /root/gr_ros2_ws/install/setup.bash" >> /root/.bashrc && \
source /root/.bashrc
```

Check, with a simulator running in another terminal:

```bash
export ROS_DOMAIN_ID=124                          # the simulator's default DDS domain
ros2 interface list | grep -c -E "ghost_manager_interfaces|robot_control_msgs|draper_msgs"   # expect 25
ros2 topic echo /state/heartbeat
cd ~/gr_sim && scripts/rviz.sh                    # the SDK's perception output in RViz2
```

### Setting Modes in the Simulator

Without the robot's services, there are two ways to set modes:

| Way | Sets | How |
| --- | --- | --- |
| `/command/setControlMode` | the control mode | `std_msgs/msg/UInt32`, the mode (e.g. `140`) |
| `gr_ros2_link.py` | every mode | The SDK's helper: sets the mode and confirms it from `/state/heartbeat`. |

The control mode topic:

```bash
ros2 topic pub --once /command/setControlMode std_msgs/msg/UInt32 "{data: 140}"
```

The helper (it sets control mode 140 first):

```bash
G=/ws/GhostSDK_bullet_linux/share/gr/examples/gr_ros2_link.py
python3 -B $G -m action 2                         # stand up and walk
python3 -B $G -m action 0                         # sit
python3 -B $G -b                                  # stand, walk blind for a few seconds, sit
python3 -B $G --help                              # the other modes and tests
```

Then command the robot over the `/mcu/command/*` topics (for the details of each topic, see
"List of Subscribed Topics" on the "ROS2 Interface" page of Ghost's documentation), for example:

```bash
ros2 topic pub -r 20 /mcu/command/manual_twist geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
```

## Without Docker

MuJoCo and Bullet also build directly on an Ubuntu 24.04 host (no Gazebo, and ROS 2 only if it is
installed separately). Do steps 2-3, then instead of step 4:

```bash
sudo apt install build-essential cmake git pkg-config libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev libegl-dev libgtest-dev

# Python 3.10 env for mavgui and examples/ (the SDK's grmblinkpy is built for 3.10)
conda create -y -n ghost_mavgui python=3.10
conda activate ghost_mavgui
pip install -r docker/requirements-mavgui.txt
```

then run steps 5-6 on the host. `scripts/env.sh` uses the SDK in `third_party/` unless
`docker/.env` or an exported `CONTROLS_INSTALL_DIR` names another one. The scripts find the Python env through `$GSIM_PY` (default
`~/miniconda3/envs/ghost_mavgui/bin/python`), so it never needs activating; any Python 3.10 env
with the same requirements works if `GSIM_PY` points at its interpreter. The host build goes to
`build/`, the container's to `build-docker/`, so both can exist side by side.

## CMake Options

| Option | Default | Meaning |
| --- | --- | --- |
| `GSIM_WITH_MUJOCO` | `ON` | Build `simulators/gsim_mujoco`. |
| `GSIM_WITH_BULLET` | `ON` | Build `simulators/gsim_bullet`. |
| `GSIM_WITH_GAZEBO` | `ON` when `gz-sim8` is found | Build `simulators/gsim_gazebo` (on in the container). |
| `GSIM_MUJOCO_DIR` | (download) | An unpacked MuJoCo 4.0.0 release to use instead of downloading it. |
