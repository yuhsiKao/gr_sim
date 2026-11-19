# gsim_remote

Remote backend (`gsim_backend remote`). The physics does not run inside `ghost_sim` but in a
second process, [`gsim_server`](../../gsim_server/src/main.cpp), reached over a unix domain
socket. This is what puts several robots into one world: every controller process needs its own
network namespace (the SDK's MBLink port is fixed), so the world they share has to live outside
them.

The package holds both ends and the protocol between them. It depends only on `gsim_core`.

| File | Purpose |
| --- | --- |
| `protocol.hpp` | Message layout and the socket helpers. |
| `RemoteBackend` | `ISimBackend` for `ghost_sim`: one round trip to the server per physics step, one more per depth frame. |
| `WorldServer` | Serves an `ISimWorld` to its controllers, one robot each; real-time pacing and pause. |

The world behind the server is MuJoCo ([`MujocoWorld`](../gsim_mujoco/README.md), the default) or
Gazebo ([`GazeboWorld`](../gsim_gazebo/README.md), `gsim_backend gazebo` on the server).

## Run: One Robot

```bash
# terminal 1: the physics (MuJoCo), with its window
scripts/run_server.sh gsim_server /tmp/gsim.sock
# ... or Gazebo:  scripts/run_server.sh gsim_server /tmp/gsim.sock gsim_backend gazebo
# terminal 2: the controller
scripts/run_sim.sh gsim_backend remote gsim_server /tmp/gsim.sock
# terminal 3: command the robot as usual
scripts/mavgui.sh
```

Start the server first. It stops when the controller leaves, so both are started again for the
next run. Ctrl-C in either terminal, or closing the window, stops both.

## Run: Several Robots

Each controller runs in its own container, next to one container for the physics
([`docker/compose.multi.yaml`](../../docker/compose.multi.yaml)). On the host:

```bash
scripts/multi.sh up            # once: gr-physics, gr-robot-0, gr-robot-1
scripts/multi.sh server        # terminal 1: the world and its window, waits for 2 robots
                               #   (Gazebo: scripts/multi.sh server gsim_backend gazebo)
scripts/multi.sh robot 0       # terminal 2
scripts/multi.sh robot 1       # terminal 3; the simulation starts when all robots have joined
scripts/multi.sh mavgui 0      # terminal 4: commands robot 0
scripts/multi.sh mavgui 1      # terminal 5: commands robot 1
scripts/multi.sh down          # when done
```

- **Each robot is its own controller.** It has its own MBLink (reached from inside its container,
  which is what `multi.sh mavgui <i>` does) and its own DDS domain, 124 + index.
- **The robots share the world.** They collide with each other, and the window, the real-time
  factor and pause (space bar) apply to all of them.
- **Each robot is configured by its own controller.** `scripts/multi.sh robot 0 arm 1` gives robot
  0 the arm while robot 1 has none; every robot has its own depth camera, on while its
  `vision_mode` turns the cameras on.
- **One gamepad drives one robot at a time.** Start every mavgui with `--joystick`
  (`scripts/multi.sh mavgui 0 --joystick`, `... mavgui 1 --joystick`): the gamepad starts on robot
  0; D-pad right hands it to the next robot, D-pad left to the previous one, among the robots of
  the server that was started last (`GSIM_ROBOTS=1 scripts/multi.sh server`: nothing to switch).
  The window title shows `[GAMEPAD]` on the robot that has it; the others stand still, and the
  gamepad's buttons (soft e-stop, arm) only reach that robot.
  Without `--joystick` a mavgui takes the keyboard while its window has the focus.
- **Robot `i` starts 1.5 m beside robot `i - 1`.** `gsim_spawn "x y yaw_deg"` after
  `multi.sh robot <i>` puts it elsewhere; robots must start at least 1 m apart.
- **A controller that stops leaves its robot in the world.** The robot gets no torque any more and
  collapses where it is; the others carry on. The server stops when the last controller has left.

## Parameters

`gsim_server` takes the parameters of the physics; `ghost_sim` keeps the ones of the controller.
Giving one program a parameter that belongs to the other is an error.

| Parameter | `gsim_server` | `ghost_sim` (remote) |
| --- | --- | --- |
| `gsim_server <path>` | required: the socket to listen on | required: the socket to connect to |
| `gsim_backend` | the physics: `mujoco` (default) or `gazebo` | `remote` |
| `gsim_robots <n>` | robots in the world, 1 to 8 (default `1`) | error |
| `gsim_robot <i>` | error | which robot this controller drives (default `0`) |
| `gsim_spawn "x y yaw_deg"` | error | where the robot starts (default `"0 0 0"`) |
| `gsim_headless` | `1` = no window (default `0`) | always headless |
| `gsim_rtf` | real-time factor (default `1.0`, `0` = as fast as possible) | error |
| `gsim_scene`, `gsim_toe_friction`, `gsim_toe_solref` | as for `gsim_backend mujoco` | error |
| `gsim_world` | as for `gsim_backend gazebo` | error |
| `gsim_urdf_dir` | where the URDF is loaded from | where the robot type is looked up |
| `gsim_control_rate`, `gsim_log` | error | as usual; may differ between robots |
| `arm` (the SDK's own argument) | not used | `1` = this robot has the arm |

All processes must be the same build and see the same SDK: a controller sends the URDF's file
name and the server loads that file from its own `gsim_urdf_dir`.

## Protocol

Packed structs in host byte order behind a 12-byte header (magic `GSIM`, version, type, size);
both ends run on one machine. A different version on the other end is refused.

| Message | Direction | Contents |
| --- | --- | --- |
| `Hello` / `Welcome` | controller → server / back | The robot's index; the server's physics step and number of robots. |
| `LoadRobot` / `Loaded` | controller → server / back | URDF file name, joint count, fixed base, start pose, arm joints; success or the reason. Answered once every robot has joined and the world is built. |
| `ResetJoint` | controller → server | Initial joint angle. |
| `Step` / `State` | controller → server / back | Once per physics step: joint torques, external force, arm lock and wrist command; base pose, velocities, joint and wrist state. |
| `RenderDepth` / `Cloud` | controller → server / back | Every 66 steps while the robot's cameras are on: the depth camera's point cloud (75 × 1500 points, 1.35 MB). |
| `Bye` | controller → server | The controller is leaving. |
| `Error` | server → controller | A message the server could not accept. |

The world steps once every connected controller has asked for the step, so the controllers and
the physics share one clock. With one robot the result is identical to the same simulator inside
`ghost_sim`, bit for bit: joints, wrist and (MuJoCo) depth cloud. One round trip costs about 5 µs,
a depth frame about 1 ms.

Gazebo renders its cameras on its own schedule, every 66 steps of the world, so a controller gets
the newest of those frames: up to 33 ms old, where `gsim_backend gazebo` gets the frame of the
very step.

## Real Time

A controller sleeps while it waits for the world's step. A Gazebo step is long enough for the
idle cores to enter a deep idle state, and waking them on every step costs more time than the
step has left. While a Gazebo world runs, `gsim_server` therefore asks Linux to keep the CPUs out
of idle states that take more than 50 µs to leave: it holds `/dev/cpu_dma_latency` open. The
request covers the whole machine, which draws more power meanwhile, and ends with the server,
however it stops. A MuJoCo world does not need it and does not ask.

Real-time factor with `gsim_rtf 0` (as fast as possible), standing on flat ground, no window, on
an Intel Core Ultra 9 285K:

| Robots on Gazebo | Deep idle states held off | Not held off |
| --- | --- | --- |
| 1 | 3.5 to 3.9 | 1.8 to 2.0 |
| 2 | 2.2 | 0.8 to 1.0 |
| 3 | 1.5 to 1.6 | 0.65 to 0.7 |

The server says which case applies when it starts:

| Message | Meaning |
| --- | --- |
| `deep CPU idle states are held off while the server runs` | The request is in force. |
| `deep CPU idle states are not held off (<reason>)` | The device is missing or cannot be written (it needs root). One robot keeps real time anyway. |

`gr-physics` gets the device from [`docker/compose.multi.yaml`](../../docker/compose.multi.yaml);
a container created before that line was added gets it with `scripts/multi.sh up`. Where the
device cannot be had, `sudo cpupower idle-set -D 100` on the host does the same for the whole
machine until `sudo cpupower idle-set -E`.

## Behaviour

| Situation | What happens |
| --- | --- |
| No server at the socket | `ghost_sim` exits with code 2 and names the path. |
| Not every robot has a controller yet | The controllers that joined wait; Ctrl-C ends a waiting controller and frees its robot for another one. |
| A robot index the server does not have, or one that is taken | That `ghost_sim` exits with code 2 and the reason; the others are not disturbed. |
| Two robots start less than 1 m apart | The server refuses to build the world and stops; every controller reports the reason. |
| A controller connects after the simulation has started | Refused; the running robots are not disturbed. |
| The server stops (Ctrl-C, window closed) | Every `ghost_sim` stops by itself and exits normally. |
| A controller stops or crashes while the simulation runs | Its robot stays in the world without torque, its arm locked; the server stops when the last controller has left. |
| A controller hangs without disconnecting | The world waits for it, and the server names the robot every 5 s. |
| The server is paused (space bar in its window) | Every controller waits in its step; Ctrl-C still stops it. |
| Another user on the machine tries to connect | Refused: the socket file is readable and writable by its owner only. |
| A socket file left by a crashed server | Replaced by the next server. A path another server is using is refused. |

## Limits

- The server's physics is MuJoCo or Gazebo. The Bullet simulator is the SDK's own and holds one
  robot.
- Gazebo keeps real time with several robots only while deep CPU idle states are held off (Real
  Time above). Each robot adds about 0.1 ms to its 0.5 ms step, so expect real time to be lost at
  about five robots. MuJoCo runs two robots at more than six times real time.
- The number of robots is fixed when the server starts; a robot cannot join a running simulation.
- Robots in different containers only see their own MBLink and DDS domain: tools for robot `i`
  run in `gr-robot-<i>` (or in the same DDS domain on the same Docker network).
