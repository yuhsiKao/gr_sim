# gsim_remote

Remote backend (`gsim_backend remote`). The physics does not run inside `ghost_sim` but in a
second process, [`gsim_server`](../../gsim_server/src/main.cpp), reached over a unix domain
socket. It is the first step towards several robots in one world: every controller process needs
its own network namespace (the SDK's MBLink port is fixed), so the world they share has to live
outside them.

The package holds both ends and the protocol between them. It depends only on `gsim_core`.

| File | Purpose |
| --- | --- |
| `protocol.hpp` | Message layout and the socket helpers. |
| `RemoteBackend` | `ISimBackend` for `ghost_sim`: one round trip to the server per physics step. |
| `BackendServer` | Serves any `ISimBackend` to one controller; real-time pacing and pause. |

## Run

```bash
# terminal 1: the physics (MuJoCo), with its window
scripts/run_server.sh gsim_server /tmp/gsim.sock
# terminal 2: the controller
scripts/run_sim.sh gsim_backend remote gsim_server /tmp/gsim.sock
# terminal 3: command the robot as usual
scripts/mavgui.sh
```

Start the server first. It waits for one controller and stops when that controller leaves, so
both are started again for the next run. Ctrl-C in either terminal, or closing the window, stops
both.

## Parameters

`gsim_server` takes the parameters of the physics; `ghost_sim` keeps the ones of the controller.
Giving `ghost_sim` a parameter that belongs to the server is an error.

| Parameter | `gsim_server` | `ghost_sim` (remote) |
| --- | --- | --- |
| `gsim_server <path>` | required: the socket to listen on | required: the socket to connect to |
| `gsim_headless` | `1` = no window (default `0`) | always headless |
| `gsim_rtf` | real-time factor (default `1.0`, `0` = as fast as possible) | error |
| `gsim_scene`, `gsim_toe_friction`, `gsim_toe_solref` | as for `gsim_backend mujoco` | error |
| `gsim_urdf_dir` | where the URDF is loaded from | where the robot type is looked up |
| `gsim_control_rate`, `gsim_log` | not used | as usual |

Both processes must be the same build and see the same SDK: the controller sends the URDF's
file name and the server loads that file from its own `gsim_urdf_dir`.

## Protocol

Packed structs in host byte order behind a 12-byte header (magic `GSIM`, version, type, size);
both ends run on one machine. A different version on the other end is refused.

| Message | Direction | Contents |
| --- | --- | --- |
| `Hello` / `Welcome` | controller → server / back | Handshake; the server's physics step. |
| `LoadRobot` / `Loaded` | controller → server / back | URDF file name, joint count, fixed base, start pose; success or the reason. |
| `ResetJoint` | controller → server | Initial joint angle. |
| `Step` / `State` | controller → server / back | Once per physics step: joint torques and external force; base pose, velocities and joint state. |
| `Bye` | controller → server | The controller is leaving. |
| `Error` | server → controller | A message the server could not accept. |

The world steps only when the controller asks, so the controller and the physics share one clock
and the result is identical to `gsim_backend mujoco`, bit for bit. One round trip costs about
5 µs.

## Behaviour

| Situation | What happens |
| --- | --- |
| No server at the socket | `ghost_sim` exits with code 2 and names the path. |
| The server stops (Ctrl-C, window closed) | `ghost_sim` stops by itself and exits normally. |
| The controller stops or crashes | The server stops. |
| The server is paused (space bar in its window) | The controller waits in its step; Ctrl-C still stops it. |
| Another user on the machine tries to connect | Refused: the socket file is readable and writable by its owner only. |
| A socket file left by a crashed server | Replaced by the next server. A path another server is using is refused. |

## Limits

- One robot per server.
- Legs only: no arm (`arm 1` is refused) and no depth camera, so no terrain perception.
- The server's physics is MuJoCo.
