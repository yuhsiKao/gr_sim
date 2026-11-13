# gsim_mujoco

MuJoCo backend (`gsim_backend mujoco`, the default). The SDK's URDF is converted to an mjSpec in
memory and attached to a scene file; the window is MuJoCo's simulate in passive mode. MuJoCo
4.0.0 is the official release, downloaded by CMake; simulate is compiled from the same version's
sources (the version is pinned in `cmake/Mujoco.cmake`).

## Run

```bash
scripts/run_sim.sh                                   # flat ground, with the window
scripts/run_sim.sh gsim_headless 1                   # no window
scripts/run_sim.sh gsim_scene simulators/gsim_mujoco/models/scene_stairs.xml
```

## Scenes

| File | Contents |
| --- | --- |
| `models/scene_flat.xml` | Ground, lights, physics options (`cone="elliptic"`, `impratio="10"`, `noslip_iterations="5"`, 0.5 ms timestep). Default. |
| `models/scene_stairs.xml` | `scene_flat.xml` plus Ghost's staircase (first step at x = 2.851 m). |

`gsim_scene` takes any MJCF scene with a 0.5 ms timestep. Keep `cone="elliptic"` (with the default
pyramidal cone the robot sinks onto its shins) and `impratio="10"` (with the default 1 the stance
feet creep and the robot keeps taking correcting steps while standing in walk mode) and
`noslip_iterations="5"` (without it the toes' torsional friction is soft, and the body slowly yaws
over planted feet once the arm is unstowed; Ghost's Bullet toes do not let it). There is no door: Ghost's door is in the Bullet
world only.

## Window

Passive viewer: physics and the SDK run inside `ghost_sim`; commands come from mavgui (or ROS 2).

| Action | How |
| --- | --- |
| Pause / resume | **Space** (simulation and controller stop together). |
| Push the robot | Double-click a body, then **Ctrl + right-drag** (Ctrl + left-drag twists). |
| Camera | Follows the robot; left-drag rotates, right-drag pans, scroll zooms, **Esc** = free camera. |
| More room | **Tab** / **Shift + Tab** hide the left / right panel. |
| Stop | Close the window, File → Quit, or Ctrl-C. |

Not supported while the controller runs, because moving the robot behind the SDK's back breaks
its state estimate: Reset (and Load key), the Joint sliders, and timestep changes (set it in the
scene file). The Control sliders have no effect. File → Save xml / Print model / Screenshot write
into the current directory.

## Toe Contact

| Parameter | Format | Note |
| --- | --- | --- |
| `gsim_toe_friction` | `"slide torsional rolling"` | Raise `slide` against slipping. |
| `gsim_toe_solref` | `"timeconst dampratio"` | Keep `timeconst` ≥ 0.001 s. |

## Behaviour

- **Depth camera**: `depth_underside`, Ghost's synthetic downward camera, rendered offscreen with
  EGL with or without the window, while the SDK's `vision_mode` has the cameras on.
  `GSIM_EGL_DEVICE=<n>` picks the EGL device (default: the first one that works).
- **Against Bullet**: standing height 0.530 m vs 0.533 m; walking at about 0.63 m/s on both; a
  full turn command turns at 98 °/s vs 88 °/s.

## Layout

```text
gsim_mujoco/
├── include/gsim_mujoco/
│   └── sync_schedule.hpp  # when the window copies the state (60 Hz of simulated time)
├── src/
│   ├── model_builder.cpp  # SDK URDF (read-only) -> mjSpec, toe contact parameters, self-check
│   ├── MujocoBackend.cpp  # ISimBackend: stepping, state, torques, instability reporting
│   ├── MujocoViewer.cpp   # simulate in passive mode on the main thread
│   └── egl_context.cpp    # headless OpenGL context (EGL) for the depth camera
└── models/                # scene_flat.xml, scene_stairs.xml
```
