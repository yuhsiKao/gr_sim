# gsim_bullet

Reference backend (`gsim_backend bullet`): the Ghost SDK's own Bullet wrapper,
`share/gr/bullet3/SimWrapper.cpp`, compiled from the SDK path (read-only, never copied or
modified). It gives Ghost's physics and Ghost's world under the same loop as the other backends.

## Run

```bash
scripts/run_sim.sh gsim_backend bullet               # Bullet's own GUI
scripts/run_sim.sh gsim_backend bullet arm 1         # with the arm; the door is at (-2, 3, 0)
```

The world (ground, stairs, door, boxes from about x = 3 m) is fixed by `SimWrapper::setup()`;
there is no world parameter.

## Behaviour

- **Start pose**: differs from the factory sim, whose bsp resets the wrong joints at start-up;
  this backend resets the ones the SDK names.
- **Torques**: the SDK's joint slots that have no joint in the URDF (Vision 60's 3 arm slots
  without `arm 1`) are kept off the robot; in the factory wiring they would land on joint 11.
- **Depth camera**: SimWrapper's own (`perceptionUpdate`). Headless, Bullet renders it on the CPU
  (about 52 ms per frame), so with the cameras on it runs at about 0.4× real time; the window
  keeps real time.

## Layout

```text
gsim_bullet/
├── include/gsim_bullet/   # BulletBackend (ISimBackend)
└── src/BulletBackend.cpp  # wraps SimWrapper: start pose, torques, state, depth cloud
```
