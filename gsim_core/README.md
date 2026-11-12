# gsim_core

`gsim_core` is the engine-agnostic layer shared by every simulator: the backend interface, the
simulation loop, the `gsim_*` parameters and the CSV log. It knows neither a physics engine nor
the Ghost SDK. Pure library.

## Contents

| Header (`include/gsim_core/`) | Purpose |
| --- | --- |
| `sim_backend.hpp` | `ISimBackend`, the interface every simulator implements; `SimConfig`, `RobotSpec`, `BaseState` and their conventions. |
| `sim_loop.hpp` | `SimLoop`: physics steps, the SDK control tick every `substeps` steps, pausing, stopping; the 0.5 ms step rule (`kPhysicsTimestep`). |
| `pacer.hpp` | Real-time pacing (`gsim_rtf`), without catching up after pauses. |
| `gsim_args.hpp` | The `gsim_*` parameters, their validation, and the SDK write-path guard. |
| `robot_select.hpp` | URDF choice and start pose (`RobotSpec`) for the robot the SDK reports. |
| `csv_logger.hpp` | `gsim_log`: state CSV at 100 Hz. |
| `sim_math.hpp` | Orientation helpers (Bullet's Euler convention used by the SDK). |
| `depth_camera.hpp` | Ghost's synthetic depth camera (`FactoryDepthCamera`) and depth image → body-frame point cloud. |
| `bullet_motor.hpp` | Bullet's joint motor as Ghost uses it (a velocity constraint with gains and a force limit), behind the arm lock and the wrist servo. |

The interface contract is documented on `ISimBackend` in `sim_backend.hpp`; the parameters are listed in
[`ghost_sim/README.md`](../ghost_sim/README.md).
