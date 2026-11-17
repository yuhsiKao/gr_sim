# simulators

One package per simulator. Each implements `gsim::ISimBackend`
(`gsim_core/include/gsim_core/sim_backend.hpp`) and depends only on
[`gsim_core`](../gsim_core/README.md) and its own engine: simulators never include each other or
`ghost_sim`.

## Package Layout

| Package | `gsim_backend` | CMake option | Engine | Window |
| --- | --- | --- | --- | --- |
| [`gsim_mujoco`](gsim_mujoco/README.md) | `mujoco` (default) | `GSIM_WITH_MUJOCO` | MuJoCo 4.0.0 (official release, downloaded by CMake) | MuJoCo simulate, passive mode |
| [`gsim_bullet`](gsim_bullet/README.md) | `bullet` | `GSIM_WITH_BULLET` | Bullet, through the SDK's SimWrapper | Bullet's own GUI |
| [`gsim_gazebo`](gsim_gazebo/README.md) | `gazebo` | `GSIM_WITH_GAZEBO` (on when `gz-sim8` is found) | Gazebo Harmonic, DART | `gz sim -g`, a separate process |
| [`gsim_remote`](gsim_remote/README.md) | `remote` | `GSIM_WITH_REMOTE` | none: the physics runs in `gsim_server` (MuJoCo, one or several robots), over a unix socket | the server's |
