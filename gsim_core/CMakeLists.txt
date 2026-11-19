# Engine-agnostic layer: the ISimBackend interface, the simulation loop, pacing, the gsim_*
# parameters, CSV logging and the CPU idle-state hold. Depends on no simulator and not on the Ghost SDK.
add_library(gsim_core STATIC
  src/sim_math.cpp
  src/pacer.cpp
  src/gsim_args.cpp
  src/robot_select.cpp
  src/sim_loop.cpp
  src/csv_logger.cpp
  src/sim_backend.cpp
  src/bullet_motor.cpp
  src/toe_torsion.cpp
  src/depth_camera.cpp
  src/cpu_latency.cpp
)
target_include_directories(gsim_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(gsim_core PUBLIC Eigen3::Eigen)
