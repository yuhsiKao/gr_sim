# ghost_sim: the Ghost SDK's platform hooks (bspInit/bspBegin) on top of gsim_core, and the
# factory that turns `gsim_backend <name>` into one of the simulators built in.
add_library(gsim_backends STATIC src/backend_factory.cpp)
target_include_directories(gsim_backends PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(gsim_backends PUBLIC gsim_core)
foreach(sim mujoco bullet gazebo remote)
  string(TOUPPER ${sim} SIM)
  if(GSIM_WITH_${SIM})
    target_link_libraries(gsim_backends PUBLIC gsim_${sim})
    target_compile_definitions(gsim_backends PRIVATE GSIM_WITH_${SIM})
  endif()
endforeach()

add_executable(ghost_sim src/main.cpp src/sim_bsp.cpp)
target_link_libraries(ghost_sim PRIVATE gsim_backends gsim_sdk)
gsim_add_path_defs(ghost_sim)
# Output stays at build/ghost_sim, where scripts/run_sim.sh and the README expect it.
set_target_properties(ghost_sim PROPERTIES
  BUILD_RPATH "${GSIM_RPATH}"
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
