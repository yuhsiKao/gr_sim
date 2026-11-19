# gsim_server: the physics for `ghost_sim gsim_backend remote`. A MuJoCo or Gazebo world served to
# controller processes over a unix socket (simulators/gsim_remote). Does not link the Ghost SDK.
add_executable(gsim_server src/main.cpp)
target_link_libraries(gsim_server PRIVATE gsim_remote gsim_mujoco)
if(GSIM_WITH_GAZEBO)
  target_link_libraries(gsim_server PRIVATE gsim_gazebo)
  target_compile_definitions(gsim_server PRIVATE GSIM_WITH_GAZEBO)
endif()
gsim_add_path_defs(gsim_server)
# Output stays at build/gsim_server, where scripts/run_server.sh expects it.
set_target_properties(gsim_server PROPERTIES
  BUILD_RPATH "${GSIM_MUJOCO_LIBDIR}"
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
