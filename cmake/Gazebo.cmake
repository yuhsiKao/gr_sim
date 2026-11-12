# Gazebo Harmonic (gz-sim 8) for simulators/gsim_gazebo. The option defaults to whether Gazebo is
# installed, so a host without it (MuJoCo / Bullet development) builds unchanged; the gr-sim
# container (docker/) has it. find_package(gz-sim8) needs cppzmq-dev as well.
#
# Gazebo's own dependencies (tinyxml2, uuid, protobuf, ...) must come from the system, not from
# the SDK prefix that CMAKE_PREFIX_PATH puts first for everything else: the SDK's tinyxml2.pc
# points at Ghost's build machine and at the SDK's static libtinyxml2.a, the very copy ghost_sim
# must not link (tests/CMakeLists.txt, GhostSimLink.NoTinyxml2Exports). So the lookup runs with
# the SDK prefix removed and pkg-config not following CMAKE_PREFIX_PATH.
set(_gsim_prefix_path "${CMAKE_PREFIX_PATH}")
list(REMOVE_ITEM CMAKE_PREFIX_PATH "${GSIM_SDK_DIR}")
set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH OFF)
find_package(gz-sim8 QUIET)
find_package(gz-rendering8 QUIET)   # the depth camera hides the robot's body in the server's sensor scene
unset(PKG_CONFIG_USE_CMAKE_PREFIX_PATH)
set(CMAKE_PREFIX_PATH "${_gsim_prefix_path}")

option(GSIM_WITH_GAZEBO "Build the Gazebo simulator (simulators/gsim_gazebo); default: when Gazebo Harmonic is found" ${gz-sim8_FOUND})
if(GSIM_WITH_GAZEBO AND NOT gz-sim8_FOUND)
  message(FATAL_ERROR "GSIM_WITH_GAZEBO=ON but gz-sim8 was not found: build inside the gr-sim container (README, Docker) or pass -DGSIM_WITH_GAZEBO=OFF")
endif()
message(STATUS "GSIM_WITH_GAZEBO: ${GSIM_WITH_GAZEBO} (gz-sim8 ${gz-sim8_FOUND})")
