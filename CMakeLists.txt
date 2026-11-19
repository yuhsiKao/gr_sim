cmake_minimum_required(VERSION 3.16)
project(gr_sim C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Simulators compiled into ghost_sim; each is a package under simulators/.
option(GSIM_WITH_MUJOCO "Build the MuJoCo simulator (simulators/gsim_mujoco)" ON)
option(GSIM_WITH_BULLET "Build the Bullet simulator from the SDK's SimWrapper (simulators/gsim_bullet)" ON)
option(GSIM_WITH_REMOTE "Build the remote backend and the physics server (simulators/gsim_remote, gsim_server)" ON)
# GSIM_WITH_GAZEBO is defined in cmake/Gazebo.cmake (default: ON when gz-sim8 is found).

# Ghost SDK: read-only. Taken from CONTROLS_INSTALL_DIR (source scripts/env.sh first).
set(GSIM_SDK_DIR "$ENV{CONTROLS_INSTALL_DIR}" CACHE PATH "Ghost SDK directory (read-only)")
if(NOT GSIM_SDK_DIR OR NOT EXISTS "${GSIM_SDK_DIR}/lib/libgrcontrols.a")
  message(FATAL_ERROR
    "GSIM_SDK_DIR='${GSIM_SDK_DIR}' is not a Ghost SDK directory. "
    "Run `source scripts/env.sh` before configuring, or pass -DGSIM_SDK_DIR=...")
endif()
list(PREPEND CMAKE_PREFIX_PATH "${GSIM_SDK_DIR}")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Third-party modules are included here rather than in the packages: the imported targets
# they create (mujoco::mujoco, glfw, the SDK's packages) are only visible below the
# directory that creates them.
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/GhostSDK.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Gazebo.cmake)
if(GSIM_WITH_MUJOCO)
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Mujoco.cmake)
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/MujocoSimulate.cmake)
  find_package(OpenGL REQUIRED COMPONENTS EGL)   # headless rendering of the synthetic depth camera
endif()

# Absolute paths compiled into targets that need them (no reliance on the working directory).
function(gsim_add_path_defs target)
  target_compile_definitions(${target} PRIVATE
    GSIM_SDK_DIR="${GSIM_SDK_DIR}"
    GSIM_TEST_URDF_DIR="${GSIM_SDK_DIR}/share/gr/urdf"
    GSIM_DEFAULT_SCENE="${PROJECT_SOURCE_DIR}/simulators/gsim_mujoco/models/scene_flat.xml"
    GSIM_STAIRS_SCENE="${PROJECT_SOURCE_DIR}/simulators/gsim_mujoco/models/scene_stairs.xml"
    GSIM_DEFAULT_WORLD="${PROJECT_SOURCE_DIR}/simulators/gsim_gazebo/models/world_flat.sdf"
    GSIM_STAIRS_WORLD="${PROJECT_SOURCE_DIR}/simulators/gsim_gazebo/models/world_stairs.sdf")
endfunction()

set(GSIM_RPATH "${GSIM_SDK_DIR}/lib")
if(GSIM_WITH_MUJOCO)
  list(PREPEND GSIM_RPATH "${GSIM_MUJOCO_LIBDIR}")
endif()

# Dependencies only point downwards: gsim_core <- simulators/* <- ghost_sim, gsim_server.
add_subdirectory(gsim_core)    # engine-agnostic layer: ISimBackend, simulation loop, parameters
add_subdirectory(simulators)   # one package per simulator
# the executable: SDK platform hooks + simulator factory. Its build files go to
# build/ghost_sim-build because build/ghost_sim is the executable itself.
add_subdirectory(ghost_sim ghost_sim-build)
# the physics server for `gsim_backend remote`: MuJoCo or Gazebo behind a unix socket, without the SDK.
if(GSIM_WITH_REMOTE AND GSIM_WITH_MUJOCO)
  add_subdirectory(gsim_server gsim_server-build)
endif()

# Development tests. The project builds without them, so tests/ can be deleted.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt)
  find_package(GTest REQUIRED)
  include(GoogleTest)
  enable_testing()
  add_subdirectory(tests)
endif()
