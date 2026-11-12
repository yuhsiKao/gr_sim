# MuJoCo's interactive viewer ("simulate") is not built into libmujoco, but its sources come with
# the release (cmake/Mujoco.cmake) and are compiled here. Nothing is installed; the SDK's bundled
# simulate 3.4.0 (include/simulate, lib/libsimulate.a) is never used.
include(FetchContent)
# lodepng commit pinned by MuJoCo 4.0.0 (cmake/third_party_deps/lodepng.cmake; the same in 3.17.0).
FetchContent_Declare(lodepng
  GIT_REPOSITORY https://github.com/lvandeve/lodepng.git
  GIT_TAG 22561883dd63fd1850f18e1f6adac321e4f609b0
  SOURCE_SUBDIR _none_)
FetchContent_MakeAvailable(lodepng)

find_package(glfw3 3.3 REQUIRED)

# Same sources and settings as simulate/CMakeLists.txt (libmujoco_simulate) on Linux.
set(_gsim_simulate_dir ${GSIM_MUJOCO_ROOT}/simulate)
add_library(gsim_simulate STATIC
  ${_gsim_simulate_dir}/simulate.cc
  ${_gsim_simulate_dir}/platform_ui_adapter.cc
  ${_gsim_simulate_dir}/glfw_adapter.cc
  ${_gsim_simulate_dir}/glfw_dispatch.cc
  ${lodepng_SOURCE_DIR}/lodepng.cpp
)
set_target_properties(gsim_simulate PROPERTIES CXX_STANDARD 20 POSITION_INDEPENDENT_CODE ON)
target_include_directories(gsim_simulate PUBLIC ${_gsim_simulate_dir} PRIVATE ${lodepng_SOURCE_DIR})
target_link_libraries(gsim_simulate PUBLIC mujoco::mujoco glfw)
