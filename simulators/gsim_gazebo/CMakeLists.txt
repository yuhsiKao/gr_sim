# Gazebo Harmonic backend: a gz::sim::Server inside ghost_sim, stepped by SimLoop (cmake/Gazebo.cmake
# finds gz-sim8). Gazebo's headers stay out of the public header.
add_library(gsim_gazebo STATIC
  src/gazebo_model.cpp
  src/gazebo_system.cpp
  src/GazeboBackend.cpp
  src/egl_probe.cpp
)
find_package(OpenGL REQUIRED COMPONENTS EGL)   # probe for offscreen rendering before the depth camera starts Gazebo's renderer
target_include_directories(gsim_gazebo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(gsim_gazebo PUBLIC gsim_core PRIVATE gz-sim8::gz-sim8 gz-rendering8::gz-rendering8 OpenGL::EGL)
