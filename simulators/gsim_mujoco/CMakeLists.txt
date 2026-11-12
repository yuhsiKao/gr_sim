# MuJoCo from the official release, with the simulate viewer built from the pinned sources
# (cmake/Mujoco.cmake, cmake/MujocoSimulate.cmake).
add_library(gsim_mujoco STATIC
  src/model_builder.cpp
  src/MujocoBackend.cpp
  src/MujocoViewer.cpp
  src/egl_context.cpp
)
target_include_directories(gsim_mujoco PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(gsim_mujoco PUBLIC gsim_core mujoco::mujoco gsim_simulate PRIVATE OpenGL::EGL)
