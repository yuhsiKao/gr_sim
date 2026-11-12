# One package per simulator. Each implements gsim::ISimBackend and depends only on
# gsim_core and its own engine; see README.md for adding one.
if(GSIM_WITH_MUJOCO)
  add_subdirectory(gsim_mujoco)
endif()
if(GSIM_WITH_BULLET)
  add_subdirectory(gsim_bullet)
endif()
if(GSIM_WITH_GAZEBO)
  add_subdirectory(gsim_gazebo)
endif()
