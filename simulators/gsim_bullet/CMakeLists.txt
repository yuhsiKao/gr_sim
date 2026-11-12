# Ghost's Bullet wrapper, compiled from the SDK path (read-only, never copied or modified).
add_library(gsim_bullet STATIC
  src/BulletBackend.cpp
  ${GSIM_SDK_DIR}/share/gr/bullet3/SimWrapper.cpp
)
target_include_directories(gsim_bullet
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE ${GSIM_SDK_DIR}/share/gr/bullet3)
target_link_libraries(gsim_bullet PUBLIC gsim_core gsim_sdk)
