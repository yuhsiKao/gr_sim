# The physics server's wire protocol, the backend that talks to it (RemoteBackend) and the
# server side that serves any ISimBackend (BackendServer). Depends only on gsim_core.
add_library(gsim_remote STATIC
  src/protocol.cpp
  src/RemoteBackend.cpp
  src/BackendServer.cpp
)
target_include_directories(gsim_remote PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(gsim_remote PUBLIC gsim_core)
