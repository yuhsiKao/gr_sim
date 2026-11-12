# Ghost SDK dependencies as an INTERFACE target `gsim_sdk`.
# Mirrors the SDK's share/gr/examples/CMakeLists.txt for ARCH=bullet3 on Linux
# (same packages, pinocchio fix-ups and link order), with ${CMAKE_PREFIX_PATH}
# replaced by GSIM_SDK_DIR. libbullet3_llc.a is deliberately NOT linked: it
# defines its own gr::Robot::bspInit/bspBegin.
set(Boost_NO_WARN_NEW_VERSIONS ON)
set(COAL_DISABLE_HPP_FCL_WARNINGS ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(Threads REQUIRED)
find_package(nanopb REQUIRED)
find_package(ryml REQUIRED)
find_package(octomap REQUIRED)
set_target_properties(octomap PROPERTIES SYSTEM true)
find_package(ompl REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFTDI1 REQUIRED IMPORTED_TARGET libftdi1)
set_target_properties(PkgConfig::LIBFTDI1 PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${GSIM_SDK_DIR}/include")
find_package(Boost REQUIRED)
find_package(pinocchio REQUIRED)
set_target_properties(pinocchio::pinocchio_default PROPERTIES
  INTERFACE_COMPILE_DEFINITIONS "PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION;COAL_DISABLE_HPP_FCL_WARNINGS"
  INTERFACE_INCLUDE_DIRECTORIES "${GSIM_SDK_DIR}/include")
set_target_properties(pinocchio::pinocchio_parsers PROPERTIES
  INTERFACE_COMPILE_DEFINITIONS "PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION;PINOCCHIO_WITH_URDFDOM"
  INTERFACE_INCLUDE_DIRECTORIES "${GSIM_SDK_DIR}/include;${GSIM_SDK_DIR}/lib/urdfdom/cmake/../../../include;${GSIM_SDK_DIR}/lib/urdfdom_headers/cmake/../../../include"
  INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${GSIM_SDK_DIR}/lib/urdfdom/cmake/../../../include;${GSIM_SDK_DIR}/lib/urdfdom_headers/cmake/../../../include"
  INTERFACE_LINK_LIBRARIES "pinocchio::pinocchio_headers;pinocchio::pinocchio_default;pinocchio::pinocchio_collision;Boost::filesystem;${GSIM_SDK_DIR}/lib/liburdfdom_sensor.so;${GSIM_SDK_DIR}/lib/liburdfdom_model_state.so;${GSIM_SDK_DIR}/lib/liburdfdom_model.so;${GSIM_SDK_DIR}/lib/liburdfdom_world.so")
find_package(CycloneDDS REQUIRED)
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)

add_library(gsim_sdk INTERFACE)
target_include_directories(gsim_sdk SYSTEM INTERFACE
  ${GSIM_SDK_DIR}/include
  ${GSIM_SDK_DIR}/include/gr
  ${GSIM_SDK_DIR}/include/bullet
  ${Boost_INCLUDE_DIRS})
target_link_directories(gsim_sdk INTERFACE ${GSIM_SDK_DIR}/lib)
# CMake merges this list with other targets' dependencies and reorders items (it
# hoisted libryml.a and the pinocchio/octomap .so files ahead of the archives that
# need them). The SDK's static archives therefore go in one rescanned link group
# (--start-group/--end-group), and shared libs are kept with --no-as-needed, so the
# result no longer depends on the order CMake emits.
target_link_options(gsim_sdk INTERFACE "LINKER:--no-as-needed")
# SDK .so files depend on further SDK .so files (libgrqpsolver.so -> libosqp.so) and
# carry no runpath. Old-style DT_RPATH also applies to those indirect dependencies,
# so executables run without LD_LIBRARY_PATH.
target_link_options(gsim_sdk INTERFACE "LINKER:--disable-new-dtags")
# Bullet's archives embed an old tinyxml2; exporting it from the executable makes the
# SDK's liburdfdom bind to it instead of its own copy (Ghost's prebuilt grcontrols_proc
# exports none). Only these archives are hidden: SDK .so files do need other symbols
# from the executable (e.g. loguru).
target_link_options(gsim_sdk INTERFACE "LINKER:--exclude-libs,libBulletRobotics.a:libBulletXmlWorldImporter.a")
target_link_libraries(gsim_sdk INTERFACE
  "$<LINK_GROUP:RESCAN,grcontrols,minitrace,grwbc,grarm,grmodemgr,grmpc1,grcommonqp,grstepgen,cilantro,grpipe,grperception,grfullkinematics,ryml::ryml,loguru,grllc,BulletRoboticsGUI,BulletRobotics,BulletExampleBrowserLib,BulletFileLoader,BulletWorldImporter,BulletDynamics,BulletCollision,BulletInverseDynamicsUtils,BulletInverseDynamics,LinearMath,OpenGLWindow,gwen,Bullet3Common,grupst_lib,grmblink>"
  # shared libraries (order-independent with --no-as-needed)
  grqpsolver octomap pinocchio::pinocchio_parsers ompl nanopb::protobuf-nanopb CycloneDDS::ddsc
  Threads::Threads Eigen3::Eigen dl PkgConfig::LIBFTDI1 ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})
