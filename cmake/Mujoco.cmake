# MuJoCo: libmujoco, the C headers and the simulate sources from the official release archive,
# downloaded into the build tree (build/_deps). Nothing is installed, and no Python environment
# is involved. -DGSIM_MUJOCO_DIR=... uses an unpacked release of the same version instead
# (lib/, include/, simulate/). The SDK's bundled libmujoco 3.4.0 is never used.
#
# To change the version, edit these two lines; the hash is published next to the archive
# (mujoco-<version>-linux-x86_64.tar.gz.sha256). 3.17.0:
# b3246fc698ca27e05ce36047fe2b6866fe23c9e95a26b3a84dd48f011b85e3e4
set(GSIM_MUJOCO_VERSION 4.0.0)
set(GSIM_MUJOCO_RELEASE_SHA256 3d5d2d96b6d286b758c75198aa7862590c960256626d3aac53ac5721b0a697f3)

set(GSIM_MUJOCO_DIR "" CACHE PATH "Unpacked MuJoCo ${GSIM_MUJOCO_VERSION} release (lib/, include/, simulate/); empty = download it")
if(GSIM_MUJOCO_DIR)
  set(GSIM_MUJOCO_ROOT "${GSIM_MUJOCO_DIR}")
else()
  include(FetchContent)
  FetchContent_Declare(mujoco_release
    URL https://github.com/google-deepmind/mujoco/releases/download/${GSIM_MUJOCO_VERSION}/mujoco-${GSIM_MUJOCO_VERSION}-linux-x86_64.tar.gz
    URL_HASH SHA256=${GSIM_MUJOCO_RELEASE_SHA256}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SOURCE_SUBDIR _none_)  # prebuilt: nothing to configure
  FetchContent_MakeAvailable(mujoco_release)
  set(GSIM_MUJOCO_ROOT "${mujoco_release_SOURCE_DIR}")
endif()

set(GSIM_MUJOCO_SONAME "libmujoco.so.${GSIM_MUJOCO_VERSION}")
set(GSIM_MUJOCO_LIBDIR "${GSIM_MUJOCO_ROOT}/lib")
if(NOT EXISTS "${GSIM_MUJOCO_LIBDIR}/${GSIM_MUJOCO_SONAME}"
   OR NOT EXISTS "${GSIM_MUJOCO_ROOT}/include/mujoco/mujoco.h"
   OR NOT EXISTS "${GSIM_MUJOCO_ROOT}/simulate/simulate.cc")
  message(FATAL_ERROR
    "Expected ${GSIM_MUJOCO_LIBDIR}/${GSIM_MUJOCO_SONAME}, include/mujoco/mujoco.h and simulate/simulate.cc "
    "under ${GSIM_MUJOCO_ROOT}. "
    "GSIM_MUJOCO_DIR must be an unpacked MuJoCo ${GSIM_MUJOCO_VERSION} release.")
endif()

add_library(mujoco::mujoco SHARED IMPORTED)
set_target_properties(mujoco::mujoco PROPERTIES
  IMPORTED_LOCATION "${GSIM_MUJOCO_LIBDIR}/${GSIM_MUJOCO_SONAME}"
  IMPORTED_SONAME "${GSIM_MUJOCO_SONAME}"
  INTERFACE_INCLUDE_DIRECTORIES "${GSIM_MUJOCO_ROOT}/include")
