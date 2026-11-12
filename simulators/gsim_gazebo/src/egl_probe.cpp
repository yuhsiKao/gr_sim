#include "egl_probe.hpp"

#include <EGL/egl.h>
#include <EGL/eglext.h>

namespace gsim {

bool offscreenRenderingAvailable(std::string* why) {
  const auto query_devices = reinterpret_cast<PFNEGLQUERYDEVICESEXTPROC>(eglGetProcAddress("eglQueryDevicesEXT"));
  const auto platform_display = reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(eglGetProcAddress("eglGetPlatformDisplayEXT"));
  if (!query_devices || !platform_display) {
    if (why) *why = "no EGL device extension (EGL_EXT_device_enumeration / EGL_EXT_platform_device)";
    return false;
  }
  EGLDeviceEXT devices[16];
  EGLint count = 0;
  if (!query_devices(16, devices, &count) || count <= 0) {
    if (why) *why = "no EGL device";
    return false;
  }
  for (EGLint i = 0; i < count; ++i) {
    EGLDisplay dpy = platform_display(EGL_PLATFORM_DEVICE_EXT, devices[i], nullptr);
    EGLint major = 0, minor = 0;
    if (dpy != EGL_NO_DISPLAY && eglInitialize(dpy, &major, &minor)) {
      eglTerminate(dpy);
      return true;
    }
  }
  if (why) *why = "no EGL device could be initialized";
  return false;
}

}  // namespace gsim
