#include "gsim_mujoco/egl_context.hpp"

#include <EGL/egl.h>
#include <EGL/eglext.h>

#include <cstdlib>
#include <string>
#include <vector>

namespace gsim {

namespace {

constexpr unsigned kGlRenderer = 0x1F01;   // GL_RENDERER (no GL headers needed)

std::string eglErrorText(const char* what) {
  char hex[16];
  std::snprintf(hex, sizeof(hex), "0x%04x", eglGetError());
  return std::string(what) + " failed (EGL error " + hex + ")";
}

}  // namespace

EglContext::~EglContext() {
  if (!display_) return;
  EGLDisplay dpy = static_cast<EGLDisplay>(display_);
  eglMakeCurrent(dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
  if (context_) eglDestroyContext(dpy, static_cast<EGLContext>(context_));
  eglTerminate(dpy);
}

bool EglContext::create(std::string* error) {
  auto fail = [error](const std::string& msg) {
    if (error) *error = msg;
    return false;
  };
  auto query_devices = reinterpret_cast<PFNEGLQUERYDEVICESEXTPROC>(eglGetProcAddress("eglQueryDevicesEXT"));
  auto platform_display = reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(eglGetProcAddress("eglGetPlatformDisplayEXT"));
  if (!query_devices || !platform_display) return fail("EGL device enumeration (EGL_EXT_device_enumeration) is unavailable");

  EGLint count = 0;
  if (!query_devices(0, nullptr, &count) || count <= 0) return fail("no EGL devices");
  std::vector<EGLDeviceEXT> devices(count);
  query_devices(count, devices.data(), &count);

  int first = 0, last = count - 1;
  if (const char* env = std::getenv("GSIM_EGL_DEVICE")) {
    first = last = std::atoi(env);
    if (first < 0 || first >= count)
      return fail("GSIM_EGL_DEVICE=" + std::string(env) + " but there are " + std::to_string(count) + " EGL devices");
  }

  std::string last_error = "no usable EGL device";
  for (int i = first; i <= last; ++i) {
    EGLDisplay dpy = platform_display(EGL_PLATFORM_DEVICE_EXT, devices[i], nullptr);
    if (dpy == EGL_NO_DISPLAY || !eglInitialize(dpy, nullptr, nullptr)) {
      last_error = "device " + std::to_string(i) + ": " + eglErrorText("eglInitialize");
      continue;
    }
    const EGLint config_attribs[] = {EGL_SURFACE_TYPE, EGL_PBUFFER_BIT, EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
                                     EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_DEPTH_SIZE, 24,
                                     EGL_NONE};
    EGLConfig config;
    EGLint n = 0;
    EGLContext ctx = EGL_NO_CONTEXT;
    if (eglChooseConfig(dpy, config_attribs, &config, 1, &n) && n == 1 && eglBindAPI(EGL_OPENGL_API))
      ctx = eglCreateContext(dpy, config, EGL_NO_CONTEXT, nullptr);
    // MuJoCo renders into its own offscreen framebuffer, so no surface is needed.
    if (ctx == EGL_NO_CONTEXT || !eglMakeCurrent(dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, ctx)) {
      last_error = "device " + std::to_string(i) + ": " + eglErrorText("OpenGL context creation");
      if (ctx != EGL_NO_CONTEXT) eglDestroyContext(dpy, ctx);
      eglTerminate(dpy);
      continue;
    }
    display_ = dpy;
    context_ = ctx;
    auto get_string = reinterpret_cast<const unsigned char* (*)(unsigned)>(eglGetProcAddress("glGetString"));
    const unsigned char* name = get_string ? get_string(kGlRenderer) : nullptr;
    renderer_ = "device " + std::to_string(i) + ": " + (name ? reinterpret_cast<const char*>(name) : "unknown");
    return true;
  }
  return fail(last_error);
}

void EglContext::disableGlvndPatching() { setenv("__GLVND_DISALLOW_PATCHING", "1", /*overwrite=*/0); }

bool EglContext::makeCurrent() {
  return display_ && eglMakeCurrent(static_cast<EGLDisplay>(display_), EGL_NO_SURFACE, EGL_NO_SURFACE,
                                    static_cast<EGLContext>(context_));
}

void EglContext::release() {
  if (display_) eglMakeCurrent(static_cast<EGLDisplay>(display_), EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
}

}  // namespace gsim
