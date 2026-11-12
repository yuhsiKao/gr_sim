// A headless OpenGL context through EGL, for rendering without a window (the synthetic depth
// camera). It works with or without a display. EGL types stay out of this header.
#pragma once
#include <string>

namespace gsim {

class EglContext {
 public:
  EglContext() = default;
  ~EglContext();
  EglContext(const EglContext&) = delete;
  EglContext& operator=(const EglContext&) = delete;

  // Creates a desktop-OpenGL context on an EGL device: the one numbered GSIM_EGL_DEVICE if that
  // environment variable is set, otherwise the first device that works. On failure returns false
  // and sets *error. The context is left current on the calling thread.
  bool create(std::string* error);
  bool makeCurrent();          // on the calling thread
  void release();              // detach from the calling thread
  const std::string& renderer() const { return renderer_; }   // e.g. "NVIDIA GeForce RTX 5070/PCIe/SSE2"

  // With glvnd, the first OpenGL context made current in the process (the viewer's GLX one) may patch
  // the GL entry points, and then no EGL context can be made current on another thread. Call this
  // before any OpenGL context is made current; it turns the patching off (__GLVND_DISALLOW_PATCHING=1)
  // unless that variable is already set.
  static void disableGlvndPatching();

 private:
  void* display_ = nullptr;    // EGLDisplay
  void* context_ = nullptr;    // EGLContext
  std::string renderer_;
};

}  // namespace gsim
