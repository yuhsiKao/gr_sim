// Whether offscreen (EGL device) rendering is available, checked before Gazebo's renderer is
// started: without it Ogre2 falls back to GLX, fails and crashes the process.
#pragma once
#include <string>

namespace gsim {

// true when an EGL device can be initialized; otherwise false with the reason in *why.
bool offscreenRenderingAvailable(std::string* why);

}  // namespace gsim
