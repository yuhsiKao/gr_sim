#pragma once

namespace gsim {

// Process exit code decided by the platform layer:
// 0 = normal, 1 = selftest failed, 2 = configuration or startup error.
int bspExitCode();

}  // namespace gsim
