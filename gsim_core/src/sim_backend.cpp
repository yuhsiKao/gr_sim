#include "gsim_core/sim_backend.hpp"

#include <cmath>

namespace gsim {

std::optional<WristMode> wristModeFromSdkValue(int v) {
  switch (v) {
    case 0: return WristMode::kOff;
    case 2: return WristMode::kCurrent;
    case 4: return WristMode::kPositionOverCurrent;
    default: return std::nullopt;
  }
}

int sanitizeWristCommand(WristCommand& cmd) {
  int bad = 0;
  for (int i = 0; i < 4; ++i) {
    if (std::isfinite(cmd.pos_des[i]) && std::isfinite(cmd.vel_des[i]) && std::isfinite(cmd.max_current[i])) continue;
    cmd.pos_des[i] = cmd.vel_des[i] = cmd.max_current[i] = 0.f;
    ++bad;
  }
  return bad;
}

std::string wristCommandWarning(WristCommand& cmd, long& nan_count, const std::string& backend) {
  if (sanitizeWristCommand(cmd) == 0 || !shouldReport(nan_count)) return "";
  return backend + ": NaN/inf in the wrist command; that joint gets no force (count=" + std::to_string(nan_count) + ")";
}

std::string armSupportError(const ISimBackend& backend, bool has_arm, const std::string& backend_name) {
  if (!has_arm || backend.supportsArm()) return "";
  return "the " + backend_name + " simulator does not support the arm";
}

}  // namespace gsim
