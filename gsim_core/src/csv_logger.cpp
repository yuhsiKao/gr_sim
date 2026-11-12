#include "gsim_core/csv_logger.hpp"

#include <cerrno>
#include <cstring>
#include <limits>

#include "gsim_core/sim_math.hpp"

namespace gsim {

namespace {

constexpr size_t kBufferBytes = 1 << 20;

void writeArray(std::FILE* f, const Eigen::Ref<const Eigen::ArrayXf>& a, int n) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  for (int j = 0; j < n; ++j) std::fprintf(f, ",%.6g", j < a.size() ? a[j] : nan);
}

}  // namespace

CsvLogger::CsvLogger(int decimation) : decimation_(decimation < 1 ? 1 : decimation) {}

CsvLogger::~CsvLogger() { close(); }

bool CsvLogger::open(const std::string& path, int num_joints, std::string* error, bool arm) {
  close();
  file_ = std::fopen(path.c_str(), "w");
  if (!file_) {
    if (error) *error = "cannot open CSV log " + path + ": " + std::strerror(errno);
    return false;
  }
  buffer_.resize(kBufferBytes);
  std::setvbuf(file_, buffer_.data(), _IOFBF, buffer_.size());
  num_joints_ = num_joints;
  arm_ = arm;
  calls_ = 0;

  std::fputs("t,px,py,pz,roll,pitch,yaw,vx,vy,vz", file_);
  for (const char* prefix : {"q", "dq", "tau"})
    for (int j = 0; j < num_joints_; ++j) std::fprintf(file_, ",%s%d", prefix, j);
  if (arm_) {
    std::fputs(",arm_locked,wmode", file_);
    for (const char* prefix : {"wq", "wdq", "wcur"})
      for (int j = 0; j < 4; ++j) std::fprintf(file_, ",%s%d", prefix, j);
  }
  std::fputc('\n', file_);
  return true;
}

void CsvLogger::write(const TickInfo& tick) {
  if (!file_) return;
  if (calls_++ % decimation_ != 0) return;
  const BaseState& b = tick.base;
  const Eigen::Vector3f rpy = rpyFromQuat(b.quat_WB);
  std::fprintf(file_, "%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g", tick.sim_time, b.pos_W.x(), b.pos_W.y(),
               b.pos_W.z(), rpy.x(), rpy.y(), rpy.z(), b.vel_W.x(), b.vel_W.y(), b.vel_W.z());
  writeArray(file_, tick.q, num_joints_);
  writeArray(file_, tick.dq, num_joints_);
  writeArray(file_, tick.last_tau, num_joints_);
  if (arm_) {
    if (tick.last_arm)
      std::fprintf(file_, ",%d,%d", tick.last_arm->locked ? 1 : 0, static_cast<int>(tick.last_arm->wrist.mode));
    else
      std::fputs(",nan,nan", file_);
    static const WristState kNoWrist;   // all NaN
    const WristState& w = tick.wrist ? *tick.wrist : kNoWrist;
    writeArray(file_, w.q, 4);
    writeArray(file_, w.dq, 4);
    writeArray(file_, w.current, 4);
  }
  std::fputc('\n', file_);
}

void CsvLogger::close() {
  if (file_) std::fclose(file_);
  file_ = nullptr;
}

}  // namespace gsim
