#include "gsim_core/pacer.hpp"

#include <chrono>
#include <thread>

namespace gsim {

namespace {

double steadyNow() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void steadySleepUntil(double t_s) {
  using clock = std::chrono::steady_clock;
  std::this_thread::sleep_until(
      clock::time_point(std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(t_s))));
}

}  // namespace

Pacer::Pacer(double rtf, NowFn now, SleepUntilFn sleep_until, double max_lag_s)
    : rtf_(rtf),
      now_(now ? std::move(now) : NowFn(steadyNow)),
      sleep_until_(sleep_until ? std::move(sleep_until) : SleepUntilFn(steadySleepUntil)),
      max_lag_s_(max_lag_s) {}

void Pacer::anchor(double now_s, double sim_time_s) {
  anchored_ = true;
  wall0_ = now_s;
  sim0_ = sim_time_s;
}

void Pacer::reanchor(double sim_time_s) {
  const double now = now_();
  anchor(now, sim_time_s);
  // Restart the report window so pauses do not show up as a low real-time factor.
  report_wall0_ = now;
  report_sim0_ = sim_time_s;
}

void Pacer::wait(double sim_time_s) {
  const double now = now_();
  if (!anchored_) reanchor(sim_time_s);

  if (report_ && now - report_wall0_ >= report_period_s_) {
    report_((sim_time_s - report_sim0_) / (now - report_wall0_), lag_resets_);
    report_wall0_ = now;
    report_sim0_ = sim_time_s;
  }

  if (rtf_ <= 0.0) return;
  const double target = wall0_ + (sim_time_s - sim0_) / rtf_;
  if (now - target > max_lag_s_) {
    // Too far behind: do not try to catch up, restart pacing from here.
    anchor(now, sim_time_s);
    ++lag_resets_;
    return;
  }
  if (target > now) sleep_until_(target);
}

int Pacer::lagResets() const { return lag_resets_; }

void Pacer::setReport(ReportFn fn, double period_s) {
  report_ = std::move(fn);
  report_period_s_ = period_s;
}

}  // namespace gsim
