// Keeps simulated time aligned with wall-clock time at a target real-time factor.
#pragma once
#include <functional>

namespace gsim {

class Pacer {
 public:
  using NowFn = std::function<double()>;              // monotonic seconds
  using SleepUntilFn = std::function<void(double)>;   // absolute seconds on NowFn's clock
  using ReportFn = std::function<void(double measured_rtf, int lag_resets)>;

  // rtf = 0 runs as fast as possible. Empty now/sleep_until use std::chrono::steady_clock.
  explicit Pacer(double rtf, NowFn now = {}, SleepUntilFn sleep_until = {}, double max_lag_s = 0.1);

  void reanchor(double sim_time_s);                   // explicit: start / resume after pause
  void wait(double sim_time_s);                       // call once per control tick
  int lagResets() const;                              // only lag-triggered re-anchors
  void setReport(ReportFn fn, double period_s = 5.0); // wall-clock period

 private:
  void anchor(double now_s, double sim_time_s);

  double rtf_;
  NowFn now_;
  SleepUntilFn sleep_until_;
  double max_lag_s_;
  bool anchored_ = false;
  double wall0_ = 0.0, sim0_ = 0.0;
  int lag_resets_ = 0;
  ReportFn report_;
  double report_period_s_ = 5.0;
  double report_wall0_ = 0.0, report_sim0_ = 0.0;
};

}  // namespace gsim
