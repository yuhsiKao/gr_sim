// When the viewer copies the simulation state: once per `period` of simulated time. A jump
// back in time (MuJoCo resets mjData after an instability) restarts the schedule instead of
// freezing the window until the clock catches up with the old time.
#pragma once

namespace gsim {

class SyncSchedule {
 public:
  explicit SyncSchedule(double period) : period_(period) {}

  bool due(double sim_time) {
    if (sim_time + period_ < next_) next_ = sim_time;  // time went backwards
    if (sim_time < next_) return false;
    next_ = sim_time + period_;
    return true;
  }

 private:
  double period_;
  double next_ = 0.0;
};

}  // namespace gsim
