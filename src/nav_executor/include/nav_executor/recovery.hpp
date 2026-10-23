#pragma once
#include "map_server/map_view.hpp"
#include <optional>
#include <string>

namespace nav_executor {
struct RecoveryConfig {
  bool enabled{true};
  double stall_duration{3.0}, minimum_progress{0.05}, stopped_speed{0.05};
  double reverse_speed{0.15}, reverse_distance{0.30}, timeout{3.0},
      cooldown{5.0};
  double safety_margin{0.01}, hazard_band{0.10}, maximum_sample_gap{0.25};
  int max_attempts{3};
};
// Pure geometric/time state, driven only by fresh observations. Direction is
// map-frame.
class Recovery {
public:
  explicit Recovery(RecoveryConfig config = {});
  bool update(double now, double x, double y, double ux, double uy,
              double measured_speed, bool hazard, bool demand, bool stalled,
              const map_server::MapView &grid, double radius);
  void reset();
  void invalidate();
  bool running() const { return running_; }
  double vx() const { return running_ ? ux_ * config_.reverse_speed : 0.0; }
  double vy() const { return running_ ? uy_ * config_.reverse_speed : 0.0; }
  const std::string &status() const { return status_; }
  static bool corridorClear(const map_server::MapView &grid, double ax,
                            double ay, double bx, double by, double radius);

private:
  RecoveryConfig config_;
  bool running_{false};
  int attempts_{0};
  std::optional<double> previous_, stationary_since_;
  double anchor_x_{0}, anchor_y_{0}, start_x_{0}, start_y_{0}, start_time_{0};
  double ux_{0}, uy_{0}, next_attempt_{0};
  std::string status_{"IDLE"};
};
} // namespace nav_executor
