#include "nav_executor/recovery.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace nav_executor {
Recovery::Recovery(RecoveryConfig config) : config_(config) {
  for (double v :
       {config.stall_duration, config.minimum_progress, config.stopped_speed,
        config.reverse_speed, config.reverse_distance, config.timeout,
        config.cooldown, config.maximum_sample_gap})
    if (!std::isfinite(v) || v <= 0)
      throw std::invalid_argument("Invalid recovery threshold");
  if (!std::isfinite(config.safety_margin) || config.safety_margin < 0 ||
      !std::isfinite(config.hazard_band) || config.hazard_band < 0 ||
      config.max_attempts < 1)
    throw std::invalid_argument("Invalid recovery margin or attempt limit");
}
void Recovery::reset() {
  running_ = false;
  attempts_ = 0;
  previous_.reset();
  stationary_since_.reset();
  next_attempt_ = 0;
  status_ = "IDLE";
}
void Recovery::invalidate() {
  running_ = false;
  previous_.reset();
  stationary_since_.reset();
  status_ = "INPUT_INVALID";
}
bool Recovery::corridorClear(const map_server::MapView &grid, double ax,
                             double ay, double bx, double by, double radius) {
  if (!grid.ready() || !std::isfinite(ax) || !std::isfinite(ay) ||
      !std::isfinite(bx) || !std::isfinite(by) || !std::isfinite(radius) ||
      radius <= 0)
    return false;
  const auto &g = grid.geometry();
  // Sample with an additional half-step radius: the swept disc is covered
  // continuously.
  const int steps =
      std::max(1, static_cast<int>(std::ceil(std::hypot(bx - ax, by - ay) /
                                             (g.resolution * 0.25))));
  const double rr = radius + std::hypot(bx - ax, by - ay) / (2 * steps);
  for (int i = 0; i <= steps; ++i) {
    const double x = ax + (bx - ax) * i / steps, y = ay + (by - ay) * i / steps;
    if (x - rr < g.origin_x || y - rr < g.origin_y ||
        x + rr >= g.origin_x + g.width * g.resolution ||
        y + rr >= g.origin_y + g.height * g.resolution)
      return false;
    const int x0 = std::floor((x - rr - g.origin_x) / g.resolution),
              x1 = std::floor((x + rr - g.origin_x) / g.resolution);
    const int y0 = std::floor((y - rr - g.origin_y) / g.resolution),
              y1 = std::floor((y + rr - g.origin_y) / g.resolution);
    for (int cy = y0; cy <= y1; ++cy)
      for (int cx = x0; cx <= x1; ++cx) {
        if (!grid.latestOccupied(cx, cy))
          continue;
        const double dx = std::max(
            0.0, std::abs(x - (g.origin_x + (cx + 0.5) * g.resolution)) -
                     g.resolution * 0.5);
        const double dy = std::max(
            0.0, std::abs(y - (g.origin_y + (cy + 0.5) * g.resolution)) -
                     g.resolution * 0.5);
        if (dx * dx + dy * dy <= rr * rr)
          return false;
      }
  }
  return true;
}
bool Recovery::update(double now, double x, double y, double ux, double uy,
                      double measured_speed, bool hazard, bool demand,
                      bool stalled, const map_server::MapView &grid,
                      double radius) {
  if (!config_.enabled) {
    reset();
    return false;
  }
  if (!std::isfinite(now) || !std::isfinite(x) || !std::isfinite(y) ||
      !std::isfinite(ux) || !std::isfinite(uy) ||
      !std::isfinite(measured_speed)) {
    invalidate();
    return false;
  }
  if (previous_ &&
      (now <= *previous_ || now - *previous_ > config_.maximum_sample_gap)) {
    invalidate();
    previous_ = now;
    return false;
  }
  previous_ = now;
  if (running_) {
    const double along = (x - start_x_) * ux_ + (y - start_y_) * uy_;
    const double lateral =
        std::abs((x - start_x_) * uy_ - (y - start_y_) * ux_);
    if (along >= config_.reverse_distance ||
        now - start_time_ >= config_.timeout || lateral > 0.10 ||
        along < -0.05) {
      running_ = false;
      stationary_since_.reset();
      next_attempt_ = now + config_.cooldown;
      status_ = along >= config_.reverse_distance
                    ? "COMPLETE"
                    : "ABORTED_TIMEOUT_OR_DEVIATION";
      return false;
    }
    if (!corridorClear(grid, x, y, start_x_ + ux_ * config_.reverse_distance,
                       start_y_ + uy_ * config_.reverse_distance,
                       radius + config_.safety_margin)) {
      running_ = false;
      stationary_since_.reset();
      next_attempt_ = now + config_.cooldown;
      status_ = "REVERSE_BLOCKED";
      return false;
    }
    return true;
  }
  if ((!hazard && !demand && !stalled) ||
      measured_speed > config_.stopped_speed) {
    stationary_since_.reset();
    return false;
  }
  if (!stationary_since_ ||
      std::hypot(x - anchor_x_, y - anchor_y_) >= config_.minimum_progress) {
    stationary_since_ = now;
    anchor_x_ = x;
    anchor_y_ = y;
  }
  if ((!stalled && now - *stationary_since_ < config_.stall_duration) ||
      now < next_attempt_)
    return false;
  if (attempts_ >= config_.max_attempts) {
    status_ = "ATTEMPTS_EXHAUSTED";
    return false;
  }
  ++attempts_;
  next_attempt_ = now + config_.cooldown;
  stationary_since_.reset();
  const double norm = std::hypot(ux, uy);
  if (norm < 1e-6 ||
      !corridorClear(grid, x, y, x + ux / norm * config_.reverse_distance,
                     y + uy / norm * config_.reverse_distance,
                     radius + config_.safety_margin)) {
    status_ = "REVERSE_BLOCKED";
    return false;
  }
  ux_ = ux / norm;
  uy_ = uy / norm;
  start_x_ = x;
  start_y_ = y;
  start_time_ = now;
  running_ = true;
  status_ = "STUCK_REVERSE";
  return true;
}
} // namespace nav_executor
