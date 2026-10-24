#pragma once
#include <Eigen/Core>
#include <cmath>
#include <deque>

// Estimate the output prior from corrected LiDAR positions. The filter's
// instantaneous velocity can retain IMU bias despite stationary scan matches.
// This helper never changes the LiDAR filter state or its measurement updates.
class CorrectedVelocity {
 public:
  Eigen::Vector3d update(double stamp, const Eigen::Vector3d & position,
                        const Eigen::Vector3d & fallback) {
    if (!std::isfinite(stamp) || !position.allFinite()) {
      history_.clear(); return fallback;
    }
    if (!history_.empty()) {
      const double dt=stamp-history_.back().stamp;
      if (dt<=0 || dt>1.0 ||
          (position-history_.back().position).norm()>2.5*dt+0.15) history_.clear();
    }
    history_.push_back({stamp,position});
    while (history_.size()>128 || (history_.size()>2 && history_[1].stamp<stamp-0.5)) history_.pop_front();
    if (history_.size()<3 || stamp-history_.front().stamp<0.3) return fallback;
    double mean_time=0;
    Eigen::Vector3d mean_position=Eigen::Vector3d::Zero();
    for (const auto & item:history_) {
      mean_time+=item.stamp-stamp; mean_position+=item.position;
    }
    mean_time/=history_.size(); mean_position/=history_.size();
    double variance=0;
    Eigen::Vector3d covariance=Eigen::Vector3d::Zero();
    for (const auto & item:history_) {
      const double dt=item.stamp-stamp-mean_time;
      variance+=dt*dt; covariance+=dt*(item.position-mean_position);
    }
    const Eigen::Vector3d velocity=covariance/variance;
    return velocity.allFinite() && velocity.norm()<2.5 ? velocity : fallback;
  }
 private:
  struct Sample {double stamp; Eigen::Vector3d position;};
  std::deque<Sample> history_;
};
