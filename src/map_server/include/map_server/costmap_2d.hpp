#pragma once
#include "map_server/derived_maps.hpp"
#include <algorithm>
#include <mutex>
namespace map_server
{
// Consumer view of one immutable DerivedMaps. Copies retain the same version.
// The mutex protects replacing the pointer only; no mutable uint8 cost buffer.
class Costmap2D
{
public:
  using mutex_t = std::recursive_mutex;
  Costmap2D() = default;
  Costmap2D(const Costmap2D& other) { *this = other; }
  Costmap2D& operator=(const Costmap2D& other)
  {
    if (this != &other)
    {
      std::scoped_lock lock(mutex_, other.mutex_);
      maps_ = other.maps_;
      dynamic_ = other.dynamic_;
    }
    return *this;
  }
  void setSnapshot(std::shared_ptr<const DerivedMaps> maps)
  {
    std::lock_guard<mutex_t> lock(mutex_);
    maps_ = std::move(maps);
  }
  std::shared_ptr<const DerivedMaps> getDerivedMaps() const
  {
    std::lock_guard<mutex_t> lock(mutex_);
    return maps_;
  }
  bool worldToMap(double wx, double wy, unsigned int& x, unsigned int& y) const
  {
    if (!maps_)
      return false;
    auto i = maps_->geometry.index({ wx, wy });
    if (!i)
      return false;
    x = *i % maps_->geometry.width;
    y = *i / maps_->geometry.width;
    return true;
  }
  void mapToWorld(unsigned x, unsigned y, double& wx, double& wy) const
  {
    const auto p = maps_->geometry.center(static_cast<size_t>(y) * getSizeInCellsX() + x);
    wx = p.x();
    wy = p.y();
  }
  float getCost(unsigned x, unsigned y) const
  {
    return getCost(static_cast<size_t>(y) * getSizeInCellsX() + x);
  }
  float getCost(size_t index) const { return maps_->costmap.data.at(index); }
  void setLatestDynamic(std::shared_ptr<const DynamicObstacleView> view)
  {
    dynamic_ = std::move(view);
  }
  bool rawObstacle(unsigned x, unsigned y) const
  {
    if (maps_->fused_obstacles.data.at(static_cast<size_t>(y) * getSizeInCellsX() + x))
      return true;
    if (!dynamic_)
      return false;
    const auto p = maps_->geometry.center(static_cast<size_t>(y) * getSizeInCellsX() + x);
    const double half = getResolution() * 0.5;
    if (dynamic_->cloud_mask)
    {
      const auto& cloud = *dynamic_->cloud_mask;
      const auto& g = cloud.geometry;
      const auto bound=[](double v, unsigned n) {
        return static_cast<int>(std::clamp(v,0.0,static_cast<double>(n)));
      };
      const int x0=bound(std::floor((p.x()-half-g.origin_x)/g.resolution),g.width);
      const int y0=bound(std::floor((p.y()-half-g.origin_y)/g.resolution),g.height);
      const int x1=bound(std::ceil((p.x()+half-g.origin_x)/g.resolution),g.width);
      const int y1=bound(std::ceil((p.y()+half-g.origin_y)/g.resolution),g.height);
      for(int cy=y0;cy<y1;++cy)for(int cx=x0;cx<x1;++cx)
        if(cloud.data.at(static_cast<size_t>(cy)*g.width+cx)) return true;
    }
    for (const auto* targets : { &dynamic_->allies, &dynamic_->enemies })
      for (const auto& t : *targets)
      {
        const double dx = std::max(0.0, std::abs(t.xy_map.x() - p.x()) - half);
        const double dy = std::max(0.0, std::abs(t.xy_map.y() - p.y()) - half);
        if (dx * dx + dy * dy <= t.physical_radius_m * t.physical_radius_m)
          return true;
      }
    return false;
  }
  unsigned getSizeInCellsX() const { return maps_ ? maps_->geometry.width : 0; }
  unsigned getSizeInCellsY() const { return maps_ ? maps_->geometry.height : 0; }
  double getResolution() const { return maps_ ? maps_->geometry.resolution : 1; }
  double getOriginX() const { return maps_ ? maps_->geometry.origin_x : 0; }
  double getOriginY() const { return maps_ ? maps_->geometry.origin_y : 0; }
  const float* getCostData() const { return maps_->costmap.data.data(); }
  mutex_t* getMutex() const { return &mutex_; }

private:
  std::shared_ptr<const DerivedMaps> maps_;
  std::shared_ptr<const DynamicObstacleView> dynamic_;
  mutable mutex_t mutex_;
};
}
