#pragma once
#include "map_server/radar_target_cache.hpp"
#include <memory>
namespace map_server {
struct DynamicHealth {
  SourceStatus cloud{SourceStatus::MISSING},
      radar_allies{SourceStatus::MISSING}, radar_enemies{SourceStatus::MISSING};
  double cloud_observation_age_s{INFINITY}, ally_observation_age_s{INFINITY},
      enemy_observation_age_s{INFINITY};
  bool self_filter_verified{false}, radar_time_degraded{false},
      map_error{false};
};
struct ObstacleCircle {
  Eigen::Vector2d center;
  double radius_m;
};
struct DynamicObstacleView {
  uint64_t input_version{0};
  std::shared_ptr<const BinaryGrid> cloud_mask;
  std::vector<ObstacleCircle> circles;
};
struct DerivedMaps {
  uint64_t version{0},
      input_version{0}; // version = obstacle_version = derived_version
  GridGeometry geometry;
  BinaryGrid fused_obstacles;
  DistanceGrid distance_field;
  CostGrid costmap;
  BinaryGrid collision_mask;
  double generated_at_s{0}, hard_radius_m{0}, grid_margin_m{0};
  double fusion_ms{0}, edt_ms{0}, costmap_ms{0}, cloud_ms{0};
};
} // namespace map_server
