#include "map_server/costmap_generator.hpp"
#include <algorithm>
#include <limits>
namespace map_server
{
void
CostmapOptions::validate() const
{
  if (!std::isfinite(robot_radius) || robot_radius <= 0 || !std::isfinite(safety_margin) ||
      safety_margin < 0 || !std::isfinite(grid_margin_factor) || grid_margin_factor < 0 || !std::isfinite(soft_radius) ||
      soft_radius <= robot_radius + safety_margin || !std::isfinite(alpha) || alpha < 0 ||
      !std::isfinite(beta) || beta <= 0)
    throw std::invalid_argument("Invalid costmap options");
}
void
generateCosts(DerivedMaps& m, const CostmapOptions& o)
{
  o.validate();
  m.fused_obstacles.validate();
  m.distance_field.validate();
  if (!(m.geometry == m.distance_field.geometry) || !(m.geometry == m.fused_obstacles.geometry))
    throw std::invalid_argument("Derived geometry mismatch");
  m.hard_radius_m = o.robot_radius + o.safety_margin;
  m.costmap = { m.geometry, std::vector<float>(m.geometry.size(), 1) };
  m.collision_mask = { m.geometry, std::vector<uint8_t>(m.geometry.size(), 0) };
  const auto& g = m.geometry;
  const double discrete = g.resolution * o.grid_margin_factor;
  m.grid_margin_m = discrete;
  for (size_t i = 0; i < g.size(); ++i)
  {
    double d = m.distance_field.data[i];
    if (std::isnan(d) || d < 0)
      throw std::invalid_argument("Invalid distance field");
    const auto p = g.center(i);
    const double boundary = std::min({ p.x() - g.origin_x,
                                       p.y() - g.origin_y,
                                       g.origin_x + g.width * g.resolution - p.x(),
                                       g.origin_y + g.height * g.resolution - p.y() });
    const bool collision = m.fused_obstacles.data[i] || d <= m.hard_radius_m + discrete ||
                           boundary <= m.hard_radius_m + discrete;
    m.collision_mask.data[i] = collision ? 1 : 0;
    double cost = 1;
    if (collision)
      cost = INFINITY;
    else if (o.tdt_unbounded_inverse_distance)
      cost = 1 + o.alpha / (o.beta + d);
    else if (d < o.soft_radius)
      cost = 1 + o.alpha * (1 / (o.beta + d) - 1 / (o.beta + o.soft_radius));
    if (!collision && (!std::isfinite(cost) || cost > std::numeric_limits<float>::max()))
      throw std::overflow_error("Float costmap overflow");
    m.costmap.data[i] = static_cast<float>(cost);
  }
}
}
