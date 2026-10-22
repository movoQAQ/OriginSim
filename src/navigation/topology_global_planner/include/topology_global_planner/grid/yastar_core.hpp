#pragma once
#include "map_server/derived_maps.hpp"
#include <algorithm>
#include <functional>
#include <queue>
namespace topology_global_planner::grid
{
// Float-map A* consumer boundary, independent of ROS and map production.
// Edges integrate trapezoidal endpoint cost times metric edge length.
// No internal EDT, mask history or inflation; an inf cell is never enterable.
class YAstarCore
{
public:
  static std::vector<size_t> search(const map_server::DerivedMaps& m,
                                    size_t start,
                                    size_t goal,
                                    double tolerance,
                                    const std::function<bool()>& cancelled)
  {
    m.costmap.validate();
    m.collision_mask.validate();
    const auto& g = m.geometry;
    const size_t n = g.size();
    if (start >= n || goal >= n || tolerance < 0 || !std::isfinite(tolerance))
      throw std::invalid_argument("Invalid A* endpoints");
    const auto free = [&](size_t i)
    {
      return !m.collision_mask.data.at(i) && std::isfinite(m.costmap.data.at(i)) &&
             m.costmap.data[i] >= 1;
    };
    if (!free(start))
      throw std::runtime_error("A* start is hard-blocked");
    const auto target = g.center(goal);
    const auto heuristic = [&](size_t i)
    { return std::max(0.0, (g.center(i) - target).norm() - tolerance); };
    struct Item
    {
      double priority, distance;
      size_t index;
      bool operator<(const Item& b) const { return priority > b.priority; }
    };
    std::priority_queue<Item> open;
    std::vector<double> distance(n, INFINITY);
    std::vector<size_t> parent(n, n);
    distance[start] = 0;
    open.push({ heuristic(start), 0, start });
    size_t found = n, expansions = 0;
    while (!open.empty())
    {
      auto current = open.top();
      open.pop();
      const auto i = current.index;
      if (current.distance != distance[i])
        continue;
      if ((++expansions % 256) == 0 && cancelled())
        throw std::runtime_error("Planning cancelled");
      if (i == goal || (tolerance > 0 && (g.center(i) - target).norm() <= tolerance))
      {
        found = i;
        break;
      }
      int x = i % g.width, y = i / g.width;
      for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx)
        {
          if (!dx && !dy)
            continue;
          int xx = x + dx, yy = y + dy;
          if (xx < 0 || yy < 0 || xx >= static_cast<int>(g.width) ||
              yy >= static_cast<int>(g.height))
            continue;
          size_t j = static_cast<size_t>(yy) * g.width + xx;
          if (!free(j))
            continue;
          if (dx && dy &&
              (!free(static_cast<size_t>(y) * g.width + xx) ||
               !free(static_cast<size_t>(yy) * g.width + x)))
            continue;
          double d = distance[i] + g.resolution * std::hypot(dx, dy) *
                                     (m.costmap.data[i] + m.costmap.data[j]) * 0.5;
          if (d < distance[j])
          {
            distance[j] = d;
            parent[j] = i;
            open.push({ d + heuristic(j), d, j });
          }
        }
    }
    if (cancelled())
      throw std::runtime_error("Planning cancelled");
    if (found == n)
      return {};
    std::vector<size_t> path;
    for (size_t i = found;; i = parent[i])
    {
      path.push_back(i);
      if (i == start)
        break;
    }
    std::reverse(path.begin(), path.end());
    return path;
  }
};
}
