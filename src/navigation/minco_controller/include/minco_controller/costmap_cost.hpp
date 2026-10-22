#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "map_server/map_view.hpp"
#include "minco_controller/mpc_types.hpp"

namespace minco_controller
{
// An immutable snapshot keeps every sequential QP on one float map version.
// Gaussian averaging regularizes the reciprocal-distance potential; radius is 3 sigma.
class CostmapCost
{
public:
  CostmapCost(const map_server::MapView & grid, double sigma,
    double collision_radius = 0.0, double trust_region = 0.15)
  : grid_(grid), sigma_(std::max(sigma, grid_.geometry().resolution)),
    collision_radius_(collision_radius), trust_region_(trust_region)
  {
    if (!std::isfinite(collision_radius) || collision_radius < 0.0 ||
      !std::isfinite(trust_region) || trust_region <= 0.0 ||
      !std::isfinite(sigma) || sigma <= 0.0 || grid_.geometry().width < 3 ||
      grid_.geometry().height < 3 || grid_.geometry().resolution <= 0.0)
    {throw std::invalid_argument("MPC requires a valid cost grid and smoothing distance");}
  }

  bool sample(const Eigen::Vector2d & position, CostLinearization & result) const
  {
    unsigned int mx, my;
    if (!grid_.worldToCell(position.x(), position.y(), mx, my)) {return false;}
    const double resolution = grid_.geometry().resolution;
    const int cells = static_cast<int>(std::ceil(3.0 * sigma_ / resolution));
    double weights = 0.0, weighted_cost = 0.0;
    Eigen::Vector2d weight_gradient = Eigen::Vector2d::Zero();
    Eigen::Vector2d cost_gradient = Eigen::Vector2d::Zero();
    for (int dy = -cells; dy <= cells; ++dy) {
      for (int dx = -cells; dx <= cells; ++dx) {
        const int ix = static_cast<int>(mx) + dx, iy = static_cast<int>(my) + dy;
        const Eigen::Vector2d center(grid_.geometry().origin_x + (ix + 0.5) * resolution,
          grid_.geometry().origin_y + (iy + 0.5) * resolution);
        const Eigen::Vector2d delta = center - position;
        const double weight = std::exp(-0.5 * delta.squaredNorm() / (sigma_ * sigma_));
        // Unknown cells and the exterior are high cost, never free space.
        const double cost = ix < 0 || iy < 0 || ix >= static_cast<int>(grid_.geometry().width) ||
          iy >= static_cast<int>(grid_.geometry().height) ? 1.0 :
          (std::isfinite(grid_.cost(ix, iy)) ? 1.0 - 1.0 / grid_.cost(ix, iy) : 1.0);
        const Eigen::Vector2d gradient = weight * delta / (sigma_ * sigma_);
        weights += weight;
        weighted_cost += weight * cost;
        weight_gradient += gradient;
        cost_gradient += cost * gradient;
      }
    }
    result.position = position;
    result.cost = weighted_cost / weights;
    result.gradient = (cost_gradient - result.cost * weight_gradient) / weights;
    result.safety_planes.clear();
    if (collision_radius_ > 0.0) {
      // A supporting plane of cell (+) disc is conservative and linear. Use
      // raw fused cell squares, without expanding CollisionMask again.
      const auto add_plane = [&](const Eigen::Vector2d & normal, double offset) {
          for (auto & plane : result.safety_planes) {
            if ((plane.normal - normal).squaredNorm() < 1e-12) {
              plane.offset = std::max(plane.offset, offset);
              return;
            }
          }
          result.safety_planes.push_back({normal, offset});
        };
      const double radius = collision_radius_ + 0.001;
      add_plane(Eigen::Vector2d(1, 0), grid_.geometry().origin_x + radius);
      add_plane(Eigen::Vector2d(-1, 0),
        -(grid_.geometry().origin_x + grid_.geometry().width * resolution - radius));
      add_plane(Eigen::Vector2d(0, 1), grid_.geometry().origin_y + radius);
      add_plane(Eigen::Vector2d(0, -1),
        -(grid_.geometry().origin_y + grid_.geometry().height * resolution - radius));
      const double reach = radius + std::sqrt(2.0) * trust_region_ + resolution;
      const int extent = static_cast<int>(std::ceil(reach / resolution)) + 1;
      for (int dy = -extent; dy <= extent; ++dy) {
        for (int dx = -extent; dx <= extent; ++dx) {
          const int ix = static_cast<int>(mx) + dx, iy = static_cast<int>(my) + dy;
          if (ix < 0 || iy < 0 || ix >= static_cast<int>(grid_.geometry().width) ||
            iy >= static_cast<int>(grid_.geometry().height) ||
            !grid_.latestOccupied(ix, iy)) {continue;}
          const Eigen::Vector2d lower(grid_.geometry().origin_x + ix * resolution,
            grid_.geometry().origin_y + iy * resolution);
          const Eigen::Vector2d nearest = position.cwiseMax(lower).cwiseMin(
            lower + Eigen::Vector2d::Constant(resolution));
          const Eigen::Vector2d delta = position - nearest;
          const double distance = delta.norm();
          if (distance > reach) {continue;}
          // A nominal center inside the actual obstacle has no unique local
          // separating normal. Report invalid data instead of picking a side.
          if (distance < 1e-9) {return false;}
          const Eigen::Vector2d normal = delta / distance;
          add_plane(normal, normal.dot(nearest) + radius);
        }
      }
    }
    return std::isfinite(result.cost) && result.gradient.allFinite();
  }

private:
  const map_server::MapView grid_;
  double sigma_;
  double collision_radius_, trust_region_;
};
}  // namespace minco_controller
