#pragma once
#include "map_server/derived_maps.hpp"
namespace map_server
{
struct CostmapOptions
{
  double robot_radius{ 0.3 }, safety_margin{ 0.05 }, soft_radius{ 0.8 }, alpha{ 0.7 }, beta{ 0.15 };
  bool tdt_unbounded_inverse_distance{ false };
  void validate() const;
};
void
generateCosts(DerivedMaps& maps, const CostmapOptions& options);
}
