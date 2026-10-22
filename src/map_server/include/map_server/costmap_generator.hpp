#pragma once
#include "map_server/derived_maps.hpp"
namespace map_server
{
struct CostmapOptions
{
  double robot_radius{ 0.43 }, safety_margin{ 0.05 }, soft_radius{ 1.5 }, alpha{ 1.0 }, beta{ 0.1 };
  bool tdt_unbounded_inverse_distance{ false };
  void validate() const;
};
void
generateCosts(DerivedMaps& maps, const CostmapOptions& options);
}
