#pragma once
#include "map_server/radar_target_cache.hpp"
namespace map_server
{
// Circle vs cell square, including tangent coverage; no ROI clipping.
void
rasterizeRadar(BinaryGrid& grid, const std::vector<RadarTarget>& targets);
void
overlayConservative(BinaryGrid& destination, const BinaryGrid& source);
}
