#pragma once

#include <limits>
#include <optional>
#include "map_server/map_view.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav_executor
{
// 固定只读 MapView，沿路径段扫描圆形车体轮廓；无需调用方加锁
std::optional<double> findPathObstacle(const nav_msgs::msg::Path & path,
  const map_server::MapView & grid,
  double radius,
  double begin = 0.0,
  double end = std::numeric_limits<double>::infinity());
bool pathIsClear(
  const nav_msgs::msg::Path & path, const map_server::MapView & grid, double radius);
} // 命名空间 nav_executor
