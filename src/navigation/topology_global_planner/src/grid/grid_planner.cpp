#include "topology_global_planner/grid/grid_planner.hpp"
#include "topology_global_planner/grid/yastar_core.hpp"
namespace topology_global_planner::grid
{
GridPlanner::GridPlanner(const std::string& frame,
                         rclcpp::Clock::SharedPtr clock,
                         Options options)
  : global_frame_(frame)
  , clock_(std::move(clock))
  , options_(options)
{
  if (!clock_ || frame != "map" || !std::isfinite(options.tolerance) ||
      options.tolerance < 0)
    throw std::invalid_argument("Invalid float grid planner configuration");
}
nav_msgs::msg::Path
GridPlanner::plan(const geometry_msgs::msg::PoseStamped& start,
                  const geometry_msgs::msg::PoseStamped& goal,
                  const map_server::MapView & view,
                  GoalPolicy policy,
                  std::function<bool()> cancelled)
{
  if (!cancelled)
    cancelled = [] { return false; };
  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_)
    throw std::invalid_argument("Planner frame mismatch");
  const auto maps = view.derivedMaps();
  if (!maps)
    throw std::runtime_error("No derived map");
  const auto& g = maps->geometry;
  auto a = g.index({ start.pose.position.x, start.pose.position.y }),
       b = g.index({ goal.pose.position.x, goal.pose.position.y });
  if (!a || !b)
    throw std::runtime_error("Planning endpoint outside map");
  auto cells = YAstarCore::search(*maps, *a, *b, 0, cancelled);
  if (cells.empty() && policy == GoalPolicy::WITHIN_TOLERANCE && options_.tolerance > 0)
    cells = YAstarCore::search(*maps, *a, *b, options_.tolerance, cancelled);
  if (cells.empty())
    throw std::runtime_error("No reachable float-cost path");
  nav_msgs::msg::Path out;
  out.header.frame_id = global_frame_;
  out.header.stamp = clock_->now();
  auto pose = start;
  pose.header = out.header;
  out.poses.push_back(pose);
  for (auto cell : cells)
  {
    const auto p = g.center(cell);
    pose.pose.position.x = p.x();
    pose.pose.position.y = p.y();
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    out.poses.push_back(pose);
  }
  if (cells.back() == *b)
  {
    pose = goal;
    pose.header = out.header;
    out.poses.push_back(pose);
  }
  if (options_.use_final_approach_orientation && out.poses.size() > 1)
  {
    const auto& p = out.poses.back().pose.position;
    size_t j = out.poses.size() - 2;
    while (j > 0 && std::hypot(p.x - out.poses[j].pose.position.x,
                               p.y - out.poses[j].pose.position.y) < 1e-6)
      --j;
    const auto& q = out.poses[j].pose.position;
    double yaw = std::atan2(p.y - q.y, p.x - q.x);
    out.poses.back().pose.orientation.z = std::sin(yaw / 2);
    out.poses.back().pose.orientation.w = std::cos(yaw / 2);
  }
  return out;
}
}
